#!/usr/bin/env python3
"""Line-oriented, watchdog-protected bridge for the two moteus wrist axes.

Protocol on stdin (SI units):
  P <q1_rad> <q2_rad>      position mode
  P8 <q1_rad>              position first configured axis; STOP second axis
  T <tau1_Nm> <tau2_Nm>    pure feed-forward torque mode
  Z                         position both axes at calibrated zero
  A                         arm the runtime command watchdog
  S                         STOP both axes and exit

At startup, only axes selected by ``--zero-hold-mask`` move to their configured
zero. Unselected axes remain in STOP and are read only. It never changes
persistent moteus configuration.
"""

from __future__ import annotations

import argparse
import asyncio
from dataclasses import dataclass
import math
import select
import sys
import time
from typing import Any, Sequence

try:
    import moteus
except ImportError as exc:
    raise SystemExit("请使用 .venv_moteus/bin/python 运行 wrist_moteus_bridge.py") from exc


TAU = 2.0 * math.pi
COMMAND_TIMEOUT_S = 1.0


@dataclass(frozen=True)
class Sample:
    position_rev: float
    velocity_rev_s: float
    torque_nm: float
    mode: int | None
    fault: int | None


def _value(state: Any, register: Any, default: Any = None) -> Any:
    return state.values.get(register, default)


def _sample(state: Any) -> Sample:
    return Sample(
        position_rev=float(_value(state, moteus.Register.POSITION, math.nan)),
        velocity_rev_s=float(_value(state, moteus.Register.VELOCITY, math.nan)),
        torque_nm=float(_value(state, moteus.Register.TORQUE, math.nan)),
        mode=(None if _value(state, moteus.Register.MODE) is None else int(_value(state, moteus.Register.MODE))),
        fault=(None if _value(state, moteus.Register.FAULT) is None else int(_value(state, moteus.Register.FAULT))),
    )


def _validate(sample: Sample, motor_id: int) -> None:
    if not all(math.isfinite(value) for value in (sample.position_rev, sample.velocity_rev_s)):
        raise RuntimeError(
            "电机 ID {} 返回无效状态: position_rev={} velocity_rev_s={} "
            "mode={} fault={}".format(
                motor_id,
                sample.position_rev,
                sample.velocity_rev_s,
                sample.mode,
                sample.fault,
            )
        )
    if sample.fault not in (None, 0):
        raise RuntimeError(f"电机 ID {motor_id} fault={sample.fault}")
    if sample.mode == 1:
        raise RuntimeError(f"电机 ID {motor_id} 进入 FAULT mode")


def _q_from_sample(sample: Sample, zero_rev: float, sign: float) -> tuple[float, float, float]:
    return (
        sign * (sample.position_rev - zero_rev) * TAU,
        sign * sample.velocity_rev_s * TAU,
        sign * sample.torque_nm,
    )


def _nearest_equivalent_zero_rev(configured_zero_rev: float, current_rev: float) -> tuple[float, int]:
    """Shift a periodic absolute zero by whole turns to be nearest current.

    Some output encoders report the same physical pose in a neighboring
    revolution after controller/USB power cycles.  Position commands must use
    the equivalent zero in that same revolution, otherwise a short move could
    be misread or commanded as an almost-complete turn.
    """
    turn_offset = int(round(current_rev - configured_zero_rev))
    return configured_zero_rev + turn_offset, turn_offset


async def _command_with_timeout(awaitable: Any, motor_id: int, operation: str) -> Any:
    try:
        return await asyncio.wait_for(awaitable, timeout=COMMAND_TIMEOUT_S)
    except asyncio.TimeoutError as error:
        raise TimeoutError(
            f"电机 ID {motor_id} {operation} 通信超时 {COMMAND_TIMEOUT_S:.1f}s"
        ) from error


async def _stop_all(
    controllers: Sequence[Any], ids: Sequence[int], *, strict: bool = False
) -> None:
    errors: list[str] = []
    for controller, motor_id in zip(controllers, ids):
        try:
            await _command_with_timeout(controller.set_stop(), motor_id, "STOP")
        except BaseException as error:
            details = f"ID{motor_id}:{type(error).__name__}:{error}"
            errors.append(details)
            print(f"WRIST_WARNING stop_failed={details}", file=sys.stderr)
    if errors and strict:
        raise RuntimeError("腕部启动 STOP 未确认: " + "; ".join(errors))


async def _query_all(controllers: Sequence[Any], ids: Sequence[int]) -> list[Sample]:
    result: list[Sample] = []
    for controller, motor_id in zip(controllers, ids):
        state = await _command_with_timeout(controller.query(), motor_id, "QUERY")
        if state is None:
            raise RuntimeError(f"电机 ID {motor_id} 没有响应")
        sample = _sample(state)
        _validate(sample, motor_id)
        result.append(sample)
    return result


async def _capture_position_mode_at_current_pose(
    controller: Any,
    motor_id: int,
    sample: Sample,
    *,
    maximum_torque_nm: float,
    kp_scale: float,
    kd_scale: float,
    watchdog_s: float,
) -> Sample:
    """Enter position mode at the sensed pose without a trajectory jump.

    The user's controllers run 2024 firmware while the installed Python
    package exposes newer registers.  Do not depend on the later
    RECAPTURE_POSITION_VELOCITY command here.  Instead, enter position mode
    with the measured position as an immediate (unprofiled) target.  The next
    normal command enables the configured velocity/acceleration trajectory
    from that captured pose.

    This changes no persistent moteus configuration and does not redefine the
    calibrated output position.
    """
    state = await _command_with_timeout(
        controller.set_position(
            position=sample.position_rev,
            velocity=0.0,
            # Explicit NaN overrides disable both trajectory limits for this
            # single current-pose capture.  Omitting them would allow the
            # controller's persistent defaults to remain active.
            velocity_limit=math.nan,
            accel_limit=math.nan,
            maximum_torque=maximum_torque_nm,
            kp_scale=kp_scale,
            kd_scale=kd_scale,
            watchdog_timeout=watchdog_s,
            # Application-level q limits are checked on every sample. Ignore
            # stale persistent servopos min/max values that were configured
            # around an older zero and can otherwise clamp a new target to an
            # unrelated raw revolution.
            ignore_position_bounds=1,
            query=True,
        ),
        motor_id,
        "POSITION_CAPTURE",
    )
    if state is None:
        raise RuntimeError(f"电机 ID {motor_id} 位置接管没有状态响应")
    captured = _sample(state)
    _validate(captured, motor_id)
    return captured


async def _move_to_zero(
    controllers: Sequence[Any],
    ids: Sequence[int],
    zero_rev: Sequence[float],
    *,
    velocity_deg_s: float,
    accel_deg_s2: float,
    maximum_torque_nm: Sequence[float],
    kp_scale: Sequence[float],
    kd_scale: Sequence[float],
    loop_hz: float,
    tolerance_deg: float,
    settle_seconds: float,
    timeout_s: float,
    maximum_error_deg: Sequence[float],
    active_mask: Sequence[int],
) -> list[Sample]:
    period = 1.0 / loop_hz
    started = time.monotonic()
    settled_since: float | None = None
    samples: list[Sample] = []
    next_status = started
    while True:
        cycle = time.monotonic()
        samples = []
        for controller, motor_id, target, torque_limit, kp, kd, active in zip(
            controllers,
            ids,
            zero_rev,
            maximum_torque_nm,
            kp_scale,
            kd_scale,
            active_mask,
        ):
            if active:
                state = await _command_with_timeout(
                    controller.set_position(
                        position=target,
                        velocity=0.0,
                        velocity_limit=velocity_deg_s / 360.0,
                        accel_limit=accel_deg_s2 / 360.0,
                        maximum_torque=torque_limit,
                        kp_scale=kp,
                        kd_scale=kd,
                        watchdog_timeout=max(0.1, 3.0 * period),
                        ignore_position_bounds=1,
                        query=True,
                    ),
                    motor_id,
                    "ZERO_POSITION",
                )
            else:
                state = await _command_with_timeout(
                    controller.query(), motor_id, "PASSIVE_QUERY"
                )
            if state is None:
                raise RuntimeError(f"电机 ID {motor_id} 零位命令没有状态响应")
            sample = _sample(state)
            _validate(sample, motor_id)
            samples.append(sample)
        errors_deg = [abs((sample.position_rev - target) * 360.0) for sample, target in zip(samples, zero_rev)]
        velocities_deg_s = [abs(sample.velocity_rev_s * 360.0) for sample in samples]
        now = time.monotonic()
        for motor_id, error, error_limit, active in zip(
            ids, errors_deg, maximum_error_deg, active_mask
        ):
            if active and error > error_limit:
                raise RuntimeError(
                    f"电机 ID {motor_id} 启动目标偏差扩大到 {error:.2f}deg，"
                    f"超过保护值 {error_limit:.2f}deg"
                )
        if now >= next_status:
            details = " ".join(
                "ID{}[pos={:+.6f}rev error={:.3f}deg vel={:+.3f}deg/s "
                "torque={:+.3f}Nm mode={} fault={}]".format(
                    motor_id,
                    sample.position_rev,
                    error,
                    sample.velocity_rev_s * 360.0,
                    sample.torque_nm,
                    sample.mode,
                    sample.fault,
                )
                for motor_id, sample, error in zip(ids, samples, errors_deg)
            )
            print(
                f"WRIST_ZEROING_STATUS elapsed_s={now - started:.1f} {details}",
                flush=True,
            )
            next_status = now + 1.0
        active_errors = [
            error for error, active in zip(errors_deg, active_mask) if active
        ]
        active_speeds = [
            speed for speed, active in zip(velocities_deg_s, active_mask) if active
        ]
        if all(error <= tolerance_deg for error in active_errors) and all(
            speed <= 2.0 for speed in active_speeds
        ):
            if settled_since is None:
                settled_since = now
            elif now - settled_since >= settle_seconds:
                return samples
        else:
            settled_since = None
        if now - started > timeout_s:
            details = ", ".join(f"ID {motor_id} error={error:.2f}deg" for motor_id, error in zip(ids, errors_deg))
            raise TimeoutError(f"腕部回绝对零位超时: {details}")
        await asyncio.sleep(max(0.0, period - (time.monotonic() - cycle)))


def _parse_command(line: str) -> tuple[str, list[float]]:
    fields = line.strip().split()
    if not fields:
        return "", []
    operation = fields[0].upper()
    expected = {"P": 2, "P8": 1, "T": 2, "Z": 0, "A": 0, "S": 0}
    if operation not in expected or len(fields) - 1 != expected[operation]:
        raise ValueError("命令必须是 P q1 q2、P8 q1、T tau1 tau2、Z、A 或 S")
    values = [float(value) for value in fields[1:]]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("命令包含 NaN/Inf")
    return operation, values


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="两轴 moteus 腕部进程桥")
    parser.add_argument("--ids", nargs=2, type=int, default=(1, 2))
    parser.add_argument("--zero-rev", nargs=2, type=float, required=True)
    parser.add_argument(
        "--zero-hold-mask",
        nargs=2,
        type=int,
        choices=(0, 1),
        default=(1, 1),
        help="1=启动时回标定零位，0=保持 STOP、只读取当前位置",
    )
    parser.add_argument("--motor-sign", nargs=2, type=float, default=(1.0, 1.0))
    parser.add_argument(
        "--limit-deg",
        nargs=2,
        type=float,
        default=(180.0, 90.0),
        help="以逻辑零位为中心、按 --ids 顺序给出的两轴半行程",
    )
    parser.add_argument(
        "--reduction-ratio",
        nargs=2,
        type=float,
        required=True,
        help="仅用于记录/核对；moteus position/torque 命令本身已使用输出轴单位",
    )
    parser.add_argument("--max-torque-nm", nargs=2, type=float, default=(0.5, 0.5))
    parser.add_argument("--position-torque-nm", nargs=2, type=float, default=(0.5, 0.5))
    parser.add_argument(
        "--position-kp-scale",
        nargs=2,
        type=float,
        default=(0.05, 0.05),
        help="位置命令对 moteus 永久 kp 的临时缩放，不写入固件",
    )
    parser.add_argument(
        "--position-kd-scale",
        nargs=2,
        type=float,
        default=(0.5, 0.5),
        help="位置命令对 moteus 永久 kd 的临时缩放，不写入固件",
    )
    parser.add_argument("--velocity-deg-s", type=float, default=20.0)
    parser.add_argument("--accel-deg-s2", type=float, default=40.0)
    parser.add_argument("--zero-tolerance-deg", type=float, default=2.0)
    parser.add_argument("--zero-settle-seconds", type=float, default=0.2)
    parser.add_argument("--zero-timeout-s", type=float, default=30.0)
    parser.add_argument("--runtime-velocity-deg-s", type=float, default=45.0)
    parser.add_argument("--runtime-accel-deg-s2", type=float, default=180.0)
    parser.add_argument(
        "--position-following-error-deg",
        type=float,
        default=20.0,
        help=(
            "位置模式测量角与实际下发目标的最大允许偏差；这是异常跟随检测，"
            "不是工作空间限位"
        ),
    )
    parser.add_argument("--loop-hz", type=float, default=100.0)
    parser.add_argument("--command-timeout-s", type=float, default=1.0)
    parser.add_argument("--watchdog-ms", type=float, default=100.0)
    parser.add_argument("--soft-limit-margin-deg", type=float, default=5.0)
    parser.add_argument("--soft-limit-stiffness-nm-rad", type=float, default=1.0)
    parser.add_argument("--soft-limit-damping-nm-s-rad", type=float, default=0.08)
    parser.add_argument("--allow-torque-control", action="store_true")
    parser.add_argument("--confirm-torque", default="")
    moteus.make_transport_args(parser)
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if len(set(args.ids)) != 2 or any(value <= 0 for value in args.ids):
        raise ValueError("--ids 必须是两个不同正整数")
    if any(sign not in (-1.0, 1.0) for sign in args.motor_sign):
        raise ValueError("--motor-sign 只能是 -1 或 +1")
    if any(value not in (0, 1) for value in args.zero_hold_mask):
        raise ValueError("--zero-hold-mask 只能包含 0 或 1")
    positive_fields = (
        *args.limit_deg,
        *args.reduction_ratio,
        *args.max_torque_nm,
        *args.position_torque_nm,
        *args.position_kp_scale,
        *args.position_kd_scale,
        args.velocity_deg_s,
        args.accel_deg_s2,
        args.zero_tolerance_deg,
        args.zero_settle_seconds,
        args.zero_timeout_s,
        args.runtime_velocity_deg_s,
        args.runtime_accel_deg_s2,
        args.position_following_error_deg,
        args.loop_hz,
        args.command_timeout_s,
        args.watchdog_ms,
        args.soft_limit_margin_deg,
        args.soft_limit_stiffness_nm_rad,
        args.soft_limit_damping_nm_s_rad,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in positive_fields):
        raise ValueError("腕部限位、速率、扭矩和 watchdog 参数必须为有限正数")
    if any(value > 1.0 for value in (*args.position_kp_scale, *args.position_kd_scale)):
        raise ValueError("位置环 kp/kd 临时缩放必须在 (0, 1] 范围内")
    if args.soft_limit_margin_deg >= min(args.limit_deg):
        raise ValueError("soft-limit margin 必须小于关节角度范围")
    if any(value <= 1.0 for value in args.reduction_ratio):
        raise ValueError("--reduction-ratio 必须全部大于 1")
    if args.loop_hz < 20.0 or args.loop_hz > 500.0:
        raise ValueError("--loop-hz 必须在 20..500 Hz")
    if args.allow_torque_control and args.confirm_torque != "MOTEUS_TORQUE_MODE":
        raise ValueError("扭矩模式必须同时设置 --confirm-torque MOTEUS_TORQUE_MODE")


async def _run(args: argparse.Namespace) -> None:
    global COMMAND_TIMEOUT_S
    COMMAND_TIMEOUT_S = args.command_timeout_s
    transport = moteus.get_singleton_transport(args)
    controllers = [moteus.Controller(id=motor_id, transport=transport) for motor_id in args.ids]
    limit_rad = [math.radians(value) for value in args.limit_deg]
    soft_margin_rad = math.radians(args.soft_limit_margin_deg)
    period = 1.0 / args.loop_hz
    operation = "P"
    command = [0.0, 0.0]
    last_command = time.monotonic()
    watchdog_armed = False
    next_position_status = time.monotonic()
    try:
        await _stop_all(controllers, args.ids, strict=True)
        samples = await _query_all(controllers, args.ids)
        session_zero_rev: list[float] = []
        for motor_id, sample, configured_zero in zip(
            args.ids, samples, args.zero_rev
        ):
            equivalent_zero, turn_offset = _nearest_equivalent_zero_rev(
                configured_zero, sample.position_rev
            )
            session_zero_rev.append(equivalent_zero)
            if turn_offset:
                print(
                    "WRIST_POSITION_WRAP ID{} configured_zero={:+.6f}rev "
                    "session_zero={:+.6f}rev turn_offset={:+d}".format(
                        motor_id,
                        configured_zero,
                        equivalent_zero,
                        turn_offset,
                    ),
                    flush=True,
                )
        for motor_id, sample, zero, sign, limit in zip(
            args.ids, samples, session_zero_rev, args.motor_sign, limit_rad
        ):
            q, _, _ = _q_from_sample(sample, zero, sign)
            if abs(q) > limit + math.radians(5.0):
                raise RuntimeError(
                    f"电机 ID {motor_id} 启动角 {math.degrees(q):+.1f}deg 已超出配置范围"
                )
        startup_target_rev = [
            zero if should_zero else sample.position_rev
            for sample, zero, should_zero in zip(
                samples, session_zero_rev, args.zero_hold_mask
            )
        ]
        startup_hold_q = [
            sign * (target - zero) * TAU
            for target, zero, sign in zip(
                startup_target_rev, session_zero_rev, args.motor_sign
            )
        ]
        startup_initial_error_deg = [
            abs(sample.position_rev - target) * 360.0
            for sample, target in zip(samples, startup_target_rev)
        ]
        startup_maximum_error_deg = [
            error + 5.0 for error in startup_initial_error_deg
        ]
        print(
            "WRIST_ZEROING starting=1 zero_hold_mask={}".format(
                ",".join(str(value) for value in args.zero_hold_mask)
            ),
            flush=True,
        )
        samples = await _move_to_zero(
            controllers,
            args.ids,
            startup_target_rev,
            velocity_deg_s=args.velocity_deg_s,
            accel_deg_s2=args.accel_deg_s2,
            maximum_torque_nm=args.position_torque_nm,
            kp_scale=args.position_kp_scale,
            kd_scale=args.position_kd_scale,
            loop_hz=args.loop_hz,
            tolerance_deg=args.zero_tolerance_deg,
            settle_seconds=args.zero_settle_seconds,
            timeout_s=args.zero_timeout_s,
            maximum_error_deg=startup_maximum_error_deg,
            active_mask=args.zero_hold_mask,
        )
        print(
            "WRIST_READY ids={} configured_zero_rev={} session_zero_rev={} "
            "zero_hold_mask={} startup_hold_deg={} "
            "limit_deg={} reduction_ratio={} position_kp_scale={} "
            "position_kd_scale={} torque_control={}".format(
                ",".join(str(value) for value in args.ids),
                ",".join(f"{value:.9g}" for value in args.zero_rev),
                ",".join(f"{value:.9g}" for value in session_zero_rev),
                ",".join(str(value) for value in args.zero_hold_mask),
                ",".join(f"{math.degrees(value):.3f}" for value in startup_hold_q),
                ",".join(f"{value:g}" for value in args.limit_deg),
                ",".join(f"{value:g}" for value in args.reduction_ratio),
                ",".join(f"{value:g}" for value in args.position_kp_scale),
                ",".join(f"{value:g}" for value in args.position_kd_scale),
                int(args.allow_torque_control),
            ),
            flush=True,
        )
        # Stay read-only after startup. In this configuration ID2/joint 8 has
        # reached zero, while ID1/joint 9 remains in STOP until P or T arrives.
        operation = "I"
        command: list[float] = []
        last_command = time.monotonic()
        last_applied_operation = "I"

        while True:
            cycle = time.monotonic()
            while select.select([sys.stdin], [], [], 0.0)[0]:
                line = sys.stdin.readline()
                if line == "":
                    return
                new_operation, values = _parse_command(line)
                if not new_operation:
                    continue
                if new_operation == "S":
                    return
                if new_operation == "A":
                    watchdog_armed = True
                    last_command = time.monotonic()
                    continue
                operation = "P" if new_operation == "Z" else new_operation
                command = [0.0, 0.0] if new_operation == "Z" else values
                last_command = time.monotonic()

            age_ms = (time.monotonic() - last_command) * 1000.0
            if watchdog_armed and operation in ("P", "P8", "T") and age_ms > args.watchdog_ms:
                raise TimeoutError(f"腕部命令 watchdog 超时 {age_ms:.1f}ms")

            states: list[Any] = []
            commanded_q: list[float | None] = [None, None]
            commanded_rev: list[float | None] = [None, None]
            if operation == "I":
                if last_applied_operation != "I":
                    await _stop_all(controllers, args.ids, strict=True)
                states = [
                    await _command_with_timeout(
                        controller.query(), motor_id, "IDLE_QUERY"
                    )
                    for controller, motor_id in zip(controllers, args.ids)
                ]
            elif operation == "P":
                if last_applied_operation != "P":
                    transition_samples = await _query_all(controllers, args.ids)
                    for controller, motor_id, sample, torque_limit, kp, kd in zip(
                        controllers,
                        args.ids,
                        transition_samples,
                        args.position_torque_nm,
                        args.position_kp_scale,
                        args.position_kd_scale,
                    ):
                        await _capture_position_mode_at_current_pose(
                            controller,
                            motor_id,
                            sample,
                            maximum_torque_nm=torque_limit,
                            kp_scale=kp,
                            kd_scale=kd,
                            watchdog_s=5.0,
                        )
                    print(
                        "WRIST_POSITION_CAPTURE operation=P ids={}".format(
                            ",".join(str(value) for value in args.ids)
                        ),
                        flush=True,
                    )
                for index, (controller, motor_id, q_target, zero, sign, limit, torque_limit, kp, kd) in enumerate(zip(
                    controllers,
                    args.ids,
                    command,
                    session_zero_rev,
                    args.motor_sign,
                    limit_rad,
                    args.position_torque_nm,
                    args.position_kp_scale,
                    args.position_kd_scale,
                )):
                    q_target = float(np_clip(q_target, -limit, limit))
                    target_rev = zero + sign * q_target / TAU
                    commanded_q[index] = q_target
                    commanded_rev[index] = target_rev
                    state = await _command_with_timeout(
                        controller.set_position(
                            position=target_rev,
                            velocity=0.0,
                            velocity_limit=args.runtime_velocity_deg_s / 360.0,
                            accel_limit=args.runtime_accel_deg_s2 / 360.0,
                            maximum_torque=torque_limit,
                            kp_scale=kp,
                            kd_scale=kd,
                            watchdog_timeout=max(0.05, args.watchdog_ms / 1000.0),
                            ignore_position_bounds=1,
                            query=True,
                        ),
                        motor_id,
                        "POSITION",
                    )
                    states.append(state)
            elif operation == "P8":
                if last_applied_operation != "P8":
                    await _command_with_timeout(
                        controllers[1].set_stop(), args.ids[1], "PASSIVE_STOP"
                    )
                    transition_sample = _sample(
                        await _command_with_timeout(
                            controllers[0].query(), args.ids[0], "JOINT8_CAPTURE_QUERY"
                        )
                    )
                    _validate(transition_sample, args.ids[0])
                    await _capture_position_mode_at_current_pose(
                        controllers[0],
                        args.ids[0],
                        transition_sample,
                        maximum_torque_nm=args.position_torque_nm[0],
                        kp_scale=args.position_kp_scale[0],
                        kd_scale=args.position_kd_scale[0],
                        watchdog_s=5.0,
                    )
                    print(
                        f"WRIST_POSITION_CAPTURE operation=P8 ids={args.ids[0]}",
                        flush=True,
                    )
                q_target = float(np_clip(command[0], -limit_rad[0], limit_rad[0]))
                target_rev = (
                    session_zero_rev[0] + args.motor_sign[0] * q_target / TAU
                )
                commanded_q[0] = q_target
                commanded_rev[0] = target_rev
                state = await _command_with_timeout(
                    controllers[0].set_position(
                        position=target_rev,
                        velocity=0.0,
                        velocity_limit=args.runtime_velocity_deg_s / 360.0,
                        accel_limit=args.runtime_accel_deg_s2 / 360.0,
                        maximum_torque=args.position_torque_nm[0],
                        kp_scale=args.position_kp_scale[0],
                        kd_scale=args.position_kd_scale[0],
                        watchdog_timeout=max(0.05, args.watchdog_ms / 1000.0),
                        ignore_position_bounds=1,
                        query=True,
                    ),
                    args.ids[0],
                    "JOINT8_POSITION",
                )
                states.append(state)
                states.append(
                    await _command_with_timeout(
                        controllers[1].query(), args.ids[1], "PASSIVE_QUERY"
                    )
                )
            elif operation == "T":
                if not args.allow_torque_control:
                    raise RuntimeError("收到 T 命令，但未显式允许 moteus 扭矩模式")
                for index, (controller, requested, zero, sign, limit, torque_limit) in enumerate(
                    zip(
                        controllers,
                        command,
                        session_zero_rev,
                        args.motor_sign,
                        limit_rad,
                        args.max_torque_nm,
                    )
                ):
                    previous = _sample_from_optional(states[index] if index < len(states) else None)
                    if previous is None:
                        state_query = await _command_with_timeout(
                            controller.query(), args.ids[index], "QUERY"
                        )
                        if state_query is None:
                            raise RuntimeError(f"电机 ID {args.ids[index]} 无状态响应")
                        previous = _sample(state_query)
                    q, dq, _ = _q_from_sample(previous, zero, sign)
                    tau = float(np_clip(requested, -torque_limit, torque_limit))
                    upper_soft = limit - soft_margin_rad
                    lower_soft = -limit + soft_margin_rad
                    if q > upper_soft:
                        tau = min(tau, -args.soft_limit_stiffness_nm_rad * (q - upper_soft) - args.soft_limit_damping_nm_s_rad * max(dq, 0.0))
                    elif q < lower_soft:
                        tau = max(tau, -args.soft_limit_stiffness_nm_rad * (q - lower_soft) - args.soft_limit_damping_nm_s_rad * min(dq, 0.0))
                    tau = float(np_clip(tau, -torque_limit, torque_limit))
                    state = await _command_with_timeout(
                        controller.set_position(
                            position=math.nan,
                            velocity=0.0,
                            feedforward_torque=sign * tau,
                            kp_scale=0.0,
                            kd_scale=0.0,
                            maximum_torque=torque_limit,
                            watchdog_timeout=max(0.05, args.watchdog_ms / 1000.0),
                            ignore_position_bounds=1,
                            query=True,
                        ),
                        args.ids[index],
                        "TORQUE",
                    )
                    states.append(state)
            else:
                raise RuntimeError(f"内部未知腕部模式 {operation}")

            last_applied_operation = operation

            samples = []
            for motor_id, state in zip(args.ids, states):
                if state is None:
                    raise RuntimeError(f"电机 ID {motor_id} 命令无状态响应")
                sample = _sample(state)
                _validate(sample, motor_id)
                samples.append(sample)
            joint_states = [
                _q_from_sample(sample, zero, sign)
                for sample, zero, sign in zip(samples, session_zero_rev, args.motor_sign)
            ]
            following_violations = []
            for motor_id, values, target_q, target_rev, sample in zip(
                args.ids,
                joint_states,
                commanded_q,
                commanded_rev,
                samples,
            ):
                if target_q is None:
                    continue
                error_deg = abs(math.degrees(values[0] - target_q))
                if error_deg > args.position_following_error_deg:
                    following_violations.append(
                        "ID{} measured_q={:+.2f}deg target_q={:+.2f}deg "
                        "measured_rev={:+.6f} target_rev={:+.6f} error={:.2f}deg "
                        "velocity={:+.2f}deg/s torque={:+.3f}Nm mode={} fault={}".format(
                            motor_id,
                            math.degrees(values[0]),
                            math.degrees(target_q),
                            sample.position_rev,
                            target_rev,
                            error_deg,
                            sample.velocity_rev_s * 360.0,
                            sample.torque_nm,
                            sample.mode,
                            sample.fault,
                        )
                    )
            if following_violations:
                raise RuntimeError(
                    "腕部位置目标未被正确跟随；已 STOP（这不是行程限位）："
                    + "; ".join(following_violations)
                )
            limit_violations = [
                (motor_id, math.degrees(values[0]), math.degrees(limit))
                for motor_id, values, limit in zip(
                    args.ids, joint_states, limit_rad
                )
                if abs(values[0]) > limit + math.radians(2.0)
            ]
            if limit_violations:
                details = ", ".join(
                    f"ID{motor_id} q={angle_deg:+.1f}deg limit=+/-{limit_deg:.1f}deg"
                    for motor_id, angle_deg, limit_deg in limit_violations
                )
                raise RuntimeError(
                    f"腕部测量位置越过硬软件限位（operation={operation}）：{details}"
                )
            now = time.monotonic()
            if any(value is not None for value in commanded_q) and now >= next_position_status:
                details = " ".join(
                    "ID{}[measured_q={:+.2f}deg target_q={:+.2f}deg "
                    "measured_rev={:+.6f} target_rev={:+.6f}]".format(
                        motor_id,
                        math.degrees(values[0]),
                        math.degrees(target_q),
                        sample.position_rev,
                        target_rev,
                    )
                    for motor_id, values, target_q, target_rev, sample in zip(
                        args.ids,
                        joint_states,
                        commanded_q,
                        commanded_rev,
                        samples,
                    )
                    if target_q is not None and target_rev is not None
                )
                print(
                    f"WRIST_POSITION_STATUS operation={operation} {details}",
                    flush=True,
                )
                next_position_status = now + 0.5
            flattened = [value for values in joint_states for value in values]
            print(
                "WRIST_SAMPLE {} {} {} {} {} {} {} mode={}".format(
                    time.monotonic_ns(),
                    *[f"{value:.9g}" for value in flattened],
                    operation,
                ),
                flush=True,
            )
            await asyncio.sleep(max(0.0, period - (time.monotonic() - cycle)))
    finally:
        try:
            await _stop_all(controllers, args.ids)
        finally:
            try:
                transport.close()
            except BaseException as error:
                print(
                    f"WRIST_WARNING transport_close_failed={type(error).__name__}:{error}",
                    file=sys.stderr,
                )


def np_clip(value: float, lower: float, upper: float) -> float:
    return min(max(value, lower), upper)


def _sample_from_optional(state: Any | None) -> Sample | None:
    return None if state is None else _sample(state)


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    try:
        _validate_args(args)
        asyncio.run(_run(args))
    except KeyboardInterrupt:
        print("WRIST_ERROR interrupted", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"WRIST_ERROR {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
