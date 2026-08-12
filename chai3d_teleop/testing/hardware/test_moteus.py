#!/usr/bin/env python3
"""Safely exercise two moteus servos around their startup positions.

The position reported by each servo at program start is treated as the logical
0 degree position.  This is an application-level offset only: no encoder or
persistent moteus configuration is changed.
"""

from __future__ import annotations

import argparse
import asyncio
import math
import sys
import time
from dataclasses import dataclass
from typing import Any, Sequence

try:
    import moteus
except ImportError as exc:  # Give a useful message instead of a traceback.
    raise SystemExit(
        "未安装 moteus Python 包。请先执行：\n"
        "  python3 -m venv .venv_moteus\n"
        "  .venv_moteus/bin/python -m pip install moteus\n"
        "然后使用 .venv_moteus/bin/python 运行本脚本。"
    ) from exc


DEGREES_PER_REVOLUTION = 360.0
MIN_LOOP_HZ = 10.0
MAX_TEST_ANGLE_DEG = 90.0
COMMAND_TIMEOUT_S = 1.0


@dataclass(frozen=True)
class ServoSample:
    motor_id: int
    position_rev: float
    velocity_rps: float
    torque_nm: float
    mode: int | None
    fault: int | None


def _register_value(state: Any, register: Any, default: Any = None) -> Any:
    """Read a register while tolerating older moteus query layouts."""
    return state.values.get(register, default)


def _sample(motor_id: int, state: Any) -> ServoSample:
    position = float(_register_value(state, moteus.Register.POSITION, math.nan))
    velocity = float(_register_value(state, moteus.Register.VELOCITY, math.nan))
    torque = float(_register_value(state, moteus.Register.TORQUE, math.nan))
    mode_raw = _register_value(state, moteus.Register.MODE)
    fault_raw = _register_value(state, moteus.Register.FAULT)
    return ServoSample(
        motor_id=motor_id,
        position_rev=position,
        velocity_rps=velocity,
        torque_nm=torque,
        mode=None if mode_raw is None else int(mode_raw),
        fault=None if fault_raw is None else int(fault_raw),
    )


def _validate_sample(sample: ServoSample) -> None:
    if not math.isfinite(sample.position_rev):
        raise RuntimeError(f"电机 ID {sample.motor_id} 返回了无效位置")
    if sample.fault not in (None, 0):
        raise RuntimeError(
            f"电机 ID {sample.motor_id} 报告 fault={sample.fault}，已中止"
        )
    if sample.mode == 1:
        raise RuntimeError(f"电机 ID {sample.motor_id} 处于 FAULT 模式，已中止")


async def _stop_all(controllers: Sequence[Any]) -> None:
    errors: list[BaseException] = []
    # Keep access to the shared CAN transport serialized, including shutdown.
    for controller in controllers:
        try:
            await asyncio.wait_for(controller.set_stop(), timeout=COMMAND_TIMEOUT_S)
        except BaseException as exc:
            errors.append(exc)
    if errors:
        print(f"警告：{len(errors)} 台电机未确认停机：{errors}", file=sys.stderr)


async def _preflight(controllers: Sequence[Any], motor_ids: Sequence[int]) -> list[ServoSample]:
    # STOP both servos first.  Besides producing no motion, STOP clears a stale
    # fault/timeout so that the following query describes the current state.
    await _stop_all(controllers)
    samples: list[ServoSample] = []
    for motor_id, controller in zip(motor_ids, controllers):
        state = await asyncio.wait_for(controller.query(), timeout=COMMAND_TIMEOUT_S)
        if state is None:
            raise RuntimeError(f"电机 ID {motor_id} 没有响应")
        sample = _sample(motor_id, state)
        _validate_sample(sample)
        samples.append(sample)
    return samples


def _logical_angle_deg(position_rev: float, zero_rev: float) -> float:
    return (position_rev - zero_rev) * DEGREES_PER_REVOLUTION


async def _move_pair(
    controllers: Sequence[Any],
    motor_ids: Sequence[int],
    zero_positions_rev: Sequence[float],
    target_deg: float,
    *,
    velocity_deg_s: float,
    accel_deg_s2: float,
    maximum_torque_nm: float,
    loop_hz: float,
    tolerance_deg: float,
    settle_seconds: float,
) -> None:
    target_offset_rev = target_deg / DEGREES_PER_REVOLUTION
    target_positions_rev = [zero + target_offset_rev for zero in zero_positions_rev]
    velocity_limit_rps = velocity_deg_s / DEGREES_PER_REVOLUTION
    accel_limit_rps2 = accel_deg_s2 / DEGREES_PER_REVOLUTION
    period = 1.0 / loop_hz

    # Conservative timeout based on the worst-case 180 degree leg, with ample
    # time for acceleration and settling.
    timeout_s = max(8.0, 3.0 * (180.0 / velocity_deg_s) + settle_seconds)
    started = time.monotonic()
    settled_since: float | None = None
    next_print = 0.0

    while True:
        cycle_started = time.monotonic()
        states = []
        # A shared transport is used by all Controller objects.  Sequential
        # requests avoid concurrent access while keeping the two commands only
        # one CAN transaction apart.
        for controller, target_rev in zip(controllers, target_positions_rev):
            state = await asyncio.wait_for(
                controller.set_position(
                    position=target_rev,
                    velocity=0.0,
                    velocity_limit=velocity_limit_rps,
                    accel_limit=accel_limit_rps2,
                    maximum_torque=maximum_torque_nm,
                    ignore_position_bounds=1,
                    query=True,
                ),
                timeout=COMMAND_TIMEOUT_S,
            )
            if state is None:
                raise RuntimeError("位置命令没有收到状态响应")
            states.append(state)

        samples = [
            _sample(motor_id, state)
            for motor_id, state in zip(motor_ids, states)
        ]
        for sample, zero_rev in zip(samples, zero_positions_rev):
            _validate_sample(sample)
            logical_deg = _logical_angle_deg(sample.position_rev, zero_rev)
            if abs(logical_deg) > MAX_TEST_ANGLE_DEG + max(5.0, tolerance_deg):
                raise RuntimeError(
                    f"电机 ID {sample.motor_id} 超出软件安全范围："
                    f"{logical_deg:+.1f}°"
                )

        now = time.monotonic()
        if now >= next_print:
            readings = " | ".join(
                f"ID {sample.motor_id}: "
                f"{_logical_angle_deg(sample.position_rev, zero_rev):+7.2f}°, "
                f"{sample.velocity_rps * DEGREES_PER_REVOLUTION:+7.2f}°/s, "
                f"{sample.torque_nm:+6.3f} Nm"
                for sample, zero_rev in zip(samples, zero_positions_rev)
            )
            print(f"目标 {target_deg:+.1f}° | {readings}")
            next_print = now + 0.25

        all_in_tolerance = all(
            abs(_logical_angle_deg(sample.position_rev, zero_rev) - target_deg)
            <= tolerance_deg
            and abs(sample.velocity_rps * DEGREES_PER_REVOLUTION) <= 2.0
            for sample, zero_rev in zip(samples, zero_positions_rev)
        )
        if all_in_tolerance:
            if settled_since is None:
                settled_since = now
            elif now - settled_since >= settle_seconds:
                return
        else:
            settled_since = None

        if now - started > timeout_s:
            positions = ", ".join(
                f"ID {sample.motor_id}="
                f"{_logical_angle_deg(sample.position_rev, zero_rev):+.1f}°"
                for sample, zero_rev in zip(samples, zero_positions_rev)
            )
            raise TimeoutError(
                f"到达 {target_deg:+.1f}° 超时（{positions}）；已中止"
            )

        sleep_time = period - (time.monotonic() - cycle_started)
        if sleep_time > 0.0:
            await asyncio.sleep(sleep_time)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "将两台 moteus 启动位置作为逻辑 0°，依次运动到 "
            "0→+角度→0→-角度→0。"
        )
    )
    parser.add_argument(
        "--ids",
        nargs=2,
        type=int,
        default=(1, 2),
        metavar=("ID1", "ID2"),
        help="两台 moteus 的 CAN ID（默认：1 2）",
    )
    parser.add_argument(
        "--angle-deg",
        type=float,
        default=90.0,
        help="相对启动零位的测试角度，必须在 (0, 90]（默认：90）",
    )
    parser.add_argument(
        "--velocity-deg-s",
        type=float,
        default=15.0,
        help="最大速度，度/秒（默认：15）",
    )
    parser.add_argument(
        "--accel-deg-s2",
        type=float,
        default=30.0,
        help="最大加速度，度/秒²（默认：30）",
    )
    parser.add_argument(
        "--max-torque-nm",
        type=float,
        default=2.0,
        help="每台电机的命令扭矩上限，Nm（默认：2）",
    )
    parser.add_argument(
        "--loop-hz",
        type=float,
        default=50.0,
        help="命令刷新率，Hz；不得低于 10（默认：50）",
    )
    parser.add_argument(
        "--tolerance-deg",
        type=float,
        default=2.0,
        help="到位容差，度（默认：2）",
    )
    parser.add_argument(
        "--settle-seconds",
        type=float,
        default=0.5,
        help="每个目标的稳定停留时间，秒（默认：0.5）",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="只连接、发送 STOP、读取当前位置和故障，不使能运动",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="跳过键盘 MOVE 确认（仅用于已验证的自动测试）",
    )
    # Add official moteus transport flags, such as --pi3hat-cfg and
    # --fdcanusb.  With no such option the library auto-detects its adapter.
    moteus.make_transport_args(parser)
    return parser


def _validate_args(args: argparse.Namespace) -> None:
    if args.ids[0] == args.ids[1] or any(motor_id <= 0 for motor_id in args.ids):
        raise ValueError("--ids 必须是两个不同的正整数")
    if not (0.0 < args.angle_deg <= MAX_TEST_ANGLE_DEG):
        raise ValueError("--angle-deg 必须在 (0, 90] 范围内")
    if args.velocity_deg_s <= 0.0:
        raise ValueError("--velocity-deg-s 必须大于 0")
    if args.accel_deg_s2 <= 0.0:
        raise ValueError("--accel-deg-s2 必须大于 0")
    if args.max_torque_nm <= 0.0:
        raise ValueError("--max-torque-nm 必须大于 0")
    if args.loop_hz < MIN_LOOP_HZ:
        raise ValueError(f"--loop-hz 不得低于 {MIN_LOOP_HZ:g}")
    if args.tolerance_deg <= 0.0:
        raise ValueError("--tolerance-deg 必须大于 0")
    if args.settle_seconds < 0.0:
        raise ValueError("--settle-seconds 不得小于 0")


async def _run(args: argparse.Namespace) -> None:
    transport = moteus.get_singleton_transport(args)
    controllers = [
        moteus.Controller(id=motor_id, transport=transport)
        for motor_id in args.ids
    ]

    try:
        print(f"连接电机 ID：{args.ids[0]}, {args.ids[1]}")
        samples = await _preflight(controllers, args.ids)
        zero_positions_rev = [sample.position_rev for sample in samples]
        for sample in samples:
            print(
                f"ID {sample.motor_id}: 当前原始位置 "
                f"{sample.position_rev:+.6f} 圈 -> 逻辑 0.0°，"
                f"mode={sample.mode}, fault={sample.fault}"
            )

        if args.check_only:
            print("停机状态检查完成；只发送了 STOP，没有使能运动。")
            return

        print(
            "\n即将同时测试两台电机：\n"
            f"  轨迹：0° -> +{args.angle_deg:g}° -> 0° -> "
            f"-{args.angle_deg:g}° -> 0°\n"
            f"  速度：{args.velocity_deg_s:g}°/s，"
            f"加速度：{args.accel_deg_s2:g}°/s²，"
            f"扭矩上限：{args.max_torque_nm:g} Nm\n"
            "  Ctrl-C、通信异常、故障或越界都会请求两台电机 STOP。"
        )
        if not args.yes:
            confirmation = input("确认机构无碰撞并已准备急停后，输入 MOVE：").strip()
            if confirmation != "MOVE":
                print("未收到 MOVE，取消测试。")
                return

        for target_deg in (
            args.angle_deg,
            0.0,
            -args.angle_deg,
            0.0,
        ):
            await _move_pair(
                controllers,
                args.ids,
                zero_positions_rev,
                target_deg,
                velocity_deg_s=args.velocity_deg_s,
                accel_deg_s2=args.accel_deg_s2,
                maximum_torque_nm=args.max_torque_nm,
                loop_hz=args.loop_hz,
                tolerance_deg=args.tolerance_deg,
                settle_seconds=args.settle_seconds,
            )

        print("双电机 ±角度测试完成，返回逻辑 0°。")
    finally:
        print("正在向两台电机发送 STOP...")
        try:
            await _stop_all(controllers)
        finally:
            transport.close()


def main() -> int:
    parser = _build_parser()
    args = parser.parse_args()
    try:
        _validate_args(args)
        asyncio.run(_run(args))
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C；停机请求已执行。", file=sys.stderr)
        return 130
    except (ValueError, RuntimeError, TimeoutError) as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    except Exception as exc:
        print(f"通信或运行错误：{type(exc).__name__}: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
