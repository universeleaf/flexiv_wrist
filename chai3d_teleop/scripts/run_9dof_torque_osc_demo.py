#!/usr/bin/env python3
"""Launch the separate 9-DoF multi-rate torque + OSC closed-loop demo."""

from __future__ import annotations

import argparse
import json
import mmap
import os
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time

import numpy as np

from rt_osc_demo_common import (
    binary,
    realtime_preflight,
    validate_active_flexiv_tool,
    write_python_dry_run,
    write_python_orientation_dry_run,
    write_python_rectangle_dry_run,
)


SHARED_MAGIC = 0x3957524953544F53
SHARED_SIZE = 128


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--config", type=Path, default=Path(__file__).parents[1] / "config" / "nine_dof_teleop.toml")
    result.add_argument(
        "--duration-s",
        type=float,
        default=20.0,
        help="one loop period in seconds; real motion repeats until Ctrl-C",
    )
    result.add_argument("--radius-m", type=float, default=0.030)
    result.add_argument("--orientation-deg", type=float, default=3.0)
    result.add_argument(
        "--trajectory",
        choices=("orientation", "rectangle", "loop"),
        default="orientation",
        help="orientation fixes the real wrist tip and only changes attitude",
    )
    result.add_argument("--rectangle-width-m", type=float, default=0.060)
    result.add_argument("--rectangle-height-m", type=float, default=0.040)
    result.add_argument("--rectangle-corner-radius-m", type=float, default=0.010)
    result.add_argument("--tangent-axis", choices=("x", "z"), default="x")
    result.add_argument(
        "--inertia-mode",
        choices=("auto", "legacy-block"),
        default="auto",
        help=(
            "auto builds the coupled 9x9 joint mass matrix from 6x6 body "
            "spatial inertias; legacy-block keeps the old uncoupled model"
        ),
    )
    result.add_argument(
        "--wrist-state-timeout-ms",
        type=float,
        default=None,
        help="override [wrist].rt_osc_state_timeout_ms",
    )
    result.add_argument(
        "--wrist-hard-timeout-ms",
        type=float,
        default=None,
        help="override [wrist].rt_osc_hard_timeout_ms",
    )
    result.add_argument("--cpu-affinity", type=int, default=2)
    result.add_argument("--output", type=Path, default=Path("/tmp/flexiv_9dof_osc_loop.csv"))
    result.add_argument("--preflight", action="store_true")
    result.add_argument("--real", action="store_true")
    result.add_argument("--confirm", default="")
    return result


def _quaternion_wxyz_to_rotation(quaternion) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion, dtype=float)
    norm = float(np.linalg.norm([w, x, y, z]))
    if norm < 1e-9:
        raise ValueError("Flexiv Tool TCP quaternion is zero")
    w, x, y, z = np.asarray([w, x, y, z]) / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _live_tool_zero_geometry(profile, active_tool: dict[str, object]):
    """Map the live Elements TCP to the articulated wrist's q8=q9=0 model."""
    geometry = profile.document["wrist_geometry"]
    tcp = np.asarray(active_tool["tcp_location"], dtype=float)
    joint1_origin = np.asarray(geometry["joint1_origin_flange_m"], dtype=float)
    joint2_offset = np.asarray(
        geometry["joint2_offset_after_joint1_m"], dtype=float
    )
    tip_offset = tcp[:3] - joint1_origin - joint2_offset
    return tip_offset, _quaternion_wxyz_to_rotation(tcp[3:])


def _select_inertia_parameters(profile, calibration: dict, inertia_mode: str):
    """Quality-gate identified inertial terms before real-time inverse dynamics."""
    payload = profile.payload
    nominal_reflected = np.asarray(
        payload["reflected_joint_inertia_kg_m2"], dtype=float
    )
    identified_reflected = np.asarray(
        calibration.get("reflected_joint_inertia_kg_m2", nominal_reflected),
        dtype=float,
    )
    identified_scale = float(calibration.get("rigid_body_scale", 1.0))
    if inertia_mode == "legacy-block":
        return identified_scale, identified_reflected, "legacy_identified"

    reasons: list[str] = []
    rms = float(calibration.get("fit_rms_torque_nm", float("inf")))
    condition = float(calibration.get("regression_condition_number", float("inf")))
    if not np.isfinite(rms) or rms > 0.15:
        reasons.append(f"fit_rms={rms:.3f}Nm")
    if not np.isfinite(condition) or condition > 500.0:
        reasons.append(f"condition={condition:.1f}")
    if not 0.5 <= identified_scale <= 1.5:
        reasons.append(f"rigid_body_scale={identified_scale:.3f}")
    if identified_reflected.shape != (2,) or np.any(identified_reflected <= 1e-5):
        reasons.append(f"reflected_inertia={identified_reflected.tolist()}")
    if reasons:
        source = "elements_baseline+nominal_wrist(" + ", ".join(reasons) + ")"
        return 1.0, nominal_reflected, source
    return identified_scale, identified_reflected, "elements_baseline+identified_wrist"


def _write_model(
    profile,
    path: Path,
    active_tool: dict[str, object],
    inertia_mode: str,
) -> None:
    payload = profile.payload
    geometry = profile.document["wrist_geometry"]
    calibration_path = profile.path.parent / str(payload["inertia_calibration_path"])
    calibration = {}
    if calibration_path.is_file():
        calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
        print(f"加载腕部惯量辨识: {calibration_path}")
    live_tip_offset, live_probe_rotation = _live_tool_zero_geometry(
        profile, active_tool
    )
    rigid_body_scale, reflected_inertia, inertia_source = _select_inertia_parameters(
        profile, calibration, inertia_mode
    )
    print(f"自动惯量来源: {inertia_source}", flush=True)
    def line(name: str, values) -> str:
        array = np.asarray(values, dtype=float).reshape(-1)
        return name + " " + " ".join(f"{value:.12g}" for value in array)
    text = "\n".join(
        [
            line("joint1_origin", geometry["joint1_origin_flange_m"]),
            line("joint1_axis", geometry["joint1_axis_flange"]),
            line("joint2_offset", geometry["joint2_offset_after_joint1_m"]),
            line("joint2_axis", geometry["joint2_axis_after_joint1"]),
            line("tip_offset", live_tip_offset),
            line("probe_rotation_zero", live_probe_rotation),
            f"active_tool_mass {float(active_tool['mass_kg']):.12g}",
            line("active_tool_com", active_tool["com_m"]),
            line("active_tool_inertia", active_tool["inertia_kg_m2"]),
            f"mass1 {float(payload['link1_mass_kg']):.12g}",
            f"mass2 {float(payload['link2_and_probe_mass_kg']):.12g}",
            line("com1", payload["link1_com_after_joint1_m"]),
            line("com2", payload["link2_and_probe_com_after_joint2_m"]),
            line("inertia1", payload["link1_inertia_com_kg_m2_row_major"]),
            line("inertia2", payload["link2_and_probe_inertia_com_kg_m2_row_major"]),
            line(
                "reflected_inertia",
                reflected_inertia,
            ),
            line(
                "viscous_friction",
                calibration.get(
                    "viscous_friction_nm_s_rad",
                    payload["viscous_friction_nm_s_rad"],
                ),
            ),
            line(
                "coulomb_friction",
                calibration.get(
                    "coulomb_friction_nm", payload["coulomb_friction_nm"]
                ),
            ),
            line(
                "torque_bias",
                calibration.get("torque_bias_nm", payload["torque_bias_nm"]),
            ),
            f"rigid_body_scale {rigid_body_scale:.12g}",
            f"priority_inertia_scale {float(profile.allocation['osc_wrist_inertia_weight']):.12g}",
            f"wrist_priority_gain {float(profile.allocation['wrist_priority_gain']):.12g}",
            line(
                "wrist_tracking_kp",
                profile.allocation["wrist_tracking_kp_nm_per_rad"],
            ),
            line(
                "wrist_tracking_kd",
                profile.allocation["wrist_tracking_kd_nm_s_per_rad"],
            ),
            line(
                "wrist_rectangle_excursion_rad",
                np.deg2rad(profile.allocation["wrist_rectangle_excursion_deg"]),
            ),
            f"ik_damping {float(profile.allocation['damping']):.12g}",
            f"ik_max_step_rad {np.deg2rad(float(profile.allocation['max_iteration_step_deg'])):.12g}",
            f"ik_max_iterations {int(profile.allocation['max_iterations'])}",
            f"joint_limit_margin_rad {np.deg2rad(float(profile.allocation['joint_limit_margin_deg'])):.12g}",
            line("joint_min", profile.geometry.joint_min_rad),
            line("joint_max", profile.geometry.joint_max_rad),
        ]
    )
    path.write_text(text + "\n", encoding="utf-8")


def _publish_state(mapping: mmap.mmap, sequence: int, sample) -> None:
    # Odd sequence means "being written"; the C++ reader only accepts two
    # equal even sequence samples. This prevents a mixed q/dq/tau frame.
    struct.pack_into("<Q", mapping, 8, 2 * sequence - 1)
    struct.pack_into(
        "<6d", mapping, 24,
        float(sample.q_rad[0]), float(sample.q_rad[1]),
        float(sample.dq_rad_s[0]), float(sample.dq_rad_s[1]),
        float(sample.torque_nm[0]), float(sample.torque_nm[1]),
    )
    # Publish the actual bridge receive time, not merely the time at which this
    # Python loop copied an old sample into shared memory.
    struct.pack_into("<Q", mapping, 16, int(sample.received_at_s * 1e9))
    struct.pack_into("<Q", mapping, 8, 2 * sequence)


def _read_torque_command(mapping: mmap.mmap):
    for _ in range(3):
        first = struct.unpack_from("<Q", mapping, 72)[0]
        if first == 0 or first & 1:
            continue
        command_time_ns = struct.unpack_from("<Q", mapping, 80)[0]
        torque = np.asarray(struct.unpack_from("<2d", mapping, 88))
        target_q = np.asarray(struct.unpack_from("<2d", mapping, 112))
        second = struct.unpack_from("<Q", mapping, 72)[0]
        if first == second:
            return first, command_time_ns, torque, target_q
    return None


def run_real(args) -> int:
    from run_9dof_teleop import WristBridge, load_profile

    profile = load_profile(args.config.resolve())
    wrist_state_timeout_ms = (
        float(args.wrist_state_timeout_ms)
        if args.wrist_state_timeout_ms is not None
        else float(profile.wrist["rt_osc_state_timeout_ms"])
    )
    if not 100.0 <= wrist_state_timeout_ms <= 1000.0:
        raise ValueError("wrist state timeout 必须在 100..1000 ms")
    wrist_hard_timeout_ms = (
        float(args.wrist_hard_timeout_ms)
        if args.wrist_hard_timeout_ms is not None
        else float(profile.wrist["rt_osc_hard_timeout_ms"])
    )
    if not wrist_state_timeout_ms < wrist_hard_timeout_ms <= 5000.0:
        raise ValueError("wrist hard timeout 必须大于状态超时且不超过 5000 ms")
    calibration_path = profile.path.parent / str(
        profile.payload["inertia_calibration_path"]
    )
    if not calibration_path.is_file():
        raise FileNotFoundError(
            "9-DoF 实时扭矩 OSC 必须先完成腕部惯量辨识；缺少 "
            f"{calibration_path}。按 docs/RT_TORQUE_OSC_ZH.md 执行采集和分析。"
        )
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    assembly_id = str(profile.payload["assembly_id"])
    if calibration.get("assembly_id") != assembly_id:
        raise RuntimeError(
            "腕部惯量辨识属于旧硬件总成；当前 assembly_id="
            f"{assembly_id!r}。拆装探针后必须重新 collect + analyze"
        )
    executable = binary("flexiv_9dof_torque_osc")
    print("\n".join(realtime_preflight(executable, args.cpu_affinity)))
    active_tool = validate_active_flexiv_tool(profile)
    fdcanusb = Path(str(profile.wrist["fdcanusb"]))
    if not fdcanusb.exists() or not os.access(fdcanusb, os.R_OK | os.W_OK):
        raise PermissionError(f"fdcanusb 不存在或当前会话无读写权限: {fdcanusb}")

    shared_file = tempfile.NamedTemporaryFile(prefix="flexiv_9dof_wrist_", suffix=".bin", dir="/tmp", delete=False)
    shared_path = Path(shared_file.name)
    model_path = shared_path.with_suffix(".model")
    process: subprocess.Popen[str] | None = None
    try:
        shared_file.truncate(SHARED_SIZE)
        shared_file.flush()
        mapping = mmap.mmap(shared_file.fileno(), SHARED_SIZE)
        struct.pack_into("<Q", mapping, 0, SHARED_MAGIC)
        _write_model(profile, model_path, active_tool, args.inertia_mode)
        print(
            "9DoF 腕部优先控制: inertia_mode={}, orientation amplification={:.1f}x, "
            "joint Kp={}, joint Kd={}".format(
                args.inertia_mode,
                float(profile.allocation["wrist_priority_gain"]),
                profile.allocation["wrist_tracking_kp_nm_per_rad"],
                profile.allocation["wrist_tracking_kd_nm_s_per_rad"],
            ),
            flush=True,
        )
        with WristBridge(
            profile,
            zero_hold_mask_override=[False, False],
            watchdog_ms_override=wrist_state_timeout_ms,
        ) as wrist:
            wrist.wait_ready(float(profile.wrist["zero_timeout_s"]) + 5.0)
            first = wrist.wait_first_sample(2.0)
            _publish_state(mapping, 1, first)
            wrist.finish_startup()
            command = [
                str(executable),
                "--robot-sn", str(profile.robot["robot_sn"]),
                "--wrist-shm", str(shared_path),
                "--wrist-model", str(model_path),
                "--duration-s", str(args.duration_s),
                "--radius-m", str(args.radius_m),
                "--orientation-deg", str(args.orientation_deg),
                "--trajectory", args.trajectory,
                "--rectangle-width-m", str(args.rectangle_width_m),
                "--rectangle-height-m", str(args.rectangle_height_m),
                "--rectangle-corner-radius-m", str(args.rectangle_corner_radius_m),
                "--tangent-axis", args.tangent_axis,
                "--inertia-mode", args.inertia_mode,
                "--wrist-state-timeout-ms", str(wrist_state_timeout_ms),
                "--wrist-hard-timeout-ms", str(wrist_hard_timeout_ms),
                "--cpu-affinity", str(args.cpu_affinity),
                "--real-confirm", "RUN_9DOF_TORQUE_OSC",
            ]
            print("执行:", " ".join(command), flush=True)
            print(
                f"循环周期={args.duration_s:g}s；将持续重复运行，直到 Ctrl-C。",
                flush=True,
            )
            process = subprocess.Popen(command, cwd=Path(__file__).parents[1], text=True)
            sequence = 1
            last_command_sequence = 0
            last_torque = np.zeros(2)
            last_target_q = first.q_rad.copy()
            have_torque_command = False
            period = 1.0 / float(profile.wrist["loop_rate_hz"])
            next_tick = time.monotonic()
            next_status = time.monotonic()
            while process.poll() is None:
                sequence += 1
                sample = wrist.latest()
                _publish_state(mapping, sequence, sample)
                stop_requested = struct.unpack_from("<I", mapping, 104)[0]
                if stop_requested:
                    break
                command = _read_torque_command(mapping)
                if command is not None and command[0] != last_command_sequence:
                    command_sequence, command_time_ns, torque, target_q = command
                    age_ms = (time.monotonic_ns() - command_time_ns) / 1e6
                    if age_ms > wrist_state_timeout_ms:
                        print(
                            f"警告: 丢弃过期腕部力矩帧 age={age_ms:.1f}ms，发送 0 Nm",
                            file=sys.stderr,
                        )
                        last_torque = np.zeros(2)
                    else:
                        last_torque = torque
                        if np.all(np.isfinite(target_q)):
                            last_target_q = target_q
                    have_torque_command = True
                    last_command_sequence = command_sequence
                # Refresh even when the shared-memory sequence did not change;
                # this prevents a short Python scheduling pause from tripping
                # the moteus command watchdog after torque mode is engaged.
                if have_torque_command:
                    wrist.command_torque(last_torque)
                if time.monotonic() >= next_status:
                    print(
                        "WRIST_OSC q_deg=[{:+.1f}, {:+.1f}] "
                        "target_q_deg=[{:+.1f}, {:+.1f}] "
                        "dq_deg_s=[{:+.1f}, {:+.1f}] tau_cmd_Nm=[{:+.3f}, {:+.3f}]".format(
                            *np.rad2deg(sample.q_rad),
                            *np.rad2deg(last_target_q),
                            *np.rad2deg(sample.dq_rad_s),
                            *last_torque,
                        ),
                        flush=True,
                    )
                    next_status = time.monotonic() + 1.0
                next_tick += period
                time.sleep(max(0.0, next_tick - time.monotonic()))
            if process.poll() is None:
                process.send_signal(2)
            # Keep the bridge alive and explicitly refresh zero torque while
            # the C++ process leaves RT mode.  Previously process.wait() left
            # the wrist stdin silent long enough to trip its 100 ms watchdog.
            shutdown_deadline = time.monotonic() + 3.0
            while process.poll() is None and time.monotonic() < shutdown_deadline:
                wrist.command_torque(np.zeros(2))
                sequence += 1
                _publish_state(mapping, sequence, wrist.latest())
                time.sleep(period)
            return process.wait(timeout=2.0)
    finally:
        if process is not None and process.poll() is None:
            process.terminate()
            process.wait(timeout=3.0)
        try:
            mapping.close()
        except (NameError, BufferError):
            pass
        shared_file.close()
        shared_path.unlink(missing_ok=True)
        model_path.unlink(missing_ok=True)


def main() -> int:
    args = parser().parse_args()
    try:
        if not args.real:
            if args.preflight:
                print("\n".join(realtime_preflight(binary("flexiv_9dof_torque_osc"), args.cpu_affinity)))
                return 0
            if args.trajectory == "orientation":
                write_python_orientation_dry_run(
                    args.output.resolve(), args.duration_s, args.orientation_deg
                )
            elif args.trajectory == "rectangle":
                write_python_rectangle_dry_run(
                    args.output.resolve(),
                    args.duration_s,
                    args.rectangle_width_m,
                    args.rectangle_height_m,
                    args.rectangle_corner_radius_m,
                )
            else:
                write_python_dry_run(
                    args.output.resolve(), args.duration_s, args.radius_m, args.orientation_deg
                )
            print(f"仅生成一圈 9-DoF 周期轨迹: {args.output.resolve()}")
            print("未连接 Flexiv 或腕部，也未发送任何力矩。")
            return 0
        if args.confirm != "RUN_9DOF_TORQUE_OSC":
            raise RuntimeError("真实运行必须加 --confirm RUN_9DOF_TORQUE_OSC")
        print("确认：腕部/探针无障碍，机械臂已使能无故障，急停可立即触及。")
        if args.trajectory == "orientation":
            print(
                "OSC 固定点将在进入实时控制前从当前真实腕部末端自动采集；"
                "先把机械臂移动到需要的中心点，再启动本命令。"
            )
        return run_real(args)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f"错误: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
