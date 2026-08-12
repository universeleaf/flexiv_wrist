#!/usr/bin/env python3
"""Launch the separate 7-DoF Flexiv RT_JOINT_TORQUE + OSC loop demo."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import time

import numpy as np

from tools.osc_launcher import (
    binary,
    realtime_preflight,
    validate_active_flexiv_tool,
    write_python_dry_run,
    write_python_orientation_dry_run,
    write_python_rectangle_dry_run,
    write_python_spin_dry_run,
)


def parser() -> argparse.ArgumentParser:
    result = argparse.ArgumentParser(description=__doc__)
    result.add_argument("--config", type=Path, default=Path(__file__).parents[1] / "config" / "nine_dof_teleop.toml")
    result.add_argument(
        "--robot-sn",
        default=None,
        help="override [robot].robot_sn; normally leave unset",
    )
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
        choices=("orientation", "spin", "rectangle", "loop"),
        default="orientation",
        help="orientation keeps the selected endpoint fixed and only changes attitude",
    )
    result.add_argument("--rectangle-width-m", type=float, default=0.060)
    result.add_argument("--rectangle-height-m", type=float, default=0.040)
    result.add_argument("--rectangle-corner-radius-m", type=float, default=0.010)
    result.add_argument("--tangent-axis", choices=("x", "z"), default="x")
    result.add_argument("--endpoint", choices=("tcp", "pivot", "flange"), default="tcp")
    result.add_argument("--cpu-affinity", type=int, default=2)
    result.add_argument("--output", type=Path, default=Path("/tmp/flexiv_7dof_osc_loop.csv"))
    result.add_argument("--preflight", action="store_true")
    result.add_argument("--real", action="store_true")
    result.add_argument("--confirm", default="")
    return result


def main() -> int:
    args = parser().parse_args()
    executable = binary("flexiv_7dof_torque_osc")
    try:
        if not args.real:
            if args.preflight:
                print("\n".join(realtime_preflight(executable, args.cpu_affinity)))
                return 0
            if args.trajectory == "orientation":
                write_python_orientation_dry_run(
                    args.output.resolve(), args.duration_s, args.orientation_deg
                )
            elif args.trajectory == "spin":
                write_python_spin_dry_run(
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
            print(f"仅生成一圈 7-DoF 周期轨迹: {args.output.resolve()}")
            print("未连接 Flexiv，也未发送任何力矩。")
            return 0
        if args.confirm != "RUN_7DOF_TORQUE_OSC":
            raise RuntimeError("真实运行必须加 --confirm RUN_7DOF_TORQUE_OSC")
        from scripts.teleoperate import WristBridge, load_profile

        profile = load_profile(args.config.resolve())
        print("\n".join(realtime_preflight(executable, args.cpu_affinity)))
        validate_active_flexiv_tool(profile)
        print("确认：机械臂已使能、无故障，工作空间清空，急停可立即触及。")
        if args.trajectory in ("orientation", "spin"):
            print(
                "OSC 固定点将在进入实时控制前从当前姿态自动采集；"
                "先把机械臂移动到需要的中心点，再启动本命令。"
            )
        print(f"循环周期={args.duration_s:g}s；将持续重复运行，直到 Ctrl-C。")
        command = [
                str(executable),
                "--robot-sn", str(args.robot_sn or profile.robot["robot_sn"]),
                "--duration-s", str(args.duration_s),
                "--radius-m", str(args.radius_m),
                "--orientation-deg", str(args.orientation_deg),
                "--trajectory", args.trajectory,
                "--rectangle-width-m", str(args.rectangle_width_m),
                "--rectangle-height-m", str(args.rectangle_height_m),
                "--rectangle-corner-radius-m", str(args.rectangle_corner_radius_m),
                "--tangent-axis", args.tangent_axis,
                "--endpoint", args.endpoint,
                "--cpu-affinity", str(args.cpu_affinity),
                "--real-confirm", "RUN_7DOF_TORQUE_OSC",
            ]
        print("7DoF comparison: moving q8/q9 to zero and holding both axes at zero.")
        process: subprocess.Popen[str] | None = None
        with WristBridge(profile, zero_hold_mask_override=(True, True)) as wrist:
            wrist.wait_ready(float(profile.wrist["zero_timeout_s"]) + 5.0)
            wrist.wait_first_sample(2.0)
            wrist.finish_startup()
            print("执行:", " ".join(command), flush=True)
            process = subprocess.Popen(command, cwd=Path(__file__).parents[1], text=True)
            period = 1.0 / float(profile.wrist["loop_rate_hz"])
            try:
                while process.poll() is None:
                    wrist.command_position(np.zeros(2))
                    time.sleep(period)
            except KeyboardInterrupt:
                if process.poll() is None:
                    process.send_signal(2)
                raise
            finally:
                if process.poll() is None:
                    process.send_signal(2)
                    process.wait(timeout=3.0)
            return int(process.returncode or 0)
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f"错误: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
