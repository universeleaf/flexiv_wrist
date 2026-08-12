#!/usr/bin/env python3
"""Move q8/q9 and stream the live 2x2 mass and 6x6 spatial inertia matrices."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import sys
import time

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from tools.identify_wrist_inertia import (  # noqa: E402
    _parameters_from_profile,
    _smoothstep,
)
from scripts.teleoperate import (  # noqa: E402
    DEFAULT_CONFIG,
    WristBridge,
    load_profile,
)
from scripts.demo_9dof import (  # noqa: E402
    _select_inertia_parameters,
)
from controllers.wrist_dynamics import WristDynamics  # noqa: E402


def _target(
    center_rad: np.ndarray,
    amplitudes_rad: np.ndarray,
    elapsed_s: float,
    duration_s: float,
    period_s: float,
) -> np.ndarray:
    """Smooth, non-synchronous q8/q9 excitation around the current pose."""
    envelope = _smoothstep(elapsed_s / 3.0) * _smoothstep(
        (duration_s - elapsed_s) / 3.0
    )
    phase = 2.0 * math.pi * elapsed_s / period_s
    return center_rad + envelope * amplitudes_rad * np.array(
        [math.sin(phase), math.sin(math.sqrt(2.0) * phase + 0.65)]
    )


def _load_dynamics(profile) -> tuple[WristDynamics, str]:
    calibration_path = profile.path.parent / str(
        profile.payload["inertia_calibration_path"]
    )
    calibration = (
        json.loads(calibration_path.read_text(encoding="utf-8"))
        if calibration_path.is_file()
        else {}
    )
    scale, reflected, source = _select_inertia_parameters(
        profile, calibration, "auto"
    )
    effective = dict(calibration)
    effective["rigid_body_scale"] = scale
    effective["reflected_joint_inertia_kg_m2"] = reflected.tolist()
    parameters = _parameters_from_profile(profile, effective)
    return WristDynamics(profile.geometry, parameters), source


def _csv_header() -> list[str]:
    return [
        "time_s",
        "q8_rad",
        "q9_rad",
        "dq8_rad_s",
        "dq9_rad_s",
        "target_q8_rad",
        "target_q9_rad",
        *[f"M_wrist_{row}{column}" for row in range(2) for column in range(2)],
        *[f"I_spatial_flange_{row}{column}" for row in range(6) for column in range(6)],
        "M_eigen_0",
        "M_eigen_1",
        *[f"I_eigen_{index}" for index in range(6)],
    ]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--duration-s", type=float, default=45.0)
    parser.add_argument("--period-s", type=float, default=8.0)
    parser.add_argument(
        "--amplitude-deg", nargs=2, type=float, default=[20.0, 45.0]
    )
    parser.add_argument("--sample-hz", type=float, default=100.0)
    parser.add_argument(
        "--print-hz",
        type=float,
        default=2.0,
        help="terminal matrix rate; the CSV still records every sample",
    )
    parser.add_argument(
        "--position-kp-scale", nargs=2, type=float, default=[0.35, 0.35]
    )
    parser.add_argument(
        "--position-kd-scale", nargs=2, type=float, default=[1.0, 1.0]
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("/tmp/wrist_dynamic_inertia.csv"),
    )
    parser.add_argument("--confirm-move", default="")
    return parser


def run(args: argparse.Namespace) -> None:
    if args.confirm_move != "RUN_WRIST_DYNAMIC_INERTIA":
        raise RuntimeError(
            "该脚本会移动 q8/q9；必须加 --confirm-move RUN_WRIST_DYNAMIC_INERTIA"
        )
    if not math.isfinite(args.duration_s) or args.duration_s < 8.0:
        raise ValueError("--duration-s 必须至少为 8 秒")
    if not math.isfinite(args.period_s) or args.period_s < 4.0:
        raise ValueError("--period-s 必须至少为 4 秒")
    if not 1.0 <= args.sample_hz <= 100.0:
        raise ValueError("--sample-hz 必须在 1..100 Hz")
    if not 0.1 <= args.print_hz <= args.sample_hz:
        raise ValueError("--print-hz 必须在 0.1..sample-hz 范围")
    amplitudes_deg = np.asarray(args.amplitude_deg, dtype=float)
    if (
        amplitudes_deg.shape != (2,)
        or np.any(~np.isfinite(amplitudes_deg))
        or np.any(amplitudes_deg <= 0.0)
    ):
        raise ValueError("--amplitude-deg 必须是两个有限正数")
    amplitudes = np.deg2rad(amplitudes_deg)

    profile = load_profile(args.config.resolve())
    dynamics, inertia_source = _load_dynamics(profile)
    following_error_deg = float(profile.wrist["position_following_error_deg"])
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    print("腕部动态惯量演示：只移动 q8/q9，不连接或移动 Flexiv。")
    print("不会重定义或保存零位；Ctrl-C 会立即向两台 moteus 发送 STOP。")
    print(f"auto_inertia_source={inertia_source}")
    print(
        "矩阵定义：M_wrist=2x2 joint mass；I_spatial_flange=6x6，"
        "twist 顺序 [vx,vy,vz,wx,wy,wz]。"
    )

    with WristBridge(
        profile,
        zero_hold_mask_override=[False, False],
        position_following_error_deg_override=following_error_deg,
        position_kp_scale_override=args.position_kp_scale,
        position_kd_scale_override=args.position_kd_scale,
    ) as wrist:
        wrist.wait_ready(float(profile.wrist["zero_timeout_s"]) + 5.0)
        initial = wrist.wait_first_sample(2.0)
        wrist.finish_startup()
        center = initial.q_rad.copy()
        limits = np.deg2rad(
            np.asarray(profile.wrist["joint_limit_deg"], dtype=float)
        )
        if np.any(np.abs(center) + amplitudes >= limits):
            raise RuntimeError("当前姿态加运动幅度会超过腕部配置的物理行程")
        print(
            "center_deg=[{}] amplitude_deg=[{}] sample_hz={:g} output={}".format(
                ", ".join(f"{value:+.2f}" for value in np.rad2deg(center)),
                ", ".join(f"{value:g}" for value in amplitudes_deg),
                args.sample_hz,
                output,
            )
        )
        started = time.monotonic()
        next_tick = started
        next_print = started
        sample_period = 1.0 / args.sample_hz
        print_period = 1.0 / args.print_hz
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(_csv_header())
            while True:
                now = time.monotonic()
                elapsed = now - started
                if elapsed >= args.duration_s:
                    break
                target = _target(
                    center, amplitudes, elapsed, args.duration_s, args.period_s
                )
                wrist.command_position(target)
                sample = wrist.latest()
                mass = dynamics.mass_matrix(sample.q_rad)
                spatial = dynamics.composite_spatial_inertia(sample.q_rad)
                mass_eigen = np.linalg.eigvalsh(mass)
                spatial_eigen = np.linalg.eigvalsh(spatial)
                writer.writerow(
                    [
                        elapsed,
                        *sample.q_rad,
                        *sample.dq_rad_s,
                        *target,
                        *mass.reshape(-1),
                        *spatial.reshape(-1),
                        *mass_eigen,
                        *spatial_eigen,
                    ]
                )
                if now >= next_print:
                    print(
                        "t={:.3f}s q_deg=[{}]\nM_wrist_2x2=\n{}\n"
                        "I_spatial_flange_6x6=\n{}".format(
                            elapsed,
                            ", ".join(
                                f"{value:+.2f}"
                                for value in np.rad2deg(sample.q_rad)
                            ),
                            np.array2string(mass, precision=8),
                            np.array2string(spatial, precision=8),
                        ),
                        flush=True,
                    )
                    next_print += print_period
                next_tick += sample_period
                time.sleep(max(0.0, next_tick - time.monotonic()))
    print(f"完成：每个采样时刻的矩阵已保存到 {output}")


def main() -> int:
    args = build_parser().parse_args()
    try:
        run(args)
        return 0
    except KeyboardInterrupt:
        print("收到 Ctrl-C：腕部 STOP。", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"错误: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
