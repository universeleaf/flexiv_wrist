#!/usr/bin/env python3
"""Identify wrist inertia/friction, then evaluate M(q8,q9) in real time.

Collection moves only the two moteus wrist joints around their current pose;
it never commands the Flexiv arm.  Analysis is ordinary least squares over a
physical rigid-body model and can be rerun without hardware.
"""

from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from typing import Any

import numpy as np


SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.run_9dof_teleop import (  # noqa: E402
    DEFAULT_CONFIG,
    WristBridge,
    load_profile,
)
from src.nine_dof_core import pose_to_transform  # noqa: E402
from src.wrist_dynamics import (  # noqa: E402
    WristDynamics,
    WristInertialParameters,
)


def _parameters_from_profile(profile, calibration: dict[str, Any] | None = None):
    payload = profile.payload
    calibration = calibration or {}
    return WristInertialParameters(
        link1_mass_kg=float(payload["link1_mass_kg"]),
        link1_com_after_joint1_m=np.asarray(
            payload["link1_com_after_joint1_m"], dtype=float
        ),
        link1_inertia_com_kg_m2=np.asarray(
            payload["link1_inertia_com_kg_m2_row_major"], dtype=float
        ).reshape(3, 3),
        link2_mass_kg=float(payload["link2_and_probe_mass_kg"]),
        link2_com_after_joint2_m=np.asarray(
            payload["link2_and_probe_com_after_joint2_m"], dtype=float
        ),
        link2_inertia_com_kg_m2=np.asarray(
            payload["link2_and_probe_inertia_com_kg_m2_row_major"], dtype=float
        ).reshape(3, 3),
        reflected_joint_inertia_kg_m2=np.asarray(
            calibration.get(
                "reflected_joint_inertia_kg_m2",
                payload["reflected_joint_inertia_kg_m2"],
            ),
            dtype=float,
        ),
        viscous_friction_nm_s_rad=np.asarray(
            calibration.get(
                "viscous_friction_nm_s_rad", payload["viscous_friction_nm_s_rad"]
            ),
            dtype=float,
        ),
        coulomb_friction_nm=np.asarray(
            calibration.get("coulomb_friction_nm", payload["coulomb_friction_nm"]),
            dtype=float,
        ),
        torque_bias_nm=np.asarray(
            calibration.get("torque_bias_nm", payload["torque_bias_nm"]), dtype=float
        ),
        rigid_body_scale=float(calibration.get("rigid_body_scale", 1.0)),
    )


def _smoothstep(value: float) -> float:
    value = float(np.clip(value, 0.0, 1.0))
    return value * value * (3.0 - 2.0 * value)


def _set_current_pose_as_saved_zero(config_path: Path) -> None:
    """STOP/query both axes, then persist their current raw positions as q=0."""
    executable = PROJECT_ROOT / ".venv_moteus" / "bin" / "python"
    helper = PROJECT_ROOT / "scripts" / "set_current_wrist_zero.py"
    if not executable.is_file():
        raise FileNotFoundError(f"找不到 moteus Python: {executable}")
    command = [
        str(executable),
        str(helper),
        "--config",
        str(config_path),
        "--confirm-set-zero",
        "SET_CURRENT_WRIST_ZERO",
    ]
    print("辨识启动：先将当前两轴姿态保存为新的应用层 q8=q9=0。", flush=True)
    result = subprocess.run(command, cwd=PROJECT_ROOT, check=False)
    if result.returncode != 0:
        raise RuntimeError(f"自动保存当前腕部零位失败，返回码 {result.returncode}")


def collect(
    profile,
    output: Path,
    duration_s: float,
    amplitudes_deg: np.ndarray,
    position_kp_scale: np.ndarray,
    position_kd_scale: np.ndarray,
) -> None:
    import flexivrdk

    if duration_s < 20.0:
        raise ValueError("辨识采集至少需要 20 秒")
    limits = np.deg2rad(np.asarray(profile.wrist["joint_limit_deg"], dtype=float))
    amplitudes = np.deg2rad(np.asarray(amplitudes_deg, dtype=float))
    if amplitudes.shape != (2,) or np.any(amplitudes <= 0.0):
        raise ValueError("--amplitude-deg 必须是两个正数")
    position_kp_scale = np.asarray(position_kp_scale, dtype=float)
    position_kd_scale = np.asarray(position_kd_scale, dtype=float)
    for name, values in (
        ("--position-kp-scale", position_kp_scale),
        ("--position-kd-scale", position_kd_scale),
    ):
        if values.shape != (2,) or np.any(~np.isfinite(values)) or np.any(values <= 0.0) or np.any(values > 1.0):
            raise ValueError(f"{name} 必须是两个 (0, 1] 内的有限数")

    print("连接 Flexiv 仅用于读取法兰姿态；不会发送任何 Flexiv 运动/力矩命令。")
    robot = flexivrdk.Robot(
        str(profile.robot["robot_sn"]), [str(profile.robot["network_interface_ip"])]
    )
    # Identification data is invalid if a joint is several degrees away from
    # its commanded excitation.  Keep this tighter than teleoperation so an
    # unstable/misconfigured moteus position loop is stopped promptly instead
    # of being hidden by the general runtime threshold.
    following_error_deg = min(
        float(profile.wrist["position_following_error_deg"]),
        max(2.0, 0.5 * float(np.max(amplitudes_deg))),
    )
    with WristBridge(
        profile,
        zero_hold_mask_override=[False, False],
        position_following_error_deg_override=following_error_deg,
        position_kp_scale_override=position_kp_scale,
        position_kd_scale_override=position_kd_scale,
    ) as wrist:
        wrist.wait_ready(float(profile.wrist["zero_timeout_s"]) + 5.0)
        initial = wrist.wait_first_sample(2.0)
        wrist.finish_startup()
        center = initial.q_rad.copy()
        # Do not apply the teleoperation soft-limit margin to identification.
        # Only reject a waveform whose requested target itself crosses the
        # actuator's stated physical travel.
        if np.any(np.abs(center) + amplitudes >= limits):
            raise RuntimeError(
                "当前中心角加辨识振幅会超出腕部物理行程；先把该轴移回行程中部，"
                "或减小 --amplitude-deg"
            )
        print(
            "开始多正弦辨识：中心角=[{}] deg，位置增益缩放 Kp=[{}] Kd=[{}]，"
            "目标跟随异常阈值={:.1f}deg，"
            "Ctrl-C 可随时 STOP。".format(
                ", ".join(f"{x:+.2f}" for x in np.rad2deg(center)),
                ", ".join(f"{x:g}" for x in position_kp_scale),
                ", ".join(f"{x:g}" for x in position_kd_scale),
                following_error_deg,
            )
        )
        output.parent.mkdir(parents=True, exist_ok=True)
        rate_hz = min(100.0, float(profile.wrist["loop_rate_hz"]))
        period = 1.0 / rate_hz
        started = time.monotonic()
        next_tick = started
        with output.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "time_s",
                    "q8_rad",
                    "q9_rad",
                    "dq8_rad_s",
                    "dq9_rad_s",
                    "tau8_nm",
                    "tau9_nm",
                    "target_q8_rad",
                    "target_q9_rad",
                    *[f"R_world_flange_{r}{c}" for r in range(3) for c in range(3)],
                ]
            )
            while True:
                now = time.monotonic()
                elapsed = now - started
                if elapsed >= duration_s:
                    break
                envelope = _smoothstep(elapsed / 3.0) * _smoothstep(
                    (duration_s - elapsed) / 3.0
                )
                # Incommensurate components excite q, dq and ddq without a
                # simultaneous direction reversal on both joints.
                q_target = center + envelope * np.array(
                    [
                        amplitudes[0]
                        * (0.72 * math.sin(2 * math.pi * 0.13 * elapsed)
                           + 0.28 * math.sin(2 * math.pi * 0.31 * elapsed)),
                        amplitudes[1]
                        * (0.68 * math.sin(2 * math.pi * 0.17 * elapsed + 0.8)
                           + 0.32 * math.sin(2 * math.pi * 0.37 * elapsed)),
                    ]
                )
                wrist.command_position(q_target)
                sample = wrist.latest()
                flange_rotation = pose_to_transform(robot.states().flange_pose)[:3, :3]
                writer.writerow(
                    [
                        elapsed,
                        *sample.q_rad,
                        *sample.dq_rad_s,
                        *sample.torque_nm,
                        *q_target,
                        *flange_rotation.reshape(-1),
                    ]
                )
                next_tick += period
                time.sleep(max(0.0, next_tick - time.monotonic()))
    print(f"辨识数据已保存: {output}")
    metadata_path = output.with_suffix(output.suffix + ".meta.json")
    metadata_path.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "assembly_id": str(profile.payload["assembly_id"]),
                "zero_position_rev": list(profile.wrist["zero_position_rev"]),
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"总成元数据已保存: {metadata_path}")


def _lowpass(samples: np.ndarray, dt: float, cutoff_hz: float) -> np.ndarray:
    result = np.asarray(samples, dtype=float).copy()
    alpha = 1.0 - math.exp(-2.0 * math.pi * cutoff_hz * dt)
    for index in range(1, len(result)):
        result[index] = result[index - 1] + alpha * (result[index] - result[index - 1])
    return result


def analyze(profile, input_path: Path, output_path: Path) -> dict[str, Any]:
    metadata_path = input_path.with_suffix(input_path.suffix + ".meta.json")
    if not metadata_path.is_file():
        raise RuntimeError(
            f"缺少辨识总成元数据 {metadata_path}；请用当前版本重新采集，不能复用拆探针前的 CSV"
        )
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    assembly_id = str(profile.payload["assembly_id"])
    if metadata.get("assembly_id") != assembly_id:
        raise RuntimeError(
            "辨识 CSV 来自不同硬件总成："
            f"{metadata.get('assembly_id')!r} != {assembly_id!r}"
        )
    data = np.genfromtxt(input_path, delimiter=",", names=True)
    if data.size < 1000:
        raise ValueError("辨识数据太短（至少需要 1000 个样本）")
    time_s = np.asarray(data["time_s"], dtype=float)
    dt = float(np.median(np.diff(time_s)))
    q = np.column_stack((data["q8_rad"], data["q9_rad"]))
    dq = _lowpass(
        np.column_stack((data["dq8_rad_s"], data["dq9_rad_s"])), dt, 8.0
    )
    ddq = _lowpass(np.gradient(dq, time_s, axis=0), dt, 5.0)
    torque = _lowpass(
        np.column_stack((data["tau8_nm"], data["tau9_nm"])), dt, 8.0
    )
    rotations = np.column_stack(
        [data[f"R_world_flange_{r}{c}"] for r in range(3) for c in range(3)]
    ).reshape(-1, 3, 3)

    base_parameters = _parameters_from_profile(profile)
    base_parameters = WristInertialParameters(
        **{
            **asdict(base_parameters),
            "reflected_joint_inertia_kg_m2": np.array([1e-6, 1e-6]),
            "viscous_friction_nm_s_rad": np.zeros(2),
            "coulomb_friction_nm": np.zeros(2),
            "torque_bias_nm": np.zeros(2),
            "rigid_body_scale": 1.0,
        }
    )
    dynamics = WristDynamics(profile.geometry, base_parameters)
    rows: list[np.ndarray] = []
    observations: list[float] = []
    # Exclude the three-second entry/exit envelopes where finite-difference
    # acceleration is least reliable.
    keep = np.flatnonzero((time_s > 3.5) & (time_s < time_s[-1] - 3.5))
    for index in keep:
        rigid = (
            dynamics.rigid_body_mass_matrix(q[index]) @ ddq[index]
            + dynamics.coriolis_torque(q[index], dq[index])
            + dynamics.gravity_compensation(q[index], rotations[index])
        )
        signs = np.tanh(dq[index] / 0.02)
        rows.append(
            np.array([rigid[0], ddq[index, 0], 0, dq[index, 0], 0, signs[0], 0, 1, 0])
        )
        observations.append(float(torque[index, 0]))
        rows.append(
            np.array([rigid[1], 0, ddq[index, 1], 0, dq[index, 1], 0, signs[1], 0, 1])
        )
        observations.append(float(torque[index, 1]))
    design = np.asarray(rows)
    measured = np.asarray(observations)
    parameters, _, rank, singular_values = np.linalg.lstsq(design, measured, rcond=1e-8)
    if rank < design.shape[1]:
        raise RuntimeError("辨识轨迹激励不足，回归矩阵不满秩")
    predicted = design @ parameters
    rms = float(np.sqrt(np.mean((predicted - measured) ** 2)))
    result = {
        "schema_version": 1,
        "assembly_id": assembly_id,
        "source_csv": str(input_path.resolve()),
        "sample_period_s": dt,
        "samples_used": int(len(keep)),
        "rigid_body_scale": max(0.2, float(parameters[0])),
        "reflected_joint_inertia_kg_m2": [
            max(1e-6, float(parameters[1])),
            max(1e-6, float(parameters[2])),
        ],
        "viscous_friction_nm_s_rad": [
            max(0.0, float(parameters[3])),
            max(0.0, float(parameters[4])),
        ],
        "coulomb_friction_nm": [
            max(0.0, float(parameters[5])),
            max(0.0, float(parameters[6])),
        ],
        "torque_bias_nm": [float(parameters[7]), float(parameters[8])],
        "fit_rms_torque_nm": rms,
        "regression_condition_number": float(singular_values[0] / singular_values[-1]),
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    print(f"辨识结果已保存: {output_path}")
    return result


def monitor(profile, calibration_path: Path, seconds: float) -> None:
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    dynamics = WristDynamics(
        profile.geometry, _parameters_from_profile(profile, calibration)
    )
    with WristBridge(profile, zero_hold_mask_override=[False, False]) as wrist:
        wrist.wait_ready(float(profile.wrist["zero_timeout_s"]) + 5.0)
        wrist.wait_first_sample(2.0)
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            sample = wrist.latest()
            matrix = dynamics.mass_matrix(sample.q_rad)
            print(
                "q_deg=[{}] M_kgm2=[[{:.6f},{:.6f}],[{:.6f},{:.6f}]] eig=[{}]".format(
                    ", ".join(f"{x:+.2f}" for x in np.rad2deg(sample.q_rad)),
                    matrix[0, 0], matrix[0, 1], matrix[1, 0], matrix[1, 1],
                    ", ".join(f"{x:.6f}" for x in np.linalg.eigvalsh(matrix)),
                )
            )
            time.sleep(0.1)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    action = parser.add_mutually_exclusive_group(required=True)
    action.add_argument("--collect", type=Path, metavar="CSV")
    action.add_argument("--analyze", type=Path, metavar="CSV")
    action.add_argument("--monitor", action="store_true")
    parser.add_argument("--output", type=Path, default=PROJECT_ROOT / "config" / "wrist_inertia_calibration.json")
    parser.add_argument("--duration-s", type=float, default=40.0)
    parser.add_argument("--amplitude-deg", nargs=2, type=float, default=[6.0, 10.0])
    parser.add_argument(
        "--position-kp-scale",
        nargs=2,
        type=float,
        default=[0.2, 0.2],
        metavar=("ID2", "ID1"),
        help="辨识专用 moteus Kp 缩放，默认 0.2 0.2；不修改 TOML/固件",
    )
    parser.add_argument(
        "--position-kd-scale",
        nargs=2,
        type=float,
        default=[1.0, 1.0],
        metavar=("ID2", "ID1"),
        help="辨识专用 moteus Kd 缩放，默认 1.0 1.0；不修改 TOML/固件",
    )
    parser.add_argument("--confirm-move", default="")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        config_path = args.config.resolve()
        if args.collect is not None:
            if args.confirm_move != "CALIBRATE_WRIST_INERTIA":
                raise RuntimeError(
                    "真实辨识会移动腕部；必须加 --confirm-move CALIBRATE_WRIST_INERTIA"
                )
            _set_current_pose_as_saved_zero(config_path)
        # The automatic zero helper updates TOML atomically, so load/reload the
        # profile only after it finishes.
        profile = load_profile(config_path)
        if args.collect is not None:
            collect(
                profile,
                args.collect.resolve(),
                args.duration_s,
                np.asarray(args.amplitude_deg),
                np.asarray(args.position_kp_scale),
                np.asarray(args.position_kd_scale),
            )
        elif args.analyze is not None:
            analyze(profile, args.analyze.resolve(), args.output.resolve())
        else:
            monitor(profile, args.output.resolve(), args.duration_s)
        return 0
    except KeyboardInterrupt:
        print("收到 Ctrl-C：腕部 STOP。", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"错误: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
