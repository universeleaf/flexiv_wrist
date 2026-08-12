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
import sys
import time
from typing import Any

import numpy as np


SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.teleoperate import (  # noqa: E402
    DEFAULT_CONFIG,
    WristBridge,
    load_profile,
)
from controllers.nine_dof import pose_to_transform  # noqa: E402
from controllers.wrist_dynamics import (  # noqa: E402
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
    # Use the commissioned drive following-error diagnostic. A geared joint
    # can remain static until the sine target overcomes breakaway friction;
    # treating half the experiment amplitude as a fault caused every useful
    # identification run to stop before excitation began.
    following_error_deg = float(profile.wrist["position_following_error_deg"])
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
                # Deliberately visible, incommensurate multi-sine excitation.
                # The old 3/5 degree waveform was dominated by reducer static
                # friction, so its regression could not identify real inertia.
                # These components stay below the configured 45 deg/s and
                # 180 deg/s^2 position-profile limits for the new 15/30 deg
                # defaults while exciting q, dq and ddq independently.
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


def _bounded_least_squares(
    design: np.ndarray,
    measured: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    *,
    iterations: int = 500,
) -> np.ndarray:
    """Small dependency-free box-constrained linear least-squares solver."""
    unconstrained, *_ = np.linalg.lstsq(design, measured, rcond=1e-8)
    parameters = np.clip(unconstrained, lower, upper)
    residual = measured - design @ parameters
    column_energy = np.sum(design * design, axis=0)
    for _ in range(iterations):
        previous = parameters.copy()
        for column_index in range(design.shape[1]):
            energy = float(column_energy[column_index])
            if energy <= 1e-18:
                continue
            column = design[:, column_index]
            old = float(parameters[column_index])
            candidate = float(column @ (residual + column * old) / energy)
            new = float(np.clip(candidate, lower[column_index], upper[column_index]))
            parameters[column_index] = new
            residual -= column * (new - old)
        if np.linalg.norm(parameters - previous, ord=np.inf) < 1e-10:
            break
    return parameters


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
    validation_rows = np.repeat(np.arange(len(keep)) % 5 == 0, 2)
    fit_design = design[~validation_rows]
    fit_measured = measured[~validation_rows]
    normalized = fit_design / np.maximum(
        np.linalg.norm(fit_design, axis=0, keepdims=True), 1e-12
    )
    singular_values = np.linalg.svd(normalized, compute_uv=False)
    rank = np.linalg.matrix_rank(normalized, tol=1e-8)
    if rank < design.shape[1]:
        raise RuntimeError("辨识轨迹激励不足，回归矩阵不满秩")
    # [rigid scale, J8, J9, viscous8, viscous9, coulomb8, coulomb9,
    #  bias8, bias9]. Box constraints prevent an apparently low residual from
    # producing negative inertia/friction or a 20x fictitious rigid body.
    lower = np.asarray([0.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -3.0, -3.0])
    upper = np.asarray([1.5, 0.10, 0.10, 2.0, 2.0, 3.0, 3.0, 3.0, 3.0])
    parameters = _bounded_least_squares(
        fit_design, fit_measured, lower, upper
    )
    predicted = design @ parameters
    rms = float(np.sqrt(np.mean((predicted - measured) ** 2)))
    validation_residual = (
        design[validation_rows] @ parameters - measured[validation_rows]
    )
    validation_rms = float(np.sqrt(np.mean(validation_residual**2)))
    torque_rms = float(np.sqrt(np.mean(measured**2)))
    normalized_rms = rms / max(torque_rms, 1e-9)
    validation_normalized_rms = validation_rms / max(
        float(np.sqrt(np.mean(measured[validation_rows] ** 2))), 1e-9
    )
    q_span_deg = np.ptp(np.rad2deg(q[keep]), axis=0)
    dq_rms_rad_s = np.sqrt(np.mean(dq[keep] ** 2, axis=0))
    ddq_rms_rad_s2 = np.sqrt(np.mean(ddq[keep] ** 2, axis=0))
    bound_hits: list[str] = []
    parameter_names = (
        "rigid_body_scale", "reflected_joint8", "reflected_joint9",
        "viscous_joint8", "viscous_joint9", "coulomb_joint8",
        "coulomb_joint9", "bias_joint8", "bias_joint9",
    )
    for name, value, low, high in zip(parameter_names, parameters, lower, upper):
        tolerance = 1e-5 * max(1.0, abs(low), abs(high))
        if abs(value - low) <= tolerance or abs(value - high) <= tolerance:
            bound_hits.append(name)
    result = {
        "schema_version": 1,
        "assembly_id": assembly_id,
        "source_csv": str(input_path.resolve()),
        "sample_period_s": dt,
        "samples_used": int(len(keep)),
        "rigid_body_scale": float(parameters[0]),
        "reflected_joint_inertia_kg_m2": [
            float(parameters[1]),
            float(parameters[2]),
        ],
        "viscous_friction_nm_s_rad": [
            float(parameters[3]),
            float(parameters[4]),
        ],
        "coulomb_friction_nm": [
            float(parameters[5]),
            float(parameters[6]),
        ],
        "torque_bias_nm": [float(parameters[7]), float(parameters[8])],
        "fit_rms_torque_nm": rms,
        "fit_normalized_rms": normalized_rms,
        "validation_rms_torque_nm": validation_rms,
        "validation_normalized_rms": validation_normalized_rms,
        "regression_condition_number": float(singular_values[0] / singular_values[-1]),
        "excitation_q_span_deg": q_span_deg.tolist(),
        "excitation_dq_rms_rad_s": dq_rms_rad_s.tolist(),
        "excitation_ddq_rms_rad_s2": ddq_rms_rad_s2.tolist(),
        "parameter_bound_hits": bound_hits,
    }
    fitted_dynamics = WristDynamics(profile.geometry, _parameters_from_profile(profile, result))
    minimum_mass_eigenvalue = float("inf")
    maximum_mass_condition = 0.0
    q8_limit = np.deg2rad(float(profile.wrist["joint_limit_deg"][0]))
    q9_limit = np.deg2rad(float(profile.wrist["joint_limit_deg"][1]))
    for q8 in np.linspace(-q8_limit, q8_limit, 9):
        for q9 in np.linspace(
            -q9_limit,
            q9_limit,
            17,
        ):
            matrix = fitted_dynamics.mass_matrix(np.asarray([q8, q9]))
            eigenvalues = np.linalg.eigvalsh(matrix)
            minimum_mass_eigenvalue = min(minimum_mass_eigenvalue, float(eigenvalues[0]))
            maximum_mass_condition = max(
                maximum_mass_condition, float(eigenvalues[-1] / eigenvalues[0])
            )
    result["wrist_mass_matrix_min_eigenvalue"] = minimum_mass_eigenvalue
    result["wrist_mass_matrix_max_condition"] = maximum_mass_condition
    failure_reasons: list[str] = []
    if np.any(q_span_deg < np.asarray([8.0, 15.0])):
        failure_reasons.append(f"insufficient_q_span_deg={q_span_deg.tolist()}")
    if np.any(ddq_rms_rad_s2 < np.asarray([0.08, 0.15])):
        failure_reasons.append(
            f"insufficient_ddq_rms={ddq_rms_rad_s2.tolist()}"
        )
    if rms > 0.35 or normalized_rms > 0.65:
        failure_reasons.append(
            f"fit_residual={rms:.3f}Nm normalized={normalized_rms:.3f}"
        )
    if validation_rms > 0.40 or validation_normalized_rms > 0.70:
        failure_reasons.append(
            "validation_residual={:.3f}Nm normalized={:.3f}".format(
                validation_rms, validation_normalized_rms
            )
        )
    if result["regression_condition_number"] > 500.0:
        failure_reasons.append(
            f"condition={result['regression_condition_number']:.1f}"
        )
    # Reflected actuator inertia is not separately identifiable when the
    # rigid-body acceleration columns explain the same measured torque. A
    # zero optimum is valid for this *effective* model as long as the total
    # M(q) remains positive definite and validation residuals pass. Only a
    # rigid-body scale pinned to its artificial box boundary invalidates the
    # model.
    inertial_bound_hits = [
        name for name in bound_hits if name == "rigid_body_scale"
    ]
    if inertial_bound_hits:
        failure_reasons.append(f"inertial_parameter_bound_hits={inertial_bound_hits}")
    if minimum_mass_eigenvalue <= 1e-6 or maximum_mass_condition > 1e5:
        failure_reasons.append(
            "wrist_mass_matrix_invalid=min_eig={:.3e}, max_condition={:.1f}".format(
                minimum_mass_eigenvalue, maximum_mass_condition
            )
        )
    result["calibration_status"] = "PASS" if not failure_reasons else "FAIL"
    result["calibration_failure_reasons"] = failure_reasons
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
    parser.add_argument("--duration-s", type=float, default=90.0)
    parser.add_argument(
        "--amplitude-deg",
        nargs=2,
        type=float,
        default=[15.0, 30.0],
        help="joint8 joint9 peak excitation; default is a clearly visible 15/30 deg",
    )
    parser.add_argument(
        "--position-kp-scale",
        nargs=2,
        type=float,
        default=[0.35, 0.35],
        metavar=("ID2", "ID1"),
        help="辨识专用 moteus Kp 缩放，默认 0.35 0.35；不修改 TOML/固件",
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
        # Zero is a geometric calibration and must never be silently redefined
        # by an inertia experiment. Use `python run.py set-zero` explicitly
        # after mechanically aligning q8/q9, then identify around that zero.
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
