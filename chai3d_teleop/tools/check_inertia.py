#!/usr/bin/env python3
"""Read-only inspection of the live Elements Tool and 6x6 spatial inertia."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from tools.osc_launcher import validate_active_flexiv_tool  # noqa: E402
from scripts.teleoperate import DEFAULT_CONFIG, load_profile  # noqa: E402
from scripts.demo_9dof import _select_inertia_parameters  # noqa: E402


def spatial_inertia_at_origin(
    mass_kg: float, com_m: np.ndarray, inertia_values: np.ndarray
) -> np.ndarray:
    """Return a 6x6 spatial inertia for twist ordering [linear; angular]."""
    com = np.asarray(com_m, dtype=float)
    ixx, iyy, izz, ixy, ixz, iyz = np.asarray(inertia_values, dtype=float)
    inertia_com = np.asarray(
        [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float
    )
    cross = np.asarray(
        [[0.0, -com[2], com[1]], [com[2], 0.0, -com[0]], [-com[1], com[0], 0.0]]
    )
    result = np.zeros((6, 6))
    result[:3, :3] = mass_kg * np.eye(3)
    result[:3, 3:] = -mass_kg * cross
    result[3:, :3] = mass_kg * cross
    result[3:, 3:] = inertia_com + mass_kg * cross.T @ cross
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    args = parser.parse_args()
    try:
        profile = load_profile(args.config.resolve())
        print("只读 auto inertia 检查：不会切换模式或发送任何运动/力矩命令。")
        tool = validate_active_flexiv_tool(profile)
        matrix = spatial_inertia_at_origin(
            float(tool["mass_kg"]),
            np.asarray(tool["com_m"]),
            np.asarray(tool["inertia_kg_m2"]),
        )
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
        print("IDENTIFICATION_STATUS=PASS")
        print(
            "fit_rms_torque_nm={:.6g} validation_rms_torque_nm={:.6g} "
            "condition={:.6g}".format(
                float(calibration.get("fit_rms_torque_nm", float("nan"))),
                float(calibration.get("validation_rms_torque_nm", float("nan"))),
                float(calibration.get("regression_condition_number", float("nan"))),
            )
        )
        print(
            "excitation_q_span_deg="
            + np.array2string(
                np.asarray(calibration.get("excitation_q_span_deg", [])),
                precision=3,
            )
        )
        print(
            "wrist_mass_min_eigen={:.9g} wrist_mass_max_condition={:.6g} "
            "parameter_bound_hits={}".format(
                float(
                    calibration.get(
                        "wrist_mass_matrix_min_eigenvalue", float("nan")
                    )
                ),
                float(
                    calibration.get(
                        "wrist_mass_matrix_max_condition", float("nan")
                    )
                ),
                calibration.get("parameter_bound_hits", []),
            )
        )
        print("tcp_location=" + np.array2string(np.asarray(tool["tcp_location"]), precision=6))
        print("spatial_inertia_6x6_flange=")
        print(np.array2string(matrix, precision=9, suppress_small=False))
        print("spatial_eigen=" + np.array2string(np.linalg.eigvalsh(matrix), precision=9))
        print(f"spatial_condition={np.linalg.cond(matrix):.3f}")
        print(f"auto_source={source}")
        print(f"auto_rigid_body_scale={scale:.9g}")
        print("auto_reflected_joint_inertia=" + np.array2string(reflected, precision=9))
        print("runtime_dimensions=body_spatial:6x6 joint_mass:9x9 task_lambda:6x6")
        return 0
    except Exception as error:
        print(f"错误: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
