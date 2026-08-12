from __future__ import annotations

from types import SimpleNamespace
from pathlib import Path
import sys

import numpy as np

sys.path.insert(0, str(Path(__file__).parents[2]))

from scripts.demo_9dof import (
    _live_tool_zero_geometry,
    _select_inertia_parameters,
    parser,
)
from tools.check_inertia import spatial_inertia_at_origin


def test_live_elements_tcp_sets_wrist_zero_tip_and_rotation() -> None:
    profile = SimpleNamespace(
        document={
            "wrist_geometry": {
                "joint1_origin_flange_m": [0.0, 0.0, 0.112],
                "joint2_offset_after_joint1_m": [0.0, 0.0, 0.0],
            }
        }
    )
    active_tool = {
        "tcp_location": np.asarray(
            [0.0166, 0.0469, 0.1448, 1.0, 0.0, 0.0, 0.0]
        )
    }

    tip_offset, rotation = _live_tool_zero_geometry(profile, active_tool)

    np.testing.assert_allclose(tip_offset, [0.0166, 0.0469, 0.0328], atol=1e-12)
    np.testing.assert_allclose(rotation, np.eye(3), atol=1e-12)


def test_9dof_demo_defaults_to_auto_inertia() -> None:
    args = parser().parse_args([])
    assert args.inertia_mode == "auto"


def test_auto_inertia_rejects_current_floor_hit_and_large_residual() -> None:
    profile = SimpleNamespace(
        payload={"reflected_joint_inertia_kg_m2": [0.008, 0.003]}
    )
    calibration = {
        "rigid_body_scale": 0.605,
        "reflected_joint_inertia_kg_m2": [1e-6, 0.0021],
        "fit_rms_torque_nm": 0.266,
        "regression_condition_number": 119.0,
    }

    scale, reflected, source = _select_inertia_parameters(
        profile, calibration, "auto"
    )

    assert scale == 1.0
    np.testing.assert_allclose(reflected, [0.008, 0.003])
    assert source.startswith("elements_baseline+nominal_wrist")


def test_elements_payload_builds_symmetric_positive_6x6_spatial_inertia() -> None:
    matrix = spatial_inertia_at_origin(
        0.906,
        np.asarray([-0.0011, -0.0011, 0.0641]),
        np.asarray([0.001647, 0.001996, 0.001730, 0.000691, -0.000958, 0.000507]),
    )

    assert matrix.shape == (6, 6)
    np.testing.assert_allclose(matrix, matrix.T, atol=1e-12)
    assert np.min(np.linalg.eigvalsh(matrix)) > 0.0
