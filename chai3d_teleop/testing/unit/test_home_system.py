from __future__ import annotations

import numpy as np
import pytest

from tools.home_system import elevated_home_tcp_pose, lifted_tcp_pose


def test_lifted_tcp_pose_changes_only_world_z() -> None:
    current = np.asarray([0.40, -0.20, 0.30, 0.5, 0.5, -0.5, 0.5])
    target = lifted_tcp_pose(current, 0.20)
    np.testing.assert_allclose(target, [0.40, -0.20, 0.50, 0.5, 0.5, -0.5, 0.5])
    np.testing.assert_allclose(current, [0.40, -0.20, 0.30, 0.5, 0.5, -0.5, 0.5])


@pytest.mark.parametrize("lift", [0.0, -0.1, float("nan")])
def test_lifted_tcp_pose_rejects_invalid_distance(lift: float) -> None:
    with pytest.raises(ValueError, match="finite positive"):
        lifted_tcp_pose(np.asarray([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]), lift)


def test_elevated_home_tcp_pose_uses_home_fk_and_active_tool() -> None:
    world_to_home_flange = np.eye(4)
    world_to_home_flange[:3, 3] = [0.40, -0.10, 0.30]
    flange_to_tcp_pose = np.asarray([0.02, 0.03, 0.15, 1.0, 0.0, 0.0, 0.0])
    target = elevated_home_tcp_pose(
        world_to_home_flange, flange_to_tcp_pose, 0.20
    )
    np.testing.assert_allclose(
        target, [0.42, -0.07, 0.65, 1.0, 0.0, 0.0, 0.0]
    )
