from __future__ import annotations

import math
from pathlib import Path
import tomllib

import numpy as np

from src.nine_dof_core import (
    PedalModeStateMachine,
    TeleopMode,
    WristGeometry,
    allocate_wrist_orientation,
    axis_angle_rotation,
    flange_target_for_probe,
    operational_space_wrist_torque,
    orientation_only_target,
    parse_runtime_mode_command,
    pose_to_transform,
    probe_force_from_world,
    probe_pose_from_flange,
    rotation_vector,
    scale_rotation,
    wrist_hold_axes,
    wrist_gravity_compensation,
)
from src.teleop_core import HapticSample, MappingConfig, RelativePoseMapper, parse_axis_map


def geometry() -> WristGeometry:
    return WristGeometry(
        joint1_origin_flange_m=np.array([0.0, 0.0, 0.04]),
        joint1_axis_flange=np.array([1.0, 0.0, 0.0]),
        joint2_offset_after_joint1_m=np.array([0.0, 0.0, 0.03]),
        joint2_axis_after_joint1=np.array([0.0, 1.0, 0.0]),
        tip_offset_after_joint2_m=np.array([0.0, 0.0, 0.12]),
        probe_rotation_at_zero=np.eye(3),
        joint_min_rad=np.deg2rad([-180.0, -90.0]),
        joint_max_rad=np.deg2rad([180.0, 90.0]),
    )


def test_runtime_ui_mode_commands_map_to_the_same_three_modes() -> None:
    assert parse_runtime_mode_command("MODE 1\n") is TeleopMode.ARM_7DOF
    assert parse_runtime_mode_command(" mode   2 ") is TeleopMode.ARM_WRIST_9DOF
    assert parse_runtime_mode_command("MODE 3") is TeleopMode.PIVOT_ORIENTATION


def test_commissioning_profile_matches_measured_zero_and_positive_directions() -> None:
    config_path = Path(__file__).parents[1] / "config" / "nine_dof_teleop.toml"
    with config_path.open("rb") as stream:
        document = tomllib.load(stream)
    wrist = document["wrist"]
    configured = document["wrist_geometry"]
    limits = np.deg2rad(np.asarray(wrist["joint_limit_deg"], dtype=float))
    model = WristGeometry(
        joint1_origin_flange_m=np.asarray(
            configured["joint1_origin_flange_m"], dtype=float
        ),
        joint1_axis_flange=np.asarray(configured["joint1_axis_flange"], dtype=float),
        joint2_offset_after_joint1_m=np.asarray(
            configured["joint2_offset_after_joint1_m"], dtype=float
        ),
        joint2_axis_after_joint1=np.asarray(
            configured["joint2_axis_after_joint1"], dtype=float
        ),
        tip_offset_after_joint2_m=np.asarray(
            configured["tip_offset_after_joint2_m"], dtype=float
        ),
        probe_rotation_at_zero=np.asarray(
            configured["probe_rotation_at_zero_row_major"], dtype=float
        ).reshape(3, 3),
        joint_min_rad=-limits,
        joint_max_rad=limits,
    )

    neutral = model.forward(np.zeros(2))
    expected_tool_tcp = np.asarray(
        document["flexiv_tool"]["expected_tcp_location"], dtype=float
    )
    assert wrist["ids"] == [2, 1]
    assert np.allclose(neutral[:3, 3], expected_tool_tcp[:3])
    assert np.allclose(neutral[:3, :3], np.eye(3))

    # Positive joint 8 moves the neutral tip toward flange +X (forward).
    joint8_positive = model.forward(np.deg2rad([5.0, 0.0]))
    assert joint8_positive[0, 3] > neutral[0, 3]
    # Positive joint 9 is clockwise when viewed from the probe (+Z) side.
    joint9_positive = model.forward(np.deg2rad([0.0, 5.0]))
    assert joint9_positive[1, 0] < 0.0


def test_mode_change_while_clutched_requires_release() -> None:
    state = PedalModeStateMachine()
    transition = state.select(TeleopMode.ARM_7DOF, clutch_pressed=False)
    assert transition.ready
    assert state.teleoperation_enabled(True)

    transition = state.select(TeleopMode.ARM_WRIST_9DOF, clutch_pressed=True)
    assert not transition.ready
    assert not state.teleoperation_enabled(True)
    assert not state.observe_clutch(True).ready
    assert state.observe_clutch(False).ready
    assert state.teleoperation_enabled(True)


def test_joint9_remains_stopped_until_selected_mode_has_engaged() -> None:
    assert wrist_hold_axes(None, False) == (False, False)
    assert wrist_hold_axes(TeleopMode.ARM_7DOF, False) == (True, False)
    assert wrist_hold_axes(TeleopMode.ARM_7DOF, True) == (True, False)
    assert wrist_hold_axes(TeleopMode.ARM_WRIST_9DOF, False) == (True, False)
    assert wrist_hold_axes(TeleopMode.PIVOT_ORIENTATION, False) == (True, False)
    assert wrist_hold_axes(TeleopMode.ARM_WRIST_9DOF, True) == (True, True)
    assert wrist_hold_axes(TeleopMode.PIVOT_ORIENTATION, True) == (True, True)


def test_geometric_jacobian_matches_finite_difference() -> None:
    model = geometry()
    q = np.array([0.2, -0.35])
    jacobian = model.jacobian(q)
    initial = model.forward(q)
    epsilon = 1e-7
    for joint in range(2):
        perturbed_q = q.copy()
        perturbed_q[joint] += epsilon
        perturbed = model.forward(perturbed_q)
        linear_fd = (perturbed[:3, 3] - initial[:3, 3]) / epsilon
        rotation_delta = perturbed[:3, :3] @ initial[:3, :3].T
        angular_fd = np.array(
            [
                rotation_delta[2, 1] - rotation_delta[1, 2],
                rotation_delta[0, 2] - rotation_delta[2, 0],
                rotation_delta[1, 0] - rotation_delta[0, 1],
            ]
        ) / (2.0 * epsilon)
        assert np.allclose(jacobian[:3, joint], linear_fd, atol=1e-6)
        assert np.allclose(jacobian[3:, joint], angular_fd, atol=1e-6)


def test_flange_compensation_reconstructs_exact_probe_target() -> None:
    model = geometry()
    q = np.array([0.4, -0.25])
    target = np.eye(4)
    target[:3, :3] = axis_angle_rotation(np.array([0.2, 0.3, 0.9]), 0.7)
    target[:3, 3] = [0.45, -0.12, 0.31]
    flange = flange_target_for_probe(target, model, q)
    reconstructed = probe_pose_from_flange(flange, model, q)
    assert np.allclose(reconstructed, target, atol=1e-10)


def test_flexiv_world_force_is_projected_by_live_probe_orientation() -> None:
    rotation = axis_angle_rotation(np.array([0.0, 1.0, 0.0]), math.pi / 2.0)
    # Probe +Z points along world +X after the 90-degree Y rotation.
    force_probe = probe_force_from_world(
        np.array([15.5, 2.0, -1.0]), rotation, np.array([0.5, 2.0, -1.0])
    )
    assert np.allclose(force_probe, [0.0, 0.0, 15.0], atol=1e-10)


def test_force_projection_does_not_depend_on_tcp_translation() -> None:
    model = geometry()
    q = np.array([0.31, -0.47])
    rotation = model.forward(q)[:3, :3]
    expected = rotation.T @ np.array([1.0, -2.0, 3.0])
    assert np.allclose(
        probe_force_from_world(np.array([1.0, -2.0, 3.0]), rotation), expected
    )


def test_mode3_discards_haptic_translation_and_uses_robot_position() -> None:
    first = np.eye(4)
    first[:3, :3] = axis_angle_rotation(
        np.array([0.0, 1.0, 0.0]), math.radians(15.0)
    )
    first[:3, 3] = [1.0, 2.0, 3.0]
    second = first.copy()
    second[:3, 3] = [-4.0, 8.0, 0.5]
    robot_position = np.array([0.42, -0.11, 0.33])

    first_target = orientation_only_target(first, robot_position)
    second_target = orientation_only_target(second, robot_position)
    assert np.allclose(first_target[:3, 3], robot_position)
    assert np.allclose(second_target[:3, 3], robot_position)
    assert np.allclose(first_target, second_target)

    moved_robot_position = np.array([0.44, -0.08, 0.31])
    moved_target = orientation_only_target(first, moved_robot_position)
    assert np.allclose(moved_target[:3, 3], moved_robot_position)
    assert np.allclose(moved_target[:3, :3], first_target[:3, :3])


def test_mode3_pure_handle_translation_cannot_change_robot_target() -> None:
    mapper = RelativePoseMapper(
        MappingConfig(
            translation_scale=2.0,
            max_translation_m=None,
            max_step_m=None,
            enable_rotation=True,
            max_rotation_rad=None,
            max_angular_step_rad=None,
        ),
        parse_axis_map("-x,-y,z"),
    )
    anchor_sample = HapticSample(1, np.zeros(3), np.eye(3), 1)
    translated_sample = HapticSample(
        2, np.array([0.08, -0.04, 0.06]), np.eye(3), 1
    )
    probe_anchor_pose = np.array([0.42, -0.11, 0.33, 1.0, 0.0, 0.0, 0.0])
    measured_robot_position = np.array([0.421, -0.109, 0.329])
    mapper.capture(anchor_sample, probe_anchor_pose)

    mapped = pose_to_transform(mapper.target(translated_sample))
    mode3_target = orientation_only_target(mapped, measured_robot_position)

    assert np.allclose(mode3_target[:3, 3], measured_robot_position)
    assert np.allclose(mode3_target[:3, :3], np.eye(3))


def test_wrist_priority_allocator_uses_both_reachable_axes() -> None:
    model = geometry()
    desired_q = np.array([0.35, -0.28])
    desired = model.forward(desired_q)[:3, :3]
    result = allocate_wrist_orientation(model, np.zeros(2), desired)
    assert np.allclose(result.q_target_rad, desired_q, atol=2e-4)
    assert np.linalg.norm(result.residual_rotvec_rad) < 2e-4


def test_allocator_leaves_unreachable_roll_for_flexiv_arm() -> None:
    model = geometry()
    desired = axis_angle_rotation(np.array([0.0, 0.0, 1.0]), math.radians(20.0))
    result = allocate_wrist_orientation(model, np.zeros(2), desired)
    assert np.linalg.norm(result.residual_rotvec_rad) > math.radians(10.0)


def test_mode2_rotation_priority_gain_doubles_reachable_motion() -> None:
    relative = axis_angle_rotation(np.array([0.0, 1.0, 0.0]), math.radians(12.0))
    amplified = scale_rotation(relative, 2.0)
    assert math.isclose(
        np.linalg.norm(rotation_vector(amplified)),
        math.radians(24.0),
        abs_tol=1e-10,
    )


def test_operational_space_torque_is_zero_at_target_and_clamped() -> None:
    model = geometry()
    q = np.array([0.1, -0.2])
    target = model.forward(q)[:3, :3]
    zero = operational_space_wrist_torque(
        model,
        q,
        np.zeros(2),
        target,
        rotational_stiffness_nm_per_rad=4.0,
        rotational_damping_nm_s_per_rad=0.25,
        max_torque_nm=np.array([0.3, 0.25]),
    )
    assert np.allclose(zero.torque_nm, 0.0, atol=1e-10)

    far_target = axis_angle_rotation(np.array([1.0, 0.0, 0.0]), 1.2) @ target
    driven = operational_space_wrist_torque(
        model,
        q,
        np.zeros(2),
        far_target,
        rotational_stiffness_nm_per_rad=4.0,
        rotational_damping_nm_s_per_rad=0.25,
        max_torque_nm=np.array([0.3, 0.25]),
    )
    assert np.all(np.abs(driven.torque_nm) <= np.array([0.3, 0.25]) + 1e-12)
    assert np.linalg.norm(driven.torque_nm) > 0.0


def test_operational_space_impedance_generates_physical_task_moment() -> None:
    model = geometry()
    q = np.zeros(2)
    target = axis_angle_rotation(
        model.joint1_axis_flange, math.radians(10.0)
    ) @ model.forward(q)[:3, :3]
    result = operational_space_wrist_torque(
        model,
        q,
        np.zeros(2),
        target,
        rotational_stiffness_nm_per_rad=6.0,
        rotational_damping_nm_s_per_rad=0.35,
        max_torque_nm=np.array([6.0, 2.0]),
    )
    assert math.isclose(result.torque_nm[0], math.radians(60.0), rel_tol=1e-6)
    assert math.isclose(result.torque_nm[1], 0.0, abs_tol=1e-10)


def test_gravity_compensation_cancels_virtual_work() -> None:
    model = geometry()
    q = np.array([0.3, -0.4])
    compensation = wrist_gravity_compensation(
        model,
        q,
        np.eye(3),
        link1_mass_kg=0.2,
        link1_com_after_joint1_m=np.array([0.0, 0.0, 0.02]),
        link2_mass_kg=0.35,
        link2_com_after_joint2_m=np.array([0.0, 0.0, 0.08]),
    )
    assert compensation.shape == (2,)
    assert np.all(np.isfinite(compensation))
    # Rotating the whole flange 180 degrees about X reverses gravity in its frame.
    inverted = wrist_gravity_compensation(
        model,
        q,
        axis_angle_rotation(np.array([1.0, 0.0, 0.0]), math.pi),
        link1_mass_kg=0.2,
        link1_com_after_joint1_m=np.array([0.0, 0.0, 0.02]),
        link2_mass_kg=0.35,
        link2_com_after_joint2_m=np.array([0.0, 0.0, 0.08]),
    )
    assert np.allclose(inverted, -compensation, atol=1e-10)
