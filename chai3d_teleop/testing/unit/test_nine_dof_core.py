from __future__ import annotations

import math
from pathlib import Path
import tomllib

import numpy as np
import scripts.teleoperate as teleoperate_module

from controllers.nine_dof import (
    Mode3ForceGate,
    PedalModeStateMachine,
    TeleopMode,
    WristGeometry,
    WristTargetShaper,
    allocate_wrist_orientation,
    axis_angle_rotation,
    flange_target_for_probe,
    flange_target_for_probe_decoupled,
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
from controllers.teleop import HapticSample, MappingConfig, RelativePoseMapper, parse_axis_map
from scripts.teleoperate import DEFAULT_CONFIG, build_ui_telemetry, load_profile


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


def test_saved_common_safe_loop_demo_profile_is_accepted() -> None:
    profile = load_profile(DEFAULT_CONFIG)
    demo = profile.document["demo"]
    assert demo["trajectory"] == "loop"
    assert demo["radius_m"] == 0.010
    assert demo["orientation_amplitude_deg"] == 35.0
    identification = profile.document["inertia_identification"]
    assert identification["duration_s"] == 120.0
    assert identification["amplitude_deg"] == [45.0, 60.0]
    assert profile.runtime["teleop_controller"] == "flexiv_impedance"
    assert profile.runtime["ui_telemetry_rate_hz"] == 20.0
    assert profile.teleop_impedance["stiffness_scale"] == [
        0.25,
        0.25,
        0.25,
        0.35,
        0.35,
        0.35,
    ]
    assert profile.teleop_impedance["released_stiffness_scale"] == [
        0.04,
        0.04,
        0.04,
        0.08,
        0.08,
        0.08,
    ]
    assert profile.teleop_impedance["mode3_stiffness_scale"] == [
        0.35,
        0.35,
        0.35,
        0.20,
        0.20,
        0.20,
    ]
    assert profile.osc["force_axis_max_velocity_m_s"] == 0.005
    assert profile.osc["force_command_ramp_s"] == 3.0
    assert profile.wrist["drive_watchdog_ms"] == 1000.0
    assert profile.allocation["teleop_wrist_max_tracking_lead_deg"] == [
        15.0,
        15.0,
    ]


def test_default_real_teleop_dispatches_to_flexiv_impedance(monkeypatch) -> None:
    profile = load_profile(DEFAULT_CONFIG)
    calls: list[tuple[object, object, bool]] = []

    def fake_impedance(selected_profile, mode3_force_n, *, ui_control_stdin):
        calls.append((selected_profile, mode3_force_n, ui_control_stdin))
        return 23

    monkeypatch.setattr(
        teleoperate_module, "_run_real_nrt_legacy", fake_impedance
    )
    result = teleoperate_module._run_real(
        profile, None, ui_control_stdin=True
    )

    assert result == 23
    assert calls == [(profile, None, True)]


def test_ui_telemetry_reports_probe_and_all_nine_joint_errors() -> None:
    actual = np.eye(4)
    actual[:3, 3] = [0.10, -0.20, 0.30]
    target = np.eye(4)
    target[:3, :3] = axis_angle_rotation(np.array([0.0, 0.0, 1.0]), math.radians(5.0))
    target[:3, 3] = [0.102, -0.203, 0.304]
    arm_actual = np.deg2rad(np.arange(7, dtype=float))
    arm_reference = arm_actual + math.radians(1.0)
    wrist_actual = np.deg2rad([10.0, -20.0])
    wrist_target = np.deg2rad([12.0, -23.0])

    sample = build_ui_telemetry(
        timestamp_s=1.25,
        mode=TeleopMode.ARM_WRIST_9DOF,
        enabled=True,
        actual_probe_world=actual,
        target_probe_world=target,
        arm_actual_q_rad=arm_actual,
        arm_reference_q_rad=arm_reference,
        wrist_actual_q_rad=wrist_actual,
        wrist_target_q_rad=wrist_target,
        force_measured_tool_z_n=-13.8,
        force_estimated_tool_z_n=-14.2,
        force_target_tool_z_n=-15.0,
        force_command_tool_z_n=-14.5,
        force_control_active=True,
    )

    assert np.allclose(sample["position_error_mm"], [2.0, -3.0, 4.0])
    assert math.isclose(sample["position_error_norm_mm"], math.sqrt(29.0))
    assert np.allclose(sample["orientation_error_rotvec_deg"], [0.0, 0.0, 5.0])
    assert len(sample["joint_error_deg"]) == 9
    assert np.allclose(sample["joint_error_deg"][:7], np.ones(7))
    assert np.allclose(sample["joint_error_deg"][7:], [2.0, -3.0])
    assert sample["arm_joint_target_kind"] == "nullspace_reference"
    assert sample["force_measured_tool_z_n"] == -13.8
    assert sample["force_estimated_tool_z_n"] == -14.2
    assert sample["force_target_tool_z_n"] == -15.0
    assert sample["force_command_tool_z_n"] == -14.5
    assert sample["force_control_active"] is True


def test_controller_parser_keeps_explicit_torque_osc_choice() -> None:
    args = teleoperate_module.build_parser().parse_args(
        ["--controller", "torque-osc"]
    )
    assert args.controller == "torque-osc"


def test_mode3_force_gate_waits_for_contact_then_ramps_gently() -> None:
    gate = Mode3ForceGate(
        target_force_n=-15.0,
        contact_enable_threshold_n=2.0,
        contact_release_threshold_n=1.0,
        contact_release_delay_s=0.05,
        force_ramp_s=3.0,
    )
    waiting = gate.update(-0.5, teleoperation_enabled=True, now_s=10.0)
    assert not waiting.force_axis_enabled
    assert waiting.commanded_force_n == 0.0

    acquired = gate.update(-2.5, teleoperation_enabled=True, now_s=11.0)
    assert acquired.force_axis_enabled
    assert acquired.changed
    assert acquired.commanded_force_n == -2.5

    halfway = gate.update(-3.0, teleoperation_enabled=True, now_s=12.5)
    assert halfway.force_axis_enabled
    assert math.isclose(halfway.commanded_force_n, -8.75)

    settled = gate.update(-15.0, teleoperation_enabled=True, now_s=15.0)
    assert settled.commanded_force_n == -15.0


def test_mode3_force_gate_releases_after_persistent_contact_loss() -> None:
    gate = Mode3ForceGate(
        target_force_n=-15.0,
        contact_enable_threshold_n=2.0,
        contact_release_threshold_n=1.0,
        contact_release_delay_s=0.05,
        force_ramp_s=3.0,
    )
    gate.update(-3.0, teleoperation_enabled=True, now_s=1.0)
    transient = gate.update(-0.2, teleoperation_enabled=True, now_s=2.0)
    assert transient.force_axis_enabled
    lost = gate.update(-0.2, teleoperation_enabled=True, now_s=2.06)
    assert not lost.force_axis_enabled
    assert lost.changed
    assert lost.reason == "contact_lost"


def test_runtime_ui_mode_commands_map_to_the_same_three_modes() -> None:
    assert parse_runtime_mode_command("MODE 1\n") is TeleopMode.ARM_7DOF
    assert parse_runtime_mode_command(" mode   2 ") is TeleopMode.ARM_WRIST_9DOF
    assert parse_runtime_mode_command("MODE 3") is TeleopMode.PIVOT_ORIENTATION


def test_commissioning_profile_matches_measured_zero_and_positive_directions() -> None:
    config_path = Path(__file__).parents[2] / "config" / "nine_dof_teleop.toml"
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
    assert wrist["joint_limit_deg"] == [90.0, 360.0]
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


def test_mode_state_can_be_cleared_to_a_disabled_hold() -> None:
    state = PedalModeStateMachine()
    state.select(TeleopMode.ARM_WRIST_9DOF, clutch_pressed=False)
    assert state.teleoperation_enabled(True)

    transition = state.clear()
    assert transition.changed
    assert transition.selected_mode is None
    assert not transition.ready
    assert not state.teleoperation_enabled(True)
    assert not state.observe_clutch(False).ready


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


def test_mode2_joint8_rotation_keeps_tcp_fixed_without_arm_rotation() -> None:
    model = geometry()
    q_target = np.array([math.radians(25.0), 0.0])
    initial_probe = model.forward(np.zeros(2))
    target_probe = initial_probe.copy()
    target_probe[:3, :3] = model.forward(q_target)[:3, :3]

    flange = flange_target_for_probe(target_probe, model, q_target)
    reconstructed = probe_pose_from_flange(flange, model, q_target)

    # Joint 8 supplies all requested orientation, so the Flexiv flange does
    # not rotate. It only translates slightly to cancel the offset-tip arc.
    assert np.allclose(flange[:3, :3], np.eye(3), atol=1e-10)
    assert np.linalg.norm(flange[:3, 3]) > 1e-3
    assert np.allclose(
        reconstructed[:3, 3], initial_probe[:3, 3], atol=1e-10
    )
    assert np.allclose(
        reconstructed[:3, :3], target_probe[:3, :3], atol=1e-10
    )


def test_mode2_slow_joint_keeps_tcp_position_without_arm_rotation() -> None:
    model = geometry()
    desired_q = np.array([math.radians(25.0), 0.0])
    measured_q = np.array([math.radians(8.0), 0.0])
    initial_probe = model.forward(np.zeros(2))
    target_probe = initial_probe.copy()
    target_probe[:3, :3] = model.forward(desired_q)[:3, :3]

    flange = flange_target_for_probe_decoupled(
        target_probe, model, desired_q, measured_q
    )
    live_probe = probe_pose_from_flange(flange, model, measured_q)

    # Desired q8 owns all target rotation, so Flexiv does not rotate while the
    # physical wrist catches up. Its small translation keeps live TCP exact.
    assert np.allclose(flange[:3, :3], np.eye(3), atol=1e-10)
    assert np.allclose(
        live_probe[:3, 3], target_probe[:3, 3], atol=1e-10
    )
    assert not np.allclose(
        live_probe[:3, :3], target_probe[:3, :3], atol=1e-4
    )


def test_mode1_xz_rotation_sign_is_reversed_from_mode2() -> None:
    profile = load_profile(DEFAULT_CONFIG)
    mode2_sign = np.asarray(
        profile.haptic["rotation_command_sign"], dtype=float
    )
    mode1_sign = np.asarray(
        profile.haptic["mode1_rotation_command_sign"], dtype=float
    )
    assert np.array_equal(mode2_sign, [-1.0, -1.0, 1.0])
    assert np.array_equal(mode1_sign, [-1.0, 1.0, 1.0])
    assert mode1_sign[1] == -mode2_sign[1]


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


def test_wrist_target_shaper_limits_velocity_and_acceleration() -> None:
    shaper = WristTargetShaper(
        filter_hz=1.5,
        max_velocity_rad_s=np.deg2rad([30.0, 40.0]),
        max_acceleration_rad_s2=np.deg2rad([120.0, 160.0]),
        joint_min_rad=np.deg2rad([-90.0, -360.0]),
        joint_max_rad=np.deg2rad([90.0, 360.0]),
    )
    previous = shaper.reset(np.zeros(2))
    previous_velocity = np.zeros(2)
    dt = 0.01
    for _ in range(200):
        current = shaper.step(np.deg2rad([80.0, 180.0]), dt)
        velocity = (current - previous) / dt
        acceleration = (velocity - previous_velocity) / dt
        assert np.all(np.abs(velocity) <= np.deg2rad([30.0, 40.0]) + 1e-10)
        assert np.all(np.abs(acceleration) <= np.deg2rad([120.0, 160.0]) + 1e-8)
        previous = current
        previous_velocity = velocity


def test_wrist_target_shaper_reversal_is_continuous() -> None:
    shaper = WristTargetShaper(
        filter_hz=1.5,
        max_velocity_rad_s=np.ones(2),
        max_acceleration_rad_s2=np.full(2, 2.0),
        joint_min_rad=np.full(2, -4.0),
        joint_max_rad=np.full(2, 4.0),
    )
    shaper.reset(np.zeros(2))
    before = shaper.step(np.ones(2), 0.01)
    after = shaper.step(-np.ones(2), 0.01)
    assert np.all(np.abs(after - before) <= 0.01 + 1e-12)


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
