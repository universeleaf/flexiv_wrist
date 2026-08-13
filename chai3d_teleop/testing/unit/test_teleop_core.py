import math
from pathlib import Path

import numpy as np
import pytest

from controllers.teleop import (
    HapticSample,
    MappingConfig,
    RelativePoseMapper,
    StableHapticFeedback,
    add_velocity_damping,
    matrix_to_quaternion,
    limit_quaternion_step,
    map_feedback_force,
    map_progressive_feedback_force,
    parse_axis_map,
    parse_sample,
    quaternion_angular_distance,
    quaternion_to_rotation_vector,
    quaternion_to_matrix,
    rotation_vector_to_matrix,
)
from hardware.haptic_bridge import (
    build_parser,
    config_to_argv,
    parse_device_capabilities,
    validate_args,
)


def sample(position=(0.0, 0.0, 0.0), rotation=None, switches=0):
    return HapticSample(
        1,
        np.asarray(position, dtype=float),
        np.eye(3) if rotation is None else np.asarray(rotation, dtype=float),
        switches,
    )


def _stable_feedback(**overrides) -> StableHapticFeedback:
    settings = dict(
        update_rate_hz=100.0,
        lowpass_hz=6.0,
        force_slew_n_per_s=20.0,
        engagement_ramp_s=0.5,
        local_damping_n_per_m_s=8.0,
        max_device_force_n=12.0,
        initial_tank_energy_j=0.02,
        max_tank_energy_j=0.1,
        passivity_enabled=True,
    )
    settings.update(overrides)
    return StableHapticFeedback(**settings)


def test_stable_feedback_ramps_and_slew_limits_force() -> None:
    controller = _stable_feedback()
    first = controller.update(np.array([10.0, 0.0, 0.0]), np.zeros(3), dt_s=0.01)
    assert np.linalg.norm(first.force_device_n) <= 0.2 + 1e-12
    assert 0.0 < first.engagement < 1.0
    previous = first.force_device_n
    for _ in range(20):
        current = controller.update(
            np.array([10.0, 0.0, 0.0]), np.zeros(3), dt_s=0.01
        )
        assert np.linalg.norm(current.force_device_n - previous) <= 0.2 + 1e-12
        previous = current.force_device_n


def test_passivity_tank_never_becomes_negative() -> None:
    controller = _stable_feedback(
        force_slew_n_per_s=1000.0,
        engagement_ramp_s=0.001,
        initial_tank_energy_j=0.001,
        max_tank_energy_j=0.01,
        local_damping_n_per_m_s=0.0,
    )
    result = controller.update(
        np.array([12.0, 0.0, 0.0]), np.array([1.0, 0.0, 0.0]), dt_s=0.01
    )
    assert result.passivity_limited
    assert result.tank_energy_j >= 0.0
    assert float(np.dot(result.force_device_n, [1.0, 0.0, 0.0])) <= 0.1 + 1e-9


def test_dissipative_motion_recharges_bounded_tank() -> None:
    controller = _stable_feedback(
        force_slew_n_per_s=1000.0,
        engagement_ramp_s=0.001,
        initial_tank_energy_j=0.0,
        max_tank_energy_j=0.01,
        local_damping_n_per_m_s=0.0,
    )
    result = controller.update(
        np.array([-2.0, 0.0, 0.0]), np.array([0.5, 0.0, 0.0]), dt_s=0.01
    )
    assert not result.passivity_limited
    assert 0.0 < result.tank_energy_j <= 0.01


def test_parse_sample():
    parsed = parse_sample("1 0.1 0.2 0.3 1 0 0 0 1 0 0 0 1 5")
    assert np.allclose(parsed.position, [0.1, 0.2, 0.3])
    assert parsed.switch_pressed(0)
    assert not parsed.switch_pressed(1)
    assert parsed.switch_pressed(2)


def test_parse_sample_rejects_invalid_rotation():
    with pytest.raises(ValueError, match="旋转矩阵"):
        parse_sample("1 0.1 0.2 0.3 0 0 0 0 0 0 0 0 0 1")


def test_axis_map_must_be_right_handed():
    assert np.allclose(parse_axis_map("x,-z,y") @ [1, 2, 3], [1, 3, -2])
    with pytest.raises(ValueError, match="右手"):
        parse_axis_map("x,y,-z")


def test_quaternion_round_trip():
    angle = math.radians(42.0)
    quaternion = np.array([math.cos(angle / 2), 0.0, 0.0, math.sin(angle / 2)])
    recovered = matrix_to_quaternion(quaternion_to_matrix(quaternion))
    assert np.allclose(recovered, quaternion)


def test_quaternion_step_is_rate_limited():
    current = np.array([1.0, 0.0, 0.0, 0.0])
    target_angle = math.radians(30.0)
    target = np.array(
        [math.cos(target_angle / 2), 0.0, 0.0, math.sin(target_angle / 2)]
    )
    limited = limit_quaternion_step(current, target, math.radians(2.0))
    assert math.degrees(quaternion_angular_distance(current, limited)) == pytest.approx(2.0)


def test_quaternion_to_rotation_vector():
    angle = math.radians(12.0)
    quaternion = np.array([math.cos(angle / 2), math.sin(angle / 2), 0.0, 0.0])
    rotation_vector = np.rad2deg(quaternion_to_rotation_vector(quaternion))
    assert np.allclose(rotation_vector, [12.0, 0.0, 0.0])


def test_xy_flip_is_right_handed():
    mapping = parse_axis_map("-x,-y,z")
    assert np.allclose(mapping @ [1, 2, 3], [-1, -2, 3])


def test_translation_scaling_and_step_limit():
    mapper = RelativePoseMapper(
        MappingConfig(translation_scale=0.5, max_translation_m=0.05, max_step_m=0.002),
        np.eye(3),
    )
    robot_pose = np.array([0.4, -0.1, 0.3, 1.0, 0.0, 0.0, 0.0])
    mapper.capture(sample(), robot_pose)
    target = mapper.target(sample((0.02, 0.0, 0.0)))
    assert np.allclose(target[:3], [0.402, -0.1, 0.3])


def test_hard_workspace_limit():
    mapper = RelativePoseMapper(MappingConfig(max_translation_m=0.01), np.eye(3))
    mapper.capture(sample(), np.array([0, 0, 0, 1, 0, 0, 0], dtype=float))
    with pytest.raises(RuntimeError, match="硬限制"):
        mapper.target(sample((0.1, 0.0, 0.0)))


def test_unlimited_translation_and_step_map_full_scaled_delta():
    mapper = RelativePoseMapper(
        MappingConfig(
            translation_scale=2.0,
            max_translation_m=None,
            max_step_m=None,
        ),
        np.eye(3),
    )
    mapper.capture(sample(), np.array([0, 0, 0, 1, 0, 0, 0], dtype=float))
    target = mapper.target(sample((0.08, -0.04, 0.02)))
    assert np.allclose(target[:3], [0.16, -0.08, 0.04])


def test_reclutch_recenters_device_and_accumulates_from_measured_robot_pose():
    mapper = RelativePoseMapper(
        MappingConfig(
            translation_scale=2.0,
            max_translation_m=None,
            max_step_m=None,
            enable_rotation=True,
            max_rotation_rad=None,
            max_angular_step_rad=None,
        ),
        np.eye(3),
    )
    initial_robot = np.array([0, 0, 0, 1, 0, 0, 0], dtype=float)
    mapper.capture(sample(), initial_robot)
    first = mapper.target(sample((0.04, 0.0, 0.0)))
    np.testing.assert_allclose(first[:3], [0.08, 0.0, 0.0])

    # Simulate release after the robot reached only 60 mm, move the haptic
    # handle back while released, and engage again. The new command must start
    # exactly at the measured robot pose, without an old-target jump.
    measured_on_reclutch = np.array([0.06, 0, 0, 1, 0, 0, 0], dtype=float)
    recentered_device = sample((-0.03, 0.0, 0.0))
    mapper.capture(recentered_device, measured_on_reclutch)
    np.testing.assert_allclose(
        mapper.target(recentered_device), measured_on_reclutch, atol=1e-12
    )
    continued = mapper.target(sample((0.02, 0.0, 0.0)))
    np.testing.assert_allclose(continued[:3], [0.16, 0.0, 0.0])


def test_mapping_deadband_suppresses_feedback_micro_motion_continuously():
    mapper = RelativePoseMapper(
        MappingConfig(
            translation_scale=2.0,
            translation_deadband_m=0.0004,
            rotation_deadband_rad=math.radians(0.25),
            max_translation_m=None,
            max_step_m=None,
            enable_rotation=True,
            max_rotation_rad=None,
            max_angular_step_rad=None,
        ),
        np.eye(3),
    )
    mapper.capture(sample(), np.array([0, 0, 0, 1, 0, 0, 0], dtype=float))
    assert np.allclose(mapper.target(sample((0.0003, 0.0, 0.0)))[:3], np.zeros(3))
    assert np.allclose(
        mapper.target(sample((0.0005, 0.0, 0.0)))[:3],
        [0.0002, 0.0, 0.0],
    )


def test_rotation_mapping_and_angular_step_limit():
    mapper = RelativePoseMapper(
        MappingConfig(
            enable_rotation=True,
            max_rotation_rad=math.radians(20.0),
            max_angular_step_rad=math.radians(1.0),
        ),
        parse_axis_map("-x,-y,z"),
    )
    robot_pose = np.array([0, 0, 0, 1, 0, 0, 0], dtype=float)
    mapper.capture(sample(), robot_pose)
    angle = math.radians(10.0)
    rotation_z = np.array(
        [
            [math.cos(angle), -math.sin(angle), 0.0],
            [math.sin(angle), math.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    target = mapper.target(sample(rotation=rotation_z))
    assert math.degrees(quaternion_angular_distance(robot_pose[3:], target[3:])) == pytest.approx(1.0)


def test_rotation_axis_map_is_independent_from_translation_axis_map():
    translation_map = parse_axis_map("-x,-y,z")
    rotation_map = parse_axis_map("-x,-y,z")
    mapper = RelativePoseMapper(
        MappingConfig(
            translation_scale=1.0,
            enable_rotation=True,
            max_translation_m=None,
            max_step_m=None,
            max_rotation_rad=None,
            max_angular_step_rad=None,
        ),
        translation_map,
        rotation_axis_map=rotation_map,
    )
    robot_pose = np.array([0, 0, 0, 1, 0, 0, 0], dtype=float)
    mapper.capture(sample(), robot_pose)
    angle = math.radians(10.0)
    # Forward/back handle pitch is rotation about device +Y.
    rotation_device_y = np.array(
        [
            [math.cos(angle), 0.0, math.sin(angle)],
            [0.0, 1.0, 0.0],
            [-math.sin(angle), 0.0, math.cos(angle)],
        ]
    )
    target = mapper.target(
        sample(position=(0.01, 0.02, 0.03), rotation=rotation_device_y)
    )
    # Translation retains the original -X/-Y/+Z mapping.
    assert np.allclose(target[:3], [-0.01, -0.02, 0.03])
    # Device +Y pitch stays on robot Y instead of being swapped onto robot X.
    assert np.allclose(
        np.rad2deg(quaternion_to_rotation_vector(target[3:])),
        [0.0, -10.0, 0.0],
        atol=1e-8,
    )


def test_yaw_command_can_be_reversed_without_changing_pitch_axis():
    mapper = RelativePoseMapper(
        MappingConfig(
            enable_rotation=True,
            max_rotation_rad=None,
            max_angular_step_rad=None,
        ),
        parse_axis_map("-x,-y,z"),
        rotation_axis_map=parse_axis_map("-x,-y,z"),
        rotation_command_sign=np.array([1.0, 1.0, -1.0]),
    )
    robot_pose = np.array([0, 0, 0, 1, 0, 0, 0], dtype=float)
    mapper.capture(sample(), robot_pose)
    yaw_target = mapper.target(
        sample(rotation=rotation_vector_to_matrix(np.deg2rad([0.0, 0.0, 10.0])))
    )
    assert np.allclose(
        np.rad2deg(quaternion_to_rotation_vector(yaw_target[3:])),
        [0.0, 0.0, -10.0],
        atol=1e-8,
    )
    mapper.capture(sample(), robot_pose)
    opposite_yaw_target = mapper.target(
        sample(rotation=rotation_vector_to_matrix(np.deg2rad([0.0, 0.0, -10.0])))
    )
    assert np.allclose(
        np.rad2deg(quaternion_to_rotation_vector(opposite_yaw_target[3:])),
        [0.0, 0.0, 10.0],
        atol=1e-8,
    )


def test_rotation_mapping_uses_handle_local_frame_at_nonidentity_anchor():
    mapper = RelativePoseMapper(
        MappingConfig(
            enable_rotation=True,
            max_rotation_rad=None,
            max_angular_step_rad=None,
        ),
        np.eye(3),
        rotation_axis_map=np.eye(3),
        rotation_command_sign=np.ones(3),
    )
    device_anchor = rotation_vector_to_matrix(
        np.deg2rad([23.0, -17.0, 31.0])
    )
    robot_anchor_rotation = rotation_vector_to_matrix(
        np.deg2rad([-35.0, 12.0, 8.0])
    )
    robot_pose = np.r_[
        [0.4, -0.2, 0.5], matrix_to_quaternion(robot_anchor_rotation)
    ]
    mapper.capture(sample(rotation=device_anchor), robot_pose)

    # Apply a +12 degree rotation about the handle's own local Y axis. The
    # robot must receive +12 degrees about its captured local Y axis, with no
    # start-attitude-dependent X/Z cross-coupling.
    local_delta = rotation_vector_to_matrix(np.deg2rad([0.0, 12.0, 0.0]))
    target = mapper.target(sample(rotation=device_anchor @ local_delta))
    target_local_delta = (
        robot_anchor_rotation.T @ quaternion_to_matrix(target[3:])
    )
    np.testing.assert_allclose(
        np.rad2deg(
            quaternion_to_rotation_vector(
                matrix_to_quaternion(target_local_delta)
            )
        ),
        [0.0, 12.0, 0.0],
        atol=1e-8,
    )


def test_unlimited_rotation_still_has_angular_step_limit():
    mapper = RelativePoseMapper(
        MappingConfig(
            enable_rotation=True,
            max_rotation_rad=None,
            max_angular_step_rad=math.radians(0.5),
        ),
        parse_axis_map("-x,-y,z"),
    )
    robot_pose = np.array([0, 0, 0, 1, 0, 0, 0], dtype=float)
    mapper.capture(sample(), robot_pose)
    angle = math.radians(120.0)
    rotation_z = np.array(
        [
            [math.cos(angle), -math.sin(angle), 0.0],
            [math.sin(angle), math.cos(angle), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    target = mapper.target(sample(rotation=rotation_z))
    assert math.degrees(quaternion_angular_distance(robot_pose[3:], target[3:])) == pytest.approx(0.5)


def test_unlimited_translation_flag_is_explicit_and_valid():
    args = build_parser().parse_args(["--unlimited-translation"])
    validate_args(args)
    assert args.unlimited_translation is True
    assert args.max_translation > 0.0
    assert args.max_step > 0.0


def test_transparent_defaults_are_high_rate_two_to_one_and_raw_force():
    args = build_parser().parse_args([])
    assert args.scale == pytest.approx(2.0)
    assert args.device_rate == 1000
    assert args.gravity_compensation is True
    assert args.startup_center is False
    assert args.hold_when_released is False
    assert args.command_rate == 100
    assert args.force_feedback_gain == pytest.approx(1.0)
    assert args.force_feedback_deadband == pytest.approx(0.0)
    assert args.max_device_force == pytest.approx(12.0)
    assert args.max_force_feedback_bias == pytest.approx(5.0)
    assert args.force_feedback_source == "raw"
    assert args.force_slew_rate == pytest.approx(0.0)
    assert args.teleop_damping == pytest.approx(0.0)


def test_feedback_force_uses_inverse_axis_map_and_bias():
    force = map_feedback_force(
        np.array([5.0, -4.0, 3.0]),
        np.array([1.0, -2.0, 1.0]),
        parse_axis_map("-x,-y,z"),
        force_gain=0.5,
        force_deadband_n=0.0,
        max_device_force_n=10.0,
    )
    assert np.allclose(force, [-2.0, 1.0, 1.0])


def test_feedback_force_deadband_and_device_saturation():
    force = map_feedback_force(
        np.array([20.0, 0.0, 0.0]),
        np.zeros(3),
        np.eye(3),
        force_gain=1.0,
        force_deadband_n=1.0,
        max_device_force_n=3.0,
    )
    assert np.allclose(force, [3.0, 0.0, 0.0])


def test_progressive_feedback_does_not_abort_or_stop_at_four_newtons():
    force, excess = map_progressive_feedback_force(
        np.array([20.0, 0.0, 0.0]),
        np.zeros(3),
        np.eye(3),
        base_gain=0.25,
        force_deadband_n=0.0,
        overload_threshold_n=15.0,
        overload_gain=0.8,
        max_device_force_n=12.0,
    )
    assert excess == pytest.approx(5.0)
    assert np.allclose(force, [9.0, 0.0, 0.0])

    saturated, excess = map_progressive_feedback_force(
        np.array([100.0, 0.0, 0.0]),
        np.zeros(3),
        np.eye(3),
        base_gain=0.25,
        force_deadband_n=0.0,
        overload_threshold_n=15.0,
        overload_gain=0.8,
        max_device_force_n=12.0,
    )
    assert excess == pytest.approx(85.0)
    assert np.linalg.norm(saturated) == pytest.approx(12.0)


def test_overload_velocity_damping_opposes_motion_and_stays_capped():
    damped = add_velocity_damping(
        np.array([5.0, 0.0, 0.0]),
        np.array([0.2, 0.0, 0.0]),
        damping_n_per_m_s=20.0,
        activation=1.0,
        max_device_force_n=12.0,
    )
    assert np.allclose(damped, [1.0, 0.0, 0.0])


def test_parse_omega7_force_capability():
    capabilities = parse_device_capabilities(
        'CHAI3D_READY device=0 model="omega.7" rotation=1 '
        "force=1 torque=0 max_force_N=12 max_torque_Nm=0 rate_hz=250"
    )
    assert capabilities.actuated_force is True
    assert capabilities.actuated_torque is False
    assert capabilities.max_force_n == pytest.approx(12.0)


def test_force_feedback_requires_explicit_real_robot_arm():
    args = build_parser().parse_args(["--enable-force-feedback"])
    with pytest.raises(ValueError, match="只能与 --arm"):
        validate_args(args)

    armed = build_parser().parse_args(
        [
            "Rizon4s-062232",
            "--enable-force-feedback",
            "--arm",
            "--confirm",
            "MOVE_RIZON",
            "--confirm-force-feedback",
            "FORCE_1_TO_1",
        ]
    )
    validate_args(armed)


def test_full_force_confirmation_is_required():
    args = build_parser().parse_args(
        [
            "Rizon4s-062232",
            "--enable-force-feedback",
            "--arm",
            "--confirm",
            "MOVE_RIZON",
        ]
    )
    with pytest.raises(ValueError, match="FORCE_1_TO_1"):
        validate_args(args)
