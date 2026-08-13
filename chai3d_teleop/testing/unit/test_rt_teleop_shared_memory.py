from __future__ import annotations

import mmap
import struct
import time
from types import SimpleNamespace

import numpy as np

from controllers.teleop import matrix_to_quaternion, rotation_vector_to_matrix

from hardware.rt_teleop_bridge import (
    MODE_9DOF_OSC,
    TELEOP_SHARED_MAGIC,
    TELEOP_SHARED_SIZE,
    initialize,
    publish_command,
    read_state,
)
from scripts.teleoperate_rt import _make_mapper, _orientation_only_target, _target_twist


def test_python_command_layout_matches_cpp_abi() -> None:
    with mmap.mmap(-1, TELEOP_SHARED_SIZE) as mapping:
        initialize(mapping)
        pose = np.asarray([0.1, 0.2, 0.3, 1.0, 0.0, 0.0, 0.0])
        publish_command(
            mapping,
            7,
            mode=MODE_9DOF_OSC,
            enabled=True,
            target_pose=pose,
            target_linear_velocity=[1.0, 2.0, 3.0],
            target_angular_velocity=[4.0, 5.0, 6.0],
            target_force_z_n=-15.0,
        )

        assert struct.unpack_from("<Q", mapping, 0)[0] == TELEOP_SHARED_MAGIC
        assert struct.unpack_from("<Q", mapping, 8)[0] == 14
        assert struct.unpack_from("<II", mapping, 24) == (MODE_9DOF_OSC, 1)
        np.testing.assert_allclose(struct.unpack_from("<7d", mapping, 32), pose)
        np.testing.assert_allclose(
            struct.unpack_from("<7d", mapping, 88), [1, 2, 3, 4, 5, 6, -15]
        )


def test_python_reads_coherent_cpp_state_layout() -> None:
    with mmap.mmap(-1, TELEOP_SHARED_SIZE) as mapping:
        initialize(mapping)
        struct.pack_into("<Q", mapping, 144, 1)
        struct.pack_into("<Q", mapping, 152, time.monotonic_ns())
        struct.pack_into("<7d", mapping, 160, 1, 2, 3, 1, 0, 0, 0)
        struct.pack_into("<6d", mapping, 216, 4, 5, 6, 7, 8, 9)
        struct.pack_into("<2d", mapping, 264, 0.1, -0.2)
        struct.pack_into("<II", mapping, 280, MODE_9DOF_OSC, 3)
        struct.pack_into("<4d", mapping, 288, -14.8, 0.001, 0.02, -0.2)
        struct.pack_into("<5d", mapping, 320, 0.01, -0.02, 0.03, 0.04, 0.7)
        struct.pack_into("<Q", mapping, 144, 2)

        state = read_state(mapping)

        assert state is not None
        assert state.mode == MODE_9DOF_OSC
        assert state.command_fresh and state.enabled
        assert not state.force_control_active
        assert not state.fixed_point_recovery_active
        np.testing.assert_allclose(state.external_wrench_world, [4, 5, 6, 7, 8, 9])
        assert state.probe_force_z_n == -14.8
        np.testing.assert_allclose(
            state.arm_orientation_error_world, [0.01, -0.02, 0.03]
        )
        assert state.arm_singularity_sigma_min == 0.04
        assert state.arm_motion_scale == 0.7


def test_python_decodes_force_status_flag() -> None:
    with mmap.mmap(-1, TELEOP_SHARED_SIZE) as mapping:
        initialize(mapping)
        struct.pack_into("<Q", mapping, 144, 1)
        struct.pack_into("<Q", mapping, 152, time.monotonic_ns())
        struct.pack_into("<7d", mapping, 160, 0, 0, 0, 1, 0, 0, 0)
        struct.pack_into("<6d", mapping, 216, 0, 0, 0, 0, 0, 0)
        struct.pack_into("<2d", mapping, 264, 0, 0)
        struct.pack_into("<II", mapping, 280, MODE_9DOF_OSC, 16 | 32)
        struct.pack_into("<4d", mapping, 288, 0, 0, 0, 0)
        struct.pack_into("<Q", mapping, 144, 2)

        state = read_state(mapping)

        assert state is not None
        assert state.force_control_active
        assert state.fixed_point_recovery_active
        assert not state.command_fresh
        assert not state.enabled


def test_target_twist_supplies_motion_feedforward_and_zero_when_stationary() -> None:
    previous = np.asarray([0.1, -0.2, 0.3, 1.0, 0.0, 0.0, 0.0])
    target = previous.copy()
    target[:3] += [0.002, -0.004, 0.006]
    target[3:] = matrix_to_quaternion(
        rotation_vector_to_matrix(np.asarray([0.01, -0.02, 0.03]))
    )

    linear, angular = _target_twist(previous, target, 0.02)

    np.testing.assert_allclose(linear, [0.1, -0.2, 0.3], atol=1e-12)
    np.testing.assert_allclose(angular, [0.5, -1.0, 1.5], atol=1e-10)
    stopped_linear, stopped_angular = _target_twist(target, target, 0.02)
    np.testing.assert_allclose(stopped_linear, np.zeros(3), atol=1e-12)
    np.testing.assert_allclose(stopped_angular, np.zeros(3), atol=1e-12)


def test_rt_mapper_keeps_last_github_cycle_limits() -> None:
    profile = SimpleNamespace(
        robot={
            "command_rate_hz": 100.0,
            "max_linear_velocity_m_s": 0.5,
            "max_angular_velocity_rad_s": 1.0,
        },
        haptic={
            "translation_scale": 2.0,
            "translation_deadband_m": 0.0004,
            "rotation_deadband_deg": 0.25,
            "axis_map": "-x,-y,z",
            "rotation_axis_map": "-x,-y,z",
            "rotation_command_sign": [1.0, 1.0, 1.0],
        },
    )

    mapper = _make_mapper(profile)

    assert mapper.config.max_translation_m is None
    assert mapper.config.max_rotation_rad is None
    assert mapper.config.max_step_m == 0.005
    assert mapper.config.max_angular_step_rad == 0.01


def test_mode3_orientation_target_never_moves_captured_contact_point() -> None:
    mapped = np.asarray([0.8, -0.7, 0.6, 0.5, 0.5, -0.5, 0.5])
    anchor = np.asarray([0.31, -0.22, 0.47])

    target = _orientation_only_target(mapped, anchor)

    np.testing.assert_allclose(target[:3], anchor)
    np.testing.assert_allclose(target[3:], mapped[3:])
    # The helper must not mutate the mapper's pose buffer.
    np.testing.assert_allclose(mapped[:3], [0.8, -0.7, 0.6])
