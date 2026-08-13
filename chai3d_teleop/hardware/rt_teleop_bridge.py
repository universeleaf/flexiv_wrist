"""Binary shared-memory transport for the 1 kHz C++ teleoperation OSC.

Python owns slow I/O (omega.7, pedals and moteus USB).  The C++ process owns
Flexiv ``RT_JOINT_TORQUE``.  This module is deliberately transport-only so the
real-time controller never imports or calls Python.
"""

from __future__ import annotations

from dataclasses import dataclass
import mmap
import struct
import time

import numpy as np


TELEOP_SHARED_MAGIC = 0x54454C454F504F53
TELEOP_SHARED_SIZE = 360

MODE_HOLD = 0
MODE_7DOF_OSC = 1
MODE_9DOF_OSC = 2
MODE_ORIENTATION_FORCE_OSC = 3


@dataclass(frozen=True)
class RtTeleopState:
    received_at_s: float
    sequence: int
    controller_time_ns: int
    probe_pose: np.ndarray
    external_wrench_world: np.ndarray
    wrist_q_rad: np.ndarray
    mode: int
    command_fresh: bool
    enabled: bool
    force_control_active: bool
    fixed_point_recovery_active: bool
    probe_force_z_n: float
    position_error_m: float
    orientation_error_rad: float
    force_error_n: float
    arm_orientation_error_world: np.ndarray
    arm_singularity_sigma_min: float
    arm_motion_scale: float

    @property
    def age_ms(self) -> float:
        return max(0.0, (time.monotonic_ns() - self.controller_time_ns) / 1e6)


def initialize(mapping: mmap.mmap) -> None:
    if len(mapping) != TELEOP_SHARED_SIZE:
        raise ValueError(f"teleop shared memory 必须是 {TELEOP_SHARED_SIZE} bytes")
    mapping[:] = bytes(TELEOP_SHARED_SIZE)
    struct.pack_into("<Q", mapping, 0, TELEOP_SHARED_MAGIC)


def publish_command(
    mapping: mmap.mmap,
    sequence: int,
    *,
    mode: int,
    enabled: bool,
    target_pose,
    target_linear_velocity=None,
    target_angular_velocity=None,
    target_force_z_n: float = 0.0,
) -> None:
    pose = np.asarray(target_pose, dtype=float)
    linear = np.zeros(3) if target_linear_velocity is None else np.asarray(
        target_linear_velocity, dtype=float
    )
    angular = np.zeros(3) if target_angular_velocity is None else np.asarray(
        target_angular_velocity, dtype=float
    )
    if mode not in range(4):
        raise ValueError("unknown RT teleoperation mode")
    if pose.shape != (7,) or linear.shape != (3,) or angular.shape != (3,):
        raise ValueError("RT teleoperation pose/velocity dimensions are invalid")
    values = np.concatenate((pose, linear, angular, [target_force_z_n]))
    if not np.all(np.isfinite(values)):
        raise ValueError("RT teleoperation command contains non-finite values")
    odd = 2 * int(sequence) - 1
    even = odd + 1
    struct.pack_into("<Q", mapping, 8, odd)
    struct.pack_into("<QII", mapping, 16, time.monotonic_ns(), mode, int(enabled))
    struct.pack_into("<7d", mapping, 32, *pose)
    struct.pack_into("<3d", mapping, 88, *linear)
    struct.pack_into("<3d", mapping, 112, *angular)
    struct.pack_into("<d", mapping, 136, float(target_force_z_n))
    struct.pack_into("<Q", mapping, 8, even)


def read_state(mapping: mmap.mmap) -> RtTeleopState | None:
    for _ in range(3):
        first = struct.unpack_from("<Q", mapping, 144)[0]
        if first == 0 or first & 1:
            continue
        timestamp = struct.unpack_from("<Q", mapping, 152)[0]
        pose = np.asarray(struct.unpack_from("<7d", mapping, 160), dtype=float)
        wrench = np.asarray(struct.unpack_from("<6d", mapping, 216), dtype=float)
        wrist_q = np.asarray(struct.unpack_from("<2d", mapping, 264), dtype=float)
        mode, flags = struct.unpack_from("<II", mapping, 280)
        metrics = struct.unpack_from("<4d", mapping, 288)
        arm_metrics = struct.unpack_from("<5d", mapping, 320)
        second = struct.unpack_from("<Q", mapping, 144)[0]
        if first == second:
            return RtTeleopState(
                received_at_s=time.monotonic(),
                sequence=first,
                controller_time_ns=timestamp,
                probe_pose=pose,
                external_wrench_world=wrench,
                wrist_q_rad=wrist_q,
                mode=mode,
                command_fresh=bool(flags & 1),
                enabled=bool(flags & 2),
                force_control_active=bool(flags & 16),
                fixed_point_recovery_active=bool(flags & 32),
                probe_force_z_n=metrics[0],
                position_error_m=metrics[1],
                orientation_error_rad=metrics[2],
                force_error_n=metrics[3],
                arm_orientation_error_world=np.asarray(
                    arm_metrics[:3], dtype=float
                ),
                arm_singularity_sigma_min=arm_metrics[3],
                arm_motion_scale=arm_metrics[4],
            )
    return None
