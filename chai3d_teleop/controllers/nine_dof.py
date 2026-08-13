"""Pure control math for a Flexiv 7-DoF arm plus a serial 2-DoF wrist.

This module deliberately contains no hardware I/O.  The geometry and state
machine can therefore be tested before any command is sent to the robot or to
the moteus servos.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

import numpy as np

from .teleop import is_rotation_matrix, matrix_to_quaternion, quaternion_to_matrix


class TeleopMode(Enum):
    """The three mutually exclusive pedal-selected operating modes."""

    ARM_7DOF = "7dof"
    ARM_WRIST_9DOF = "9dof"
    PIVOT_ORIENTATION = "pivot_orientation"


def wrist_hold_axes(
    mode: TeleopMode | None, has_engaged_since_selection: bool
) -> tuple[bool, bool]:
    """Return which wrist axes may be position-held while clutch is released.

    Joint 9 is deliberately left in STOP until a freshly selected Mode 2/3
    has actually seen a clutch engagement.  This prevents a stale angle from
    being enabled as a position target merely because a pedal was pressed.
    """
    if mode is None:
        return False, False
    if mode is TeleopMode.ARM_7DOF or not has_engaged_since_selection:
        return True, False
    return True, True


def parse_runtime_mode_command(command: str) -> TeleopMode:
    """Parse the line protocol used by the local teleoperation control UI."""
    normalized = " ".join(command.strip().upper().split())
    modes = {
        "MODE 1": TeleopMode.ARM_7DOF,
        "MODE 2": TeleopMode.ARM_WRIST_9DOF,
        "MODE 3": TeleopMode.PIVOT_ORIENTATION,
    }
    try:
        return modes[normalized]
    except KeyError as error:
        raise ValueError("UI 命令必须是 MODE 1、MODE 2 或 MODE 3") from error


@dataclass(frozen=True)
class ModeTransition:
    selected_mode: TeleopMode | None
    ready: bool
    changed: bool
    reason: str


class PedalModeStateMachine:
    """Latch pedal selections without allowing a mid-clutch target jump.

    A pedal key-down selects a mode.  If the haptic clutch is already held,
    the new mode is selected but remains inhibited until the clutch is first
    released.  The next clutch press can then capture fresh arm, wrist and
    haptic anchors.
    """

    def __init__(self) -> None:
        self.selected_mode: TeleopMode | None = None
        self.ready = False
        self._waiting_for_release = False

    def select(self, mode: TeleopMode, *, clutch_pressed: bool) -> ModeTransition:
        changed = mode is not self.selected_mode
        self.selected_mode = mode
        if clutch_pressed:
            self.ready = False
            self._waiting_for_release = True
            reason = "mode_selected_waiting_for_clutch_release"
        else:
            self.ready = True
            self._waiting_for_release = False
            reason = "mode_ready"
        return ModeTransition(mode, self.ready, changed, reason)

    def observe_clutch(self, pressed: bool) -> ModeTransition:
        changed = False
        reason = "unchanged"
        if self._waiting_for_release and not pressed:
            self._waiting_for_release = False
            self.ready = self.selected_mode is not None
            changed = self.ready
            reason = "mode_ready_after_clutch_release"
        return ModeTransition(self.selected_mode, self.ready, changed, reason)

    def teleoperation_enabled(self, clutch_pressed: bool) -> bool:
        return self.selected_mode is not None and self.ready and clutch_pressed

    def clear(self) -> ModeTransition:
        """Return to a disabled, no-mode-selected state."""
        changed = self.selected_mode is not None or self.ready
        self.selected_mode = None
        self.ready = False
        self._waiting_for_release = False
        return ModeTransition(None, False, changed, "mode_cleared")


@dataclass(frozen=True)
class Mode3ForceGateResult:
    """One update of the Mode-3 contact gate and force ramp."""

    force_axis_enabled: bool
    commanded_force_n: float
    changed: bool
    reason: str


class Mode3ForceGate:
    """Gate and gently ramp the Mode-3 Tool-Z force command.

    The force-controlled axis is never enabled in free space.  Contact must
    first be measured in the configured target-force direction.  Once contact
    is present, the command starts at the measured force and ramps to the
    requested task force.  A short, configurable loss-of-contact delay avoids
    chattering on sensor noise; a genuine loss returns the axis to position
    control without terminating teleoperation.
    """

    def __init__(
        self,
        *,
        target_force_n: float,
        contact_enable_threshold_n: float,
        contact_release_threshold_n: float,
        contact_release_delay_s: float,
        force_ramp_s: float,
    ) -> None:
        values = (
            target_force_n,
            contact_enable_threshold_n,
            contact_release_threshold_n,
            contact_release_delay_s,
            force_ramp_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Mode3 force-gate settings must be finite")
        if target_force_n == 0.0:
            raise ValueError("target_force_n must be nonzero")
        if contact_enable_threshold_n <= 0.0:
            raise ValueError("contact_enable_threshold_n must be positive")
        if not 0.0 <= contact_release_threshold_n < contact_enable_threshold_n:
            raise ValueError("release threshold must be in [0, enable threshold)")
        if contact_release_delay_s <= 0.0 or force_ramp_s <= 0.0:
            raise ValueError("release delay and force ramp must be positive")
        self.target_force_n = float(target_force_n)
        self.contact_enable_threshold_n = float(contact_enable_threshold_n)
        self.contact_release_threshold_n = float(contact_release_threshold_n)
        self.contact_release_delay_s = float(contact_release_delay_s)
        self.force_ramp_s = float(force_ramp_s)
        self._direction = math.copysign(1.0, self.target_force_n)
        self.reset()

    def reset(self) -> None:
        self._active = False
        self._ramp_started_s: float | None = None
        self._ramp_start_magnitude_n = 0.0
        self._contact_lost_since_s: float | None = None

    def update(
        self, measured_force_n: float, *, teleoperation_enabled: bool, now_s: float
    ) -> Mode3ForceGateResult:
        if not math.isfinite(measured_force_n) or not math.isfinite(now_s):
            raise ValueError("Mode3 force-gate inputs must be finite")
        if not teleoperation_enabled:
            changed = self._active
            self.reset()
            return Mode3ForceGateResult(False, 0.0, changed, "teleoperation_disabled")

        aligned_magnitude = self._direction * measured_force_n
        if not self._active:
            if aligned_magnitude < self.contact_enable_threshold_n:
                return Mode3ForceGateResult(False, 0.0, False, "waiting_for_contact")
            self._active = True
            self._ramp_started_s = now_s
            self._ramp_start_magnitude_n = float(
                np.clip(aligned_magnitude, 0.0, abs(self.target_force_n))
            )
            self._contact_lost_since_s = None
            return Mode3ForceGateResult(
                True,
                self._direction * self._ramp_start_magnitude_n,
                True,
                "contact_acquired",
            )

        if aligned_magnitude < self.contact_release_threshold_n:
            if self._contact_lost_since_s is None:
                self._contact_lost_since_s = now_s
            elif now_s - self._contact_lost_since_s >= self.contact_release_delay_s:
                self.reset()
                return Mode3ForceGateResult(False, 0.0, True, "contact_lost")
        else:
            self._contact_lost_since_s = None

        assert self._ramp_started_s is not None
        alpha = float(np.clip((now_s - self._ramp_started_s) / self.force_ramp_s, 0.0, 1.0))
        magnitude = self._ramp_start_magnitude_n + alpha * (
            abs(self.target_force_n) - self._ramp_start_magnitude_n
        )
        return Mode3ForceGateResult(
            True, self._direction * magnitude, False, "contact_active"
        )


def _as_vector(values: np.ndarray, length: int, name: str) -> np.ndarray:
    result = np.asarray(values, dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"{name} must be a finite {length}-vector")
    return result


def _unit_axis(values: np.ndarray, name: str) -> np.ndarray:
    result = _as_vector(values, 3, name)
    norm = float(np.linalg.norm(result))
    if norm < 1e-9:
        raise ValueError(f"{name} must not be zero")
    return result / norm


def skew(vector: np.ndarray) -> np.ndarray:
    x, y, z = _as_vector(vector, 3, "vector")
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def axis_angle_rotation(axis: np.ndarray, angle_rad: float) -> np.ndarray:
    if not math.isfinite(angle_rad):
        raise ValueError("angle_rad must be finite")
    unit = _unit_axis(axis, "axis")
    cross = skew(unit)
    return np.eye(3) + math.sin(angle_rad) * cross + (1.0 - math.cos(angle_rad)) * (cross @ cross)


def rotation_vector(rotation: np.ndarray) -> np.ndarray:
    """Return the shortest logarithmic rotation vector for a 3x3 rotation."""
    matrix = np.asarray(rotation, dtype=float)
    if not is_rotation_matrix(matrix):
        raise ValueError("rotation must be a valid 3x3 rotation matrix")
    cosine = float(np.clip((np.trace(matrix) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1e-9:
        return np.array(
            [matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]]
        ) * 0.5
    if math.pi - angle < 1e-5:
        # The usual sine denominator is ill-conditioned near pi.  Extract an
        # eigenvector of eigenvalue 1 and choose a deterministic sign.
        values, vectors = np.linalg.eig(matrix)
        index = int(np.argmin(np.abs(values - 1.0)))
        axis = np.real(vectors[:, index])
        axis /= np.linalg.norm(axis)
        nonzero = np.flatnonzero(np.abs(axis) > 1e-8)
        if len(nonzero) and axis[nonzero[0]] < 0.0:
            axis = -axis
        return axis * angle
    axis = np.array(
        [matrix[2, 1] - matrix[1, 2], matrix[0, 2] - matrix[2, 0], matrix[1, 0] - matrix[0, 1]]
    ) / (2.0 * math.sin(angle))
    return axis * angle


def scale_rotation(rotation: np.ndarray, gain: float) -> np.ndarray:
    """Scale a relative rotation angle while preserving its axis."""
    if not math.isfinite(gain) or gain <= 0.0:
        raise ValueError("rotation gain must be finite and positive")
    vector = rotation_vector(rotation)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    return axis_angle_rotation(vector / angle, gain * angle)


def pose_to_transform(pose_xyzw: np.ndarray) -> np.ndarray:
    """Convert Flexiv ``[x,y,z,qw,qx,qy,qz]`` pose to a transform."""
    pose = _as_vector(pose_xyzw, 7, "pose")
    result = np.eye(4)
    result[:3, :3] = quaternion_to_matrix(pose[3:])
    result[:3, 3] = pose[:3]
    return result


def probe_force_from_world(
    force_world_n: np.ndarray,
    probe_rotation_world: np.ndarray,
    bias_world_n: np.ndarray | None = None,
) -> np.ndarray:
    """Rotate a Flexiv world-frame force into the live probe/TCP frame.

    A pure force vector is unaffected by translating its reference point, so
    only the live probe rotation (including q8/q9) is required.  A torque
    would additionally require the moment-arm cross product and is therefore
    deliberately not handled by this helper.
    """
    force = _as_vector(force_world_n, 3, "force_world_n")
    rotation = np.asarray(probe_rotation_world, dtype=float)
    if not is_rotation_matrix(rotation):
        raise ValueError("probe_rotation_world must be a rotation matrix")
    bias = (
        np.zeros(3)
        if bias_world_n is None
        else _as_vector(bias_world_n, 3, "bias_world_n")
    )
    return rotation.T @ (force - bias)


def transform_to_pose(transform: np.ndarray) -> np.ndarray:
    matrix = np.asarray(transform, dtype=float)
    if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
        raise ValueError("transform must be a finite 4x4 matrix")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-9):
        raise ValueError("invalid homogeneous transform bottom row")
    if not is_rotation_matrix(matrix[:3, :3]):
        raise ValueError("transform rotation is invalid")
    return np.concatenate((matrix[:3, 3], matrix_to_quaternion(matrix[:3, :3])))


@dataclass(frozen=True)
class WristGeometry:
    """Calibrated serial 2R geometry, expressed from the Flexiv flange.

    ``joint2_offset_after_joint1_m`` is expressed in the frame after joint 1;
    ``tip_offset_after_joint2_m`` and ``probe_rotation_at_zero`` are expressed
    in the frame after joint 2.  At q=[0,0] both rotating frames are aligned
    with the flange frame.
    """

    joint1_origin_flange_m: np.ndarray
    joint1_axis_flange: np.ndarray
    joint2_offset_after_joint1_m: np.ndarray
    joint2_axis_after_joint1: np.ndarray
    tip_offset_after_joint2_m: np.ndarray
    probe_rotation_at_zero: np.ndarray
    joint_min_rad: np.ndarray
    joint_max_rad: np.ndarray

    def __post_init__(self) -> None:
        object.__setattr__(self, "joint1_origin_flange_m", _as_vector(self.joint1_origin_flange_m, 3, "joint1_origin_flange_m"))
        object.__setattr__(self, "joint1_axis_flange", _unit_axis(self.joint1_axis_flange, "joint1_axis_flange"))
        object.__setattr__(self, "joint2_offset_after_joint1_m", _as_vector(self.joint2_offset_after_joint1_m, 3, "joint2_offset_after_joint1_m"))
        object.__setattr__(self, "joint2_axis_after_joint1", _unit_axis(self.joint2_axis_after_joint1, "joint2_axis_after_joint1"))
        object.__setattr__(self, "tip_offset_after_joint2_m", _as_vector(self.tip_offset_after_joint2_m, 3, "tip_offset_after_joint2_m"))
        rotation = np.asarray(self.probe_rotation_at_zero, dtype=float)
        if not is_rotation_matrix(rotation):
            raise ValueError("probe_rotation_at_zero must be a rotation matrix")
        object.__setattr__(self, "probe_rotation_at_zero", rotation.copy())
        lower = _as_vector(self.joint_min_rad, 2, "joint_min_rad")
        upper = _as_vector(self.joint_max_rad, 2, "joint_max_rad")
        if np.any(lower >= upper):
            raise ValueError("every wrist lower limit must be below its upper limit")
        object.__setattr__(self, "joint_min_rad", lower)
        object.__setattr__(self, "joint_max_rad", upper)

    def clamp(self, q_rad: np.ndarray, margin_rad: float = 0.0) -> np.ndarray:
        q = _as_vector(q_rad, 2, "q_rad")
        if margin_rad < 0.0:
            raise ValueError("margin_rad must be non-negative")
        lower = self.joint_min_rad + margin_rad
        upper = self.joint_max_rad - margin_rad
        if np.any(lower > upper):
            raise ValueError("joint-limit margin consumes the configured range")
        return np.clip(q, lower, upper)

    def forward(self, q_rad: np.ndarray) -> np.ndarray:
        q = _as_vector(q_rad, 2, "q_rad")
        r1 = axis_angle_rotation(self.joint1_axis_flange, float(q[0]))
        r2 = axis_angle_rotation(self.joint2_axis_after_joint1, float(q[1]))
        p_joint2 = self.joint1_origin_flange_m + r1 @ self.joint2_offset_after_joint1_m
        rotation = r1 @ r2
        result = np.eye(4)
        result[:3, :3] = rotation @ self.probe_rotation_at_zero
        result[:3, 3] = p_joint2 + rotation @ self.tip_offset_after_joint2_m
        return result

    def jacobian(self, q_rad: np.ndarray) -> np.ndarray:
        """Return flange-frame geometric tip Jacobian ``[linear; angular]``."""
        q = _as_vector(q_rad, 2, "q_rad")
        r1 = axis_angle_rotation(self.joint1_axis_flange, float(q[0]))
        r2 = axis_angle_rotation(self.joint2_axis_after_joint1, float(q[1]))
        p1 = self.joint1_origin_flange_m
        p2 = p1 + r1 @ self.joint2_offset_after_joint1_m
        tip = p2 + r1 @ r2 @ self.tip_offset_after_joint2_m
        axis1 = self.joint1_axis_flange
        axis2 = r1 @ self.joint2_axis_after_joint1
        angular = np.column_stack((axis1, axis2))
        linear = np.column_stack((np.cross(axis1, tip - p1), np.cross(axis2, tip - p2)))
        return np.vstack((linear, angular))


def flange_target_for_probe(
    probe_target_world: np.ndarray, geometry: WristGeometry, wrist_q_rad: np.ndarray
) -> np.ndarray:
    """Solve ``world_T_flange`` so the measured wrist reaches probe target."""
    target = np.asarray(probe_target_world, dtype=float)
    if target.shape == (7,):
        target = pose_to_transform(target)
    if target.shape != (4, 4):
        raise ValueError("probe_target_world must be a pose or 4x4 transform")
    return target @ np.linalg.inv(geometry.forward(wrist_q_rad))


def flange_target_for_probe_decoupled(
    probe_target_world: np.ndarray,
    geometry: WristGeometry,
    wrist_orientation_q_rad: np.ndarray,
    wrist_position_q_rad: np.ndarray,
) -> np.ndarray:
    """Keep live TCP position while assigning reachable rotation to q8/q9.

    The desired wrist angle determines only the residual flange orientation.
    The measured wrist angle determines the translation needed to cancel the
    live offset-tip arc. Thus a slow wrist cannot make the arm temporarily
    perform its reachable rotation, while TCP position remains exact.
    """
    target = np.asarray(probe_target_world, dtype=float)
    if target.shape == (7,):
        target = pose_to_transform(target)
    if target.shape != (4, 4) or not np.all(np.isfinite(target)):
        raise ValueError("probe_target_world must be a finite pose or 4x4 transform")
    wrist_orientation = geometry.forward(wrist_orientation_q_rad)
    wrist_position = geometry.forward(wrist_position_q_rad)
    result = np.eye(4)
    result[:3, :3] = target[:3, :3] @ wrist_orientation[:3, :3].T
    result[:3, 3] = target[:3, 3] - result[:3, :3] @ wrist_position[:3, 3]
    return result


def probe_pose_from_flange(
    flange_pose_world: np.ndarray, geometry: WristGeometry, wrist_q_rad: np.ndarray
) -> np.ndarray:
    flange = np.asarray(flange_pose_world, dtype=float)
    if flange.shape == (7,):
        flange = pose_to_transform(flange)
    if flange.shape != (4, 4):
        raise ValueError("flange_pose_world must be a pose or 4x4 transform")
    return flange @ geometry.forward(wrist_q_rad)


def orientation_only_target(
    mapped_probe_target_world: np.ndarray, position_reference_world_m: np.ndarray
) -> np.ndarray:
    """Keep mapped orientation only and take position from robot-side state."""
    target = np.asarray(mapped_probe_target_world, dtype=float)
    if target.shape == (7,):
        target = pose_to_transform(target)
    if target.shape != (4, 4) or not np.all(np.isfinite(target)):
        raise ValueError("mapped probe target must be a finite pose or 4x4 transform")
    if not is_rotation_matrix(target[:3, :3]):
        raise ValueError("mapped probe target rotation is invalid")
    position_reference = _as_vector(
        position_reference_world_m, 3, "position_reference_world_m"
    )
    result = target.copy()
    result[:3, 3] = position_reference
    return result


@dataclass(frozen=True)
class WristAllocationResult:
    q_target_rad: np.ndarray
    achieved_rotation: np.ndarray
    residual_rotvec_rad: np.ndarray
    iterations: int


class WristTargetShaper:
    """Critically damp and rate-limit a two-joint wrist target.

    The IK target can change much faster than a geared output joint can track,
    especially when the operator reverses a large rotation.  This stateful
    controller turns that geometric target into a realizable position command
    without changing the requested end-effector pose: the Flexiv arm continues
    to compensate the measured wrist pose in the outer task-space command.
    """

    def __init__(
        self,
        *,
        filter_hz: float,
        max_velocity_rad_s: np.ndarray,
        max_acceleration_rad_s2: np.ndarray,
        joint_min_rad: np.ndarray,
        joint_max_rad: np.ndarray,
    ) -> None:
        self.filter_hz = float(filter_hz)
        self.max_velocity_rad_s = _as_vector(
            max_velocity_rad_s, 2, "max_velocity_rad_s"
        )
        self.max_acceleration_rad_s2 = _as_vector(
            max_acceleration_rad_s2, 2, "max_acceleration_rad_s2"
        )
        self.joint_min_rad = _as_vector(joint_min_rad, 2, "joint_min_rad")
        self.joint_max_rad = _as_vector(joint_max_rad, 2, "joint_max_rad")
        if not math.isfinite(self.filter_hz) or self.filter_hz <= 0.0:
            raise ValueError("filter_hz must be finite and positive")
        if np.any(self.max_velocity_rad_s <= 0.0):
            raise ValueError("max_velocity_rad_s must be positive")
        if np.any(self.max_acceleration_rad_s2 <= 0.0):
            raise ValueError("max_acceleration_rad_s2 must be positive")
        if np.any(self.joint_min_rad >= self.joint_max_rad):
            raise ValueError("joint_min_rad must be below joint_max_rad")
        self.position_rad = np.zeros(2)
        self.velocity_rad_s = np.zeros(2)
        self.initialized = False

    def reset(self, measured_q_rad: np.ndarray) -> np.ndarray:
        q = _as_vector(measured_q_rad, 2, "measured_q_rad")
        self.position_rad = np.clip(q, self.joint_min_rad, self.joint_max_rad)
        self.velocity_rad_s.fill(0.0)
        self.initialized = True
        return self.position_rad.copy()

    def step(self, raw_target_rad: np.ndarray, dt_s: float) -> np.ndarray:
        target = _as_vector(raw_target_rad, 2, "raw_target_rad")
        dt = float(dt_s)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt_s must be finite and positive")
        target = np.clip(target, self.joint_min_rad, self.joint_max_rad)
        if not self.initialized:
            return self.reset(target)
        omega = 2.0 * math.pi * self.filter_hz
        acceleration = (
            omega * omega * (target - self.position_rad)
            - 2.0 * omega * self.velocity_rad_s
        )
        acceleration = np.clip(
            acceleration,
            -self.max_acceleration_rad_s2,
            self.max_acceleration_rad_s2,
        )
        self.velocity_rad_s = np.clip(
            self.velocity_rad_s + dt * acceleration,
            -self.max_velocity_rad_s,
            self.max_velocity_rad_s,
        )
        unclipped = self.position_rad + dt * self.velocity_rad_s
        self.position_rad = np.clip(
            unclipped, self.joint_min_rad, self.joint_max_rad
        )
        hit_limit = self.position_rad != unclipped
        self.velocity_rad_s[hit_limit] = 0.0
        return self.position_rad.copy()


def allocate_wrist_orientation(
    geometry: WristGeometry,
    q_seed_rad: np.ndarray,
    desired_wrist_rotation_flange: np.ndarray,
    *,
    damping: float = 0.03,
    max_step_rad: float = math.radians(4.0),
    joint_margin_rad: float = math.radians(2.0),
    max_iterations: int = 30,
) -> WristAllocationResult:
    """Use both wrist joints as much as possible for a desired orientation.

    This is a damped least-squares orientation IK.  Any unachievable residual
    (a 2-DoF wrist cannot span arbitrary 3-D orientation at one instant) is
    intentionally left for the 7-DoF Flexiv arm.
    """
    if damping <= 0.0 or max_step_rad <= 0.0 or max_iterations <= 0:
        raise ValueError("allocation damping, max step and iteration count must be positive")
    desired = np.asarray(desired_wrist_rotation_flange, dtype=float)
    if not is_rotation_matrix(desired):
        raise ValueError("desired_wrist_rotation_flange must be a rotation matrix")
    q = geometry.clamp(q_seed_rad, joint_margin_rad)
    iterations = 0
    for iterations in range(1, max_iterations + 1):
        current = geometry.forward(q)[:3, :3]
        error = rotation_vector(desired @ current.T)
        if np.linalg.norm(error) < 1e-6:
            break
        angular_jacobian = geometry.jacobian(q)[3:]
        normal = angular_jacobian @ angular_jacobian.T + damping * damping * np.eye(3)
        dq = angular_jacobian.T @ np.linalg.solve(normal, error)
        norm = float(np.linalg.norm(dq))
        if norm > max_step_rad:
            dq *= max_step_rad / norm
        next_q = geometry.clamp(q + dq, joint_margin_rad)
        if np.linalg.norm(next_q - q) < 1e-9:
            q = next_q
            break
        q = next_q
    achieved = geometry.forward(q)[:3, :3]
    residual = rotation_vector(desired @ achieved.T)
    return WristAllocationResult(q, achieved, residual, iterations)


@dataclass(frozen=True)
class WristOscResult:
    torque_nm: np.ndarray
    orientation_error_rad: np.ndarray
    angular_velocity_rad_s: np.ndarray


def wrist_gravity_compensation(
    geometry: WristGeometry,
    q_rad: np.ndarray,
    flange_rotation_world: np.ndarray,
    *,
    link1_mass_kg: float,
    link1_com_after_joint1_m: np.ndarray,
    link2_mass_kg: float,
    link2_com_after_joint2_m: np.ndarray,
    gravity_world_m_s2: np.ndarray = np.array([0.0, 0.0, -9.81]),
) -> np.ndarray:
    """Return joint torques that compensate gravity for the articulated wrist."""
    q = _as_vector(q_rad, 2, "q_rad")
    flange_rotation = np.asarray(flange_rotation_world, dtype=float)
    if not is_rotation_matrix(flange_rotation):
        raise ValueError("flange_rotation_world must be a rotation matrix")
    if not math.isfinite(link1_mass_kg) or not math.isfinite(link2_mass_kg):
        raise ValueError("link masses must be finite")
    if link1_mass_kg < 0.0 or link2_mass_kg < 0.0:
        raise ValueError("link masses cannot be negative")
    com1 = _as_vector(link1_com_after_joint1_m, 3, "link1_com_after_joint1_m")
    com2 = _as_vector(link2_com_after_joint2_m, 3, "link2_com_after_joint2_m")
    gravity_flange = flange_rotation.T @ _as_vector(
        gravity_world_m_s2, 3, "gravity_world_m_s2"
    )

    r1 = axis_angle_rotation(geometry.joint1_axis_flange, float(q[0]))
    r2 = axis_angle_rotation(geometry.joint2_axis_after_joint1, float(q[1]))
    p1 = geometry.joint1_origin_flange_m
    p2 = p1 + r1 @ geometry.joint2_offset_after_joint1_m
    axis1 = geometry.joint1_axis_flange
    axis2 = r1 @ geometry.joint2_axis_after_joint1

    p_com1 = p1 + r1 @ com1
    jv_com1 = np.column_stack((np.cross(axis1, p_com1 - p1), np.zeros(3)))
    p_com2 = p2 + r1 @ r2 @ com2
    jv_com2 = np.column_stack(
        (np.cross(axis1, p_com2 - p1), np.cross(axis2, p_com2 - p2))
    )
    generalized_gravity = (
        jv_com1.T @ (link1_mass_kg * gravity_flange)
        + jv_com2.T @ (link2_mass_kg * gravity_flange)
    )
    return -generalized_gravity


def operational_space_wrist_torque(
    geometry: WristGeometry,
    q_rad: np.ndarray,
    dq_rad_s: np.ndarray,
    target_wrist_rotation_flange: np.ndarray,
    *,
    rotational_stiffness_nm_per_rad: float,
    rotational_damping_nm_s_per_rad: float,
    gravity_torque_nm: np.ndarray | None = None,
    max_torque_nm: np.ndarray,
) -> WristOscResult:
    """Rank-aware rotational task-space impedance for the 2R wrist.

    ``e_R`` and ``omega`` are expressed in the flange frame.  The controller
    builds a physical task-space moment (N m), then projects it to the two
    wrist joints with ``Jw.T``.  This avoids scaling the command by an
    uncertain reflected motor/reducer inertia: the earlier acceleration-form
    controller could produce only a few millinewton-metres and fail to
    overcome the geared modules' static friction.  The arm controller must
    simultaneously compensate the measured wrist transform to preserve the
    commanded probe task.
    """
    q = _as_vector(q_rad, 2, "q_rad")
    dq = _as_vector(dq_rad_s, 2, "dq_rad_s")
    torque_limit = _as_vector(max_torque_nm, 2, "max_torque_nm")
    if np.any(torque_limit <= 0.0):
        raise ValueError("joint torque limits must be positive")
    if (
        not math.isfinite(rotational_stiffness_nm_per_rad)
        or rotational_stiffness_nm_per_rad <= 0.0
        or not math.isfinite(rotational_damping_nm_s_per_rad)
        or rotational_damping_nm_s_per_rad < 0.0
    ):
        raise ValueError("invalid operational-space gains")
    target = np.asarray(target_wrist_rotation_flange, dtype=float)
    if not is_rotation_matrix(target):
        raise ValueError("target wrist rotation must be valid")

    current = geometry.forward(q)[:3, :3]
    error = rotation_vector(target @ current.T)
    jacobian = geometry.jacobian(q)[3:]
    omega = jacobian @ dq
    task_moment = (
        rotational_stiffness_nm_per_rad * error
        - rotational_damping_nm_s_per_rad * omega
    )
    torque = jacobian.T @ task_moment
    if gravity_torque_nm is not None:
        torque += _as_vector(gravity_torque_nm, 2, "gravity_torque_nm")
    torque = np.clip(torque, -torque_limit, torque_limit)
    return WristOscResult(torque, error, omega)
