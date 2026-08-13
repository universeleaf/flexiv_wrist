"""Pure mapping and safety helpers for CHAI3D-to-Flexiv teleoperation."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class HapticSample:
    device_timestamp_ns: int
    position: np.ndarray
    rotation: np.ndarray
    switches: int

    def switch_pressed(self, index: int) -> bool:
        return bool(self.switches & (1 << index))


def parse_sample(line: str) -> HapticSample:
    fields = line.split()
    if len(fields) != 14:
        raise ValueError(f"CHAI3D 数据帧应有 14 列，实际为 {len(fields)}")
    timestamp_ns = int(fields[0])
    numbers = np.asarray([float(value) for value in fields[1:13]], dtype=float)
    position = numbers[:3]
    rotation = numbers[3:].reshape(3, 3)
    switches = int(fields[13])
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
        raise ValueError("CHAI3D 数据包含 NaN 或 Inf")
    if not is_rotation_matrix(rotation):
        raise ValueError("CHAI3D 姿态不是有效旋转矩阵")
    return HapticSample(timestamp_ns, position, rotation, switches)


def is_rotation_matrix(matrix: np.ndarray, tolerance: float = 2e-3) -> bool:
    matrix = np.asarray(matrix, dtype=float)
    return (
        matrix.shape == (3, 3)
        and np.allclose(matrix.T @ matrix, np.eye(3), atol=tolerance)
        and math.isclose(float(np.linalg.det(matrix)), 1.0, abs_tol=tolerance)
    )


def parse_axis_map(spec: str) -> np.ndarray:
    """Parse e.g. ``x,-z,y`` into a proper 3-D basis mapping."""
    tokens = [token.strip().lower() for token in spec.split(",")]
    if len(tokens) != 3:
        raise ValueError("--axis-map 必须有三个逗号分隔的轴，例如 x,-z,y")
    unit = {
        "x": np.array([1.0, 0.0, 0.0]),
        "y": np.array([0.0, 1.0, 0.0]),
        "z": np.array([0.0, 0.0, 1.0]),
    }
    columns = []
    used = set()
    for token in tokens:
        sign = -1.0 if token.startswith("-") else 1.0
        name = token[1:] if token[:1] in "+-" else token
        if name not in unit or name in used:
            raise ValueError("--axis-map 每个轴 x/y/z 必须恰好使用一次")
        used.add(name)
        columns.append(sign * unit[name])
    mapping = np.column_stack(columns)
    if not is_rotation_matrix(mapping):
        raise ValueError("--axis-map 必须是右手坐标系（行列式为 +1）")
    return mapping


def quaternion_to_matrix(quaternion_wxyz: np.ndarray) -> np.ndarray:
    q = np.asarray(quaternion_wxyz, dtype=float)
    if q.shape != (4,):
        raise ValueError("四元数必须有 4 个分量")
    norm = float(np.linalg.norm(q))
    if norm < 1e-12:
        raise ValueError("零四元数无效")
    w, x, y, z = q / norm
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def matrix_to_quaternion(matrix: np.ndarray) -> np.ndarray:
    """Return a normalized [w, x, y, z] quaternion."""
    matrix = np.asarray(matrix, dtype=float)
    if not is_rotation_matrix(matrix):
        raise ValueError("输入不是有效旋转矩阵")
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            w = (matrix[2, 1] - matrix[1, 2]) / scale
            x = 0.25 * scale
            y = (matrix[0, 1] + matrix[1, 0]) / scale
            z = (matrix[0, 2] + matrix[2, 0]) / scale
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            w = (matrix[0, 2] - matrix[2, 0]) / scale
            x = (matrix[0, 1] + matrix[1, 0]) / scale
            y = 0.25 * scale
            z = (matrix[1, 2] + matrix[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            w = (matrix[1, 0] - matrix[0, 1]) / scale
            x = (matrix[0, 2] + matrix[2, 0]) / scale
            y = (matrix[1, 2] + matrix[2, 1]) / scale
            z = 0.25 * scale
    quaternion = np.asarray([w, x, y, z], dtype=float)
    quaternion /= np.linalg.norm(quaternion)
    return quaternion if quaternion[0] >= 0.0 else -quaternion


def quaternion_angular_distance(first_wxyz: np.ndarray, second_wxyz: np.ndarray) -> float:
    """Return the shortest 3-D rotation angle between two orientations."""
    first = np.asarray(first_wxyz, dtype=float).copy()
    second = np.asarray(second_wxyz, dtype=float).copy()
    if np.linalg.norm(first) < 1e-12 or np.linalg.norm(second) < 1e-12:
        raise ValueError("零四元数无效")
    first /= np.linalg.norm(first)
    second /= np.linalg.norm(second)
    dot = abs(float(np.dot(first, second)))
    return 2.0 * math.acos(float(np.clip(dot, -1.0, 1.0)))


def quaternion_to_rotation_vector(quaternion_wxyz: np.ndarray) -> np.ndarray:
    """Convert [w, x, y, z] to the shortest axis-angle rotation vector."""
    quaternion = np.asarray(quaternion_wxyz, dtype=float).copy()
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError("零四元数无效")
    quaternion /= norm
    if quaternion[0] < 0.0:
        quaternion = -quaternion
    vector_norm = float(np.linalg.norm(quaternion[1:]))
    if vector_norm < 1e-12:
        return np.zeros(3)
    angle = 2.0 * math.atan2(vector_norm, float(quaternion[0]))
    return quaternion[1:] / vector_norm * angle


def rotation_vector_to_matrix(rotation_vector_rad: np.ndarray) -> np.ndarray:
    """Convert a finite 3-D rotation vector into a rotation matrix."""
    vector = np.asarray(rotation_vector_rad, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError("旋转向量必须是长度 3 的有限数组")
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    axis = vector / angle
    cross = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    return (
        np.eye(3)
        + math.sin(angle) * cross
        + (1.0 - math.cos(angle)) * (cross @ cross)
    )


def limit_quaternion_step(
    current_wxyz: np.ndarray, target_wxyz: np.ndarray, max_step_rad: float
) -> np.ndarray:
    """Slerp toward target without rotating more than ``max_step_rad``."""
    if max_step_rad <= 0.0:
        raise ValueError("max_step_rad 必须大于 0")
    current = np.asarray(current_wxyz, dtype=float).copy()
    target = np.asarray(target_wxyz, dtype=float).copy()
    current /= np.linalg.norm(current)
    target /= np.linalg.norm(target)
    dot = float(np.dot(current, target))
    if dot < 0.0:
        target = -target
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    angle = 2.0 * math.acos(dot)
    if angle <= max_step_rad:
        return target
    fraction = max_step_rad / angle
    if dot > 0.9995:
        result = current + fraction * (target - current)
    else:
        half_angle = math.acos(dot)
        sin_half_angle = math.sin(half_angle)
        result = (
            math.sin((1.0 - fraction) * half_angle) / sin_half_angle * current
            + math.sin(fraction * half_angle) / sin_half_angle * target
        )
    result /= np.linalg.norm(result)
    return result


def _radial_deadband(vector: np.ndarray, deadband: float) -> np.ndarray:
    """Remove a radial deadband without introducing a step at its edge."""
    values = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(values))
    if norm <= deadband:
        return np.zeros_like(values)
    return values * ((norm - deadband) / norm)


def _limit_vector_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
    values = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(values))
    if norm <= maximum:
        return values.copy()
    return values * (maximum / norm)


def map_feedback_force(
    force_world: np.ndarray,
    bias_world: np.ndarray,
    axis_map: np.ndarray,
    *,
    force_gain: float,
    force_deadband_n: float,
    max_device_force_n: float,
) -> np.ndarray:
    """Map a Flexiv world-frame external force into the haptic-device frame."""
    force = np.asarray(force_world, dtype=float)
    bias = np.asarray(bias_world, dtype=float)
    mapping = np.asarray(axis_map, dtype=float)
    if force.shape != (3,) or bias.shape != (3,):
        raise ValueError("Flexiv external force 和 bias 必须都是 3 维")
    if not np.all(np.isfinite(force)) or not np.all(np.isfinite(bias)):
        raise ValueError("Flexiv external force 包含 NaN 或 Inf")
    if not is_rotation_matrix(mapping):
        raise ValueError("axis_map 必须是 3x3 右手旋转矩阵")
    for name, value in (
        ("force_gain", force_gain),
        ("force_deadband_n", force_deadband_n),
    ):
        if value < 0.0:
            raise ValueError(f"{name} 不能为负数")
    if max_device_force_n <= 0.0:
        raise ValueError("触觉设备力输出上限必须大于 0")

    external_force = _radial_deadband(force - bias, force_deadband_n)

    # Position uses x_robot = axis_map * x_device, so the reciprocal wrench
    # mapping is the transpose. The Flexiv value is the external wrench applied
    # to the TCP, which gives the operator the corresponding reaction direction.
    force_device = force_gain * (mapping.T @ external_force)
    return _limit_vector_norm(force_device, max_device_force_n)


def map_progressive_feedback_force(
    force_world: np.ndarray,
    bias_world: np.ndarray,
    axis_map: np.ndarray,
    *,
    base_gain: float,
    force_deadband_n: float,
    overload_threshold_n: float,
    overload_gain: float,
    max_device_force_n: float,
) -> tuple[np.ndarray, float]:
    """Map normal feedback plus a progressive over-force reaction.

    Below ``overload_threshold_n`` this is the normal scaled force feedback.
    Above it, every additional robot-side newton adds ``overload_gain`` device
    newtons in the same reaction direction.  The device rating is a saturation,
    never a teleoperation abort condition.

    Returns ``(device_force, overload_excess_n)``.
    """
    force = np.asarray(force_world, dtype=float)
    bias = np.asarray(bias_world, dtype=float)
    mapping = np.asarray(axis_map, dtype=float)
    if force.shape != (3,) or bias.shape != (3,):
        raise ValueError("Flexiv external force 和 bias 必须都是 3 维")
    if not np.all(np.isfinite(force)) or not np.all(np.isfinite(bias)):
        raise ValueError("Flexiv external force 包含 NaN 或 Inf")
    if not is_rotation_matrix(mapping):
        raise ValueError("axis_map 必须是 3x3 右手旋转矩阵")
    if base_gain < 0.0 or force_deadband_n < 0.0 or overload_gain < 0.0:
        raise ValueError("反馈增益和死区不能为负数")
    if overload_threshold_n <= 0.0 or max_device_force_n <= 0.0:
        raise ValueError("过载阈值和设备力上限必须大于 0")

    unbiased_force = force - bias
    unbiased_magnitude = float(np.linalg.norm(unbiased_force))
    external_force = _radial_deadband(unbiased_force, force_deadband_n)
    overload_excess = max(0.0, unbiased_magnitude - overload_threshold_n)
    robot_reaction = base_gain * external_force
    if overload_excess > 0.0 and unbiased_magnitude > 1e-12:
        robot_reaction += (
            overload_gain * overload_excess * unbiased_force / unbiased_magnitude
        )
    device_force = mapping.T @ robot_reaction
    return _limit_vector_norm(device_force, max_device_force_n), overload_excess


def add_velocity_damping(
    force_device_n: np.ndarray,
    velocity_device_m_s: np.ndarray,
    *,
    damping_n_per_m_s: float,
    activation: float,
    max_device_force_n: float,
) -> np.ndarray:
    """Add dissipative device-frame damping and preserve the hardware cap."""
    force = np.asarray(force_device_n, dtype=float)
    velocity = np.asarray(velocity_device_m_s, dtype=float)
    if force.shape != (3,) or velocity.shape != (3,):
        raise ValueError("力和速度必须是 3 维")
    if not np.all(np.isfinite(force)) or not np.all(np.isfinite(velocity)):
        raise ValueError("力或速度包含 NaN/Inf")
    if damping_n_per_m_s < 0.0 or not 0.0 <= activation <= 1.0:
        raise ValueError("阻尼不能为负，activation 必须在 0..1")
    if max_device_force_n <= 0.0:
        raise ValueError("设备力上限必须大于 0")
    damped = force - activation * damping_n_per_m_s * velocity
    return _limit_vector_norm(damped, max_device_force_n)


@dataclass(frozen=True)
class StableFeedbackResult:
    """One update of the delayed-force stabilization layer."""

    force_device_n: np.ndarray
    requested_force_device_n: np.ndarray
    tank_energy_j: float
    passivity_limited: bool
    engagement: float


class StableHapticFeedback:
    """Low-pass, slew-limit and passivate a delayed bilateral force signal.

    Flexiv wrench samples arrive much slower than the omega.7 servo loop.  A
    raw sampled force can therefore drive the handle, move the robot target,
    and return a larger force one network delay later.  This class breaks that
    active loop in three independent ways:

    * a first-order wrench low-pass removes state-estimator/gear ripple;
    * an engagement ramp and vector slew limiter prevent force steps;
    * a time-domain energy tank removes only the force component that would
      inject more energy at the haptic port than the controller has absorbed.

    Local viscous damping is included before the passivity observer.  Since it
    always has non-positive power, it replenishes rather than consumes the
    tank.  The tank is deliberately bounded so old motion cannot accumulate an
    unlimited amount of releasable energy.
    """

    def __init__(
        self,
        *,
        update_rate_hz: float,
        lowpass_hz: float,
        force_slew_n_per_s: float,
        engagement_ramp_s: float,
        local_damping_n_per_m_s: float,
        max_device_force_n: float,
        initial_tank_energy_j: float,
        max_tank_energy_j: float,
        passivity_enabled: bool = True,
    ) -> None:
        positive = {
            "update_rate_hz": update_rate_hz,
            "lowpass_hz": lowpass_hz,
            "force_slew_n_per_s": force_slew_n_per_s,
            "engagement_ramp_s": engagement_ramp_s,
            "max_device_force_n": max_device_force_n,
            "max_tank_energy_j": max_tank_energy_j,
        }
        for name, value in positive.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and positive")
        if not math.isfinite(local_damping_n_per_m_s) or local_damping_n_per_m_s < 0.0:
            raise ValueError("local_damping_n_per_m_s must be finite and non-negative")
        if (
            not math.isfinite(initial_tank_energy_j)
            or initial_tank_energy_j < 0.0
            or initial_tank_energy_j > max_tank_energy_j
        ):
            raise ValueError("initial tank energy must be inside [0, maximum]")
        self.nominal_dt = 1.0 / update_rate_hz
        self.lowpass_hz = lowpass_hz
        self.force_slew_n_per_s = force_slew_n_per_s
        self.engagement_ramp_s = engagement_ramp_s
        self.local_damping_n_per_m_s = local_damping_n_per_m_s
        self.max_device_force_n = max_device_force_n
        self.initial_tank_energy_j = initial_tank_energy_j
        self.max_tank_energy_j = max_tank_energy_j
        self.passivity_enabled = bool(passivity_enabled)
        self.reset()

    def reset(self) -> None:
        self._filtered = np.zeros(3)
        self._output = np.zeros(3)
        self._engagement = 0.0
        self._tank_energy_j = self.initial_tank_energy_j
        self._initialized = False

    @property
    def tank_energy_j(self) -> float:
        return self._tank_energy_j

    def update(
        self,
        requested_force_device_n: np.ndarray,
        velocity_device_m_s: np.ndarray,
        *,
        dt_s: float | None = None,
    ) -> StableFeedbackResult:
        requested = np.asarray(requested_force_device_n, dtype=float)
        velocity = np.asarray(velocity_device_m_s, dtype=float)
        if requested.shape != (3,) or velocity.shape != (3,):
            raise ValueError("requested force and velocity must be 3-vectors")
        if not np.all(np.isfinite(requested)) or not np.all(np.isfinite(velocity)):
            raise ValueError("requested force or velocity contains NaN/Inf")
        dt = self.nominal_dt if dt_s is None else float(dt_s)
        if not math.isfinite(dt) or dt <= 0.0:
            raise ValueError("dt_s must be finite and positive")
        # A delayed UI/process wake-up must not authorize a proportionally huge
        # force jump or drain the complete tank in one sample.
        dt = min(dt, 5.0 * self.nominal_dt)

        alpha = 1.0 - math.exp(-2.0 * math.pi * self.lowpass_hz * dt)
        if not self._initialized:
            # Start from zero; copying the first wrench would bypass the force
            # engagement ramp at exactly the most dangerous transition.
            self._initialized = True
        self._filtered += alpha * (requested - self._filtered)
        self._engagement = min(1.0, self._engagement + dt / self.engagement_ramp_s)
        desired = self._engagement * self._filtered
        desired -= self.local_damping_n_per_m_s * velocity
        desired = _limit_vector_norm(desired, self.max_device_force_n)

        delta = desired - self._output
        delta = _limit_vector_norm(delta, self.force_slew_n_per_s * dt)
        candidate = _limit_vector_norm(
            self._output + delta, self.max_device_force_n
        )

        limited = False
        if self.passivity_enabled:
            power_w = float(np.dot(candidate, velocity))
            if power_w <= 0.0:
                self._tank_energy_j = min(
                    self.max_tank_energy_j,
                    self._tank_energy_j - power_w * dt,
                )
            else:
                allowed_power_w = self._tank_energy_j / dt
                if power_w > allowed_power_w:
                    velocity_norm_sq = float(np.dot(velocity, velocity))
                    if velocity_norm_sq > 1e-12:
                        candidate -= (
                            (power_w - allowed_power_w) / velocity_norm_sq
                        ) * velocity
                    else:
                        candidate.fill(0.0)
                    power_w = max(0.0, float(np.dot(candidate, velocity)))
                    limited = True
                self._tank_energy_j = max(
                    0.0, self._tank_energy_j - power_w * dt
                )

        self._output = _limit_vector_norm(candidate, self.max_device_force_n)
        return StableFeedbackResult(
            force_device_n=self._output.copy(),
            requested_force_device_n=desired.copy(),
            tank_energy_j=self._tank_energy_j,
            passivity_limited=limited,
            engagement=self._engagement,
        )


@dataclass(frozen=True)
class MappingConfig:
    translation_scale: float = 1.0
    translation_deadband_m: float = 0.0
    rotation_deadband_rad: float = 0.0
    # None disables the per-clutch displacement range check.
    max_translation_m: float | None = 0.05
    # None disables the application-level target step limiter.
    max_step_m: float | None = 0.001
    enable_rotation: bool = False
    # None disables the per-clutch angular range limit, but not angular rate limiting.
    max_rotation_rad: float | None = math.radians(20.0)
    max_angular_step_rad: float | None = math.radians(0.2)


class RelativePoseMapper:
    """Clutched relative mapping with hard translation/orientation limits."""

    def __init__(
        self,
        config: MappingConfig,
        axis_map: np.ndarray,
        *,
        rotation_axis_map: np.ndarray | None = None,
        rotation_command_sign: np.ndarray | None = None,
    ):
        self.config = config
        self.axis_map = np.asarray(axis_map, dtype=float)
        if not is_rotation_matrix(self.axis_map):
            raise ValueError("axis_map 必须是 3x3 右手旋转矩阵")
        self.rotation_axis_map = np.asarray(
            self.axis_map if rotation_axis_map is None else rotation_axis_map,
            dtype=float,
        )
        if not is_rotation_matrix(self.rotation_axis_map):
            raise ValueError("rotation_axis_map 必须是 3x3 右手旋转矩阵")
        self.rotation_command_sign = np.asarray(
            np.ones(3) if rotation_command_sign is None else rotation_command_sign,
            dtype=float,
        )
        if (
            self.rotation_command_sign.shape != (3,)
            or not np.all(np.isin(self.rotation_command_sign, [-1.0, 1.0]))
        ):
            raise ValueError("rotation_command_sign 必须是三个 +1/-1")
        if config.translation_scale <= 0.0:
            raise ValueError("translation_scale 必须大于 0")
        if config.translation_deadband_m < 0.0:
            raise ValueError("translation_deadband_m 不能为负")
        if config.rotation_deadband_rad < 0.0:
            raise ValueError("rotation_deadband_rad 不能为负")
        if config.max_translation_m is not None and config.max_translation_m <= 0.0:
            raise ValueError("max_translation_m 必须大于 0 或为 None")
        if config.max_step_m is not None and config.max_step_m <= 0.0:
            raise ValueError("max_step_m 必须大于 0 或为 None")
        if (
            config.max_angular_step_rad is not None
            and config.max_angular_step_rad <= 0.0
        ):
            raise ValueError("max_angular_step_rad 必须大于 0 或为 None")
        self._device_anchor: HapticSample | None = None
        self._robot_anchor: np.ndarray | None = None
        self._last_target: np.ndarray | None = None

    def capture(self, sample: HapticSample, robot_pose: np.ndarray) -> None:
        pose = np.asarray(robot_pose, dtype=float).copy()
        if pose.shape != (7,) or not np.all(np.isfinite(pose)):
            raise ValueError("Flexiv TCP 位姿必须是有限的 7 维数组")
        pose[3:] = matrix_to_quaternion(quaternion_to_matrix(pose[3:]))
        self._device_anchor = sample
        self._robot_anchor = pose
        self._last_target = pose.copy()

    def target(self, sample: HapticSample) -> np.ndarray:
        if self._device_anchor is None or self._robot_anchor is None:
            raise RuntimeError("必须先调用 capture()")

        delta_device = _radial_deadband(
            sample.position - self._device_anchor.position,
            self.config.translation_deadband_m,
        )
        delta_world = self.axis_map @ delta_device * self.config.translation_scale
        norm = float(np.linalg.norm(delta_world))
        if (
            self.config.max_translation_m is not None
            and norm > self.config.max_translation_m
        ):
            raise RuntimeError(
                f"遥操作位移 {norm:.4f} m 超过硬限制 {self.config.max_translation_m:.4f} m"
            )

        target = self._robot_anchor.copy()
        target[:3] += delta_world
        if self.config.enable_rotation:
            # Measure orientation in the handle frame captured on the clutch
            # edge.  The former space-frame delta (R * R0.T) mixed pitch,
            # roll and yaw whenever the omega.7 was engaged away from the
            # identity attitude.  Body-frame relative motion (R0.T * R) gives
            # the operator the same local forward/back and left/right axes at
            # every clutch re-capture.
            relative_device = self._device_anchor.rotation.T @ sample.rotation
            angle = math.acos(float(np.clip((np.trace(relative_device) - 1.0) / 2.0, -1.0, 1.0)))
            if (
                self.config.max_rotation_rad is not None
                and angle > self.config.max_rotation_rad
            ):
                raise RuntimeError(
                    f"遥操作转角 {math.degrees(angle):.1f}° 超过硬限制 "
                    f"{math.degrees(self.config.max_rotation_rad):.1f}°"
                )
            if angle <= self.config.rotation_deadband_rad:
                relative_device = np.eye(3)
            elif angle > 1e-12 and self.config.rotation_deadband_rad > 0.0:
                # Remove the deadband continuously: no angular command step at
                # its boundary and no change to the commanded rotation axis.
                relative_quaternion = matrix_to_quaternion(relative_device)
                vector = quaternion_to_rotation_vector(relative_quaternion)
                reduced = vector * ((angle - self.config.rotation_deadband_rad) / angle)
                reduced_angle = float(np.linalg.norm(reduced))
                if reduced_angle < 1e-12:
                    relative_device = np.eye(3)
                else:
                    axis = reduced / reduced_angle
                    cross = np.array(
                        [[0.0, -axis[2], axis[1]],
                         [axis[2], 0.0, -axis[0]],
                         [-axis[1], axis[0], 0.0]]
                    )
                    relative_device = (
                        np.eye(3)
                        + math.sin(reduced_angle) * cross
                        + (1.0 - math.cos(reduced_angle)) * (cross @ cross)
                    )
            mapped_relative = (
                self.rotation_axis_map
                @ relative_device
                @ self.rotation_axis_map.T
            )
            mapped_vector = quaternion_to_rotation_vector(
                matrix_to_quaternion(mapped_relative)
            )
            mapped_relative = rotation_vector_to_matrix(
                mapped_vector * self.rotation_command_sign
            )
            robot_rotation = quaternion_to_matrix(self._robot_anchor[3:])
            # The mapped delta is likewise local to the captured probe TCP.
            # Post-multiplication is the matching body-frame composition.
            target[3:] = matrix_to_quaternion(robot_rotation @ mapped_relative)

        if self._last_target is not None:
            step = target[:3] - self._last_target[:3]
            step_norm = float(np.linalg.norm(step))
            if (
                self.config.max_step_m is not None
                and step_norm > self.config.max_step_m
            ):
                target[:3] = self._last_target[:3] + step * (
                    self.config.max_step_m / step_norm
                )
            if (
                self.config.enable_rotation
                and self.config.max_angular_step_rad is not None
            ):
                target[3:] = limit_quaternion_step(
                    self._last_target[3:],
                    target[3:],
                    self.config.max_angular_step_rad,
                )
        self._last_target = target.copy()
        return target
