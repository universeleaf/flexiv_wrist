#!/usr/bin/env python3
"""Pedal-selected 7/9-DoF and orientation/force teleoperation coordinator."""

from __future__ import annotations

import argparse
from collections import deque
from contextlib import ExitStack
from dataclasses import dataclass
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
import tomllib
from typing import Any, Sequence


SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parents[1]
FREEDRIVE_ROOT = PROJECT_ROOT.parent / "freedrive_python"
FREEDRIVE_VENV = FREEDRIVE_ROOT / ".venv"
MOTEUS_VENV = PROJECT_ROOT / ".venv_moteus"
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "nine_dof_teleop.toml"


def ensure_project_python() -> None:
    expected = FREEDRIVE_VENV.resolve()
    if Path(sys.prefix).resolve() == expected:
        return
    executable = FREEDRIVE_VENV / "bin" / "python"
    if not executable.is_file():
        raise FileNotFoundError(f"找不到 Flexiv Python 环境: {executable}")
    os.execv(str(executable), [str(executable), str(SCRIPT_PATH), *sys.argv[1:]])


ensure_project_python()

import numpy as np  # noqa: E402

sys.path.insert(0, str(PROJECT_ROOT))
sys.path.insert(0, str(FREEDRIVE_ROOT))

from hardware.haptic_bridge import BridgeReader, ensure_robot_ready, wait_for_first_sample  # noqa: E402
from controllers.nine_dof import (  # noqa: E402
    PedalModeStateMachine,
    TeleopMode,
    WristGeometry,
    WristTargetShaper,
    allocate_wrist_orientation,
    flange_target_for_probe,
    operational_space_wrist_torque,
    orientation_only_target,
    parse_runtime_mode_command,
    pose_to_transform,
    probe_force_from_world,
    probe_pose_from_flange,
    rotation_vector,
    scale_rotation,
    transform_to_pose,
    wrist_hold_axes,
    wrist_gravity_compensation,
)
from controllers.teleop import (  # noqa: E402
    MappingConfig,
    RelativePoseMapper,
    StableHapticFeedback,
    map_progressive_feedback_force,
    parse_axis_map,
)
from src.foot_pedal_configuration import load_foot_pedal_configuration  # noqa: E402
from src.foot_pedal_input import FootPedalReader, XInputFootPedalReader  # noqa: E402


MODE_BY_ACTION = {
    "teleop_7dof": TeleopMode.ARM_7DOF,
    "teleop_9dof": TeleopMode.ARM_WRIST_9DOF,
    "teleop_pivot_orientation": TeleopMode.PIVOT_ORIENTATION,
}


def _section(document: dict[str, Any], name: str) -> dict[str, Any]:
    value = document.get(name)
    if not isinstance(value, dict):
        raise ValueError(f"配置缺少 [{name}] section")
    return value


def _vector(section: dict[str, Any], key: str, length: int) -> np.ndarray:
    result = np.asarray(section.get(key), dtype=float)
    if result.shape != (length,) or not np.all(np.isfinite(result)):
        raise ValueError(f"配置 {key} 必须是长度 {length} 的有限数组")
    return result


def _positive(section: dict[str, Any], key: str) -> float:
    value = float(section.get(key, math.nan))
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"配置 {key} 必须是有限正数")
    return value


@dataclass(frozen=True)
class Profile:
    path: Path
    document: dict[str, Any]
    geometry: WristGeometry
    pedal_config_path: Path

    @property
    def robot(self) -> dict[str, Any]:
        return _section(self.document, "robot")

    @property
    def haptic(self) -> dict[str, Any]:
        return _section(self.document, "haptic")

    @property
    def wrist(self) -> dict[str, Any]:
        return _section(self.document, "wrist")

    @property
    def payload(self) -> dict[str, Any]:
        return _section(self.document, "wrist_payload")

    @property
    def flexiv_tool(self) -> dict[str, Any]:
        return _section(self.document, "flexiv_tool")

    @property
    def allocation(self) -> dict[str, Any]:
        return _section(self.document, "allocation")

    @property
    def osc_controller(self) -> dict[str, Any]:
        return _section(self.document, "osc_controller")

    @property
    def demo(self) -> dict[str, Any]:
        return _section(self.document, "demo")

    @property
    def osc(self) -> dict[str, Any]:
        return _section(self.document, "pivot_orientation_osc")

    @property
    def force_feedback(self) -> dict[str, Any]:
        return _section(self.document, "force_feedback")

    @property
    def runtime(self) -> dict[str, Any]:
        return _section(self.document, "runtime")


def load_profile(path: Path) -> Profile:
    with path.open("rb") as stream:
        document = tomllib.load(stream)
    wrist = _section(document, "wrist")
    geometry_section = _section(document, "wrist_geometry")
    limits = np.deg2rad(_vector(wrist, "joint_limit_deg", 2))
    geometry = WristGeometry(
        joint1_origin_flange_m=_vector(geometry_section, "joint1_origin_flange_m", 3),
        joint1_axis_flange=_vector(geometry_section, "joint1_axis_flange", 3),
        joint2_offset_after_joint1_m=_vector(geometry_section, "joint2_offset_after_joint1_m", 3),
        joint2_axis_after_joint1=_vector(geometry_section, "joint2_axis_after_joint1", 3),
        tip_offset_after_joint2_m=_vector(geometry_section, "tip_offset_after_joint2_m", 3),
        probe_rotation_at_zero=_vector(
            geometry_section, "probe_rotation_at_zero_row_major", 9
        ).reshape(3, 3),
        joint_min_rad=-limits,
        joint_max_rad=limits,
    )
    pedal_section = _section(document, "pedal")
    pedal_config_path = Path(str(pedal_section.get("config_path", "")))
    if not pedal_config_path.is_absolute():
        pedal_config_path = path.parent / pedal_config_path
    pedal_config = load_foot_pedal_configuration(pedal_config_path)
    for pedal_id, expected_action in (
        ("pedal_1", "teleop_7dof"),
        ("pedal_2", "teleop_9dof"),
        ("pedal_3", "teleop_pivot_orientation"),
    ):
        binding = pedal_config.pedal(pedal_id)
        if binding.action != expected_action:
            raise ValueError(f"{pedal_id} action 必须是 {expected_action}")
    _vector(wrist, "zero_position_rev", 2)
    zero_hold_mask = wrist.get("zero_hold_mask")
    if (
        not isinstance(zero_hold_mask, list)
        or len(zero_hold_mask) != 2
        or not all(isinstance(value, bool) for value in zero_hold_mask)
    ):
        raise ValueError("wrist.zero_hold_mask 必须是两个布尔值")
    signs = _vector(wrist, "motor_sign", 2)
    if not np.all(np.isin(signs, [-1.0, 1.0])):
        raise ValueError("motor_sign 只能是 -1 或 +1")
    _vector(wrist, "position_torque_limit_nm", 2)
    _vector(wrist, "torque_limit_nm", 2)
    for key in (
        "position_kp_scale",
        "position_kd_scale",
        "zero_position_kp_scale",
        "zero_position_kd_scale",
    ):
        scale = _vector(wrist, key, 2)
        if np.any(scale <= 0.0) or np.any(scale > 1.0):
            raise ValueError(f"wrist.{key} 必须是两个 (0, 1] 内的值")
    reduction_ratio = _vector(wrist, "reduction_ratio", 2)
    if np.any(reduction_ratio <= 1.0):
        raise ValueError("reduction_ratio 必须是大于 1 的两轴减速比")
    for key in (
        "loop_rate_hz",
        "command_timeout_s",
        "watchdog_ms",
        "rt_osc_state_timeout_ms",
        "rt_osc_hard_timeout_ms",
        "zero_velocity_deg_s",
        "zero_acceleration_deg_s2",
        "zero_tolerance_deg",
        "zero_settle_seconds",
        "zero_timeout_s",
        "runtime_velocity_deg_s",
        "runtime_acceleration_deg_s2",
        "position_following_error_deg",
        "position_following_error_timeout_s",
    ):
        _positive(wrist, key)
    allocation = _section(document, "allocation")
    for key in (
        "damping",
        "wrist_priority_gain",
        "osc_wrist_inertia_weight",
        "max_iteration_step_deg",
        "joint_limit_margin_deg",
    ):
        _positive(allocation, key)
    if not math.isclose(
        float(allocation["wrist_priority_gain"]), 1.0, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ValueError(
            "allocation.wrist_priority_gain 必须为 1.0；自然腕部分配不放大姿态"
        )
    for key in ("wrist_tracking_kp_nm_per_rad", "wrist_tracking_kd_nm_s_per_rad"):
        values = _vector(allocation, key, 2)
        if np.any(values <= 0.0):
            raise ValueError(f"allocation.{key} 必须包含两个有限正数")
    for key in (
        "wrist_target_max_velocity_deg_s",
        "wrist_target_max_acceleration_deg_s2",
    ):
        values = _vector(allocation, key, 2)
        if np.any(values <= 0.0):
            raise ValueError(f"allocation.{key} 必须包含两个有限正数")
    if np.any(
        _vector(allocation, "wrist_target_max_velocity_deg_s", 2)
        > float(wrist["runtime_velocity_deg_s"])
    ):
        raise ValueError(
            "allocation.wrist_target_max_velocity_deg_s 不能超过腕部运行速度"
        )
    if np.any(
        _vector(allocation, "wrist_target_max_acceleration_deg_s2", 2)
        > float(wrist["runtime_acceleration_deg_s2"])
    ):
        raise ValueError(
            "allocation.wrist_target_max_acceleration_deg_s2 不能超过腕部运行加速度"
        )
    for key in (
        "wrist_target_filter_hz",
        "wrist_state_velocity_filter_hz",
        "torque_startup_ramp_s",
        "stale_recovery_ramp_s",
    ):
        _positive(allocation, key)
    wrist_rectangle_excursion = _vector(
        allocation, "wrist_rectangle_excursion_deg", 2
    )
    if np.any(wrist_rectangle_excursion <= 0.0):
        raise ValueError("allocation.wrist_rectangle_excursion_deg 必须包含两个正数")
    if np.any(wrist_rectangle_excursion >= np.rad2deg(limits)):
        raise ValueError("腕部矩形演示行程必须小于对应 joint_limit_deg")
    max_iterations = allocation.get("max_iterations")
    if not isinstance(max_iterations, int) or not 1 <= max_iterations <= 100:
        raise ValueError("allocation.max_iterations 必须是 1..100 的整数")
    tool = _section(document, "flexiv_tool")
    if not isinstance(tool.get("calibration_ready"), bool):
        raise ValueError("flexiv_tool.calibration_ready 必须是布尔值")
    _vector(tool, "expected_tcp_location", 7)
    _vector(tool, "expected_com_m", 3)
    _vector(tool, "expected_inertia_kg_m2", 6)
    if not str(tool.get("expected_name", "")).strip():
        raise ValueError("flexiv_tool.expected_name 不能为空")
    for key in (
        "expected_mass_kg",
        "tcp_position_tolerance_m",
        "tcp_rotation_tolerance_deg",
        "mass_tolerance_kg",
        "com_tolerance_m",
        "inertia_tolerance_kg_m2",
    ):
        _positive(tool, key)
    force_feedback = _section(document, "force_feedback")
    for key in (
        "base_gain",
        "deadband_n",
        "overload_gain",
        "overload_damping_n_per_m_s",
    ):
        value = float(force_feedback.get(key, math.nan))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"配置 {key} 必须是有限非负数")
    for key in (
        "overload_threshold_n",
        "overload_full_damping_excess_n",
        "max_device_force_n",
        "wrench_lowpass_hz",
        "force_slew_n_per_s",
        "engagement_ramp_s",
        "passivity_max_energy_j",
    ):
        _positive(force_feedback, key)
    for key in (
        "local_damping_n_per_m_s",
        "passivity_initial_energy_j",
    ):
        value = float(force_feedback.get(key, math.nan))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"配置 {key} 必须是有限非负数")
    if not isinstance(force_feedback.get("passivity_enabled"), bool):
        raise ValueError("force_feedback.passivity_enabled 必须是布尔值")
    if float(force_feedback["passivity_initial_energy_j"]) > float(
        force_feedback["passivity_max_energy_j"]
    ):
        raise ValueError("passivity_initial_energy_j 不能超过最大能量")
    if str(force_feedback.get("source")) not in {"raw", "filtered"}:
        raise ValueError("force_feedback.source 只能是 raw 或 filtered")
    osc = _section(document, "pivot_orientation_osc")
    for key in (
        "rotational_stiffness_nm_per_rad",
        "wrist_priority_gain",
        "force_axis_max_velocity_m_s",
        "force_frame_update_hz",
        "force_frame_update_angle_deg",
    ):
        _positive(osc, key)
    rotational_damping = float(
        osc.get("rotational_damping_nm_s_per_rad", math.nan)
    )
    if not math.isfinite(rotational_damping) or rotational_damping < 0.0:
        raise ValueError("rotational_damping_nm_s_per_rad 不能为负数")
    target_force = float(osc.get("target_sensed_force_tool_z_n", math.nan))
    if not math.isfinite(target_force):
        raise ValueError("模式 3 target_sensed_force_tool_z_n 必须是有限数值")
    if str(osc.get("force_source")) not in {"raw", "filtered"}:
        raise ValueError("pivot_orientation_osc.force_source 只能是 raw 或 filtered")
    _positive(osc, "force_display_lowpass_hz")
    payload = _section(document, "wrist_payload")
    if not str(payload.get("assembly_id", "")).strip():
        raise ValueError("wrist_payload.assembly_id 不能为空")
    _vector(payload, "link1_inertia_com_kg_m2_row_major", 9)
    _vector(payload, "link2_and_probe_inertia_com_kg_m2_row_major", 9)
    for key in (
        "reflected_joint_inertia_kg_m2",
        "viscous_friction_nm_s_rad",
        "coulomb_friction_nm",
        "torque_bias_nm",
    ):
        _vector(payload, key, 2)
    robot = _section(document, "robot")
    if not str(robot.get("robot_sn", "")).strip():
        raise ValueError("robot_sn 不能为空")
    parse_axis_map(str(_section(document, "haptic").get("axis_map", "")))
    haptic = _section(document, "haptic")
    for key in ("translation_deadband_m", "rotation_deadband_deg"):
        value = float(haptic.get(key, math.nan))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"haptic.{key} 必须是有限非负数")
    motor_pid = _section(document, "motor_pid")
    for key in (
        "joint8_kp", "joint8_kd", "joint9_kp", "joint9_kd"
    ):
        _positive(motor_pid, key)
    for key in ("joint8_ki", "joint9_ki"):
        value = float(motor_pid.get(key, math.nan))
        if not math.isfinite(value) or value < 0.0:
            raise ValueError(f"motor_pid.{key} 必须是有限非负数")
    demo = _section(document, "demo")
    if str(demo.get("trajectory")) not in {
        "loop",
        "orientation",
        "spin",
        "rectangle",
    }:
        raise ValueError(
            "demo.trajectory 必须是 loop/orientation/spin/rectangle"
        )
    for key in ("period_s", "radius_m", "orientation_amplitude_deg"):
        _positive(demo, key)
    if not isinstance(demo.get("cpu_affinity"), int):
        raise ValueError("demo.cpu_affinity 必须是整数")
    osc_controller = _section(document, "osc_controller")
    for key in (
        "translation_kp", "translation_kd", "rotation_kp", "rotation_kd",
        "nullspace_kp", "nullspace_kd",
    ):
        _positive(osc_controller, key)
    for key, length in (
        ("arm_torque_limit_nm", 7),
        ("arm_torque_slew_nm_s", 7),
        ("wrist_torque_slew_nm_s", 2),
    ):
        values = _vector(osc_controller, key, length)
        if np.any(values <= 0.0):
            raise ValueError(f"osc_controller.{key} 必须全部为正数")
    return Profile(path.resolve(), document, geometry, pedal_config_path.resolve())


@dataclass(frozen=True)
class WristSample:
    received_at_s: float
    timestamp_ns: int
    q_rad: np.ndarray
    dq_rad_s: np.ndarray
    torque_nm: np.ndarray
    mode: str
    servo_mode: np.ndarray
    fault: np.ndarray


class WristBridge:
    def __init__(
        self,
        profile: Profile,
        *,
        zero_hold_mask_override: Sequence[bool] | None = None,
        position_following_error_deg_override: float | None = None,
        position_kp_scale_override: Sequence[float] | None = None,
        position_kd_scale_override: Sequence[float] | None = None,
        watchdog_ms_override: float | None = None,
        loop_rate_hz_override: float | None = None,
    ):
        wrist = profile.wrist
        executable = MOTEUS_VENV / "bin" / "python"
        bridge = PROJECT_ROOT / "hardware" / "wrist_moteus_bridge.py"
        if not executable.is_file():
            raise FileNotFoundError(f"找不到 moteus Python 环境: {executable}")
        ids = [int(value) for value in wrist["ids"]]
        zero_hold_mask = (
            list(zero_hold_mask_override)
            if zero_hold_mask_override is not None
            else list(wrist["zero_hold_mask"])
        )
        if len(zero_hold_mask) != 2:
            raise ValueError("zero_hold_mask_override 必须包含两个值")
        following_error_deg = (
            float(position_following_error_deg_override)
            if position_following_error_deg_override is not None
            else float(wrist["position_following_error_deg"])
        )
        if not math.isfinite(following_error_deg) or following_error_deg <= 0.0:
            raise ValueError("position following error 必须是有限正数")
        position_kp_scale = (
            list(position_kp_scale_override)
            if position_kp_scale_override is not None
            else list(wrist["position_kp_scale"])
        )
        position_kd_scale = (
            list(position_kd_scale_override)
            if position_kd_scale_override is not None
            else list(wrist["position_kd_scale"])
        )
        for name, values in (
            ("position_kp_scale_override", position_kp_scale),
            ("position_kd_scale_override", position_kd_scale),
        ):
            if len(values) != 2 or not all(
                math.isfinite(float(value)) and 0.0 < float(value) <= 1.0
                for value in values
            ):
                raise ValueError(f"{name} 必须包含两个 (0, 1] 内的有限数")
        watchdog_ms = (
            float(watchdog_ms_override)
            if watchdog_ms_override is not None
            else float(wrist["watchdog_ms"])
        )
        if not math.isfinite(watchdog_ms) or watchdog_ms <= 0.0:
            raise ValueError("wrist watchdog 必须是有限正数")
        loop_rate_hz = (
            float(loop_rate_hz_override)
            if loop_rate_hz_override is not None
            else float(wrist["loop_rate_hz"])
        )
        if not math.isfinite(loop_rate_hz) or not 20.0 <= loop_rate_hz <= 500.0:
            raise ValueError("wrist loop rate 必须在 20..500 Hz")
        command = [
            str(executable),
            str(bridge),
            "--ids",
            *(str(value) for value in ids),
            "--zero-rev",
            *(str(value) for value in wrist["zero_position_rev"]),
            "--zero-hold-mask",
            *("1" if bool(value) else "0" for value in zero_hold_mask),
            "--motor-sign",
            *(str(value) for value in wrist["motor_sign"]),
            "--limit-deg",
            *(str(value) for value in wrist["joint_limit_deg"]),
            "--reduction-ratio",
            *(str(value) for value in wrist["reduction_ratio"]),
            "--max-torque-nm",
            *(str(value) for value in wrist["torque_limit_nm"]),
            "--position-torque-nm",
            *(str(value) for value in wrist["position_torque_limit_nm"]),
            "--position-kp-scale",
            *(str(value) for value in position_kp_scale),
            "--position-kd-scale",
            *(str(value) for value in position_kd_scale),
            "--zero-position-kp-scale",
            *(str(value) for value in wrist["zero_position_kp_scale"]),
            "--zero-position-kd-scale",
            *(str(value) for value in wrist["zero_position_kd_scale"]),
            "--velocity-deg-s",
            str(wrist["zero_velocity_deg_s"]),
            "--accel-deg-s2",
            str(wrist["zero_acceleration_deg_s2"]),
            "--zero-tolerance-deg",
            str(wrist["zero_tolerance_deg"]),
            "--zero-settle-seconds",
            str(wrist["zero_settle_seconds"]),
            "--zero-timeout-s",
            str(wrist["zero_timeout_s"]),
            "--runtime-velocity-deg-s",
            str(wrist["runtime_velocity_deg_s"]),
            "--runtime-accel-deg-s2",
            str(wrist["runtime_acceleration_deg_s2"]),
            "--position-following-error-deg",
            str(following_error_deg),
            "--position-following-error-timeout-s",
            str(wrist["position_following_error_timeout_s"]),
            "--loop-hz",
            str(loop_rate_hz),
            "--command-timeout-s",
            str(wrist["command_timeout_s"]),
            "--watchdog-ms",
            str(watchdog_ms),
            "--soft-limit-margin-deg",
            str(wrist["soft_limit_margin_deg"]),
            "--fdcanusb",
            str(wrist["fdcanusb"]),
            "--allow-torque-control",
            "--confirm-torque",
            str(profile.runtime.get("confirm_wrist_torque", "")),
        ]
        self._process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self._ready = threading.Event()
        self._samples: deque[WristSample] = deque(maxlen=1)
        self._lock = threading.Lock()
        self._stdin_lock = threading.Lock()
        self._error: BaseException | None = None
        threading.Thread(target=self._read_stdout, daemon=True).start()
        threading.Thread(target=self._read_stderr, daemon=True).start()

    def _read_stdout(self) -> None:
        assert self._process.stdout is not None
        try:
            for line in self._process.stdout:
                message = line.strip()
                if message.startswith("WRIST_READY"):
                    print(f"[wrist] {message}")
                    self._ready.set()
                    continue
                if message.startswith("WRIST_POSITION_WRAP"):
                    print(f"[wrist] {message}")
                    continue
                if message.startswith((
                    "WRIST_POSITION_CAPTURE",
                    "WRIST_POSITION_STATUS",
                    "WRIST_TORQUE_STATUS",
                )):
                    print(f"[wrist] {message}")
                    continue
                if message.startswith(("WRIST_ZEROING", "WRIST_ZEROING_STATUS")):
                    print(f"[wrist] {message}")
                    continue
                fields = message.split()
                if len(fields) != 11 or fields[0] != "WRIST_SAMPLE":
                    raise ValueError(f"无法解析腕部数据: {message}")
                values = np.asarray([float(value) for value in fields[2:8]], dtype=float)
                sample = WristSample(
                    received_at_s=time.monotonic(),
                    timestamp_ns=int(fields[1]),
                    q_rad=np.array([values[0], values[3]]),
                    dq_rad_s=np.array([values[1], values[4]]),
                    torque_nm=np.array([values[2], values[5]]),
                    mode=fields[8].split("=", 1)[1],
                    servo_mode=np.asarray(
                        [int(value) for value in fields[9].split("=", 1)[1].split(",")],
                        dtype=int,
                    ),
                    fault=np.asarray(
                        [int(value) for value in fields[10].split("=", 1)[1].split(",")],
                        dtype=int,
                    ),
                )
                with self._lock:
                    self._samples.append(sample)
        except BaseException as error:
            self._error = error

    def _read_stderr(self) -> None:
        assert self._process.stderr is not None
        for line in self._process.stderr:
            print(f"[wrist] {line.rstrip()}", file=sys.stderr)

    def wait_ready(self, timeout_s: float = 20.0) -> None:
        deadline = time.monotonic() + timeout_s
        while not self._ready.wait(0.05):
            if self._error is not None:
                raise RuntimeError(f"腕部数据读取失败: {self._error}") from self._error
            return_code = self._process.poll()
            if return_code is not None:
                raise RuntimeError(f"腕部 bridge 启动失败，返回码 {return_code}")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"腕部 {timeout_s:g} 秒内未完成回零/就绪")

    def latest(self) -> WristSample:
        if self._error is not None:
            raise RuntimeError(f"腕部数据读取失败: {self._error}") from self._error
        if self._process.poll() is not None:
            raise RuntimeError(f"腕部 bridge 已退出，返回码 {self._process.returncode}")
        with self._lock:
            if not self._samples:
                raise RuntimeError("腕部尚无状态数据")
            return self._samples[-1]

    def wait_first_sample(self, timeout_s: float = 2.0) -> WristSample:
        """Wait for the first coherent sample after ``WRIST_READY``.

        READY and the first sample are written on consecutive bridge loop
        iterations.  Treating READY as if it already contained a sample is a
        race, especially after a zero-mask startup that finishes immediately.
        """
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("首帧等待时间必须是有限正数")
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                return self.latest()
            except RuntimeError as error:
                if "腕部尚无状态数据" not in str(error):
                    raise
                if time.monotonic() >= deadline:
                    raise TimeoutError(
                        f"WRIST_READY 后 {timeout_s:g} 秒内未收到第一帧状态"
                    ) from error
                time.sleep(0.005)

    def _send(self, command: str) -> None:
        if self._process.poll() is not None or self._process.stdin is None:
            raise RuntimeError("腕部 bridge 已退出")
        with self._stdin_lock:
            self._process.stdin.write(command + "\n")
            self._process.stdin.flush()

    def command_position(self, q_rad: np.ndarray) -> None:
        q = np.asarray(q_rad, dtype=float)
        self._send(f"P {q[0]:.9g} {q[1]:.9g}")

    def command_joint8_position(self, q8_rad: float) -> None:
        """Control ID2/joint 8 only; ID1/joint 9 remains in STOP."""
        self._send(f"P8 {float(q8_rad):.9g}")

    def command_torque(self, torque_nm: np.ndarray) -> None:
        torque = np.asarray(torque_nm, dtype=float)
        self._send(f"T {torque[0]:.9g} {torque[1]:.9g}")

    def command_hybrid(self, q_rad: np.ndarray, torque_nm: np.ndarray) -> None:
        """Track the OSC joint target with drive PD plus model feed-forward.

        The Flexiv arm remains in RT joint-torque mode. This wrist execution
        path is useful on the commissioned 2024 moteus firmware where a pure
        feed-forward command enters POSITION mode but produces almost no
        measured output torque. The target still comes from the same coupled
        9-DoF OSC/IK solution; moteus only closes the fast two-axis inner loop.
        """
        q = np.asarray(q_rad, dtype=float)
        torque = np.asarray(torque_nm, dtype=float)
        self._send(
            "H {:.9g} {:.9g} {:.9g} {:.9g}".format(
                q[0], q[1], torque[0], torque[1]
            )
        )

    def finish_startup(self) -> None:
        """Enable the 100 ms watchdog immediately before the main loop."""
        self._send("A")

    def close(self) -> None:
        if self._process.poll() is None:
            try:
                self._send("S")
            except Exception:
                pass
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.terminate()
                self._process.wait(timeout=2.0)

    def __enter__(self) -> "WristBridge":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


@dataclass(frozen=True)
class RuntimeModeRequest:
    mode: TeleopMode
    source: str


class RuntimeModeCommandReader:
    """Read optional local-UI mode requests without replacing pedal input."""

    def __init__(self) -> None:
        self._requests: deque[RuntimeModeRequest] = deque()
        self._lock = threading.Lock()
        threading.Thread(target=self._read_stdin, daemon=True).start()

    def _read_stdin(self) -> None:
        for line in sys.stdin:
            try:
                request = RuntimeModeRequest(
                    mode=parse_runtime_mode_command(line),
                    source="UI",
                )
            except ValueError as error:
                print(f"UI_COMMAND_ERROR {error}", flush=True)
                continue
            with self._lock:
                self._requests.append(request)

    def poll(self) -> list[RuntimeModeRequest]:
        with self._lock:
            requests = list(self._requests)
            self._requests.clear()
        return requests


def _open_pedal(profile: Profile):
    config = load_foot_pedal_configuration(profile.pedal_config_path)
    missing = [pedal_id for pedal_id in ("pedal_1", "pedal_2", "pedal_3") if config.pedal(pedal_id).key_code is None]
    if missing:
        raise ValueError(
            "脚踏板 key_code 尚未标定: "
            + ", ".join(missing)
            + "；先运行 freedrive_python/scripts/diagnose_xinput_foot_pedal.py"
        )
    backend = str(_section(profile.document, "pedal").get("backend", "xinput"))
    if backend == "xinput":
        return XInputFootPedalReader(config, emit_all_events=True)
    if backend == "evdev":
        return FootPedalReader(config, emit_all_events=True)
    raise ValueError("pedal.backend 只能是 xinput 或 evdev")


def _pedal_test(profile: Profile, seconds: float) -> int:
    selector = PedalModeStateMachine()
    with _open_pedal(profile) as pedal:
        pedal.arm()
        print(f"脚踏板模式测试 {seconds:g} 秒；不会连接 Flexiv、moteus 或 CHAI3D。")
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            for event in pedal.poll():
                mode = MODE_BY_ACTION[event.action]
                transition = selector.select(mode, clutch_pressed=False)
                print(f"{event.pedal_id} -> {mode.value}, ready={int(transition.ready)}")
            time.sleep(0.01)
    return 0


def _require_real_calibration(profile: Profile) -> None:
    if profile.wrist.get("calibration_ready") is not True:
        raise RuntimeError(
            "wrist.calibration_ready=false：腕部轴线、零位、方向、探针长度、质量/CoM/惯量"
            "尚未实测确认，拒绝启动真实 9-DoF 运动"
        )
    if profile.runtime.get("arm") is not True or profile.runtime.get("confirm_robot") != "MOVE_RIZON":
        raise RuntimeError("真实 Flexiv 运动需要 runtime.arm=true 和 confirm_robot=MOVE_RIZON")
    if profile.runtime.get("confirm_wrist_torque") != "MOTEUS_TORQUE_MODE":
        raise RuntimeError("模式 3 需要 confirm_wrist_torque=MOTEUS_TORQUE_MODE")


def _validated_active_tool_transform(
    profile: Profile, tool_name: str, params: Any
) -> np.ndarray:
    """Validate the Elements Tool and return ``flange_T_active_tcp``.

    Flexiv Cartesian commands target the active TCP, while the wrist model
    solves a flange target.  Supporting a calibrated nonzero Tool TCP avoids
    asking the user to replace their correct payload/TCP definition with a
    synthetic flange-only Tool.
    """
    config = profile.flexiv_tool
    expected_name = str(config["expected_name"])
    if tool_name != expected_name:
        raise RuntimeError(
            f"active Tool 是 {tool_name!r}，配置要求 {expected_name!r}"
        )
    actual_mass = float(params.mass)
    expected_mass = float(config["expected_mass_kg"])
    if not math.isfinite(actual_mass) or abs(actual_mass - expected_mass) > float(
        config["mass_tolerance_kg"]
    ):
        raise RuntimeError(
            f"active Tool mass={actual_mass:.3f}kg 与配置 {expected_mass:.3f}kg 不一致"
        )
    actual_pose = np.asarray(params.tcp_location, dtype=float)
    expected_pose = _vector(config, "expected_tcp_location", 7)
    actual_transform = pose_to_transform(actual_pose)
    expected_transform = pose_to_transform(expected_pose)
    position_error = float(
        np.linalg.norm(actual_transform[:3, 3] - expected_transform[:3, 3])
    )
    rotation_error = float(
        np.linalg.norm(
            rotation_vector(
                actual_transform[:3, :3] @ expected_transform[:3, :3].T
            )
        )
    )
    if position_error > float(config["tcp_position_tolerance_m"]):
        raise RuntimeError(
            f"active Tool TCP 与配置相差 {position_error * 1000.0:.1f}mm"
        )
    if rotation_error > math.radians(float(config["tcp_rotation_tolerance_deg"])):
        raise RuntimeError(
            f"active Tool 姿态与配置相差 {math.degrees(rotation_error):.2f}deg"
        )
    model_zero = profile.geometry.forward(np.zeros(2))
    model_position_error = float(
        np.linalg.norm(model_zero[:3, 3] - actual_transform[:3, 3])
    )
    model_rotation_error = float(
        np.linalg.norm(
            rotation_vector(model_zero[:3, :3] @ actual_transform[:3, :3].T)
        )
    )
    if (
        model_position_error > float(config["tcp_position_tolerance_m"])
        or model_rotation_error
        > math.radians(float(config["tcp_rotation_tolerance_deg"]))
    ):
        raise RuntimeError(
            "腕部 q=[0,0] 正运动学与 active Tool TCP 不一致："
            f"位置误差 {model_position_error * 1000.0:.1f}mm，"
            f"姿态误差 {math.degrees(model_rotation_error):.2f}deg"
        )
    return actual_transform


def _active_tcp_pose_for_flange_target(
    flange_target_world: np.ndarray, flange_to_active_tcp: np.ndarray
) -> np.ndarray:
    target = np.asarray(flange_target_world, dtype=float)
    tool_transform = np.asarray(flange_to_active_tcp, dtype=float)
    if target.shape != (4, 4) or tool_transform.shape != (4, 4):
        raise ValueError("flange target 和 Tool transform 必须是 4x4")
    return transform_to_pose(target @ tool_transform)


def _run_real(
    profile: Profile,
    mode3_force_n: float | None = None,
    *,
    ui_control_stdin: bool = False,
) -> int:
    import flexivrdk

    _require_real_calibration(profile)
    robot_cfg = profile.robot
    haptic_cfg = profile.haptic
    payload = profile.payload
    allocation = profile.allocation
    osc = profile.osc
    force_feedback = profile.force_feedback
    ui_mode_commands = RuntimeModeCommandReader() if ui_control_stdin else None
    configured_mode3_force_n = float(osc["target_sensed_force_tool_z_n"])
    effective_mode3_force_n = configured_mode3_force_n
    if mode3_force_n is not None:
        if not math.isfinite(mode3_force_n) or mode3_force_n == 0.0:
            raise ValueError("--mode3-force-n 必须是有限非零值")
        if math.copysign(1.0, mode3_force_n) != math.copysign(
            1.0, configured_mode3_force_n
        ):
            raise ValueError("--mode3-force-n 不能改变已配置的 Tool-Z 力方向")
        if abs(mode3_force_n) > abs(configured_mode3_force_n):
            raise ValueError("--mode3-force-n 不能超过配置的任务力幅值")
        effective_mode3_force_n = mode3_force_n
    mapper = RelativePoseMapper(
        MappingConfig(
            translation_scale=_positive(haptic_cfg, "translation_scale"),
            translation_deadband_m=float(haptic_cfg["translation_deadband_m"]),
            rotation_deadband_rad=math.radians(
                float(haptic_cfg["rotation_deadband_deg"])
            ),
            max_translation_m=None,
            max_step_m=float(robot_cfg["max_linear_velocity_m_s"])
            / float(robot_cfg["command_rate_hz"]),
            enable_rotation=True,
            max_rotation_rad=None,
            max_angular_step_rad=float(robot_cfg["max_angular_velocity_rad_s"])
            / float(robot_cfg["command_rate_hz"]),
        ),
        parse_axis_map(str(haptic_cfg["axis_map"])),
        rotation_axis_map=parse_axis_map(
            str(haptic_cfg.get("rotation_axis_map", haptic_cfg["axis_map"]))
        ),
        rotation_command_sign=_vector(haptic_cfg, "rotation_command_sign", 3),
    )

    with ExitStack() as stack:
        pedal = stack.enter_context(_open_pedal(profile))
        haptic = stack.enter_context(
            BridgeReader(
                PROJECT_ROOT / "build" / "chai3d_device_stream",
                int(haptic_cfg["device"]),
                int(haptic_cfg["device_rate_hz"]),
                False,
                feedback_watchdog_ms=round(_positive(haptic_cfg, "watchdog_ms")),
                teleop_damping_n_per_mps=float(
                    haptic_cfg["teleop_damping_n_per_m_s"]
                ),
                gravity_compensation=bool(haptic_cfg["gravity_compensation"]),
                hold_switch=int(haptic_cfg["switch"]),
                hold_when_released=False,
                startup_center=False,
            )
        )
        wrist = stack.enter_context(WristBridge(profile))
        pedal.arm()
        first_haptic = wait_for_first_sample(haptic)
        switch_index = int(haptic_cfg["switch"])
        if first_haptic.switch_pressed(switch_index):
            raise RuntimeError("启动前必须松开 omega.7 clutch")
        if force_feedback.get("enabled") is True:
            capabilities = haptic.capabilities()
            if not capabilities.actuated_force:
                raise RuntimeError("当前 CHAI3D 设备不支持三轴力输出")
            if float(force_feedback["max_device_force_n"]) > capabilities.max_force_n:
                raise RuntimeError(
                    f"配置触觉输出 {float(force_feedback['max_device_force_n']):.1f}N 超过 "
                    f"设备连续额定值 {capabilities.max_force_n:.1f}N"
                )
        wrist.wait_ready(timeout_s=float(profile.wrist["zero_timeout_s"]) + 5.0)
        wrist_state = wrist.wait_first_sample(2.0)
        robot = flexivrdk.Robot(str(robot_cfg["robot_sn"]), [str(robot_cfg["network_interface_ip"])])
        robot_in_motion = False
        try:
            ensure_robot_ready(robot)
            tool = flexivrdk.Tool(robot)
            tool_name = str(tool.name())
            tool_params = tool.params()
            flange_to_active_tcp = _validated_active_tool_transform(
                profile, tool_name, tool_params
            )
            print(
                f"Flexiv Tool={tool_name}, mass={float(tool_params.mass):.3f}kg, "
                "已启用 active-TCP/法兰动态换算；"
                "脚踏板选择模式，omega.7 clutch 控制当前模式启停。"
            )
            print(
                f"模式 3 本次 Tool-Z sensed force={effective_mode3_force_n:+.1f}N"
                + (
                    "（低力方向验证）"
                    if mode3_force_n is not None
                    else "（任务配置）"
                )
            )
            print(
                "模式 3：omega.7 只输入姿态；Flexiv 末端力传感器的世界坐标力"
                "按实时 q8/q9 旋转到探针坐标；X/Y 位置控制，探针 Z 使用 Flexiv 内置力控制。"
            )
            print(
                f"力控制轴：只闭环探针 Fz={effective_mode3_force_n:+.2f}N；"
                "不启动、不读取外置力传感器。"
            )
            robot.SwitchMode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
            robot_in_motion = True
            robot.SetForceControlAxis([False] * 6)
            states = robot.states()
            held_tcp_pose = np.asarray(states.tcp_pose, dtype=float).copy()
            robot.SendCartesianMotionForce(held_tcp_pose.tolist())
            feedback_field = (
                "ext_wrench_in_world_raw"
                if str(force_feedback["source"]) == "raw"
                else "ext_wrench_in_world"
            )
            feedback_bias_world = np.asarray(
                getattr(states, feedback_field)[:3], dtype=float
            ).copy()
            mode3_force_field = (
                "ext_wrench_in_world_raw"
                if str(osc["force_source"]) == "raw"
                else "ext_wrench_in_world"
            )
            stable_feedback = StableHapticFeedback(
                update_rate_hz=float(robot_cfg["command_rate_hz"]),
                lowpass_hz=float(force_feedback["wrench_lowpass_hz"]),
                force_slew_n_per_s=float(force_feedback["force_slew_n_per_s"]),
                engagement_ramp_s=float(force_feedback["engagement_ramp_s"]),
                local_damping_n_per_m_s=float(
                    force_feedback["local_damping_n_per_m_s"]
                ),
                max_device_force_n=float(force_feedback["max_device_force_n"]),
                initial_tank_energy_j=float(
                    force_feedback["passivity_initial_energy_j"]
                ),
                max_tank_energy_j=float(
                    force_feedback["passivity_max_energy_j"]
                ),
                passivity_enabled=bool(force_feedback["passivity_enabled"]),
            )
            print(
                "记录触觉反馈 bias [N]=[{}]；运行中外力无软件退出阈值，"
                "触觉输出只在 12N 额定值饱和。".format(
                    ", ".join(f"{value:+.2f}" for value in feedback_bias_world)
                )
            )

            selector = PedalModeStateMachine()
            selected_mode: TeleopMode | None = None
            was_enabled = False
            mode_has_engaged = False
            wrist_hold_q = wrist_state.q_rad.copy()
            wrist_goal_q = wrist_hold_q.copy()
            wrist_target_shaper = WristTargetShaper(
                filter_hz=float(allocation["wrist_target_filter_hz"]),
                max_velocity_rad_s=np.deg2rad(
                    _vector(allocation, "wrist_target_max_velocity_deg_s", 2)
                ),
                max_acceleration_rad_s2=np.deg2rad(
                    _vector(allocation, "wrist_target_max_acceleration_deg_s2", 2)
                ),
                joint_min_rad=profile.geometry.joint_min_rad
                + math.radians(float(allocation["joint_limit_margin_deg"])),
                joint_max_rad=profile.geometry.joint_max_rad
                - math.radians(float(allocation["joint_limit_margin_deg"])),
            )
            wrist_target_shaper.reset(wrist_state.q_rad)
            zero_hold_mask = np.asarray(
                profile.wrist["zero_hold_mask"], dtype=bool
            )
            mode1_hold_q = wrist_state.q_rad.copy()
            mode1_hold_q[zero_hold_mask] = 0.0
            frozen_probe = probe_pose_from_flange(states.flange_pose, profile.geometry, wrist_state.q_rad)
            probe_anchor = frozen_probe.copy()
            flange_anchor_rotation = pose_to_transform(states.flange_pose)[:3, :3]
            wrist_anchor_rotation = profile.geometry.forward(wrist_state.q_rad)[:3, :3]
            target_probe = frozen_probe.copy()
            centering_mode1 = False
            period = 1.0 / int(robot_cfg["command_rate_hz"])
            next_tick = time.monotonic()
            next_print = 0.0
            force_axis_active = False
            force_command_z_n = 0.0
            probe_force_filtered_n = np.zeros(3)
            probe_force_filter_initialized = False
            probe_force_z_n = math.nan
            feedback_passivity_limited = False
            feedback_tank_energy_j = stable_feedback.tank_energy_j
            last_force_frame_rotation = target_probe[:3, :3].copy()
            next_force_frame_update = 0.0
            last_haptic_position = first_haptic.position.copy()
            last_haptic_velocity_time = time.monotonic()
            filtered_haptic_velocity = np.zeros(3)
            haptic_anchor_rotation = first_haptic.rotation.copy()
            haptic_rotation_delta_deg = np.zeros(3)
            probe_target_rotation_delta_deg = np.zeros(3)
            wrist_osc_error_deg = np.zeros(3)
            wrist_command_torque_nm = np.zeros(2)
            wrist_zero_tolerance_rad = math.radians(
                float(profile.wrist["zero_tolerance_deg"])
            )
            print("等待脚踏板：Pedal 1=7DoF，Pedal 2=9DoF，Pedal 3=仅姿态+恒力。")
            wrist.finish_startup()

            while True:
                next_tick += period
                ensure_robot_ready(robot)
                states = robot.states()
                haptic_latest = haptic.latest()
                if haptic_latest is None:
                    raise RuntimeError("CHAI3D 尚无数据")
                haptic_received, haptic_sample = haptic_latest
                haptic_age_ms = (time.monotonic() - haptic_received) * 1000.0
                if haptic_age_ms > float(haptic_cfg["watchdog_ms"]):
                    raise RuntimeError(f"CHAI3D watchdog 超时 {haptic_age_ms:.1f}ms")
                wrist_state = wrist.latest()
                wrist_age_ms = (time.monotonic() - wrist_state.received_at_s) * 1000.0
                if wrist_age_ms > float(profile.wrist["watchdog_ms"]):
                    raise RuntimeError(f"腕部状态 watchdog 超时 {wrist_age_ms:.1f}ms")
                pressed = haptic_sample.switch_pressed(switch_index)
                now = time.monotonic()
                velocity_dt = now - last_haptic_velocity_time
                if velocity_dt > 1e-4:
                    raw_haptic_velocity = (
                        haptic_sample.position - last_haptic_position
                    ) / velocity_dt
                    velocity_alpha = velocity_dt / (0.03 + velocity_dt)
                    filtered_haptic_velocity += velocity_alpha * (
                        raw_haptic_velocity - filtered_haptic_velocity
                    )
                    last_haptic_position = haptic_sample.position.copy()
                    last_haptic_velocity_time = now
                current_probe = probe_pose_from_flange(
                    states.flange_pose, profile.geometry, wrist_state.q_rad
                )
                flexiv_force_world = np.asarray(
                    getattr(states, mode3_force_field)[:3], dtype=float
                )
                measured_probe_force = probe_force_from_world(
                    flexiv_force_world, current_probe[:3, :3]
                )
                force_alpha = 1.0 - math.exp(
                    -2.0 * math.pi * float(osc["force_display_lowpass_hz"]) * period
                )
                if not probe_force_filter_initialized:
                    probe_force_filtered_n = measured_probe_force.copy()
                    probe_force_filter_initialized = True
                else:
                    probe_force_filtered_n += force_alpha * (
                        measured_probe_force - probe_force_filtered_n
                    )
                probe_force_z_n = float(probe_force_filtered_n[2])

                mode_requests = [
                    RuntimeModeRequest(MODE_BY_ACTION[event.action], event.pedal_id)
                    for event in pedal.poll()
                ]
                if ui_mode_commands is not None:
                    mode_requests.extend(ui_mode_commands.poll())
                for request in mode_requests:
                    mode = request.mode
                    if force_axis_active:
                        robot.SetForceControlAxis([False] * 6)
                        force_axis_active = False
                        force_command_z_n = 0.0
                    selector.select(mode, clutch_pressed=pressed)
                    selected_mode = mode
                    was_enabled = False
                    mode_has_engaged = False
                    frozen_probe = current_probe.copy()
                    target_probe = frozen_probe.copy()
                    wrist_hold_q = wrist_state.q_rad.copy()
                    wrist_goal_q = wrist_hold_q.copy()
                    wrist_target_shaper.reset(wrist_state.q_rad)
                    mode1_hold_q = wrist_state.q_rad.copy()
                    mode1_hold_q[zero_hold_mask] = 0.0
                    held_tcp_pose = np.asarray(states.tcp_pose, dtype=float).copy()
                    centering_mode1 = (
                        mode is TeleopMode.ARM_7DOF
                        and np.any(
                            np.abs(
                                wrist_state.q_rad[zero_hold_mask]
                                - mode1_hold_q[zero_hold_mask]
                            )
                            > wrist_zero_tolerance_rad
                        )
                    )
                    print(
                        f"{request.source}: 选择 {mode.value}；"
                        + (
                            "先补偿探针位置并让 ID2 回零，ID1 保持 STOP"
                            if centering_mode1
                            else (
                                "Joint 9 保持 STOP；松开后再按 clutch 开始"
                                if mode is not TeleopMode.ARM_7DOF
                                else "松开后再按 clutch 开始"
                            )
                        )
                    )

                selector.observe_clutch(pressed)
                if centering_mode1:
                    wrist.command_joint8_position(mode1_hold_q[0])
                    held_tcp_pose = _active_tcp_pose_for_flange_target(
                        flange_target_for_probe(
                            frozen_probe, profile.geometry, wrist_state.q_rad
                        ),
                        flange_to_active_tcp,
                    )
                    if np.all(
                        np.abs(wrist_state.q_rad - mode1_hold_q)
                        <= wrist_zero_tolerance_rad
                    ) and np.linalg.norm(wrist_state.dq_rad_s) <= math.radians(2.0):
                        centering_mode1 = False
                        wrist_hold_q = mode1_hold_q.copy()
                        wrist_goal_q = mode1_hold_q.copy()
                        if pressed and selected_mode is not None:
                            selector.select(selected_mode, clutch_pressed=True)
                        print(
                            "Pedal 1：ID2 已回零，ID1 保持 STOP；"
                            "松开后按 clutch 开始 7DoF。"
                        )

                enabled = selector.teleoperation_enabled(pressed) and not centering_mode1
                if enabled and not was_enabled:
                    # Capture the newest joint state at the actual clutch edge.
                    # ID1/joint 9 stays in STOP between startup or mode
                    # selection and this point, so a stale selection-time angle
                    # must never be used as its first position target.
                    wrist_hold_q = wrist_state.q_rad.copy()
                    probe_anchor = current_probe.copy()
                    target_probe = probe_anchor.copy()
                    mapper.capture(haptic_sample, transform_to_pose(probe_anchor))
                    haptic_anchor_rotation = haptic_sample.rotation.copy()
                    haptic_rotation_delta_deg.fill(0.0)
                    probe_target_rotation_delta_deg.fill(0.0)
                    wrist_osc_error_deg.fill(0.0)
                    wrist_command_torque_nm.fill(0.0)
                    flange_anchor_rotation = pose_to_transform(states.flange_pose)[:3, :3]
                    wrist_anchor_rotation = profile.geometry.forward(wrist_state.q_rad)[:3, :3]
                    wrist_goal_q = wrist_state.q_rad.copy()
                    wrist_target_shaper.reset(wrist_state.q_rad)
                    mode_has_engaged = True
                    if selected_mode is TeleopMode.PIVOT_ORIENTATION:
                        robot.SetForceControlFrame(
                            flexivrdk.CoordType.WORLD,
                            transform_to_pose(current_probe).tolist(),
                        )
                        max_force_axis_velocity = float(
                            osc["force_axis_max_velocity_m_s"]
                        )
                        robot.SetForceControlAxis(
                            [False, False, True, False, False, False],
                            # Flexiv RDK 1.9 requires a fixed-size [Vx,Vy,Vz]
                            # vector even when only Tool-Z is force controlled.
                            [max_force_axis_velocity] * 3,
                        )
                        force_axis_active = True
                        # The Flexiv estimated external wrench is already the
                        # feedback used by its built-in force controller.  By
                        # setting the arbitrary force frame to the live probe
                        # orientation, command Z is exactly probe/TCP Z; no
                        # delayed external-sensor cascade is needed.
                        force_command_z_n = effective_mode3_force_n
                        last_force_frame_rotation = current_probe[:3, :3].copy()
                        next_force_frame_update = time.monotonic()
                    stable_feedback.reset()
                    print(f"clutch 按下：开始 {selected_mode.value if selected_mode else 'unknown'}")

                command_wrench = np.zeros(6)
                if enabled:
                    haptic_rotation_delta_deg = np.rad2deg(
                        rotation_vector(
                            haptic_sample.rotation @ haptic_anchor_rotation.T
                        )
                    )
                    mapped_probe_target = pose_to_transform(
                        mapper.target(haptic_sample)
                    )
                    if selected_mode is TeleopMode.PIVOT_ORIENTATION:
                        # Mode 3 deliberately discards every haptic translation
                        # component. X/Y stay at the engagement anchor and Z is
                        # governed only by Flexiv's probe-Z force axis.
                        target_probe = orientation_only_target(
                            mapped_probe_target, probe_anchor[:3, 3]
                        )
                    else:
                        target_probe = mapped_probe_target

                    probe_target_rotation_delta_deg = np.rad2deg(
                        rotation_vector(
                            target_probe[:3, :3] @ probe_anchor[:3, :3].T
                        )
                    )

                    if selected_mode is TeleopMode.ARM_7DOF:
                        wrist_goal_q = mode1_hold_q.copy()
                        wrist.command_joint8_position(wrist_goal_q[0])
                    else:
                        rotation_delta_world = target_probe[:3, :3] @ probe_anchor[:3, :3].T
                        rotation_delta_flange = (
                            flange_anchor_rotation.T
                            @ rotation_delta_world
                            @ flange_anchor_rotation
                        )
                        wrist_rotation_delta = rotation_delta_flange
                        if selected_mode is TeleopMode.ARM_WRIST_9DOF:
                            wrist_rotation_delta = scale_rotation(
                                rotation_delta_flange,
                                _positive(allocation, "wrist_priority_gain"),
                            )
                        elif selected_mode is TeleopMode.PIVOT_ORIENTATION:
                            wrist_rotation_delta = scale_rotation(
                                rotation_delta_flange,
                                _positive(osc, "wrist_priority_gain"),
                            )
                        desired_wrist_rotation = (
                            wrist_rotation_delta @ wrist_anchor_rotation
                        )
                        result = allocate_wrist_orientation(
                            profile.geometry,
                            wrist_goal_q,
                            desired_wrist_rotation,
                            damping=_positive(allocation, "damping"),
                            max_step_rad=math.radians(_positive(allocation, "max_iteration_step_deg")),
                            joint_margin_rad=math.radians(_positive(allocation, "joint_limit_margin_deg")),
                            max_iterations=int(allocation["max_iterations"]),
                        )
                        wrist_goal_q = wrist_target_shaper.step(
                            result.q_target_rad, period
                        )
                        if selected_mode is TeleopMode.ARM_WRIST_9DOF:
                            wrist.command_position(wrist_goal_q)
                        elif selected_mode is TeleopMode.PIVOT_ORIENTATION:
                            flange_rotation_world = pose_to_transform(states.flange_pose)[:3, :3]
                            gravity = wrist_gravity_compensation(
                                profile.geometry,
                                wrist_state.q_rad,
                                flange_rotation_world,
                                link1_mass_kg=float(payload["link1_mass_kg"]),
                                link1_com_after_joint1_m=_vector(payload, "link1_com_after_joint1_m", 3),
                                link2_mass_kg=float(payload["link2_and_probe_mass_kg"]),
                                link2_com_after_joint2_m=_vector(payload, "link2_and_probe_com_after_joint2_m", 3),
                            )
                            osc_result = operational_space_wrist_torque(
                                profile.geometry,
                                wrist_state.q_rad,
                                wrist_state.dq_rad_s,
                                profile.geometry.forward(wrist_goal_q)[:3, :3],
                                rotational_stiffness_nm_per_rad=_positive(
                                    osc, "rotational_stiffness_nm_per_rad"
                                ),
                                rotational_damping_nm_s_per_rad=float(
                                    osc["rotational_damping_nm_s_per_rad"]
                                ),
                                gravity_torque_nm=gravity,
                                max_torque_nm=_vector(profile.wrist, "torque_limit_nm", 2),
                            )
                            wrist_osc_error_deg = np.rad2deg(
                                osc_result.orientation_error_rad
                            )
                            wrist_command_torque_nm = osc_result.torque_nm.copy()
                            wrist.command_torque(osc_result.torque_nm)

                            frame_error = float(
                                np.linalg.norm(
                                    rotation_vector(
                                        target_probe[:3, :3]
                                        @ last_force_frame_rotation.T
                                    )
                                )
                            )
                            if (
                                time.monotonic() >= next_force_frame_update
                                and frame_error
                                >= math.radians(
                                    float(osc["force_frame_update_angle_deg"])
                                )
                            ):
                                robot.SetForceControlFrame(
                                    flexivrdk.CoordType.WORLD,
                                    transform_to_pose(target_probe).tolist(),
                                )
                                last_force_frame_rotation = target_probe[:3, :3].copy()
                                next_force_frame_update = time.monotonic() + 1.0 / float(
                                    osc["force_frame_update_hz"]
                                )
                            force_command_z_n = effective_mode3_force_n
                            command_wrench[2] = force_command_z_n

                    held_tcp_pose = _active_tcp_pose_for_flange_target(
                        flange_target_for_probe(
                            target_probe, profile.geometry, wrist_state.q_rad
                        ),
                        flange_to_active_tcp,
                    )
                elif not centering_mode1:
                    if was_enabled:
                        if force_axis_active:
                            robot.SetForceControlAxis([False] * 6)
                            force_axis_active = False
                            force_command_z_n = 0.0
                        held_tcp_pose = np.asarray(states.tcp_pose, dtype=float).copy()
                        wrist_hold_q = wrist_state.q_rad.copy()
                        wrist_target_shaper.reset(wrist_hold_q)
                        haptic.zero_force_feedback()
                        print("clutch 松开：Flexiv 冻结，腕部切回位置保持")
                    hold_joint8, hold_joint9 = wrist_hold_axes(
                        selected_mode, mode_has_engaged
                    )
                    if hold_joint8 and not hold_joint9:
                        if selected_mode is not TeleopMode.ARM_7DOF:
                            # Refresh passive ID1 until the clutch is pressed.
                            wrist_hold_q[1] = wrist_state.q_rad[1]
                            wrist_goal_q = wrist_hold_q.copy()
                        q8_hold = (
                            mode1_hold_q[0]
                            if selected_mode is TeleopMode.ARM_7DOF
                            else wrist_hold_q[0]
                        )
                        wrist.command_joint8_position(q8_hold)
                    elif hold_joint8 and hold_joint9:
                        wrist.command_position(wrist_hold_q)

                if force_feedback.get("enabled") is True and enabled:
                    external_force_world = np.asarray(
                        getattr(states, feedback_field)[:3], dtype=float
                    )
                    device_force, overload_excess = map_progressive_feedback_force(
                        external_force_world,
                        feedback_bias_world,
                        mapper.axis_map,
                        base_gain=float(force_feedback["base_gain"]),
                        force_deadband_n=float(force_feedback["deadband_n"]),
                        overload_threshold_n=float(
                            force_feedback["overload_threshold_n"]
                        ),
                        overload_gain=float(force_feedback["overload_gain"]),
                            max_device_force_n=float(
                                force_feedback["max_device_force_n"]
                            ),
                        )
                    overload_damping_activation = float(
                        np.clip(
                            overload_excess
                            / float(
                                force_feedback["overload_full_damping_excess_n"]
                            ),
                            0.0,
                            1.0,
                        )
                    )
                    # Overload damping is dissipative and is fed through the
                    # same passivity/slew layer as the measured contact force.
                    device_force -= (
                        overload_damping_activation
                        * float(force_feedback["overload_damping_n_per_m_s"])
                        * filtered_haptic_velocity
                    )
                    stable_result = stable_feedback.update(
                        device_force, filtered_haptic_velocity, dt_s=period
                    )
                    feedback_passivity_limited = stable_result.passivity_limited
                    feedback_tank_energy_j = stable_result.tank_energy_j
                    haptic.send_force_feedback(stable_result.force_device_n)
                else:
                    overload_excess = 0.0
                    feedback_passivity_limited = False
                    haptic.zero_force_feedback()
                    if not enabled:
                        stable_feedback.reset()

                robot.SendCartesianMotionForce(
                    held_tcp_pose.tolist(),
                    wrench=command_wrench.tolist(),
                    max_linear_vel=float(robot_cfg["max_linear_velocity_m_s"]),
                    max_angular_vel=float(robot_cfg["max_angular_velocity_rad_s"]),
                    max_linear_acc=float(robot_cfg["max_linear_acceleration_m_s2"]),
                    max_angular_acc=float(robot_cfg["max_angular_acceleration_rad_s2"]),
                )
                if time.monotonic() >= next_print:
                    print(
                        "mode={} ready={} clutch_pressed={} enabled={} "
                        "haptic_rotvec_deg=[{}] probe_target_rotvec_deg=[{}] "
                        "q_deg=[{}] target_q_deg=[{}] dq_deg_s=[{}] "
                        "wrist_tau_meas_Nm=[{}] wrist_mode={} "
                        "wrist_osc_error_deg=[{}] wrist_tau_cmd_Nm=[{}] "
                        "tip_mm=[{}] probe_fz_from_flexiv_N={:+.2f} "
                        "flexiv_force_z_cmd_N={:+.2f} overload_N={:.1f} "
                        "passivity_limited={} tank_energy_J={:.4f}".format(
                            selected_mode.value if selected_mode else "waiting",
                            int(selector.ready),
                            int(pressed),
                            int(enabled),
                            ", ".join(f"{value:+.1f}" for value in haptic_rotation_delta_deg),
                            ", ".join(f"{value:+.1f}" for value in probe_target_rotation_delta_deg),
                            ", ".join(f"{value:+.1f}" for value in np.rad2deg(wrist_state.q_rad)),
                            ", ".join(f"{value:+.1f}" for value in np.rad2deg(wrist_goal_q)),
                            ", ".join(f"{value:+.1f}" for value in np.rad2deg(wrist_state.dq_rad_s)),
                            ", ".join(f"{value:+.3f}" for value in wrist_state.torque_nm),
                            wrist_state.mode,
                            ", ".join(f"{value:+.1f}" for value in wrist_osc_error_deg),
                            ", ".join(f"{value:+.3f}" for value in wrist_command_torque_nm),
                            ", ".join(f"{value:+.1f}" for value in current_probe[:3, 3] * 1000.0),
                            probe_force_z_n,
                            force_command_z_n,
                            overload_excess,
                            int(feedback_passivity_limited),
                            feedback_tank_energy_j,
                        )
                    )
                    next_print = time.monotonic() + 0.5
                was_enabled = enabled
                time.sleep(max(0.0, next_tick - time.monotonic()))
        finally:
            try:
                haptic.zero_force_feedback()
            except Exception:
                pass
            if robot_in_motion:
                robot.Stop()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--check-config", action="store_true", help="只验证配置，不连接任何硬件")
    parser.add_argument("--pedal-test", type=float, metavar="SECONDS", help="只测试三脚踏板模式事件")
    parser.add_argument(
        "--mode3-force-n",
        type=float,
        help="仅本次运行降低模式3的 Tool-Z sensed force；不能反向或超过配置幅值",
    )
    parser.add_argument(
        "--ui-control-stdin",
        action="store_true",
        help=argparse.SUPPRESS,
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    try:
        profile = load_profile(args.config.resolve())
        if args.check_config:
            print(f"配置语法和几何结构有效: {profile.path}")
            print(f"wrist.calibration_ready={profile.wrist.get('calibration_ready')}")
            print("外置力传感器: 已从运行时与配置中移除")
            print(
                "force_feedback.enabled={} passivity_enabled={}".format(
                    profile.force_feedback.get("enabled"),
                    profile.force_feedback.get("passivity_enabled"),
                )
            )
            return 0
        if args.pedal_test is not None:
            if args.pedal_test <= 0.0:
                raise ValueError("--pedal-test 秒数必须大于 0")
            return _pedal_test(profile, args.pedal_test)
        return _run_real(
            profile,
            args.mode3_force_n,
            ui_control_stdin=args.ui_control_stdin,
        )
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C：Flexiv Stop、腕部 STOP、触觉力归零。", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"错误: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
