"""Real-time torque/operational-space backend for three-mode teleoperation."""

from __future__ import annotations

from contextlib import ExitStack
import json
import math
import mmap
import os
from pathlib import Path
import signal
import struct
import subprocess
import tempfile
import time

import numpy as np

from controllers.nine_dof import PedalModeStateMachine, TeleopMode
from controllers.teleop import (
    MappingConfig,
    RelativePoseMapper,
    StableHapticFeedback,
    map_progressive_feedback_force,
    matrix_to_quaternion,
    parse_axis_map,
    quaternion_to_matrix,
    quaternion_to_rotation_vector,
)
from hardware.haptic_bridge import BridgeReader, wait_for_first_sample
from hardware.rt_teleop_bridge import (
    MODE_7DOF_OSC,
    MODE_9DOF_OSC,
    MODE_HOLD,
    MODE_ORIENTATION_FORCE_OSC,
    TELEOP_SHARED_SIZE,
    initialize as initialize_teleop_memory,
    publish_command,
    read_state,
)
from scripts.demo_9dof import (
    SHARED_MAGIC,
    SHARED_SIZE,
    _publish_state,
    _read_torque_command,
    _write_model,
)
from tools.osc_launcher import binary, realtime_preflight, validate_active_flexiv_tool


PROJECT_ROOT = Path(__file__).resolve().parents[1]

RT_MODE = {
    TeleopMode.ARM_7DOF: MODE_7DOF_OSC,
    TeleopMode.ARM_WRIST_9DOF: MODE_9DOF_OSC,
    TeleopMode.PIVOT_ORIENTATION: MODE_ORIENTATION_FORCE_OSC,
}


def _target_twist(
    previous_pose: np.ndarray, target_pose: np.ndarray, dt_s: float
) -> tuple[np.ndarray, np.ndarray]:
    """Estimate the world-frame target twist between two coherent poses."""
    previous = np.asarray(previous_pose, dtype=float)
    target = np.asarray(target_pose, dtype=float)
    if previous.shape != (7,) or target.shape != (7,):
        raise ValueError("target poses must contain 7 values")
    if not np.all(np.isfinite(previous)) or not np.all(np.isfinite(target)):
        raise ValueError("target poses must be finite")
    if not math.isfinite(dt_s) or dt_s <= 0.0:
        raise ValueError("target velocity dt must be positive")
    linear = (target[:3] - previous[:3]) / dt_s
    rotation_delta = (
        quaternion_to_matrix(target[3:])
        @ quaternion_to_matrix(previous[3:]).T
    )
    angular = quaternion_to_rotation_vector(
        matrix_to_quaternion(rotation_delta)
    ) / dt_s
    return linear, angular


def _limit_norm(vector: np.ndarray, maximum: float) -> np.ndarray:
    values = np.asarray(vector, dtype=float)
    norm = float(np.linalg.norm(values))
    if norm <= maximum or norm < 1e-12:
        return values.copy()
    return values * (maximum / norm)


def _orientation_only_target(
    mapped_pose: np.ndarray, fixed_position: np.ndarray
) -> np.ndarray:
    """Keep the mapped orientation while enforcing one captured XYZ point."""
    pose = np.asarray(mapped_pose, dtype=float).copy()
    anchor = np.asarray(fixed_position, dtype=float)
    if pose.shape != (7,) or anchor.shape != (3,):
        raise ValueError("orientation-only target requires pose[7] and anchor[3]")
    pose[:3] = anchor
    return pose


def _effective_mode3_force(profile, override: float | None) -> float:
    configured = float(profile.osc["target_sensed_force_tool_z_n"])
    if override is None:
        return configured
    value = float(override)
    if not math.isfinite(value) or value == 0.0:
        raise ValueError("--mode3-force-n 必须是有限非零值")
    return value


def _make_mapper(profile) -> RelativePoseMapper:
    haptic = profile.haptic
    robot = profile.robot
    # Preserve the last GitHub teleoperation semantics: the clutch mapping is
    # unrestricted in accumulated workspace, while a single coordinator-cycle
    # step cannot exceed the configured Flexiv Cartesian velocity. The Python
    # coordinator runs at robot.command_rate_hz below, so these limits have the
    # same physical meaning as in the known-good NRT teleoperation version.
    return RelativePoseMapper(
        MappingConfig(
            translation_scale=float(haptic["translation_scale"]),
            translation_deadband_m=float(haptic["translation_deadband_m"]),
            rotation_deadband_rad=math.radians(float(haptic["rotation_deadband_deg"])),
            max_translation_m=None,
            max_step_m=float(robot["max_linear_velocity_m_s"])
            / float(robot["command_rate_hz"]),
            enable_rotation=True,
            max_rotation_rad=None,
            max_angular_step_rad=float(robot["max_angular_velocity_rad_s"])
            / float(robot["command_rate_hz"]),
        ),
        parse_axis_map(str(haptic["axis_map"])),
        rotation_axis_map=parse_axis_map(
            str(haptic.get("rotation_axis_map", haptic["axis_map"]))
        ),
        rotation_command_sign=np.asarray(
            haptic["rotation_command_sign"], dtype=float
        ),
    )


def _stable_feedback(profile, rate_hz: float) -> StableHapticFeedback:
    config = profile.force_feedback
    return StableHapticFeedback(
        update_rate_hz=rate_hz,
        lowpass_hz=float(config["wrench_lowpass_hz"]),
        force_slew_n_per_s=float(config["force_slew_n_per_s"]),
        engagement_ramp_s=float(config["engagement_ramp_s"]),
        local_damping_n_per_m_s=float(config["local_damping_n_per_m_s"]),
        max_device_force_n=float(config["max_device_force_n"]),
        initial_tank_energy_j=float(config["passivity_initial_energy_j"]),
        max_tank_energy_j=float(config["passivity_max_energy_j"]),
        passivity_enabled=bool(config["passivity_enabled"]),
    )


def run(
    profile,
    mode3_force_n: float | None = None,
    *,
    ui_control_stdin: bool = False,
) -> int:
    # Imported here to avoid a module cycle while scripts.teleoperate defines
    # the shared Profile/WristBridge/UI classes.
    from scripts.teleoperate import (
        MODE_BY_ACTION,
        RuntimeModeCommandReader,
        RuntimeModeRequest,
        WristBridge,
        _open_pedal,
        _require_real_calibration,
    )

    _require_real_calibration(profile)
    executable = binary("flexiv_9dof_torque_osc")
    cpu_affinity = int(profile.teleop_osc["cpu_affinity"])
    print("\n".join(realtime_preflight(executable, cpu_affinity)))
    active_tool = validate_active_flexiv_tool(profile)
    fdcanusb = Path(str(profile.wrist["fdcanusb"]))
    if not fdcanusb.exists() or not os.access(fdcanusb, os.R_OK | os.W_OK):
        raise PermissionError(f"fdcanusb 不存在或无读写权限: {fdcanusb}")

    calibration_path = profile.path.parent / str(
        profile.payload["inertia_calibration_path"]
    )
    if not calibration_path.is_file():
        raise FileNotFoundError(f"缺少真实腕部惯量辨识: {calibration_path}")
    calibration = json.loads(calibration_path.read_text(encoding="utf-8"))
    if calibration.get("assembly_id") != str(profile.payload["assembly_id"]):
        raise RuntimeError("惯量辨识 assembly_id 与当前腕部总成不一致")

    haptic_config = profile.haptic
    force_config = profile.force_feedback
    # The known-good GitHub haptic/teleoperation path ran at the Flexiv command
    # rate (100 Hz). Keep that rate for mapping, velocity estimation, feedback
    # filtering and command publication. Service the slower fdcanusb wrist path
    # independently at its commissioned rate so restoring haptics does not
    # increase moteus bus traffic.
    coordinator_rate_hz = float(profile.robot["command_rate_hz"])
    wrist_rate_hz = float(profile.wrist["rt_osc_bridge_rate_hz"])
    period = 1.0 / coordinator_rate_hz
    wrist_period = 1.0 / wrist_rate_hz
    state_timeout_ms = float(profile.wrist["rt_osc_state_timeout_ms"])
    hard_timeout_ms = float(profile.wrist["rt_osc_hard_timeout_ms"])
    wrist_execution = str(profile.wrist["osc_execution_mode"])
    target_force_n = _effective_mode3_force(profile, mode3_force_n)
    mapper = _make_mapper(profile)
    stable_feedback = _stable_feedback(profile, coordinator_rate_hz)
    ui_reader = RuntimeModeCommandReader() if ui_control_stdin else None

    wrist_file = tempfile.NamedTemporaryFile(
        prefix="flexiv_teleop_wrist_", suffix=".bin", dir="/tmp", delete=False
    )
    teleop_file = tempfile.NamedTemporaryFile(
        prefix="flexiv_teleop_command_", suffix=".bin", dir="/tmp", delete=False
    )
    wrist_path = Path(wrist_file.name)
    teleop_path = Path(teleop_file.name)
    model_path = wrist_path.with_suffix(".model")
    process: subprocess.Popen[str] | None = None
    wrist_mapping: mmap.mmap | None = None
    teleop_mapping: mmap.mmap | None = None
    wrist = None
    haptic = None

    try:
        wrist_file.truncate(SHARED_SIZE)
        wrist_file.flush()
        wrist_mapping = mmap.mmap(wrist_file.fileno(), SHARED_SIZE)
        struct.pack_into("<Q", wrist_mapping, 0, SHARED_MAGIC)
        teleop_file.truncate(TELEOP_SHARED_SIZE)
        teleop_file.flush()
        teleop_mapping = mmap.mmap(teleop_file.fileno(), TELEOP_SHARED_SIZE)
        initialize_teleop_memory(teleop_mapping)
        inertia_mode = str(profile.teleop_osc["inertia_mode"])
        _write_model(profile, model_path, active_tool, inertia_mode)
        print(
            "Mode 2 wrist-first allocation: natural 1:1 wrist IK; "
            "Flexiv position-priority OSC handles translation and only the "
            "residual orientation (arm residual <= {:.2f} rad/s, posture rate={:.2f}/s).".format(
                float(profile.teleop_osc["mode2_arm_max_angular_velocity_rad_s"]),
                float(profile.teleop_osc["mode2_posture_reference_rate_per_s"]),
            ),
            flush=True,
        )

        with ExitStack() as stack:
            pedal = stack.enter_context(_open_pedal(profile))
            haptic = stack.enter_context(
                BridgeReader(
                    PROJECT_ROOT / "build" / "chai3d_device_stream",
                    int(haptic_config["device"]),
                    int(haptic_config["device_rate_hz"]),
                    False,
                    feedback_watchdog_ms=round(float(haptic_config["watchdog_ms"])),
                    teleop_damping_n_per_mps=float(
                        haptic_config["teleop_damping_n_per_m_s"]
                    ),
                    gravity_compensation=bool(
                        haptic_config["gravity_compensation"]
                    ),
                    hold_switch=int(haptic_config["switch"]),
                    hold_when_released=False,
                    startup_center=False,
                )
            )
            wrist = stack.enter_context(
                WristBridge(
                    profile,
                    zero_hold_mask_override=profile.wrist["zero_hold_mask"],
                    position_kp_scale_override=profile.wrist[
                        "osc_position_kp_scale"
                    ],
                    position_kd_scale_override=profile.wrist[
                        "osc_position_kd_scale"
                    ],
                    watchdog_ms_override=state_timeout_ms,
                    loop_rate_hz_override=wrist_rate_hz,
                )
            )
            pedal.arm()
            first_haptic = wait_for_first_sample(haptic)
            switch_index = int(haptic_config["switch"])
            if first_haptic.switch_pressed(switch_index):
                raise RuntimeError("启动前必须松开 omega.7 clutch")
            if force_config.get("enabled") is True:
                capabilities = haptic.capabilities()
                if not capabilities.actuated_force:
                    raise RuntimeError("当前 CHAI3D 设备不支持三轴力输出")
                if float(force_config["max_device_force_n"]) > capabilities.max_force_n:
                    raise RuntimeError("触觉反馈配置超过 omega.7 连续额定力")

            wrist.wait_ready(float(profile.wrist["zero_timeout_s"]) + 5.0)
            first_wrist = wrist.wait_first_sample(2.0)
            _publish_state(wrist_mapping, 1, first_wrist)
            wrist.finish_startup()
            # Keep both axes exactly where they are while Flexiv initializes;
            # joint 9 is not sent to zero.
            wrist.command_hybrid(first_wrist.q_rad, np.zeros(2))

            publish_command(
                teleop_mapping,
                1,
                mode=MODE_HOLD,
                enabled=False,
                target_pose=np.asarray([0, 0, 0, 1, 0, 0, 0], dtype=float),
            )
            command = [
                str(executable),
                "--robot-sn", str(profile.robot["robot_sn"]),
                "--wrist-shm", str(wrist_path),
                "--teleop-shm", str(teleop_path),
                "--wrist-model", str(model_path),
                "--duration-s", "20",
                "--radius-m", "0.01",
                "--orientation-deg", "3",
                "--trajectory", "loop",
                "--endpoint", "tip",
                "--inertia-mode", inertia_mode,
                "--wrist-execution-mode", wrist_execution,
                "--wrist-state-timeout-ms", str(state_timeout_ms),
                "--wrist-hard-timeout-ms", str(hard_timeout_ms),
                "--teleop-command-timeout-ms",
                str(profile.teleop_osc["command_timeout_ms"]),
                "--cpu-affinity", str(cpu_affinity),
                "--real-confirm", "RUN_9DOF_TORQUE_OSC",
            ]
            print("执行实时三模式 OSC:", " ".join(command), flush=True)
            process = subprocess.Popen(command, cwd=PROJECT_ROOT, text=True)

            wrist_sequence = 1
            teleop_sequence = 1
            last_wrist_command_sequence = 0
            last_wrist_torque = np.zeros(2)
            last_wrist_target = first_wrist.q_rad.copy()
            have_wrist_command = False

            def service_wrist() -> object:
                nonlocal wrist_sequence, last_wrist_command_sequence
                nonlocal last_wrist_torque, last_wrist_target, have_wrist_command
                sample = wrist.latest()
                wrist_sequence += 1
                _publish_state(wrist_mapping, wrist_sequence, sample)
                item = _read_torque_command(wrist_mapping)
                if item is not None and item[0] != last_wrist_command_sequence:
                    sequence, timestamp_ns, torque, target_q = item
                    if (time.monotonic_ns() - timestamp_ns) / 1e6 <= state_timeout_ms:
                        last_wrist_torque = torque
                        if np.all(np.isfinite(target_q)):
                            last_wrist_target = target_q
                        have_wrist_command = True
                    last_wrist_command_sequence = sequence
                if have_wrist_command:
                    if wrist_execution == "hybrid_position_ff":
                        wrist.command_hybrid(last_wrist_target, last_wrist_torque)
                    else:
                        wrist.command_torque(last_wrist_torque)
                else:
                    wrist.command_hybrid(first_wrist.q_rad, np.zeros(2))
                return sample

            state = None
            startup_deadline = time.monotonic() + 12.0
            startup_tick = time.monotonic()
            while state is None or state.age_ms > state_timeout_ms:
                if process.poll() is not None:
                    raise RuntimeError(
                        f"实时 OSC 启动失败，返回码 {process.returncode}"
                    )
                service_wrist()
                teleop_sequence += 1
                publish_command(
                    teleop_mapping,
                    teleop_sequence,
                    mode=MODE_HOLD,
                    enabled=False,
                    target_pose=np.asarray([0, 0, 0, 1, 0, 0, 0], dtype=float),
                )
                state = read_state(teleop_mapping)
                if time.monotonic() >= startup_deadline:
                    raise TimeoutError("12 秒内未收到 Flexiv RT OSC 状态")
                startup_tick += wrist_period
                time.sleep(max(0.0, startup_tick - time.monotonic()))

            feedback_bias_world = state.external_wrench_world[:3].copy()
            selector = PedalModeStateMachine()
            selected_mode: TeleopMode | None = None
            was_enabled = False
            target_pose = state.probe_pose.copy()
            mode3_position_anchor = state.probe_pose[:3].copy()
            haptic_position_anchor = first_haptic.position.copy()
            haptic_rotation_anchor = first_haptic.rotation.copy()
            probe_pose_anchor = state.probe_pose.copy()
            previous_target_pose = target_pose.copy()
            previous_target_time = time.monotonic()
            target_linear_velocity = np.zeros(3)
            target_angular_velocity = np.zeros(3)
            stable_feedback.reset()
            last_device_position = first_haptic.position.copy()
            last_device_time = time.monotonic()
            filtered_device_velocity = np.zeros(3)
            feedback_passivity_limited = False
            feedback_tank_energy_j = stable_feedback.tank_energy_j
            next_status = 0.0
            next_tick = time.monotonic()
            next_wrist_service = next_tick
            wrist_sample = first_wrist
            print(
                "已恢复 GitHub 触觉管线：coordinator={:.0f}Hz, wrist_io={:.0f}Hz, "
                "feedback_bias_world_N=[{}]；保留当前 C++ OSC 与动态补偿。".format(
                    coordinator_rate_hz,
                    wrist_rate_hz,
                    ", ".join(f"{value:+.2f}" for value in feedback_bias_world),
                ),
                flush=True,
            )
            print(
                "等待脚踏板；RT OSC 已就绪：Pedal 1=7DoF OSC，Pedal 2=9DoF OSC，"
                f"Pedal 3=姿态+Tool-Z {target_force_n:+.1f}N OSC。",
                flush=True,
            )
            print(
                "omega.7 输入：读取三轴手柄位置 + 完整 3x3 手柄姿态；clutch "
                "只负责捕获相对零点。旋转按 clutch 时的手柄局部坐标计算，"
                "状态中的 haptic_local_rot_deg 与 target_local_rot_deg 可直接核对正负方向。",
                flush=True,
            )

            while process.poll() is None:
                next_tick += period
                service_time = time.monotonic()
                if service_time >= next_wrist_service:
                    wrist_sample = service_wrist()
                    # Do not issue catch-up bursts after a scheduler delay.
                    next_wrist_service = service_time + wrist_period
                state = read_state(teleop_mapping)
                if state is None or state.age_ms > hard_timeout_ms:
                    raise RuntimeError("Flexiv RT 状态连续过期，已超过 hard timeout")
                latest = haptic.latest()
                if latest is None:
                    raise RuntimeError("CHAI3D 尚无状态")
                haptic_time, haptic_sample = latest
                haptic_age_ms = (time.monotonic() - haptic_time) * 1000.0
                if haptic_age_ms > float(haptic_config["watchdog_ms"]):
                    raise RuntimeError(f"CHAI3D watchdog 超时 {haptic_age_ms:.1f}ms")
                pressed = haptic_sample.switch_pressed(switch_index)
                now = time.monotonic()
                velocity_dt = now - last_device_time
                if velocity_dt > 1e-4:
                    raw_velocity = (
                        haptic_sample.position - last_device_position
                    ) / velocity_dt
                    velocity_alpha = velocity_dt / (0.03 + velocity_dt)
                    filtered_device_velocity += velocity_alpha * (
                        raw_velocity - filtered_device_velocity
                    )
                    last_device_position = haptic_sample.position.copy()
                    last_device_time = now

                requests = [
                    RuntimeModeRequest(MODE_BY_ACTION[event.action], event.pedal_id)
                    for event in pedal.poll()
                ]
                if ui_reader is not None:
                    requests.extend(ui_reader.poll())
                for request in requests:
                    selector.select(request.mode, clutch_pressed=pressed)
                    selected_mode = request.mode
                    was_enabled = False
                    target_pose = state.probe_pose.copy()
                    mode3_position_anchor = state.probe_pose[:3].copy()
                    probe_pose_anchor = state.probe_pose.copy()
                    previous_target_pose = target_pose.copy()
                    previous_target_time = now
                    target_linear_velocity.fill(0.0)
                    target_angular_velocity.fill(0.0)
                    print(
                        f"{request.source}: 已选择 {selected_mode.value}；"
                        "松开再按 clutch 后启用。",
                        flush=True,
                    )
                selector.observe_clutch(pressed)
                enabled = selector.teleoperation_enabled(pressed)
                if enabled and not was_enabled:
                    mapper.capture(haptic_sample, state.probe_pose)
                    target_pose = state.probe_pose.copy()
                    haptic_position_anchor = haptic_sample.position.copy()
                    haptic_rotation_anchor = haptic_sample.rotation.copy()
                    probe_pose_anchor = state.probe_pose.copy()
                    if selected_mode is TeleopMode.PIVOT_ORIENTATION:
                        mode3_position_anchor = state.probe_pose[:3].copy()
                    previous_target_pose = target_pose.copy()
                    previous_target_time = now
                    target_linear_velocity.fill(0.0)
                    target_angular_velocity.fill(0.0)
                    stable_feedback.reset()
                    print(f"clutch 按下：开始 {selected_mode.value}", flush=True)
                elif not enabled and was_enabled:
                    target_pose = state.probe_pose.copy()
                    previous_target_pose = target_pose.copy()
                    previous_target_time = now
                    target_linear_velocity.fill(0.0)
                    target_angular_velocity.fill(0.0)
                    stable_feedback.reset()
                    haptic.zero_force_feedback()
                    print("clutch 松开：OSC 保持当前位置/姿态", flush=True)

                if enabled:
                    next_target_pose = mapper.target(haptic_sample)
                    target_dt = max(1e-4, now - previous_target_time)
                    raw_target_linear_velocity, raw_target_angular_velocity = (
                        _target_twist(previous_target_pose, next_target_pose, target_dt)
                    )
                    velocity_alpha = 1.0 - math.exp(
                        -2.0
                        * math.pi
                        * float(profile.teleop_osc["target_filter_hz"])
                        * target_dt
                    )
                    target_linear_velocity += velocity_alpha * (
                        raw_target_linear_velocity - target_linear_velocity
                    )
                    target_angular_velocity += velocity_alpha * (
                        raw_target_angular_velocity - target_angular_velocity
                    )
                    target_linear_velocity = _limit_norm(
                        target_linear_velocity,
                        float(profile.teleop_osc["max_linear_velocity_m_s"]),
                    )
                    target_angular_velocity = _limit_norm(
                        target_angular_velocity,
                        float(profile.teleop_osc["max_angular_velocity_rad_s"]),
                    )
                    feedforward_gain = float(
                        profile.teleop_osc["target_velocity_feedforward_gain"]
                    )
                    target_linear_velocity *= feedforward_gain
                    target_angular_velocity *= feedforward_gain
                    target_pose = next_target_pose
                    previous_target_pose = target_pose.copy()
                    previous_target_time = now
                    if selected_mode is TeleopMode.PIVOT_ORIENTATION:
                        # Keep the contact point captured on clutch engagement.
                        # Never copy the live probe position here: doing so
                        # would move the target together with position error and
                        # defeat the fixed-point controller.
                        target_pose = _orientation_only_target(
                            target_pose, mode3_position_anchor
                        )
                        target_linear_velocity.fill(0.0)

                teleop_sequence += 1
                publish_command(
                    teleop_mapping,
                    teleop_sequence,
                    mode=RT_MODE.get(selected_mode, MODE_HOLD),
                    enabled=enabled,
                    target_pose=target_pose,
                    target_linear_velocity=target_linear_velocity,
                    target_angular_velocity=target_angular_velocity,
                    target_force_z_n=target_force_n,
                )

                # Match the last GitHub teleoperation behavior: clutch/mode
                # engagement owns haptic force enable. Do not gate force again
                # on the delayed C++ acknowledgement, which previously caused
                # force dropouts and restarted the engagement ramp.
                if force_config.get("enabled") is True and enabled:
                    device_force, overload = map_progressive_feedback_force(
                        state.external_wrench_world[:3],
                        feedback_bias_world,
                        mapper.axis_map,
                        base_gain=float(force_config["base_gain"]),
                        force_deadband_n=float(force_config["deadband_n"]),
                        overload_threshold_n=float(
                            force_config["overload_threshold_n"]
                        ),
                        overload_gain=float(force_config["overload_gain"]),
                        max_device_force_n=float(
                            force_config["max_device_force_n"]
                        ),
                    )
                    activation = float(
                        np.clip(
                            overload
                            / float(force_config["overload_full_damping_excess_n"]),
                            0.0,
                            1.0,
                        )
                    )
                    device_force -= (
                        activation
                        * float(force_config["overload_damping_n_per_m_s"])
                        * filtered_device_velocity
                    )
                    feedback = stable_feedback.update(
                        device_force, filtered_device_velocity, dt_s=period
                    )
                    feedback_passivity_limited = feedback.passivity_limited
                    feedback_tank_energy_j = feedback.tank_energy_j
                    haptic.send_force_feedback(feedback.force_device_n)
                else:
                    overload = 0.0
                    feedback_passivity_limited = False
                    haptic.zero_force_feedback()
                    if not enabled:
                        stable_feedback.reset()
                        feedback_tank_energy_j = stable_feedback.tank_energy_j

                if now >= next_status:
                    if enabled:
                        haptic_delta_mm = 1000.0 * (
                            haptic_sample.position - haptic_position_anchor
                        )
                        haptic_rotation_delta_deg = np.rad2deg(
                            quaternion_to_rotation_vector(
                                matrix_to_quaternion(
                                    haptic_rotation_anchor.T
                                    @ haptic_sample.rotation
                                )
                            )
                        )
                        target_delta_mm = 1000.0 * (
                            target_pose[:3] - probe_pose_anchor[:3]
                        )
                        probe_target_rotation_delta_deg = np.rad2deg(
                            quaternion_to_rotation_vector(
                                matrix_to_quaternion(
                                    quaternion_to_matrix(probe_pose_anchor[3:]).T
                                    @ quaternion_to_matrix(target_pose[3:])
                                )
                            )
                        )
                    else:
                        haptic_delta_mm = np.zeros(3)
                        haptic_rotation_delta_deg = np.zeros(3)
                        target_delta_mm = np.zeros(3)
                        probe_target_rotation_delta_deg = np.zeros(3)
                    print(
                        "mode={} ready={} clutch_pressed={} requested_enabled={} "
                        "rt_enabled={} force_active={} fixed_point_recovery={} "
                        "haptic_xyz_mm=[{:+.1f},{:+.1f},{:+.1f}] "
                        "haptic_local_rot_deg=[{:+.1f},{:+.1f},{:+.1f}] "
                        "target_xyz_mm=[{:+.1f},{:+.1f},{:+.1f}] "
                        "target_local_rot_deg=[{:+.1f},{:+.1f},{:+.1f}] "
                        "arm_residual_rot_deg=[{:+.1f},{:+.1f},{:+.1f}] "
                        "q8/q9_deg=[{:+.1f}, {:+.1f}] "
                        "target_deg=[{:+.1f}, {:+.1f}] position_error_mm={:.2f} "
                        "orientation_error_deg={:.2f} probe_Fz_N={:+.2f} "
                        "force_error_N={:+.2f} overload_N={:.2f} "
                        "singularity_sigma={:.4f} arm_speed_scale={:.2f} "
                        "passivity_limited={} tank_energy_J={:.4f}".format(
                            selected_mode.value if selected_mode else "waiting",
                            int(selector.ready),
                            int(pressed),
                            int(enabled),
                            int(state.enabled),
                            int(state.force_control_active),
                            int(state.fixed_point_recovery_active),
                            *haptic_delta_mm,
                            *haptic_rotation_delta_deg,
                            *target_delta_mm,
                            *probe_target_rotation_delta_deg,
                            *np.rad2deg(state.arm_orientation_error_world),
                            *np.rad2deg(wrist_sample.q_rad),
                            *np.rad2deg(last_wrist_target),
                            1000.0 * state.position_error_m,
                            math.degrees(state.orientation_error_rad),
                            state.probe_force_z_n,
                            state.force_error_n,
                            overload,
                            state.arm_singularity_sigma_min,
                            state.arm_motion_scale,
                            int(feedback_passivity_limited),
                            feedback_tank_energy_j,
                        ),
                        flush=True,
                    )
                    next_status = now + 0.5
                was_enabled = enabled
                time.sleep(max(0.0, next_tick - time.monotonic()))

            if process.returncode not in (0, None):
                raise RuntimeError(f"实时 OSC 已退出，返回码 {process.returncode}")
            return 0
    finally:
        if haptic is not None:
            try:
                haptic.zero_force_feedback()
            except Exception:
                pass
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            deadline = time.monotonic() + 3.0
            while process.poll() is None and time.monotonic() < deadline:
                try:
                    if wrist is not None:
                        sample = wrist.latest()
                        wrist.command_hybrid(sample.q_rad, np.zeros(2))
                except Exception:
                    pass
                time.sleep(period)
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=2.0)
        if wrist_mapping is not None:
            wrist_mapping.close()
        if teleop_mapping is not None:
            teleop_mapping.close()
        wrist_file.close()
        teleop_file.close()
        wrist_path.unlink(missing_ok=True)
        teleop_path.unlink(missing_ok=True)
        model_path.unlink(missing_ok=True)
