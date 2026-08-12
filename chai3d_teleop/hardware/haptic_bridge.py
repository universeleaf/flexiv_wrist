#!/usr/bin/env python3
"""CHAI3D-to-Flexiv Cartesian teleoperation, dry-run unless explicitly armed."""

from __future__ import annotations

import argparse
from collections import deque
from dataclasses import dataclass
import ipaddress
import os
from pathlib import Path
import re
import subprocess
import sys
import threading
import time
import tomllib
from typing import TextIO

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from controllers.teleop import (  # noqa: E402
    HapticSample,
    MappingConfig,
    RelativePoseMapper,
    add_velocity_damping,
    map_feedback_force,
    map_progressive_feedback_force,
    parse_axis_map,
    parse_sample,
    quaternion_angular_distance,
    quaternion_to_rotation_vector,
)


@dataclass(frozen=True)
class DeviceCapabilities:
    actuated_force: bool
    actuated_torque: bool
    max_force_n: float
    max_torque_nm: float


def parse_device_capabilities(message: str) -> DeviceCapabilities:
    match = re.search(
        r"force=(\d) torque=(\d) max_force_N=([^ ]+) max_torque_Nm=([^ ]+)",
        message,
    )
    if not match:
        raise ValueError("无法解析 CHAI3D 设备能力信息")
    return DeviceCapabilities(
        actuated_force=bool(int(match.group(1))),
        actuated_torque=bool(int(match.group(2))),
        max_force_n=float(match.group(3)),
        max_torque_nm=float(match.group(4)),
    )


class BridgeReader:
    def __init__(
        self,
        executable: Path,
        device: int,
        rate_hz: int,
        calibrate_device: bool = False,
        feedback_watchdog_ms: int = 100,
        force_slew_rate_n_per_s: float = 0.0,
        teleop_damping_n_per_mps: float = 0.0,
        gravity_compensation: bool = True,
        hold_switch: int = 0,
        hold_when_released: bool = False,
        startup_center: bool = False,
        hold_stiffness_n_per_m: float = 150.0,
        hold_damping_n_per_mps: float = 5.0,
        hold_max_force_n: float = 4.0,
        hold_slew_rate_n_per_s: float = 20.0,
    ):
        if not executable.is_file() or not os.access(executable, os.X_OK):
            raise FileNotFoundError(f"CHAI3D bridge 不存在或不可执行: {executable}")
        command = [
            str(executable),
            "--device",
            str(device),
            "--rate",
            str(rate_hz),
            "--feedback-watchdog-ms",
            str(feedback_watchdog_ms),
            "--force-slew-rate",
            str(force_slew_rate_n_per_s),
            "--teleop-damping",
            str(teleop_damping_n_per_mps),
            (
                "--gravity-compensation"
                if gravity_compensation
                else "--no-gravity-compensation"
            ),
            "--hold-switch",
            str(hold_switch),
            "--hold-stiffness",
            str(hold_stiffness_n_per_m),
            "--hold-damping",
            str(hold_damping_n_per_mps),
            "--hold-max-force",
            str(hold_max_force_n),
            "--hold-slew-rate",
            str(hold_slew_rate_n_per_s),
        ]
        if hold_when_released:
            command.append("--hold-when-released")
        if startup_center:
            command.append("--startup-center")
        if calibrate_device:
            command.append("--calibrate")
        self._process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self._lock = threading.Lock()
        self._samples: deque[tuple[float, HapticSample]] = deque(maxlen=1)
        self._error: BaseException | None = None
        self._stdin_lock = threading.Lock()
        self._ready = threading.Event()
        self._capabilities: DeviceCapabilities | None = None
        self._stdout_thread = threading.Thread(target=self._read_stdout, daemon=True)
        self._stderr_thread = threading.Thread(target=self._copy_stderr, daemon=True)
        self._stdout_thread.start()
        self._stderr_thread.start()

    def _read_stdout(self) -> None:
        assert self._process.stdout is not None
        try:
            for line in self._process.stdout:
                sample = parse_sample(line)
                with self._lock:
                    self._samples.append((time.monotonic(), sample))
        except BaseException as error:
            self._error = error

    def _copy_stderr(self) -> None:
        assert self._process.stderr is not None
        for line in self._process.stderr:
            message = line.rstrip()
            if message.startswith("CHAI3D_READY"):
                try:
                    self._capabilities = parse_device_capabilities(message)
                except ValueError:
                    self._capabilities = None
                self._ready.set()
            print(f"[chai3d] {message}", file=sys.stderr)

    def capabilities(self, timeout: float = 5.0) -> DeviceCapabilities:
        if not self._ready.wait(timeout):
            raise TimeoutError("5 秒内没有收到 CHAI3D 设备能力信息")
        if self._capabilities is None:
            raise RuntimeError("无法解析 CHAI3D 设备能力信息；请重新编译 bridge")
        return self._capabilities

    def send_force_feedback(self, force_device_n: np.ndarray) -> None:
        force = np.asarray(force_device_n, dtype=float)
        if force.shape != (3,) or not np.all(np.isfinite(force)):
            raise ValueError("触觉反馈力必须是有限的 3 维数组")
        if self._process.poll() is not None or self._process.stdin is None:
            raise RuntimeError("CHAI3D bridge 已退出，无法发送力反馈")
        command = "F " + " ".join(f"{value:.9g}" for value in force) + "\n"
        with self._stdin_lock:
            try:
                self._process.stdin.write(command)
                self._process.stdin.flush()
            except (BrokenPipeError, OSError) as error:
                raise RuntimeError("向 CHAI3D bridge 发送力反馈失败") from error

    def zero_force_feedback(self) -> None:
        if self._process.poll() is not None or self._process.stdin is None:
            raise RuntimeError("CHAI3D bridge 已退出，无法归零力反馈")
        with self._stdin_lock:
            try:
                self._process.stdin.write("Z\n")
                self._process.stdin.flush()
            except (BrokenPipeError, OSError) as error:
                raise RuntimeError("向 CHAI3D bridge 发送反馈归零命令失败") from error

    def latest(self) -> tuple[float, HapticSample] | None:
        if self._error is not None:
            raise RuntimeError(f"读取 CHAI3D 数据失败: {self._error}") from self._error
        if self._process.poll() is not None:
            raise RuntimeError(f"CHAI3D bridge 已退出，返回码 {self._process.returncode}")
        with self._lock:
            return self._samples[-1] if self._samples else None

    def close(self) -> None:
        if self._process.poll() is None:
            try:
                self.zero_force_feedback()
            except Exception:
                pass
            self._process.terminate()
            try:
                self._process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._process.kill()
                self._process.wait(timeout=2.0)

    def __enter__(self) -> "BridgeReader":
        return self

    def __exit__(self, *_: object) -> None:
        self.close()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="CHAI3D -> Flexiv 笛卡尔相对位姿遥操作（默认只读 dry-run）"
    )
    parser.add_argument("robot_sn", nargs="?", help="Flexiv 序列号，如 Rizon4s-062232")
    parser.add_argument(
        "--network-interface-ip",
        default="192.168.2.27",
        help="连接 Flexiv 的本机网卡 IPv4 地址",
    )
    parser.add_argument(
        "--bridge",
        type=Path,
        default=PROJECT_ROOT / "build" / "chai3d_device_stream",
        help="chai3d_device_stream 可执行文件路径",
    )
    parser.add_argument("--device", type=int, default=0, help="CHAI3D 设备编号")
    parser.add_argument("--switch", type=int, default=0, help="dead-man/clutch 按钮编号")
    parser.add_argument(
        "--calibrate-device",
        action="store_true",
        help="仅在 dry-run 中：强制重新执行自动初始化（设备会运动）；默认也会在每次上电后按需初始化",
    )
    parser.add_argument("--device-rate", type=int, default=1000, help="设备读取/力输出频率 Hz")
    parser.add_argument(
        "--gravity-compensation",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="显式启用/关闭 Force Dimension DHD 重力补偿",
    )
    parser.add_argument(
        "--hold-when-released",
        action="store_true",
        help="clutch 松开时用 Force Dimension DRD 原生位置控制保持当前位置",
    )
    parser.add_argument(
        "--startup-center",
        action="store_true",
        help="首次 clutch 前把触觉设备缓慢拉向工作空间中心",
    )
    parser.add_argument(
        "--hold-stiffness",
        type=float,
        default=150.0,
        help="仅 startup-center 兼容模式使用的弹簧刚度 N/m",
    )
    parser.add_argument(
        "--hold-damping",
        type=float,
        default=5.0,
        help="仅 startup-center 兼容模式使用的阻尼 N/(m/s)",
    )
    parser.add_argument(
        "--hold-max-force",
        type=float,
        default=4.0,
        help="仅 startup-center 兼容模式使用的最大保持力 N",
    )
    parser.add_argument(
        "--hold-slew-rate",
        type=float,
        default=20.0,
        help="仅 startup-center 兼容模式使用的保持力变化率 N/s",
    )
    parser.add_argument("--command-rate", type=int, default=100, help="Flexiv 命令频率 Hz")
    parser.add_argument("--scale", type=float, default=2.0, help="平移比例")
    parser.add_argument(
        "--axis-map",
        default="-x,-y,z",
        help="设备轴到机器人 world 轴映射，例如 x,-z,y（必须为右手系）",
    )
    parser.add_argument("--max-translation", type=float, default=0.05, help="单次 clutch 最大平移 m")
    parser.add_argument(
        "--max-session-translation",
        type=float,
        default=0.01,
        help="整次真实运行相对启动 TCP 的最大平移 m",
    )
    parser.add_argument(
        "--unlimited-translation",
        action="store_true",
        help="取消单次 clutch 和整次运行的平移范围限制",
    )
    parser.add_argument("--max-step", type=float, default=0.001, help="每周期最大目标步长 m")
    parser.add_argument(
        "--unlimited-step",
        action="store_true",
        help="取消应用层每周期平移目标步长限制",
    )
    parser.add_argument("--enable-rotation", action="store_true", help="同时映射手柄姿态")
    parser.add_argument("--max-rotation-deg", type=float, default=20.0, help="单次 clutch 最大转角")
    parser.add_argument(
        "--max-session-rotation-deg",
        type=float,
        default=30.0,
        help="整次真实运行相对启动 TCP 的最大转角",
    )
    parser.add_argument(
        "--max-angular-step-deg",
        type=float,
        default=0.2,
        help="每个控制周期的最大姿态步长",
    )
    parser.add_argument(
        "--unlimited-rotation",
        action="store_true",
        help="取消单次和整次运行的角度范围限制；仍保留角速度限制",
    )
    parser.add_argument(
        "--unlimited-angular-step",
        action="store_true",
        help="取消应用层每周期姿态目标步长限制",
    )
    parser.add_argument(
        "--enable-force-feedback",
        action="store_true",
        help="把 Flexiv world-frame TCP 外力反馈到触觉设备（仅 clutch 按下时）",
    )
    parser.add_argument(
        "--force-feedback-gain",
        type=float,
        default=1.0,
        help="Flexiv TCP 外力到触觉设备输出力的比例",
    )
    parser.add_argument(
        "--force-feedback-deadband",
        type=float,
        default=0.0,
        help="去除传感器偏置后的外力死区 N",
    )
    parser.add_argument(
        "--max-device-force",
        type=float,
        default=12.0,
        help="触觉设备硬件输出上限 N（不能超过设备连续额定值）",
    )
    parser.add_argument(
        "--max-force-feedback-bias",
        type=float,
        default=5.0,
        help="允许开启力反馈的初始空载 TCP 外力 bias 最大模长 N",
    )
    parser.add_argument(
        "--force-feedback-source",
        choices=("filtered", "raw"),
        default="raw",
        help="反馈使用 filtered 或低延迟 raw Flexiv TCP 外力",
    )
    parser.add_argument(
        "--force-slew-rate",
        type=float,
        default=0.0,
        help="触觉力变化率 N/s；0 表示关闭应用层 slew limiter",
    )
    parser.add_argument(
        "--teleop-damping",
        type=float,
        default=0.0,
        help="clutch 按下时 omega.7 的本地速度阻尼 N/(m/s)，用于抑制双边振动",
    )
    parser.add_argument(
        "--overload-threshold",
        type=float,
        default=0.0,
        help="机器人侧过载触觉阈值 N；0 关闭渐进反弹",
    )
    parser.add_argument(
        "--overload-gain",
        type=float,
        default=0.0,
        help="超过阈值后每增加 1N 所增加的设备侧反弹力 N/N",
    )
    parser.add_argument(
        "--overload-full-damping-excess",
        type=float,
        default=5.0,
        help="超过阈值多少 N 时启用全部附加阻尼",
    )
    parser.add_argument(
        "--overload-damping",
        type=float,
        default=0.0,
        help="过载时额外的设备速度阻尼 N/(m/s)",
    )
    parser.add_argument(
        "--max-linear-velocity",
        type=float,
        default=0.5,
        help="Flexiv NRT 最大 TCP 线速度 m/s",
    )
    parser.add_argument(
        "--max-angular-velocity",
        type=float,
        default=1.0,
        help="Flexiv NRT 最大 TCP 角速度 rad/s",
    )
    parser.add_argument(
        "--max-linear-acceleration",
        type=float,
        default=2.0,
        help="Flexiv NRT 最大 TCP 线加速度 m/s^2",
    )
    parser.add_argument(
        "--max-angular-acceleration",
        type=float,
        default=5.0,
        help="Flexiv NRT 最大 TCP 角加速度 rad/s^2",
    )
    parser.add_argument("--watchdog-ms", type=float, default=100.0, help="设备数据超时停机阈值")
    parser.add_argument("--arm", action="store_true", help="允许连接并向真实 Flexiv 发送运动命令")
    parser.add_argument(
        "--confirm",
        default="",
        help="真实运动必须填写固定字符串 MOVE_RIZON",
    )
    parser.add_argument(
        "--confirm-force-feedback",
        default="",
        help="全幅力反馈必须填写固定字符串 FORCE_1_TO_1",
    )
    return parser


CONFIG_VALUE_OPTIONS = {
    "network_interface_ip": "--network-interface-ip",
    "device": "--device",
    "switch": "--switch",
    "device_rate": "--device-rate",
    "command_rate": "--command-rate",
    "scale": "--scale",
    "axis_map": "--axis-map",
    "max_linear_velocity": "--max-linear-velocity",
    "max_angular_velocity": "--max-angular-velocity",
    "max_linear_acceleration": "--max-linear-acceleration",
    "max_angular_acceleration": "--max-angular-acceleration",
    "force_feedback_source": "--force-feedback-source",
    "force_feedback_gain": "--force-feedback-gain",
    "force_feedback_deadband": "--force-feedback-deadband",
    "max_device_force": "--max-device-force",
    "max_force_feedback_bias": "--max-force-feedback-bias",
    "force_slew_rate": "--force-slew-rate",
    "teleop_damping": "--teleop-damping",
    "overload_threshold": "--overload-threshold",
    "overload_gain": "--overload-gain",
    "overload_full_damping_excess": "--overload-full-damping-excess",
    "overload_damping": "--overload-damping",
    "hold_stiffness": "--hold-stiffness",
    "hold_damping": "--hold-damping",
    "hold_max_force": "--hold-max-force",
    "hold_slew_rate": "--hold-slew-rate",
    "watchdog_ms": "--watchdog-ms",
    "confirm": "--confirm",
    "confirm_force_feedback": "--confirm-force-feedback",
}

CONFIG_BOOLEAN_OPTIONS = {
    "unlimited_translation": "--unlimited-translation",
    "unlimited_step": "--unlimited-step",
    "enable_rotation": "--enable-rotation",
    "unlimited_rotation": "--unlimited-rotation",
    "unlimited_angular_step": "--unlimited-angular-step",
    "enable_force_feedback": "--enable-force-feedback",
    "hold_when_released": "--hold-when-released",
    "startup_center": "--startup-center",
    "arm": "--arm",
}

CONFIG_NEGATABLE_BOOLEAN_OPTIONS = {
    "gravity_compensation": (
        "--gravity-compensation",
        "--no-gravity-compensation",
    ),
}


def config_to_argv(path: Path) -> list[str]:
    """Load a human-readable TOML profile and convert it to normal CLI arguments."""
    with path.open("rb") as stream:
        document = tomllib.load(stream)

    values: dict[str, object] = {}
    for section_name, section in document.items():
        if not isinstance(section, dict):
            raise ValueError(f"配置项 {section_name} 必须放在 TOML section 中")
        for key, value in section.items():
            if key in values:
                raise ValueError(f"配置项重复: {key}")
            values[key] = value

    robot_sn = values.pop("robot_sn", None)
    if not isinstance(robot_sn, str) or not robot_sn.strip():
        raise ValueError("配置文件必须提供非空 robot_sn")

    argv = [robot_sn]
    for key, value in values.items():
        if key in CONFIG_NEGATABLE_BOOLEAN_OPTIONS:
            if not isinstance(value, bool):
                raise ValueError(f"配置项 {key} 必须是 true 或 false")
            enabled_option, disabled_option = CONFIG_NEGATABLE_BOOLEAN_OPTIONS[key]
            argv.append(enabled_option if value else disabled_option)
            continue
        if key in CONFIG_BOOLEAN_OPTIONS:
            if not isinstance(value, bool):
                raise ValueError(f"配置项 {key} 必须是 true 或 false")
            if value:
                argv.append(CONFIG_BOOLEAN_OPTIONS[key])
            continue
        option = CONFIG_VALUE_OPTIONS.get(key)
        if option is None:
            raise ValueError(f"未知配置项: {key}")
        if isinstance(value, (dict, list, bool)):
            raise ValueError(f"配置项 {key} 必须是字符串或数值")
        argv.append(f"{option}={value}")
    return argv


def validate_args(args: argparse.Namespace) -> None:
    try:
        interface_ip = ipaddress.ip_address(args.network_interface_ip)
    except ValueError as error:
        raise ValueError("--network-interface-ip 必须是有效 IPv4 地址") from error
    if interface_ip.version != 4 or interface_ip.is_unspecified:
        raise ValueError("--network-interface-ip 必须是可用的 IPv4 地址")
    if args.device < 0 or args.switch < 0:
        raise ValueError("设备编号和按钮编号不能为负数")
    if not 10 <= args.device_rate <= 2000:
        raise ValueError("--device-rate 必须在 10..2000 Hz")
    if not 1 <= args.command_rate <= 100:
        raise ValueError("--command-rate 必须在 1..100 Hz")
    for name in (
        "scale",
        "max_translation",
        "max_session_translation",
        "max_step",
        "max_rotation_deg",
        "max_session_rotation_deg",
        "max_angular_step_deg",
        "force_feedback_gain",
        "max_device_force",
        "hold_stiffness",
        "hold_max_force",
        "max_linear_velocity",
        "max_angular_velocity",
        "max_linear_acceleration",
        "max_angular_acceleration",
        "watchdog_ms",
    ):
        if getattr(args, name) <= 0.0:
            raise ValueError(f"--{name.replace('_', '-')} 必须大于 0")
    if args.force_feedback_deadband < 0.0:
        raise ValueError("--force-feedback-deadband 不能为负数")
    if args.max_force_feedback_bias < 0.0:
        raise ValueError("--max-force-feedback-bias 不能为负数；0 表示只警告不退出")
    if args.force_slew_rate < 0.0:
        raise ValueError("--force-slew-rate 不能为负数")
    if args.teleop_damping < 0.0:
        raise ValueError("--teleop-damping 不能为负数")
    if args.overload_threshold < 0.0:
        raise ValueError("--overload-threshold 不能为负数")
    if args.overload_gain < 0.0:
        raise ValueError("--overload-gain 不能为负数")
    if args.overload_damping < 0.0:
        raise ValueError("--overload-damping 不能为负数")
    if args.overload_full_damping_excess <= 0.0:
        raise ValueError("--overload-full-damping-excess 必须大于 0")
    if args.hold_damping < 0.0:
        raise ValueError("--hold-damping 不能为负数")
    if args.hold_slew_rate < 0.0:
        raise ValueError("--hold-slew-rate 不能为负数")
    if args.startup_center and not args.hold_when_released:
        raise ValueError("--startup-center 必须与 --hold-when-released 一起使用")
    if args.arm and not args.robot_sn:
        raise ValueError("--arm 时必须提供 robot_sn")
    if args.arm and args.confirm != "MOVE_RIZON":
        raise ValueError("真实运动必须同时使用 --confirm MOVE_RIZON")
    if args.arm and args.calibrate_device:
        raise ValueError("不能在真实机器人模式中使用 --calibrate-device；请先单独 dry-run 初始化")
    if args.enable_force_feedback and not args.arm:
        raise ValueError("--enable-force-feedback 只能与 --arm 一起使用")
    if (
        args.enable_force_feedback
        and args.confirm_force_feedback != "FORCE_1_TO_1"
    ):
        raise ValueError(
            "启用 1:1/raw 力反馈必须同时使用 "
            "--confirm-force-feedback FORCE_1_TO_1"
        )


def wait_for_first_sample(reader: BridgeReader, timeout: float = 60.0) -> HapticSample:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        latest = reader.latest()
        if latest is not None:
            return latest[1]
        time.sleep(0.01)
    raise TimeoutError(f"{timeout:g} 秒内没有收到 CHAI3D 数据")


def ensure_robot_ready(robot: object) -> None:
    if not robot.connected():
        raise RuntimeError("Flexiv 未连接")
    if not robot.estop_released():
        raise RuntimeError("急停未释放")
    if robot.fault():
        raise RuntimeError("机器人存在 fault；本程序不会自动清 fault")
    if not robot.operational():
        raise RuntimeError(
            f"机器人未 operational（{robot.operational_status()}）；"
            "请在 Flexiv UI 中检查急停并人工 Enable"
        )


def run_dry(reader: BridgeReader, mapper: RelativePoseMapper, args: argparse.Namespace) -> int:
    current_pose = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    pressed_last = False
    next_print = 0.0
    clutch_anchor: np.ndarray | None = None
    motion_window_started = time.monotonic()
    motion_window_min: np.ndarray | None = None
    motion_window_max: np.ndarray | None = None
    print("DRY-RUN：不会连接或移动 Flexiv。输出单位为 mm。")
    print("先松开再按住 gripper；移动手柄时观察 raw 和 delta 是否变化。")
    while True:
        received_at, sample = reader.latest() or (0.0, wait_for_first_sample(reader))
        age_ms = (time.monotonic() - received_at) * 1000.0 if received_at else 0.0
        if received_at and age_ms > args.watchdog_ms:
            raise RuntimeError("CHAI3D 数据超时")
        if motion_window_min is None:
            motion_window_min = sample.position.copy()
            motion_window_max = sample.position.copy()
        else:
            motion_window_min = np.minimum(motion_window_min, sample.position)
            motion_window_max = np.maximum(motion_window_max, sample.position)
        pressed = sample.switch_pressed(args.switch)
        if pressed and not pressed_last:
            mapper.capture(sample, current_pose)
            clutch_anchor = sample.position.copy()
            print("clutch 按下：已捕获相对零点")
        if pressed:
            current_pose = mapper.target(sample)
        if pressed_last and not pressed:
            print("clutch 松开：目标冻结")
        if time.monotonic() >= next_print:
            raw_mm = sample.position * 1000.0
            delta_mm = (
                (sample.position - clutch_anchor) * 1000.0
                if clutch_anchor is not None
                else np.zeros(3)
            )
            target_mm = current_pose[:3] * 1000.0
            target_rotvec_deg = np.rad2deg(
                quaternion_to_rotation_vector(current_pose[3:])
            )
            print(
                "pressed={} mask=0x{:x} age={:5.1f}ms raw_mm=[{}] "
                "delta_mm=[{}] target_mm=[{}] target_rotvec_deg=[{}]".format(
                    int(pressed),
                    sample.switches,
                    age_ms,
                    ", ".join(f"{value:+8.3f}" for value in raw_mm),
                    ", ".join(f"{value:+8.3f}" for value in delta_mm),
                    ", ".join(f"{value:+8.3f}" for value in target_mm),
                    ", ".join(f"{value:+7.2f}" for value in target_rotvec_deg),
                )
            )
            next_print = time.monotonic() + 0.1
        now = time.monotonic()
        if now - motion_window_started >= 2.0:
            assert motion_window_min is not None and motion_window_max is not None
            observed_span_mm = float(np.linalg.norm(motion_window_max - motion_window_min) * 1000.0)
            if observed_span_mm < 0.05:
                print(
                    "警告：最近 2 秒原始位置变化小于 0.05 mm。若你确实移动了手柄，"
                    "请停止并检查设备初始化/校准或是否被其他程序占用。",
                    file=sys.stderr,
                )
            motion_window_started = now
            motion_window_min = sample.position.copy()
            motion_window_max = sample.position.copy()
        pressed_last = pressed
        time.sleep(1.0 / args.command_rate)


def run_robot(reader: BridgeReader, mapper: RelativePoseMapper, args: argparse.Namespace) -> int:
    import flexivrdk  # Imported only after explicit --arm confirmation.

    first_sample = wait_for_first_sample(reader)
    if first_sample.switch_pressed(args.switch):
        raise RuntimeError("启动真实运动前必须先松开 dead-man/clutch 按钮")
    if args.enable_force_feedback:
        capabilities = reader.capabilities()
        if not capabilities.actuated_force:
            raise RuntimeError("当前 CHAI3D 设备不支持力输出")
        if args.max_device_force > capabilities.max_force_n:
            raise ValueError(
                f"--max-device-force {args.max_device_force:.2f} N 超过设备连续额定值 "
                f"{capabilities.max_force_n:.2f} N"
            )
        print(
            f"触觉力反馈已准备：上限 {args.max_device_force:.2f} N，"
            f"设备连续额定值 {capabilities.max_force_n:.2f} N"
        )

    robot = flexivrdk.Robot(args.robot_sn, [args.network_interface_ip])
    robot_in_motion_mode = False
    try:
        ensure_robot_ready(robot)
        try:
            active_tool = flexivrdk.Tool(robot)
            active_tool_name = str(active_tool.name())
            active_tool_params = active_tool.params()
            print(
                "Flexiv active Tool: name={} mass_kg={:.4f} CoM_m=[{}] "
                "inertia=[{}]".format(
                    active_tool_name,
                    float(active_tool_params.mass),
                    ", ".join(f"{float(value):+.4f}" for value in active_tool_params.CoM),
                    ", ".join(
                        f"{float(value):+.6f}" for value in active_tool_params.inertia
                    ),
                )
            )
            if args.enable_force_feedback and active_tool_name == "Flange":
                print(
                    "警告：当前 active Tool 是 Flange；只有物理末端确实为空法兰时才正确。"
                )
        except Exception as error:
            if args.enable_force_feedback:
                raise RuntimeError(
                    f"启用力反馈前无法读取 Flexiv active Tool 参数: {error}"
                ) from error
            print(f"警告：无法读取 Flexiv active Tool 参数: {error}")
        print("真实运动将在 3 秒后进入笛卡尔模式；保持按钮松开并握住急停。")
        for remaining in (3, 2, 1):
            print(f"{remaining}...")
            time.sleep(1.0)
            latest = reader.latest()
            if latest is None or latest[1].switch_pressed(args.switch):
                raise RuntimeError("倒计时期间 CHAI3D 数据丢失或按钮被按下")
            ensure_robot_ready(robot)

        robot.SwitchMode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
        robot_in_motion_mode = True
        robot.SetForceControlAxis([False] * 6)
        initial_states = robot.states()
        hold_pose = np.asarray(initial_states.tcp_pose, dtype=float).copy()
        session_start_pose = hold_pose.copy()
        feedback_field = (
            "ext_wrench_in_world_raw"
            if args.force_feedback_source == "raw"
            else "ext_wrench_in_world"
        )
        feedback_bias_n = np.asarray(
            getattr(initial_states, feedback_field)[:3], dtype=float
        ).copy()
        if args.enable_force_feedback:
            if not np.all(np.isfinite(feedback_bias_n)):
                raise RuntimeError("Flexiv 初始 TCP 外力包含 NaN 或 Inf")
            feedback_bias_norm_n = float(np.linalg.norm(feedback_bias_n))
            print(
                "已记录空载 TCP 外力 bias [N]: ["
                + ", ".join(f"{value:+.2f}" for value in feedback_bias_n)
                + f"] norm={feedback_bias_norm_n:.2f} N"
            )
            if (
                args.max_force_feedback_bias > 0.0
                and feedback_bias_norm_n > args.max_force_feedback_bias
            ):
                raise RuntimeError(
                    "初始空载 TCP 外力 bias "
                    f"{feedback_bias_norm_n:.2f} N 超过允许值 "
                    f"{args.max_force_feedback_bias:.2f} N；程序不会用一次性 tare "
                    "掩盖错误载荷。请核对 Elements active Tool、payload/TCP、"
                    "腕部固定姿态和力/力矩传感器零点。"
                )
            if (
                args.max_force_feedback_bias == 0.0
                and feedback_bias_norm_n > 5.0
            ):
                print(
                    "警告：初始外力 bias 超过 5 N，但当前配置按要求不会因力值退出；"
                    "错误 payload 仍可能造成错误反馈或振动。",
                    file=sys.stderr,
                )
        robot.SendCartesianMotionForce(hold_pose.tolist())
        pressed_last = False
        period = 1.0 / args.command_rate
        next_tick = time.monotonic()
        next_feedback_print = 0.0
        last_haptic_position = first_sample.position.copy()
        last_haptic_velocity_time = time.monotonic()
        filtered_haptic_velocity = np.zeros(3)
        while True:
            next_tick += period
            ensure_robot_ready(robot)
            latest = reader.latest()
            if latest is None:
                raise RuntimeError("CHAI3D 数据尚未就绪")
            received_at, sample = latest
            age_ms = (time.monotonic() - received_at) * 1000.0
            if age_ms > args.watchdog_ms:
                raise RuntimeError(f"CHAI3D watchdog 超时: {age_ms:.1f} ms")

            states = robot.states()
            pressed = sample.switch_pressed(args.switch)
            velocity_now = time.monotonic()
            velocity_dt = velocity_now - last_haptic_velocity_time
            if velocity_dt > 1e-4:
                raw_haptic_velocity = (
                    sample.position - last_haptic_position
                ) / velocity_dt
                velocity_alpha = velocity_dt / (0.03 + velocity_dt)
                filtered_haptic_velocity += velocity_alpha * (
                    raw_haptic_velocity - filtered_haptic_velocity
                )
                last_haptic_position = sample.position.copy()
                last_haptic_velocity_time = velocity_now
            if pressed and not pressed_last:
                hold_pose = np.asarray(states.tcp_pose, dtype=float).copy()
                mapper.capture(sample, hold_pose)
                print("clutch 按下：开始相对遥操作")
            if pressed:
                hold_pose = mapper.target(sample)
            elif pressed_last:
                hold_pose = np.asarray(states.tcp_pose, dtype=float).copy()
                print("clutch 松开：保持当前 TCP")

            if args.enable_force_feedback:
                external_force_n = np.asarray(
                    getattr(states, feedback_field)[:3], dtype=float
                )
                overload_excess_n = 0.0
                if pressed and args.overload_threshold > 0.0:
                    device_force_n, overload_excess_n = map_progressive_feedback_force(
                        external_force_n,
                        feedback_bias_n,
                        mapper.axis_map,
                        base_gain=args.force_feedback_gain,
                        force_deadband_n=args.force_feedback_deadband,
                        overload_threshold_n=args.overload_threshold,
                        overload_gain=args.overload_gain,
                        max_device_force_n=args.max_device_force,
                    )
                    device_force_n = add_velocity_damping(
                        device_force_n,
                        filtered_haptic_velocity,
                        damping_n_per_m_s=args.overload_damping,
                        activation=float(
                            np.clip(
                                overload_excess_n
                                / args.overload_full_damping_excess,
                                0.0,
                                1.0,
                            )
                        ),
                        max_device_force_n=args.max_device_force,
                    )
                elif pressed:
                    device_force_n = map_feedback_force(
                        external_force_n,
                        feedback_bias_n,
                        mapper.axis_map,
                        force_gain=args.force_feedback_gain,
                        force_deadband_n=args.force_feedback_deadband,
                        max_device_force_n=args.max_device_force,
                    )
                else:
                    device_force_n = np.zeros(3)
                if pressed:
                    reader.send_force_feedback(device_force_n)
                else:
                    reader.zero_force_feedback()
                if pressed and time.monotonic() >= next_feedback_print:
                    tcp_moment_nm = np.asarray(
                        states.ext_wrench_in_world[3:], dtype=float
                    )
                    print(
                        "feedback ext_force_N=[{}] device_force_N=[{}] "
                        "tcp_moment_Nm=[{}] overload_N={:.2f}".format(
                            ", ".join(f"{value:+.2f}" for value in external_force_n),
                            ", ".join(f"{value:+.2f}" for value in device_force_n),
                            ", ".join(f"{value:+.2f}" for value in tcp_moment_nm),
                            overload_excess_n,
                        )
                    )
                    next_feedback_print = time.monotonic() + 0.5

            session_translation = float(
                np.linalg.norm(hold_pose[:3] - session_start_pose[:3])
            )
            if (
                not args.unlimited_translation
                and session_translation > args.max_session_translation
            ):
                raise RuntimeError(
                    f"整次运行累计平移 {session_translation:.4f} m 超过总限制 "
                    f"{args.max_session_translation:.4f} m"
                )
            session_rotation_deg = np.rad2deg(
                quaternion_angular_distance(session_start_pose[3:], hold_pose[3:])
            )
            if (
                not args.unlimited_rotation
                and session_rotation_deg > args.max_session_rotation_deg
            ):
                raise RuntimeError(
                    f"整次运行累计转角 {session_rotation_deg:.1f}° 超过总限制 "
                    f"{args.max_session_rotation_deg:.1f}°"
                )

            robot.SendCartesianMotionForce(
                hold_pose.tolist(),
                max_linear_vel=args.max_linear_velocity,
                max_angular_vel=args.max_angular_velocity,
                max_linear_acc=args.max_linear_acceleration,
                max_angular_acc=args.max_angular_acceleration,
            )
            pressed_last = pressed
            time.sleep(max(0.0, next_tick - time.monotonic()))
    finally:
        if args.enable_force_feedback:
            try:
                reader.zero_force_feedback()
            except Exception as error:
                print(f"警告：触觉反馈归零失败: {error}", file=sys.stderr)
        if robot_in_motion_mode:
            try:
                robot.Stop()
            except Exception as error:  # Keep the original safety fault visible.
                print(f"警告：Robot.Stop() 失败: {error}", file=sys.stderr)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        validate_args(args)
        mapper = RelativePoseMapper(
            MappingConfig(
                translation_scale=args.scale,
                max_translation_m=(
                    None if args.unlimited_translation else args.max_translation
                ),
                max_step_m=None if args.unlimited_step else args.max_step,
                enable_rotation=args.enable_rotation,
                max_rotation_rad=(
                    None
                    if args.unlimited_rotation
                    else np.deg2rad(args.max_rotation_deg)
                ),
                max_angular_step_rad=(
                    None
                    if args.unlimited_angular_step
                    else np.deg2rad(args.max_angular_step_deg)
                ),
            ),
            parse_axis_map(args.axis_map),
        )
        if args.calibrate_device:
            print(
                "警告：将请求 omega.7/CHAI3D 在未初始化时执行自动初始化；"
                "清空设备周围空间并松开手柄。",
                file=sys.stderr,
            )
        with BridgeReader(
            args.bridge.resolve(),
            args.device,
            args.device_rate,
            args.calibrate_device,
            feedback_watchdog_ms=max(1, round(args.watchdog_ms)),
            force_slew_rate_n_per_s=args.force_slew_rate,
            teleop_damping_n_per_mps=args.teleop_damping,
            gravity_compensation=args.gravity_compensation,
            hold_switch=args.switch,
            hold_when_released=args.hold_when_released,
            startup_center=args.startup_center,
            hold_stiffness_n_per_m=args.hold_stiffness,
            hold_damping_n_per_mps=args.hold_damping,
            hold_max_force_n=args.hold_max_force,
            hold_slew_rate_n_per_s=args.hold_slew_rate,
        ) as reader:
            return run_robot(reader, mapper, args) if args.arm else run_dry(reader, mapper, args)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C，已停止。", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"错误: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
