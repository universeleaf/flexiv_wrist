"""Shared launcher/preflight helpers for the C++ real-time OSC demos."""

from __future__ import annotations

import csv
import math
import os
from pathlib import Path
import resource
import subprocess
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
BUILD_ROOT = PROJECT_ROOT / "build_rt_osc"
RDK_PREFIX = PROJECT_ROOT.parents[1] / "rdk-install"


def binary(name: str) -> Path:
    return BUILD_ROOT / name


def realtime_preflight(executable: Path, cpu_affinity: int) -> list[str]:
    messages: list[str] = []
    kernel = os.uname().release.lower()
    if "realtime" not in kernel and "rt" not in kernel:
        raise RuntimeError(f"当前内核不是 PREEMPT_RT/realtime: {os.uname().release}")
    messages.append(f"kernel={os.uname().release}")
    if cpu_affinity < 2 or cpu_affinity >= (os.cpu_count() or 1):
        raise RuntimeError("cpu_affinity 必须为 2..CPU数量-1（0/1 为 Flexiv 保留/不建议）")
    messages.append(f"cpu_affinity={cpu_affinity}")
    soft_rtprio, _ = resource.getrlimit(resource.RLIMIT_RTPRIO)
    # Flexiv Scheduler initializes its own RT service thread at the POSIX
    # maximum (99) before user tasks are added.  Merely allowing priority 90
    # or 95 is therefore insufficient even when the user task itself would
    # request a lower value.
    if os.geteuid() != 0 and soft_rtprio < 99:
        raise RuntimeError(
            f"当前登录会话 RLIMIT_RTPRIO={soft_rtprio}，但 Flexiv Scheduler 需要 99。"
            "按 docs/RUNBOOK_ZH_EN.md 把 realtime 组改为 rtprio 99，"
            "完全注销并重新登录后再运行"
        )
    messages.append(f"rtprio_soft={soft_rtprio}")
    if os.geteuid() != 0:
        probe = subprocess.run(
            ["chrt", "-f", "99", "true"],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        if probe.returncode != 0:
            details = probe.stderr.strip() or f"exit={probe.returncode}"
            raise RuntimeError(
                "当前会话不能实际创建 SCHED_FIFO 99 线程："
                f"{details}。完全注销桌面会话并重新登录；不要只关闭终端"
            )
        messages.append("sched_fifo_99=ok")
    if not executable.is_file() or not os.access(executable, os.X_OK):
        raise FileNotFoundError(
            f"实时程序尚未编译: {executable}\n先运行 python run.py build"
        )
    messages.append(f"binary={executable}")
    return messages


def write_python_dry_run(
    path: Path, duration_s: float, radius_m: float, orientation_deg: float
) -> None:
    if duration_s < 8.0 or not 0.0 < radius_m <= 0.05:
        raise ValueError("duration_s>=8 且 radius_m 在 (0,0.05] 内")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "x_m",
                "y_m",
                "z_m",
                "tilt_x_deg",
                "tilt_y_deg",
                "tilt_z_deg",
            ]
        )
        steps = int(duration_s / 0.01) + 1
        for index in range(steps):
            t = min(duration_s, index * 0.01)
            u = t / duration_s
            smooth = 10 * u**3 - 15 * u**4 + 6 * u**5
            phase = 2 * math.pi * smooth
            writer.writerow(
                [
                    t,
                    radius_m * (math.cos(phase) - 1),
                    0.65 * radius_m * math.sin(phase),
                    0.35 * radius_m * math.sin(2 * phase),
                    0.10 * orientation_deg * math.sin(phase),
                    orientation_deg * math.sin(2 * phase),
                    0.85 * orientation_deg * math.sin(3 * phase),
                ]
            )


def write_python_orientation_dry_run(
    path: Path, duration_s: float, orientation_deg: float
) -> None:
    """Write a closed orientation loop whose Cartesian point is exactly fixed."""
    if duration_s < 8.0 or not 0.0 < orientation_deg <= 45.0:
        raise ValueError("duration_s>=8 且 orientation_deg 在 (0,45] 内")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "x_m",
                "y_m",
                "z_m",
                "tilt_x_deg",
                "tilt_y_deg",
                "tilt_z_deg",
            ]
        )
        steps = int(math.ceil(duration_s / 0.01))
        for index in range(steps + 1):
            t = min(duration_s, index * 0.01)
            phase = 2.0 * math.pi * t / duration_s
            writer.writerow(
                [
                    t,
                    0.0,
                    0.0,
                    0.0,
                    0.10 * orientation_deg * math.sin(phase),
                    orientation_deg * math.sin(2.0 * phase),
                    0.85 * orientation_deg * math.sin(3.0 * phase),
                ]
            )


def write_python_spin_dry_run(
    path: Path, duration_s: float, tilt_amplitude_deg: float
) -> None:
    """Write a fixed-point 0->360->0 orientation cycle."""
    if duration_s < 16.0 or not 0.0 < tilt_amplitude_deg <= 60.0:
        raise ValueError("spin duration_s>=16 且 tilt amplitude 在 (0,60] 内")
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "x_m",
                "y_m",
                "z_m",
                "joint8_like_tilt_deg",
                "joint9_like_unwrapped_spin_deg",
            ]
        )
        steps = int(math.ceil(duration_s / 0.01))
        for index in range(steps + 1):
            t = min(duration_s, index * 0.01)
            phase = 2.0 * math.pi * t / duration_s
            writer.writerow(
                [
                    t,
                    0.0,
                    0.0,
                    0.0,
                    tilt_amplitude_deg * math.sin(phase),
                    180.0 * (1.0 - math.cos(phase)),
                ]
            )


def _rounded_rectangle_xy(
    arc_length: float, width_m: float, height_m: float, corner_radius_m: float
) -> tuple[float, float, float]:
    """Return x/y displacement from the start and unwrapped tangent heading."""
    sx = width_m - 2.0 * corner_radius_m
    sy = height_m - 2.0 * corner_radius_m
    arc = 0.5 * math.pi * corner_radius_m
    perimeter = 2.0 * sx + 2.0 * sy + 4.0 * arc
    s = arc_length % perimeter
    hw = 0.5 * width_m
    hh = 0.5 * height_m
    start_x, start_y = -hw + corner_radius_m, -hh

    if s < sx:
        x, y, heading = start_x + s, start_y, 0.0
    else:
        s -= sx
        if s < arc:
            angle = -0.5 * math.pi + s / corner_radius_m
            x = hw - corner_radius_m + corner_radius_m * math.cos(angle)
            y = -hh + corner_radius_m + corner_radius_m * math.sin(angle)
            heading = angle + 0.5 * math.pi
        else:
            s -= arc
            if s < sy:
                x, y, heading = hw, -hh + corner_radius_m + s, 0.5 * math.pi
            else:
                s -= sy
                if s < arc:
                    angle = s / corner_radius_m
                    x = hw - corner_radius_m + corner_radius_m * math.cos(angle)
                    y = hh - corner_radius_m + corner_radius_m * math.sin(angle)
                    heading = angle + 0.5 * math.pi
                else:
                    s -= arc
                    if s < sx:
                        x, y, heading = hw - corner_radius_m - s, hh, math.pi
                    else:
                        s -= sx
                        if s < arc:
                            angle = 0.5 * math.pi + s / corner_radius_m
                            x = -hw + corner_radius_m + corner_radius_m * math.cos(angle)
                            y = hh - corner_radius_m + corner_radius_m * math.sin(angle)
                            heading = angle + 0.5 * math.pi
                        else:
                            s -= arc
                            if s < sy:
                                x, y, heading = -hw, hh - corner_radius_m - s, 1.5 * math.pi
                            else:
                                s -= sy
                                angle = math.pi + s / corner_radius_m
                                x = -hw + corner_radius_m + corner_radius_m * math.cos(angle)
                                y = -hh + corner_radius_m + corner_radius_m * math.sin(angle)
                                heading = angle + 0.5 * math.pi
    return x - start_x, y - start_y, heading


def write_python_rectangle_dry_run(
    path: Path,
    duration_s: float,
    width_m: float,
    height_m: float,
    corner_radius_m: float,
) -> None:
    if duration_s < 8.0:
        raise ValueError("duration_s 必须至少为 8 秒")
    if (
        not 0.0 < width_m <= 0.15
        or not 0.0 < height_m <= 0.15
        or corner_radius_m <= 0.0
        or 2.0 * corner_radius_m >= min(width_m, height_m)
    ):
        raise ValueError("圆角矩形尺寸无效")
    sx = width_m - 2.0 * corner_radius_m
    sy = height_m - 2.0 * corner_radius_m
    perimeter = 2.0 * sx + 2.0 * sy + 2.0 * math.pi * corner_radius_m
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "x_m", "y_m", "z_m", "tangent_heading_deg"])
        steps = int(duration_s / 0.01) + 1
        for index in range(steps):
            t = min(duration_s, index * 0.01)
            x, y, heading = _rounded_rectangle_xy(
                perimeter * t / duration_s,
                width_m,
                height_m,
                corner_radius_m,
            )
            writer.writerow([t, x, y, 0.0, math.degrees(heading)])


def run_checked(command: list[str]) -> int:
    print("执行:", " ".join(command), flush=True)
    return subprocess.run(command, cwd=PROJECT_ROOT, check=False).returncode


def validate_active_flexiv_tool(profile) -> dict[str, object]:
    """Read-only validation of the Elements payload used by gravity compensation."""
    import flexivrdk
    import numpy as np

    config = profile.flexiv_tool
    if config.get("calibration_ready") is not True:
        raise RuntimeError(
            "flexiv_tool.calibration_ready=false：工具总成已改变。先在 Flexiv "
            "Elements 重新标定无探针腕部的 payload，再更新配置"
        )
    robot = flexivrdk.Robot(
        str(profile.robot["robot_sn"]),
        [str(profile.robot["network_interface_ip"])],
    )
    tool = flexivrdk.Tool(robot)
    name = str(tool.name())
    params = tool.params()
    expected_name = str(config["expected_name"])
    if name != expected_name:
        raise RuntimeError(f"active Tool 是 {name!r}，配置要求 {expected_name!r}")
    comparisons = (
        (
            "mass",
            np.asarray([float(params.mass)]),
            np.asarray([float(config["expected_mass_kg"])]),
            float(config["mass_tolerance_kg"]),
        ),
        (
            "CoM",
            np.asarray(params.CoM, dtype=float),
            np.asarray(config["expected_com_m"], dtype=float),
            float(config["com_tolerance_m"]),
        ),
        (
            "inertia",
            np.asarray(params.inertia, dtype=float),
            np.asarray(config["expected_inertia_kg_m2"], dtype=float),
            float(config["inertia_tolerance_kg_m2"]),
        ),
    )
    for label, actual, expected, tolerance in comparisons:
        error = float(np.max(np.abs(actual - expected)))
        if not np.all(np.isfinite(actual)) or error > tolerance:
            raise RuntimeError(
                f"active Tool {label} 与配置不一致：最大误差 {error:.6g}，"
                f"容差 {tolerance:.6g}"
            )
    inertia_values = np.asarray(params.inertia, dtype=float)
    ixx, iyy, izz, ixy, ixz, iyz = inertia_values
    inertia_matrix = np.asarray(
        [[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float
    )
    principal_inertia = np.linalg.eigvalsh(inertia_matrix)
    triangle_margin = float(
        min(
            principal_inertia[0] + principal_inertia[1] - principal_inertia[2],
            principal_inertia[0] + principal_inertia[2] - principal_inertia[1],
            principal_inertia[1] + principal_inertia[2] - principal_inertia[0],
        )
    )
    # Elements values are serialized with finite precision and can land a few
    # nanounits outside a triangle equality. Reject material violations, not
    # harmless calibration/serialization roundoff.
    if principal_inertia[0] <= 0.0 or triangle_margin < -1e-6:
        raise RuntimeError(
            "active Tool inertia 不满足正定性/主惯量三角不等式："
            f"eigen={principal_inertia.tolist()}"
        )
    actual_tcp = np.asarray(params.tcp_location, dtype=float)
    expected_tcp = np.asarray(config["expected_tcp_location"], dtype=float)
    if actual_tcp.shape != (7,) or not np.all(np.isfinite(actual_tcp)):
        raise RuntimeError("active Tool TCP 不是有限的 7 维 pose")
    tcp_position_error = float(np.linalg.norm(actual_tcp[:3] - expected_tcp[:3]))
    actual_quaternion = actual_tcp[3:]
    expected_quaternion = expected_tcp[3:]
    if np.linalg.norm(actual_quaternion) < 1e-9 or np.linalg.norm(expected_quaternion) < 1e-9:
        raise RuntimeError("active Tool 或配置中的 TCP 四元数无效")
    actual_quaternion /= np.linalg.norm(actual_quaternion)
    expected_quaternion /= np.linalg.norm(expected_quaternion)
    tcp_rotation_error = 2.0 * float(
        np.arccos(np.clip(abs(np.dot(actual_quaternion, expected_quaternion)), 0.0, 1.0))
    )
    if tcp_position_error > float(config["tcp_position_tolerance_m"]):
        raise RuntimeError(
            "active Tool TCP 位置与配置不一致："
            f"误差 {tcp_position_error * 1000.0:.2f} mm"
        )
    if tcp_rotation_error > np.deg2rad(float(config["tcp_rotation_tolerance_deg"])):
        raise RuntimeError(
            "active Tool TCP 姿态与配置不一致："
            f"误差 {np.rad2deg(tcp_rotation_error):.2f} deg"
        )
    print(
        "Flexiv Tool verified: name={} mass={:.3f}kg tcp=[{}] "
        "inertia_triangle_margin={:.3e}".format(
            name,
            float(params.mass),
            ", ".join(f"{value:+.4f}" for value in actual_tcp),
            triangle_margin,
        ),
        flush=True,
    )
    return {
        "name": name,
        "mass_kg": float(params.mass),
        "com_m": np.asarray(params.CoM, dtype=float).copy(),
        "inertia_kg_m2": np.asarray(params.inertia, dtype=float).copy(),
        "tcp_location": actual_tcp.copy(),
    }
