"""Read-only Flexiv force/torque monitoring for the Annie camera UI.

This module deliberately exposes no robot command API.  It only constructs an
RDK Robot instance and calls state/status accessors so that a project can remain
under Flexiv Elements control.
"""

from __future__ import annotations

from dataclasses import dataclass
import fcntl
import math
import socket
import struct
import threading
import time
from typing import Any, Callable


def _numbers(value: Any, length: int) -> list[float]:
    """Convert an RDK fixed array/vector to finite JSON-safe floats."""
    result: list[float] = []
    try:
        values = list(value)
    except (TypeError, ValueError):
        values = []
    for item in values[:length]:
        try:
            number = float(item)
        except (TypeError, ValueError):
            number = 0.0
        result.append(number if math.isfinite(number) else 0.0)
    result.extend([0.0] * (length - len(result)))
    return result


def _robot_timestamp(value: Any) -> tuple[int, int]:
    try:
        seconds, nanoseconds = value
        return int(seconds), int(nanoseconds)
    except (TypeError, ValueError):
        return 0, 0


def _enum_name(value: Any) -> str:
    name = getattr(value, "name", None)
    return str(name if name is not None else value)


def local_ipv4_addresses() -> set[str]:
    """Return IPv4 addresses currently assigned to local Linux interfaces."""
    addresses: set[str] = set()
    try:
        interfaces = socket.if_nameindex()
    except OSError:
        return addresses
    for _, interface_name in interfaces:
        request = struct.pack("256s", interface_name.encode("utf-8")[:15])
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            response = fcntl.ioctl(sock.fileno(), 0x8915, request)  # SIOCGIFADDR
            addresses.add(socket.inet_ntoa(response[20:24]))
        except OSError:
            pass
        finally:
            sock.close()
    return addresses


def sample_from_states(states: Any, sequence: int, host_unix_ns: int) -> dict[str, Any]:
    """Build one synchronized, serializable sample from one RobotStates copy."""
    robot_seconds, robot_nanoseconds = _robot_timestamp(
        getattr(states, "timestamp", (0, 0))
    )
    tcp = _numbers(getattr(states, "ext_wrench_in_tcp", []), 6)
    world = _numbers(getattr(states, "ext_wrench_in_world", []), 6)
    sensor = _numbers(getattr(states, "ft_sensor_raw", []), 6)
    return {
        "type": "sample",
        "sequence": int(sequence),
        # Keep nanoseconds as a decimal string: JavaScript numbers cannot exactly
        # represent the current ~1.8e18 Unix-nanosecond value.
        "host_unix_ns": str(int(host_unix_ns)),
        "host_unix_ms": host_unix_ns / 1_000_000.0,
        "robot_time": {
            "seconds": robot_seconds,
            "nanoseconds": robot_nanoseconds,
        },
        "wrench": {
            "tcp": tcp,
            "world": world,
            "sensor": sensor,
        },
        "norm": {
            "tcp_force": math.hypot(*tcp[:3]),
            "tcp_moment": math.hypot(*tcp[3:]),
            "sensor_force": math.hypot(*sensor[:3]),
            "sensor_moment": math.hypot(*sensor[3:]),
        },
        "tcp_pose": _numbers(getattr(states, "tcp_pose", []), 7),
        "q": _numbers(getattr(states, "q", []), 7),
    }


class MonitorHub:
    """Thread-safe latest-sample/status exchange for one sampler and many UIs."""

    def __init__(
        self,
        project_label: str = "Annie",
        recording_profile: str = "vision-force",
    ) -> None:
        self._condition = threading.Condition()
        self._sample: dict[str, Any] | None = None
        self._status: dict[str, Any] = {
            "revision": 0,
            "phase": "starting",
            "connected": False,
            "fault": False,
            "operational": False,
            "busy": False,
            "mode": "unknown",
            "project_label": project_label,
            "read_only": True,
            "recording_api_version": 5,
            "recording_profile": recording_profile,
            "message": "Monitoring service is starting",
        }

    def publish_sample(self, sample: dict[str, Any]) -> None:
        with self._condition:
            self._sample = sample
            self._condition.notify_all()

    def update_status(self, **changes: Any) -> None:
        with self._condition:
            self._status.update(changes)
            self._status["revision"] = int(self._status["revision"]) + 1
            self._condition.notify_all()

    def snapshot(self) -> tuple[dict[str, Any] | None, dict[str, Any]]:
        with self._condition:
            sample = None if self._sample is None else dict(self._sample)
            return sample, dict(self._status)

    def wait_for_change(
        self, sample_sequence: int, status_revision: int, timeout: float = 1.0
    ) -> tuple[dict[str, Any] | None, dict[str, Any]]:
        def changed() -> bool:
            sequence = (
                -1
                if self._sample is None
                else int(self._sample.get("sequence", -1))
            )
            return sequence != sample_sequence or int(
                self._status.get("revision", -1)
            ) != status_revision

        with self._condition:
            self._condition.wait_for(changed, timeout=timeout)
            sample = None if self._sample is None else dict(self._sample)
            return sample, dict(self._status)


@dataclass(frozen=True)
class RobotMonitorConfig:
    robot_sn: str = "Rizon4s-123456"
    network_interface_ip: str = "127.0.0.1"
    sample_rate_hz: float = 50.0
    reconnect_delay_s: float = 3.0
    verbose_rdk: bool = True


class RobotMonitorWorker:
    """Background state sampler using only non-commanding RDK accessors."""

    def __init__(
        self,
        hub: MonitorHub,
        config: RobotMonitorConfig,
        *,
        robot_factory: Callable[..., Any] | None = None,
        monotonic: Callable[[], float] = time.monotonic,
        unix_ns: Callable[[], int] = time.time_ns,
    ) -> None:
        self.hub = hub
        self.config = config
        self._robot_factory = robot_factory
        self._monotonic = monotonic
        self._unix_ns = unix_ns
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(
            target=self._run, name="flexiv-read-only-monitor", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout)

    def _factory(self) -> Callable[..., Any]:
        if self._robot_factory is not None:
            return self._robot_factory
        import flexivrdk

        if str(flexivrdk.__version__) != "1.9.0":
            raise RuntimeError(
                "Robot Software 3.11 requires this project to use flexivrdk 1.9.0; "
                f"found {flexivrdk.__version__}"
            )
        return flexivrdk.Robot

    def _connect(self) -> Any:
        # lite=False is required: an RDK lite instance does not receive RobotStates.
        assigned_addresses = (
            local_ipv4_addresses() if self._robot_factory is None else set()
        )
        if (
            self._robot_factory is None
            and self.config.network_interface_ip not in assigned_addresses
        ):
            assigned = ", ".join(sorted(assigned_addresses)) or "none"
            raise ConnectionError(
                f"PC LAN1 address {self.config.network_interface_ip} is not assigned "
                f"(current local IPv4: {assigned}). Connect LAN1 and activate its "
                "NetworkManager profile before starting the monitor."
            )
        return self._factory()(
            self.config.robot_sn,
            [self.config.network_interface_ip],
            self.config.verbose_rdk,
            False,
        )

    def _status_from_robot(self, robot: Any) -> dict[str, Any]:
        changes: dict[str, Any] = {
            "connected": bool(robot.connected()),
            "fault": bool(robot.fault()),
            "operational": bool(robot.operational()),
            "busy": bool(robot.busy()),
            "mode": _enum_name(robot.mode()),
        }
        try:
            plan = robot.plan_info()
        except Exception:
            plan = None
        if plan is not None:
            changes["plan"] = {
                "assigned_plan_name": str(
                    getattr(plan, "assigned_plan_name", "")
                ),
                "node_name": str(getattr(plan, "node_name", "")),
                "node_path": str(getattr(plan, "node_path", "")),
            }
        return changes

    def _sample_connected_robot(self, robot: Any) -> None:
        info = robot.info()
        has_sensor = bool(getattr(info, "has_FT_sensor", False))
        self.hub.update_status(
            phase="streaming",
            connected=True,
            robot_sn=str(getattr(info, "serial_num", self.config.robot_sn)),
            model=str(getattr(info, "model_name", "unknown")),
            software=str(getattr(info, "software_ver", "unknown")),
            license=str(getattr(info, "license_type", "unknown")),
            has_ft_sensor=has_sensor,
            sample_rate_hz=self.config.sample_rate_hz,
            message=(
                "Reading robot states in read-only mode"
                if has_sensor
                else "The robot reports that no end-of-arm F/T sensor is installed"
            ),
        )
        if not has_sensor:
            raise RuntimeError("connected robot reports has_FT_sensor=False")

        interval = 1.0 / self.config.sample_rate_hz
        next_tick = self._monotonic()
        next_status = next_tick
        sequence = 0
        while not self._stop.is_set():
            if not robot.connected():
                raise ConnectionError("RDK connection was lost")
            states = robot.states()
            self.hub.publish_sample(
                sample_from_states(states, sequence, self._unix_ns())
            )
            sequence += 1

            now = self._monotonic()
            if now >= next_status:
                self.hub.update_status(**self._status_from_robot(robot))
                next_status = now + 0.5

            next_tick += interval
            delay = next_tick - self._monotonic()
            if delay <= -interval:
                next_tick = self._monotonic()
                delay = 0.0
            self._stop.wait(max(0.0, delay))

    def _run(self) -> None:
        while not self._stop.is_set():
            robot = None
            try:
                self.hub.update_status(
                    phase="connecting",
                    connected=False,
                    message=(
                        f"Connecting to {self.config.robot_sn} through "
                        f"{self.config.network_interface_ip}"
                    ),
                    error="",
                )
                robot = self._connect()
                self._sample_connected_robot(robot)
            except Exception as exc:
                if self._stop.is_set():
                    break
                self.hub.update_status(
                    phase="error",
                    connected=False,
                    message="Robot data connection failed; retrying automatically",
                    error=str(exc),
                )
                self._stop.wait(self.config.reconnect_delay_s)
            finally:
                robot = None
        self.hub.update_status(
            phase="stopped", connected=False, message="Monitoring service stopped"
        )


class DemoMonitorWorker:
    """Synthetic source for testing the complete UI without a robot or phone."""

    def __init__(self, hub: MonitorHub, sample_rate_hz: float = 50.0) -> None:
        self.hub = hub
        self.sample_rate_hz = sample_rate_hz
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self.hub.update_status(
            phase="demo",
            connected=True,
            fault=False,
            operational=True,
            busy=True,
            mode="DEMO",
            robot_sn="Rizon4s-123456",
            model="Rizon4s",
            software="v3.11 (demo)",
            has_ft_sensor=True,
            sample_rate_hz=self.sample_rate_hz,
            message="Demo mode: force data is synthetic",
        )
        self._thread = threading.Thread(
            target=self._run, name="annie-monitor-demo", daemon=True
        )
        self._thread.start()

    def stop(self, timeout: float = 2.0) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout)

    def _run(self) -> None:
        started = time.monotonic()
        interval = 1.0 / self.sample_rate_hz
        sequence = 0
        while not self._stop.is_set():
            t = time.monotonic() - started
            tcp = [
                5.0 * math.sin(t * 1.8),
                3.2 * math.sin(t * 1.2 + 0.8),
                7.0 + 2.5 * math.cos(t * 0.7),
                0.35 * math.sin(t * 1.1),
                0.22 * math.cos(t * 1.7),
                0.18 * math.sin(t * 0.9),
            ]
            sensor = [
                tcp[0] + 0.3 * math.sin(t * 13),
                tcp[1] + 0.2 * math.cos(t * 11),
                tcp[2] + 18.0,
                tcp[3],
                tcp[4],
                tcp[5],
            ]
            fake_states = type(
                "DemoStates",
                (),
                {
                    "timestamp": (int(time.time()), int(time.time_ns() % 1_000_000_000)),
                    "ext_wrench_in_tcp": tcp,
                    "ext_wrench_in_world": [tcp[1], -tcp[0], tcp[2], *tcp[3:]],
                    "ft_sensor_raw": sensor,
                    "tcp_pose": [0.42, -0.08, 0.52, 1.0, 0.0, 0.0, 0.0],
                    "q": [0.0] * 7,
                },
            )()
            self.hub.publish_sample(
                sample_from_states(fake_states, sequence, time.time_ns())
            )
            sequence += 1
            self._stop.wait(interval)
