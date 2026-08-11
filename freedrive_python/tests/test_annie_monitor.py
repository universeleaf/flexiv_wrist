from __future__ import annotations

import math

from src.annie_monitor import (
    MonitorHub,
    RobotMonitorConfig,
    RobotMonitorWorker,
    local_ipv4_addresses,
    sample_from_states,
)


class FakeStates:
    timestamp = (1234, 5678)
    ext_wrench_in_tcp = [1, 2, 3, 0.1, 0.2, 0.3]
    ext_wrench_in_world = [4, 5, 6, 0.4, 0.5, 0.6]
    ft_sensor_raw = [7, 8, 9, 0.7, 0.8, 0.9]
    tcp_pose = [0.1, 0.2, 0.3, 1, 0, 0, 0]
    q = [0, 1, 2, 3, 4, 5, 6]


def test_sample_contains_all_three_wrenches_and_timestamps() -> None:
    sample = sample_from_states(FakeStates(), sequence=12, host_unix_ns=123_000_000)

    assert sample["sequence"] == 12
    assert sample["host_unix_ns"] == "123000000"
    assert sample["host_unix_ms"] == 123.0
    assert sample["robot_time"] == {"seconds": 1234, "nanoseconds": 5678}
    assert sample["wrench"]["tcp"] == [1.0, 2.0, 3.0, 0.1, 0.2, 0.3]
    assert sample["wrench"]["world"] == [4.0, 5.0, 6.0, 0.4, 0.5, 0.6]
    assert sample["wrench"]["sensor"] == [7.0, 8.0, 9.0, 0.7, 0.8, 0.9]
    assert math.isclose(sample["norm"]["tcp_force"], math.sqrt(14))


def test_nonfinite_or_short_values_are_json_safe_and_padded() -> None:
    states = FakeStates()
    states.ext_wrench_in_tcp = [float("nan"), float("inf"), "bad"]
    sample = sample_from_states(states, sequence=0, host_unix_ns=0)
    assert sample["wrench"]["tcp"] == [0.0] * 6


def test_hub_reports_sample_and_status_changes() -> None:
    hub = MonitorHub()
    hub.update_status(phase="streaming", connected=True)
    hub.publish_sample({"sequence": 4})
    sample, status = hub.wait_for_change(-1, -1, timeout=0)
    assert sample == {"sequence": 4}
    assert status["phase"] == "streaming"
    assert status["connected"] is True


def test_hub_can_identify_force_only_emj_monitor() -> None:
    _, status = MonitorHub("EmJ", "force-only").snapshot()
    assert status["project_label"] == "EmJ"
    assert status["recording_profile"] == "force-only"


def test_robot_constructor_is_normal_not_lite_because_states_are_required() -> None:
    calls: list[tuple[object, ...]] = []

    def factory(*args: object) -> object:
        calls.append(args)
        return object()

    worker = RobotMonitorWorker(
        MonitorHub(),
        RobotMonitorConfig(
            robot_sn="Rizon4s-123456",
            network_interface_ip="127.0.0.1",
            verbose_rdk=False,
        ),
        robot_factory=factory,
    )
    worker._connect()
    assert calls == [
        ("Rizon4s-123456", ["127.0.0.1"], False, False)
    ]


def test_local_ipv4_discovery_returns_a_set_even_if_sandboxed() -> None:
    assert isinstance(local_ipv4_addresses(), set)
