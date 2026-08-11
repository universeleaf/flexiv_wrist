from __future__ import annotations

from scripts.run_annie_monitor import compatible_existing_monitor, monitor_url


def test_monitor_url_uses_loopback_for_wildcard_bind() -> None:
    assert monitor_url("0.0.0.0", 8765) == "http://127.0.0.1:8765"
    assert monitor_url("127.0.0.1", 8766) == "http://127.0.0.1:8766"


def test_existing_real_monitor_is_reused_for_same_robot() -> None:
    status = {
        "read_only": True,
        "project_label": "Annie",
        "phase": "streaming",
        "robot_sn": "Rizon4s-123456",
        "recording_api_version": 5,
    }
    assert compatible_existing_monitor(
        status, demo=False, robot_sn="Rizon4s-123456"
    )


def test_demo_and_real_monitors_are_not_interchanged() -> None:
    demo_status = {
        "read_only": True,
        "project_label": "Annie",
        "phase": "demo",
        "robot_sn": "Rizon4s-123456",
        "recording_api_version": 5,
    }
    assert compatible_existing_monitor(
        demo_status, demo=True, robot_sn="Rizon4s-123456"
    )
    assert not compatible_existing_monitor(
        demo_status, demo=False, robot_sn="Rizon4s-123456"
    )


def test_different_robot_monitor_is_not_reused() -> None:
    status = {
        "read_only": True,
        "project_label": "Annie",
        "phase": "streaming",
        "robot_sn": "Rizon4s-999999",
        "recording_api_version": 5,
    }
    assert not compatible_existing_monitor(
        status, demo=False, robot_sn="Rizon4s-123456"
    )


def test_old_recording_api_monitor_is_not_reused() -> None:
    status = {
        "read_only": True,
        "project_label": "Annie",
        "phase": "streaming",
        "robot_sn": "Rizon4s-123456",
        "recording_api_version": 2,
    }
    assert not compatible_existing_monitor(
        status, demo=False, robot_sn="Rizon4s-123456"
    )


def test_annie_monitor_is_not_reused_for_emj() -> None:
    status = {
        "read_only": True,
        "project_label": "Annie",
        "recording_profile": "vision-force",
        "phase": "streaming",
        "robot_sn": "Rizon4s-123456",
        "recording_api_version": 5,
    }
    assert not compatible_existing_monitor(
        status,
        demo=False,
        robot_sn="Rizon4s-123456",
        project_label="EmJ",
        recording_profile="force-only",
    )
