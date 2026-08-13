from __future__ import annotations

from ui.control_panel import (
    ANSI_ESCAPE_RE,
    INDEX_HTML,
    TeleopProcessManager,
    _primary_error,
)


def test_specific_wrist_error_is_not_hidden_by_exit_wrapper() -> None:
    specific = "[wrist] WRIST_ERROR RuntimeError: Permission denied"
    generic = "错误: 腕部 bridge 启动失败，返回码 1"
    assert _primary_error(specific, generic) == specific


def test_new_specific_error_replaces_an_old_generic_error() -> None:
    generic = "错误: 腕部 bridge 启动失败，返回码 1"
    specific = "[wrist] WRIST_ERROR TimeoutError: ID2 QUERY timeout"
    assert _primary_error(generic, specific) == specific


def test_ansi_sequences_are_removed_from_ui_log() -> None:
    colored = "[bridge] [\x1b[38;2;135;206;250mINFO\x1b[0m] stopped"
    assert ANSI_ESCAPE_RE.sub("", colored) == "[bridge] [INFO] stopped"


def test_control_panel_exposes_primary_missions_and_tools() -> None:
    for task in (
        "demo7", "demo9", "go-zero", "set-zero", "home",
        "identify-inertia", "dynamic-inertia", "apply-pid",
    ):
        assert f'data-task="{task}"' in INDEX_HTML


def test_control_panel_declares_large_force_and_all_tracking_plots() -> None:
    assert 'data-plot-count="14"' in INDEX_HTML
    assert "Mode 3 Tool-Z Force — Real-Time Measured vs Estimated" in INDEX_HTML
    assert "height:320" in INDEX_HTML
    assert "TCP Position — Actual vs Planned" in INDEX_HTML
    assert "TCP Orientation — Actual vs Planned" in INDEX_HTML
    assert "TCP Position Error" in INDEX_HTML
    assert "TCP Orientation Error" in INDEX_HTML
    assert "Array.from({length:9}" in INDEX_HTML


def test_process_manager_routes_structured_telemetry_out_of_live_log() -> None:
    manager = TeleopProcessManager()
    manager._append(
        'TELEMETRY {"timestamp_s":1.0,"mode":"9dof","enabled":true}'
    )
    status = manager.status()
    assert status["logs"] == []
    assert status["last_telemetry_sequence"] == 1
    assert status["telemetry"] == [
        {
            "sequence": 1,
            "sample": {"timestamp_s": 1.0, "mode": "9dof", "enabled": True},
        }
    ]
