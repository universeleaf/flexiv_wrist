from __future__ import annotations

from scripts.teleop_control_ui import ANSI_ESCAPE_RE, _primary_error


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
