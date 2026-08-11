import ast
from pathlib import Path

from scripts.run_9dof_teleop import WristBridge


def test_wait_first_sample_retries_ready_sample_race() -> None:
    bridge = WristBridge.__new__(WristBridge)
    expected = object()
    attempts = 0

    def latest():
        nonlocal attempts
        attempts += 1
        if attempts < 3:
            raise RuntimeError("腕部尚无状态数据")
        return expected

    bridge.latest = latest
    assert bridge.wait_first_sample(0.1) is expected
    assert attempts == 3


def test_wait_first_sample_does_not_hide_bridge_failure() -> None:
    bridge = WristBridge.__new__(WristBridge)

    def latest():
        raise RuntimeError("腕部 bridge 已退出，返回码 1")

    bridge.latest = latest
    try:
        bridge.wait_first_sample(0.1)
    except RuntimeError as error:
        assert "bridge 已退出" in str(error)
    else:
        raise AssertionError("bridge failure was incorrectly retried")


def test_every_wrist_position_command_ignores_stale_firmware_bounds() -> None:
    """A newly captured application zero must not be clamped to an old raw bound."""
    source_path = Path(__file__).parents[1] / "scripts" / "wrist_moteus_bridge.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    calls = [
        node
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Attribute)
        and node.func.attr == "set_position"
    ]
    assert calls
    for call in calls:
        keywords = {item.arg: item.value for item in call.keywords}
        assert "ignore_position_bounds" in keywords
        assert isinstance(keywords["ignore_position_bounds"], ast.Constant)
        assert keywords["ignore_position_bounds"].value == 1
        assert "kp_scale" in keywords
        assert "kd_scale" in keywords
