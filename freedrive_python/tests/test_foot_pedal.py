"""Unit tests for foot-pedal YAML configuration and Pedal 2 toggle wiring."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from src.foot_pedal_configuration import (
    FootPedalConfiguration,
    PedalBinding,
    load_foot_pedal_configuration,
)
from src.foot_pedal_input import (
    FootPedalError,
    FootPedalReader,
    InputDeviceInfo,
    PedalEvent,
    StubPedalActionHandler,
    XInputFootPedalReader,
    _KeyDebouncer,
    find_foot_pedal_devices,
    parse_xinput_product_ids,
    parse_xinput_slave_keyboards,
)
from src.freedrive_configuration import ConfigurationError
from src.freedrive_state_machine import (
    Action,
    FreedriveState,
    FreedriveStateMachine,
    StateMachineInputs,
)


def _inputs(**kwargs) -> StateMachineInputs:
    base = dict(
        button_pressed=False,
        rising_edge=False,
        falling_edge=False,
        fault=False,
        connected=True,
        quit_requested=False,
        primitive_terminated=False,
        toggle_edge=False,
        use_toggle_control=False,
    )
    base.update(kwargs)
    return StateMachineInputs(**base)


def test_workstation_yaml_arms_middle_pedal_toggle(tmp_path: Path) -> None:
    src = Path(__file__).resolve().parents[1] / "config" / "foot_pedal.yaml"
    cfg = load_foot_pedal_configuration(src)
    assert cfg.enabled is True
    assert cfg.device.name_contains == "PCsensor FootSwitch Keyboard"
    assert cfg.device.vendor_id == 0x3553
    assert cfg.device.product_id == 0xB001
    assert cfg.pedal("pedal_1").action == "none"
    assert cfg.pedal("pedal_1").intended_function == "portable_device"
    assert cfg.pedal("pedal_2").action == "freedrive_toggle"
    assert cfg.pedal("pedal_2").key_code == 48
    assert cfg.pedal("pedal_3").intended_function == "pre_programmed"
    assert cfg.freedrive_toggle_ready() is True


def test_pedal_2_armed_when_key_code_set(tmp_path: Path) -> None:
    path = tmp_path / "foot_pedal.yaml"
    path.write_text(
        yaml.dump(
            {
                "foot_pedal": {
                    "enabled": True,
                    "debounce_ms": 50,
                    "device": {"name_contains": "iKKEGOL"},
                    "pedals": {
                        "pedal_1": {
                            "key_code": None,
                            "action": "none",
                            "intended_function": "portable_device",
                        },
                        "pedal_2": {"key_code": 48, "action": "freedrive_toggle"},
                        "pedal_3": {
                            "key_code": None,
                            "action": "none",
                            "intended_function": "pre_programmed",
                        },
                    },
                }
            }
        ),
        encoding="utf-8",
    )
    cfg = load_foot_pedal_configuration(path)
    assert cfg.freedrive_toggle_ready() is True
    assert cfg.key_code_to_binding()[48].pedal_id == "pedal_2"


def test_duplicate_key_codes_rejected(tmp_path: Path) -> None:
    path = tmp_path / "bad.yaml"
    path.write_text(
        yaml.dump(
            {
                "foot_pedal": {
                    "pedals": {
                        "pedal_1": {"key_code": 30, "action": "none"},
                        "pedal_2": {"key_code": 30, "action": "freedrive_toggle"},
                    }
                }
            }
        ),
        encoding="utf-8",
    )
    with pytest.raises(ConfigurationError, match="unique"):
        load_foot_pedal_configuration(path)


def test_device_match_prefers_name_and_by_id() -> None:
    cfg = FootPedalConfiguration(
        pedals={
            "pedal_2": PedalBinding("pedal_2", 48, "freedrive_toggle"),
        }
    )
    devices = [
        InputDeviceInfo(
            path="/dev/input/event10",
            name="AT Translated Set 2 keyboard",
            phys=None,
            uniq=None,
            vendor_id=1,
            product_id=1,
            capabilities_summary="EV_KEY",
        ),
        InputDeviceInfo(
            path="/dev/input/event20",
            name="iKKEGOL USB Pedal",
            phys="usb-1",
            uniq=None,
            vendor_id=0x1234,
            product_id=0x5678,
            by_id_path="/dev/input/by-id/usb-iKKEGOL-event-kbd",
            capabilities_summary="EV_KEY",
        ),
    ]
    matched = find_foot_pedal_devices(cfg, devices=devices)
    assert len(matched) == 1
    assert matched[0].path == "/dev/input/event20"
    assert matched[0].by_id_path is not None


def test_key_debouncer_ignores_bounce_repeat_and_requires_keyup() -> None:
    clock = {"t": 0.0}

    def mono() -> float:
        return clock["t"]

    debouncer = _KeyDebouncer(debounce_s=0.08, monotonic=mono)
    assert debouncer.accept_key_down(48) is True
    # Duplicate down without up (bounce / stuck repeat) ignored.
    assert debouncer.accept_key_down(48) is False
    clock["t"] = 0.2
    assert debouncer.accept_key_down(48) is False
    debouncer.note_key_up(48)
    # Still inside debounce window relative to the last accepted down at t=0.
    clock["t"] = 0.05
    assert debouncer.accept_key_down(48) is False
    clock["t"] = 0.09
    assert debouncer.accept_key_down(48) is True


def test_reader_arms_only_after_discarding_stale_events_and_release() -> None:
    class FakeDevice:
        def __init__(self, *, active: list[int]) -> None:
            self.pending = [object(), object()]
            self.active = active

        def read_one(self) -> object | None:
            return self.pending.pop(0) if self.pending else None

        def active_keys(self) -> list[int]:
            return self.active

    config = FootPedalConfiguration(
        pedals={"pedal_2": PedalBinding("pedal_2", 48, "freedrive_toggle")}
    )
    reader = FootPedalReader.__new__(FootPedalReader)
    reader.config = config
    reader._device = FakeDevice(active=[])
    reader._debouncer = _KeyDebouncer(debounce_s=0.08, monotonic=lambda: 0.0)
    reader._armed = False

    reader.arm()
    assert reader._armed is True
    assert reader._device.pending == []

    reader._device = FakeDevice(active=[48])
    reader._armed = False
    with pytest.raises(FootPedalError, match="held down"):
        reader.arm()
    assert reader._armed is False


def test_reader_discards_events_during_stop_and_waits_for_release() -> None:
    class ReleasingDevice:
        def __init__(self) -> None:
            self.pending = [object()]
            self.active_reads = 0

        def read_one(self) -> object | None:
            return self.pending.pop(0) if self.pending else None

        def active_keys(self) -> list[int]:
            self.active_reads += 1
            return [48] if self.active_reads == 1 else []

    clock = {"now": 0.0}
    config = FootPedalConfiguration(
        pedals={"pedal_2": PedalBinding("pedal_2", 48, "freedrive_toggle")}
    )
    reader = FootPedalReader.__new__(FootPedalReader)
    reader.config = config
    reader._device = ReleasingDevice()
    reader._debouncer = _KeyDebouncer(
        debounce_s=0.08, monotonic=lambda: clock["now"]
    )
    reader._monotonic = lambda: clock["now"]
    reader._armed = True

    reader.rearm_after_stop(
        sleep=lambda seconds: clock.__setitem__("now", clock["now"] + seconds)
    )
    assert reader._armed is True
    assert reader._device.pending == []
    assert reader._device.active_reads == 2


def test_toggle_edge_starts_and_stops_without_hold() -> None:
    sm = FreedriveStateMachine()
    action = sm.on_inputs(_inputs(use_toggle_control=True, toggle_edge=True))
    assert action == Action.START_PRIMITIVE
    sm.mark_started()
    assert sm.state == FreedriveState.FREEDRIVE_ACTIVE
    # No falling edge / hold semantics: idle poll does nothing.
    assert sm.on_inputs(_inputs(use_toggle_control=True)) == Action.NONE
    action = sm.on_inputs(_inputs(use_toggle_control=True, toggle_edge=True))
    assert action == Action.STOP_SESSION
    sm.on_inputs(
        _inputs(use_toggle_control=True, stop_complete=True)
    )
    assert sm.state == FreedriveState.WAITING_FOR_ENABLE
    action = sm.on_inputs(_inputs(use_toggle_control=True, toggle_edge=True))
    assert action == Action.START_PRIMITIVE


def test_stub_handler_reports_unimplemented_extension() -> None:
    messages: list[str] = []
    handler = StubPedalActionHandler(output=messages.append)
    event = PedalEvent(
        pedal_id="pedal_1",
        key_code=30,
        action="none",
        intended_function="portable_device",
        timestamp_s=0.0,
    )
    binding = PedalBinding("pedal_1", 30, "none", intended_function="portable_device")
    handler.on_key_down(event, binding)
    assert messages
    assert "not implemented yet" in messages[0]
    assert "portable_device" in messages[0]


def test_reader_can_emit_all_three_teleop_mode_events() -> None:
    class Codes:
        EV_KEY = 1

    class FakeEvdev:
        ecodes = Codes()

    class Raw:
        type = 1
        value = 1

        def __init__(self, code: int) -> None:
            self.code = code

    config = FootPedalConfiguration(
        pedals={
            "pedal_1": PedalBinding("pedal_1", 30, "teleop_7dof"),
            "pedal_2": PedalBinding("pedal_2", 48, "teleop_9dof"),
            "pedal_3": PedalBinding("pedal_3", 46, "teleop_pivot_orientation"),
        }
    )
    config.validate()
    reader = FootPedalReader.__new__(FootPedalReader)
    reader._evdev = FakeEvdev()
    reader._bindings = config.key_code_to_binding()
    reader._handler = StubPedalActionHandler()
    reader._emit_all_events = True
    reader._monotonic = lambda: 1.0
    reader._debouncer = _KeyDebouncer(debounce_s=0.0, monotonic=lambda: 1.0)

    events = [reader._handle_raw(Raw(code)) for code in (30, 48, 46)]
    assert [event.pedal_id for event in events if event is not None] == [
        "pedal_1",
        "pedal_2",
        "pedal_3",
    ]


def test_diagnostic_reader_emits_unmapped_physical_key_code() -> None:
    class Codes:
        EV_KEY = 1

    class FakeEvdev:
        ecodes = Codes()

    class Raw:
        type = 1
        value = 1
        code = 99

    reader = FootPedalReader.__new__(FootPedalReader)
    reader._evdev = FakeEvdev()
    reader._bindings = {}
    reader._emit_unmapped_events = True
    reader._monotonic = lambda: 1.0
    reader._debouncer = _KeyDebouncer(debounce_s=0.0, monotonic=lambda: 1.0)
    event = reader._handle_raw(Raw())
    assert event is not None
    assert event.pedal_id == "unmapped"
    assert event.key_code == 99


def test_xinput_parser_keeps_only_slave_keyboards_and_usb_ids() -> None:
    listing = """
⎡ Virtual core pointer                     id=2    [master pointer  (3)]
⎜   ↳ PCsensor FootSwitch                  id=12   [slave  pointer  (2)]
⎣ Virtual core keyboard                    id=3    [master keyboard (2)]
    ↳ AT Translated Set 2 keyboard         id=11   [slave  keyboard (3)]
    ↳ PCsensor FootSwitch                  id=17   [slave  keyboard (3)]
"""
    devices = parse_xinput_slave_keyboards(listing)
    assert [(device.device_id, device.name) for device in devices] == [
        (11, "AT Translated Set 2 keyboard"),
        (17, "PCsensor FootSwitch"),
    ]
    assert parse_xinput_product_ids(
        "Device Product ID (288):\t13651, 45057"
    ) == (0x3553, 0xB001)


def test_xinput_resolver_selects_physical_footswitch_not_normal_keyboard(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    config = FootPedalConfiguration(
        device=load_foot_pedal_configuration(
            Path(__file__).resolve().parents[1] / "config" / "foot_pedal.yaml"
        ).device,
        pedals={"pedal_2": PedalBinding("pedal_2", 48, "freedrive_toggle")},
    )
    reader = XInputFootPedalReader.__new__(XInputFootPedalReader)
    reader.config = config
    listing = """
↳ AT Translated Set 2 keyboard id=11 [slave keyboard (3)]
    ↳ PCsensor FootSwitch Keyboard id=17 [slave keyboard (3)]
"""

    def fake_run(*args: str) -> str:
        if args == ("list", "--short"):
            return listing
        if args == ("list-props", "17"):
            return "Device Product ID (288): 13651, 45057"
        raise AssertionError(args)

    reader._run_xinput = fake_run  # type: ignore[method-assign]
    monkeypatch.setenv("DISPLAY", ":1")
    selected = reader._resolve_xinput_device()
    assert selected.device_id == 17
    assert selected.name == "PCsensor FootSwitch Keyboard"
    assert selected.vendor_id == 0x3553
    assert selected.product_id == 0xB001
