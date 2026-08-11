"""Physical-device readers for the iKKEGOL triple USB foot pedal.

``FootPedalReader`` uses Linux evdev and therefore needs permission to open
``/dev/input``. ``XInputFootPedalReader`` uses the already-running X11 server
to identify the physical PCsensor/iKKEGOL device, so it needs no input-group
or udev changes. Both readers emit only debounced key-down edges; key-up,
auto-repeat, and bounced duplicates are ignored.

Pedal actions:
  - freedrive_toggle: implemented (Pedal 2)
  - none / portable_device / pre_programmed: extension stubs (Pedals 1 and 3)
"""

from __future__ import annotations

import logging
import os
import queue
import re
import select
import shutil
import subprocess
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Iterable, Protocol

from src.foot_pedal_configuration import (
    FootPedalConfiguration,
    PedalBinding,
)

logger = logging.getLogger(__name__)

# Linux input event values for EV_KEY
KEY_UP = 0
KEY_DOWN = 1
KEY_REPEAT = 2

# Xorg represents Linux evdev key codes with the standard +8 X keycode offset.
XINPUT_EVDEV_KEYCODE_OFFSET = 8


class FootPedalError(RuntimeError):
    """Foot-pedal device or input failure."""


@dataclass(frozen=True)
class InputDeviceInfo:
    """One candidate Linux input device."""

    path: str
    name: str
    phys: str | None
    uniq: str | None
    vendor_id: int | None
    product_id: int | None
    by_id_path: str | None = None
    capabilities_summary: str = ""

    def matches(
        self,
        *,
        name_contains: str | None = None,
        vendor_id: int | None = None,
        product_id: int | None = None,
        path: str | None = None,
    ) -> bool:
        if path is not None:
            return Path(self.path).resolve() == Path(path).resolve() or (
                self.by_id_path is not None
                and Path(self.by_id_path).resolve() == Path(path).resolve()
            )
        if vendor_id is not None and self.vendor_id != vendor_id:
            return False
        if product_id is not None and self.product_id != product_id:
            return False
        if name_contains:
            if name_contains.lower() not in self.name.lower():
                return False
        return True


@dataclass(frozen=True)
class PedalEvent:
    """A debounced key-down event mapped to a configured pedal."""

    pedal_id: str
    key_code: int
    action: str
    intended_function: str | None
    timestamp_s: float


class PedalActionHandler(Protocol):
    """Extension point for Pedal 1 / Pedal 3 (and future actions)."""

    def on_key_down(self, event: PedalEvent, binding: PedalBinding) -> None:
        """Handle a key-down for a pedal whose action is not Freedrive toggle."""


class StubPedalActionHandler:
    """Default handler: log that the intended function is not implemented yet."""

    def __init__(self, output: Callable[[str], None] | None = None) -> None:
        self._output = output or (lambda msg: logger.info(msg))

    def on_key_down(self, event: PedalEvent, binding: PedalBinding) -> None:
        intended = binding.intended_function or event.intended_function or "unspecified"
        self._output(
            f"foot pedal {event.pedal_id}: action={event.action!r} "
            f"(intended={intended}) is not implemented yet; ignoring key-down "
            f"key_code={event.key_code}"
        )


def _by_id_index() -> dict[str, str]:
    """Map resolved /dev/input/eventX -> /dev/input/by-id/... when available."""
    by_id = Path("/dev/input/by-id")
    mapping: dict[str, str] = {}
    if not by_id.is_dir():
        return mapping
    for link in by_id.iterdir():
        try:
            resolved = str(link.resolve())
            mapping[resolved] = str(link)
        except OSError:
            continue
    return mapping


def list_input_devices() -> list[InputDeviceInfo]:
    """List Linux input devices via evdev (all nodes; caller filters)."""
    try:
        import evdev
    except ImportError as exc:
        raise FootPedalError(
            "python package 'evdev' is required; install with: "
            "python3 -m pip install -r requirements.txt"
        ) from exc

    by_id = _by_id_index()
    devices: list[InputDeviceInfo] = []
    for path in evdev.list_devices():
        try:
            dev = evdev.InputDevice(path)
        except OSError as exc:
            logger.debug("skip unreadable input device %s: %s", path, exc)
            continue
        try:
            vendor = int(dev.info.vendor) if dev.info.vendor else None
            product = int(dev.info.product) if dev.info.product else None
            caps = []
            try:
                if evdev.ecodes.EV_KEY in dev.capabilities():
                    caps.append("EV_KEY")
            except Exception:
                pass
            resolved = str(Path(path).resolve())
            devices.append(
                InputDeviceInfo(
                    path=path,
                    name=dev.name or "",
                    phys=dev.phys,
                    uniq=dev.uniq,
                    vendor_id=vendor,
                    product_id=product,
                    by_id_path=by_id.get(resolved),
                    capabilities_summary=",".join(caps),
                )
            )
        finally:
            try:
                dev.close()
            except Exception:
                pass
    return devices


def find_foot_pedal_devices(
    config: FootPedalConfiguration,
    *,
    devices: Iterable[InputDeviceInfo] | None = None,
) -> list[InputDeviceInfo]:
    """Return devices matching the configured name / USB IDs / path."""
    all_devices = list(devices) if devices is not None else list_input_devices()
    device_cfg = config.device

    if device_cfg.path:
        path = Path(device_cfg.path)
        if not path.exists():
            raise FootPedalError(f"configured foot pedal path does not exist: {path}")
        resolved = str(path.resolve())
        for info in all_devices:
            if Path(info.path).resolve() == Path(resolved) or info.by_id_path == str(
                path
            ):
                return [info]
        # Path exists but was not enumerated (permissions); still allow open later.
        return [
            InputDeviceInfo(
                path=str(path),
                name="(configured path)",
                phys=None,
                uniq=None,
                vendor_id=device_cfg.vendor_id,
                product_id=device_cfg.product_id,
                by_id_path=str(path) if "by-id" in str(path) else None,
            )
        ]

    matched = [
        info
        for info in all_devices
        if info.matches(
            name_contains=device_cfg.name_contains,
            vendor_id=device_cfg.vendor_id,
            product_id=device_cfg.product_id,
        )
    ]

    # Prefer keyboard-capable nodes and stable by-id paths.
    matched.sort(
        key=lambda d: (
            0 if d.by_id_path else 1,
            0 if "EV_KEY" in d.capabilities_summary else 1,
            d.path,
        )
    )
    return matched


def resolve_foot_pedal_device(config: FootPedalConfiguration) -> InputDeviceInfo:
    """Resolve exactly one foot-pedal input device or raise."""
    matched = find_foot_pedal_devices(config)
    if not matched:
        raise FootPedalError(
            "no foot pedal input device matched the configuration "
            f"(name_contains={config.device.name_contains!r}, "
            f"vendor_id={config.device.vendor_id}, "
            f"product_id={config.device.product_id}, "
            f"path={config.device.path!r}). "
            "Plug in the iKKEGOL pedal and run scripts/diagnose_foot_pedal.py"
        )
    if len(matched) > 1 and config.device.path is None:
        listing = ", ".join(
            f"{m.path} ({m.name!r}, by-id={m.by_id_path})" for m in matched
        )
        raise FootPedalError(
            "multiple input devices matched the foot pedal filters; "
            "set foot_pedal.device.path to a stable /dev/input/by-id/... node. "
            f"Candidates: {listing}"
        )
    return matched[0]


@dataclass
class _KeyDebouncer:
    """Accept a key-down only after debounce_s without conflicting chatter."""

    debounce_s: float
    monotonic: Callable[[], float]
    _last_accepted_down: dict[int, float] = field(default_factory=dict)
    _down_state: dict[int, bool] = field(default_factory=dict)

    def accept_key_down(self, key_code: int) -> bool:
        now = self.monotonic()
        # Ignore duplicate downs while already logically down (bounce/repeat).
        if self._down_state.get(key_code, False):
            return False
        last = self._last_accepted_down.get(key_code)
        if last is not None and (now - last) < self.debounce_s:
            return False
        self._down_state[key_code] = True
        self._last_accepted_down[key_code] = now
        return True

    def note_key_up(self, key_code: int) -> None:
        self._down_state[key_code] = False

    def reset(self) -> None:
        self._last_accepted_down.clear()
        self._down_state.clear()


class FootPedalReader:
    """Non-blocking EV_KEY reader scoped to one resolved foot-pedal device."""

    def __init__(
        self,
        config: FootPedalConfiguration,
        *,
        device_info: InputDeviceInfo | None = None,
        grab: bool = True,
        action_handler: PedalActionHandler | None = None,
        emit_all_events: bool = False,
        emit_unmapped_events: bool = False,
        monotonic: Callable[[], float] = time.monotonic,
        output: Callable[[str], None] | None = None,
    ) -> None:
        self.config = config
        self.config.validate()
        self._bindings = config.key_code_to_binding()
        self._handler = action_handler or StubPedalActionHandler(output=output)
        self._emit_all_events = emit_all_events
        self._emit_unmapped_events = emit_unmapped_events
        self._debouncer = _KeyDebouncer(
            debounce_s=config.debounce_s, monotonic=monotonic
        )
        self._monotonic = monotonic
        self._grab = grab
        self._device = None
        self._device_info = device_info
        self._armed = False
        self._open_device()

    def _open_device(self) -> None:
        try:
            import evdev
        except ImportError as exc:
            raise FootPedalError(
                "python package 'evdev' is required; install with: "
                "python3 -m pip install -r requirements.txt"
            ) from exc

        info = self._device_info or resolve_foot_pedal_device(self.config)
        self._device_info = info
        try:
            device = evdev.InputDevice(info.path)
        except PermissionError as exc:
            raise FootPedalError(
                f"permission denied opening {info.path}. "
                "Install udev/99-ikkegol-foot-pedal.rules, reload udev, "
                "replug the pedal, and ensure your user is in group 'input'."
            ) from exc
        except OSError as exc:
            raise FootPedalError(f"failed to open foot pedal {info.path}: {exc}") from exc

        if self._grab:
            try:
                device.grab()
            except OSError as exc:
                device.close()
                raise FootPedalError(
                    f"failed to grab foot pedal {info.path} exclusively: {exc}"
                ) from exc

        self._device = device
        self._evdev = evdev

    @property
    def device_info(self) -> InputDeviceInfo | None:
        return self._device_info

    def close(self) -> None:
        device = self._device
        self._device = None
        self._armed = False
        if device is None:
            return
        try:
            if self._grab:
                try:
                    device.ungrab()
                except Exception:
                    pass
            device.close()
        except Exception:
            pass

    def __enter__(self) -> FootPedalReader:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def arm(self) -> None:
        """Discard startup events and require Pedal 2 to be released.

        The reader is opened before the robot connection, so presses made while
        the robot is connecting must never be replayed as a later Freedrive
        request.
        """
        device = self._device
        if device is None:
            raise FootPedalError("cannot arm a closed foot pedal")
        self._armed = False

        try:
            while device.read_one() is not None:
                pass
            active_keys = {int(code) for code in device.active_keys()}
        except (BlockingIOError, OSError) as exc:
            raise FootPedalError(
                f"failed to establish a clean foot-pedal startup state: {exc}"
            ) from exc

        emit_all_events = bool(getattr(self, "_emit_all_events", False))
        bindings = getattr(self, "_bindings", self.config.key_code_to_binding())
        active_bindings = [
            binding
            for code, binding in bindings.items()
            if code in active_keys
        ]
        if emit_all_events and active_bindings:
            names = ", ".join(binding.pedal_id for binding in active_bindings)
            raise FootPedalError(
                f"Foot pedal(s) {names} are held while arming. Release all three "
                "pedals and restart; a held/stale press cannot select a teleop mode."
            )
        pedal_2 = self.config.pedal("pedal_2")
        if not emit_all_events and pedal_2.key_code is not None and pedal_2.key_code in active_keys:
            raise FootPedalError(
                "Pedal 2 is held down while arming. Release the middle pedal "
                "and restart; a held/stale press is never allowed to start Freedrive."
            )

        self._debouncer.reset()
        self._armed = True

    def rearm_after_stop(
        self,
        *,
        release_timeout_s: float = 3.0,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        """Ignore presses queued during Stop and rearm after Pedal 2 is released."""
        device = self._device
        if device is None:
            raise FootPedalError("cannot rearm a closed foot pedal")
        self._armed = False
        pedal_2 = self.config.pedal("pedal_2")
        deadline = self._monotonic() + release_timeout_s

        while True:
            try:
                while device.read_one() is not None:
                    pass
                active_keys = {int(code) for code in device.active_keys()}
            except (BlockingIOError, OSError) as exc:
                raise FootPedalError(
                    f"foot pedal failed while waiting for release after Stop: {exc}"
                ) from exc

            if pedal_2.key_code is None or pedal_2.key_code not in active_keys:
                self._debouncer.reset()
                self._armed = True
                return
            if self._monotonic() >= deadline:
                raise FootPedalError(
                    "Pedal 2 remained held after Stop. Freedrive stays stopped; "
                    "release the middle pedal and restart the program."
                )
            sleep(0.02)

    def poll(self) -> list[PedalEvent]:
        """Read pending events; return debounced key-down PedalEvents only."""
        if self._device is None:
            return []
        if not self._armed:
            raise FootPedalError(
                "foot pedal is not armed; arm it only after the robot is locked/stopped"
            )
        events: list[PedalEvent] = []
        try:
            while True:
                raw = self._device.read_one()
                if raw is None:
                    break
                event = self._handle_raw(raw)
                if event is not None:
                    events.append(event)
        except BlockingIOError:
            pass
        except OSError as exc:
            raise FootPedalError(f"foot pedal read failed: {exc}") from exc
        return events

    def _handle_raw(self, raw: object) -> PedalEvent | None:
        evdev = self._evdev
        if raw.type != evdev.ecodes.EV_KEY:
            return None
        code = int(raw.code)
        value = int(raw.value)
        if value == KEY_UP:
            self._debouncer.note_key_up(code)
            return None
        if value == KEY_REPEAT:
            return None
        if value != KEY_DOWN:
            return None
        if not self._debouncer.accept_key_down(code):
            return None

        binding = self._bindings.get(code)
        if binding is None:
            if bool(getattr(self, "_emit_unmapped_events", False)):
                return PedalEvent(
                    pedal_id="unmapped",
                    key_code=code,
                    action="unmapped",
                    intended_function=None,
                    timestamp_s=self._monotonic(),
                )
            # Unmapped key on the dedicated pedal device: ignore (do not steal
            # events from other keyboards because we only open the matched device).
            return None

        event = PedalEvent(
            pedal_id=binding.pedal_id,
            key_code=code,
            action=binding.action,
            intended_function=binding.intended_function,
            timestamp_s=self._monotonic(),
        )
        if self._emit_all_events or binding.action == "freedrive_toggle":
            return event
        # Extension point for Pedals 1 / 3 and future actions.
        self._handler.on_key_down(event, binding)
        return None


@dataclass(frozen=True)
class XInputDeviceInfo:
    """One XInput slave-keyboard device."""

    device_id: int
    name: str
    vendor_id: int | None = None
    product_id: int | None = None


_XINPUT_ID_RE = re.compile(r"\bid=(\d+)\b")
_XINPUT_PRODUCT_RE = re.compile(
    r"Device Product ID.*?:\s*(\d+)\s*,\s*(\d+)", re.IGNORECASE
)
_XINPUT_EVENT_RE = re.compile(r"EVENT type \d+ \((RawKeyPress|RawKeyRelease)\)")
_XINPUT_DETAIL_RE = re.compile(r"^\s*detail:\s*(\d+)\s*$")


def parse_xinput_slave_keyboards(text: str) -> list[XInputDeviceInfo]:
    """Parse ``xinput list --short`` without depending on tree glyphs."""
    devices: list[XInputDeviceInfo] = []
    for line in text.splitlines():
        if "slave" not in line.lower() or "keyboard" not in line.lower():
            continue
        match = _XINPUT_ID_RE.search(line)
        if match is None:
            continue
        prefix = line[: match.start()].strip()
        # xinput draws a Unicode tree before slave-device names.
        name = prefix.lstrip("⎡⎣⎜⎢↳ \t").strip()
        if name:
            devices.append(XInputDeviceInfo(int(match.group(1)), name))
    return devices


def parse_xinput_product_ids(text: str) -> tuple[int | None, int | None]:
    """Return decimal USB vendor/product IDs from ``xinput list-props``."""
    match = _XINPUT_PRODUCT_RE.search(text)
    if match is None:
        return None, None
    return int(match.group(1)), int(match.group(2))


class XInputFootPedalReader:
    """Read only the physical foot-switch device through XInput 2.

    The foot switch is a USB HID keyboard, but the action is selected by XInput
    *source device ID*, not by terminal text. Pressing the same letter on a
    normal keyboard therefore does not create a pedal event.
    """

    backend_name = "xinput-device"
    owns_stdin = True

    def __init__(
        self,
        config: FootPedalConfiguration,
        *,
        action_handler: PedalActionHandler | None = None,
        emit_all_events: bool = False,
        emit_unmapped_events: bool = False,
        monotonic: Callable[[], float] = time.monotonic,
        output: Callable[[str], None] | None = None,
        stdin: object | None = None,
    ) -> None:
        self.config = config
        self.config.validate()
        self._bindings = config.key_code_to_binding()
        self._handler = action_handler or StubPedalActionHandler(output=output)
        self._emit_all_events = emit_all_events
        self._emit_unmapped_events = emit_unmapped_events
        self._debouncer = _KeyDebouncer(
            debounce_s=config.debounce_s, monotonic=monotonic
        )
        self._monotonic = monotonic
        self._raw_events: queue.SimpleQueue[tuple[int, int]] = queue.SimpleQueue()
        self._active_keys: set[int] = set()
        self._active_lock = threading.Lock()
        self._armed = False
        self._closed = False
        self._quit_requested = False
        self._process: subprocess.Popen[str] | None = None
        self._thread: threading.Thread | None = None
        self._stdin = sys.stdin if stdin is None else stdin
        self._stdin_fd: int | None = None
        self._saved_terminal: list[object] | None = None

        selected = self._resolve_xinput_device()
        self._xinput_info = selected
        self._device_info = InputDeviceInfo(
            path=f"xinput:{selected.device_id}",
            name=selected.name,
            phys="X11 XInput 2",
            uniq=None,
            vendor_id=selected.vendor_id,
            product_id=selected.product_id,
            capabilities_summary="XInput2 slave keyboard",
        )
        self._start_listener()
        self._configure_terminal_guard()

    def _run_xinput(self, *args: str) -> str:
        executable = shutil.which("xinput")
        if executable is None:
            raise FootPedalError(
                "the XInput backend requires the 'xinput' command, but it was not found"
            )
        try:
            completed = subprocess.run(
                [executable, *args],
                check=False,
                capture_output=True,
                text=True,
                timeout=5.0,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            raise FootPedalError(f"failed to run xinput {' '.join(args)}: {exc}") from exc
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout).strip()
            raise FootPedalError(
                f"xinput {' '.join(args)} failed: {detail or 'unknown X11 error'}"
            )
        return completed.stdout

    def _resolve_xinput_device(self) -> XInputDeviceInfo:
        if not os.environ.get("DISPLAY"):
            raise FootPedalError(
                "XInput pedal mode needs a local X11 terminal (DISPLAY is not set)"
            )
        devices = parse_xinput_slave_keyboards(self._run_xinput("list", "--short"))
        name_filter = (self.config.device.name_contains or "").lower()
        candidates = [
            device
            for device in devices
            if not name_filter or name_filter in device.name.lower()
        ]

        matched: list[XInputDeviceInfo] = []
        for device in candidates:
            props = self._run_xinput("list-props", str(device.device_id))
            vendor_id, product_id = parse_xinput_product_ids(props)
            if (
                self.config.device.vendor_id is not None
                and vendor_id != self.config.device.vendor_id
            ):
                continue
            if (
                self.config.device.product_id is not None
                and product_id != self.config.device.product_id
            ):
                continue
            matched.append(
                XInputDeviceInfo(
                    device_id=device.device_id,
                    name=device.name,
                    vendor_id=vendor_id,
                    product_id=product_id,
                )
            )

        if not matched:
            listing = ", ".join(
                f"{device.device_id}:{device.name}" for device in devices
            )
            raise FootPedalError(
                "no XInput slave keyboard matched the physical foot pedal "
                f"(name contains {self.config.device.name_contains!r}, "
                f"USB={self.config.device.vendor_id!r}:"
                f"{self.config.device.product_id!r}). "
                f"Available slave keyboards: {listing or '(none)'}"
            )
        if len(matched) != 1:
            listing = ", ".join(
                f"{device.device_id}:{device.name}" for device in matched
            )
            raise FootPedalError(
                "multiple XInput keyboard devices matched the foot pedal; "
                f"make foot_pedal.device.name_contains more specific. Matches: {listing}"
            )
        return matched[0]

    def _start_listener(self) -> None:
        xinput = shutil.which("xinput")
        stdbuf = shutil.which("stdbuf")
        if xinput is None:
            raise FootPedalError("the 'xinput' command is required")
        command = [xinput, "test-xi2", "--root", str(self._xinput_info.device_id)]
        if stdbuf is not None:
            command = [stdbuf, "-oL", *command]
        try:
            self._process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
            )
        except OSError as exc:
            raise FootPedalError(f"failed to start XInput pedal listener: {exc}") from exc
        self._thread = threading.Thread(
            target=self._read_xinput_output,
            name="xinput-foot-pedal",
            daemon=True,
        )
        self._thread.start()
        # Catch immediate display/device errors rather than failing after robot connect.
        time.sleep(0.05)
        return_code = self._process.poll()
        if return_code is not None:
            raise FootPedalError(
                f"XInput pedal listener exited during startup (status {return_code})"
            )

    def _read_xinput_output(self) -> None:
        process = self._process
        if process is None or process.stdout is None:
            return
        pending_edge: int | None = None
        for line in process.stdout:
            event_match = _XINPUT_EVENT_RE.search(line)
            if event_match is not None:
                pending_edge = (
                    KEY_DOWN
                    if event_match.group(1) == "RawKeyPress"
                    else KEY_UP
                )
                continue
            detail_match = _XINPUT_DETAIL_RE.match(line)
            if detail_match is None or pending_edge is None:
                continue
            x_keycode = int(detail_match.group(1))
            key_code = x_keycode - XINPUT_EVDEV_KEYCODE_OFFSET
            edge = pending_edge
            pending_edge = None
            if key_code < 0:
                continue
            with self._active_lock:
                if edge == KEY_DOWN:
                    self._active_keys.add(key_code)
                else:
                    self._active_keys.discard(key_code)
                self._raw_events.put((edge, key_code))

    def _configure_terminal_guard(self) -> None:
        """Hide/drain the HID character; device identity still comes from XInput."""
        stream = self._stdin
        if stream is None or not hasattr(stream, "fileno"):
            return
        try:
            if not stream.isatty():
                return
            fd = int(stream.fileno())
            saved = termios.tcgetattr(fd)
            tty.setcbreak(fd, termios.TCSANOW)
        except (AttributeError, OSError, termios.error, ValueError):
            return
        self._stdin_fd = fd
        self._saved_terminal = saved

    def _poll_terminal_controls(self) -> None:
        """Drain typed HID characters and preserve q/Ctrl+C operator controls."""
        fd = self._stdin_fd
        if fd is None:
            return
        try:
            while select.select([fd], [], [], 0)[0]:
                data = os.read(fd, 4096)
                if not data:
                    break
                if b"q" in data.lower():
                    self._quit_requested = True
        except (OSError, ValueError):
            pass

    @property
    def device_info(self) -> InputDeviceInfo:
        return self._device_info

    @property
    def quit_requested(self) -> bool:
        return self._quit_requested

    def _drain_raw_events_locked(self) -> None:
        while True:
            try:
                self._raw_events.get_nowait()
            except queue.Empty:
                return

    def arm(self) -> None:
        """Discard connection-time events and require the physical pedal released."""
        if self._closed:
            raise FootPedalError("cannot arm a closed XInput foot pedal")
        self._armed = False
        with self._active_lock:
            self._drain_raw_events_locked()
            active_keys = set(self._active_keys)
        emit_all_events = bool(getattr(self, "_emit_all_events", False))
        bindings = getattr(self, "_bindings", self.config.key_code_to_binding())
        active_bindings = [
            binding
            for code, binding in bindings.items()
            if code in active_keys
        ]
        if emit_all_events and active_bindings:
            names = ", ".join(binding.pedal_id for binding in active_bindings)
            raise FootPedalError(
                f"Foot pedal(s) {names} are held while arming. Release all three "
                "pedals and restart; a held/stale press cannot select a teleop mode."
            )
        pedal_2 = self.config.pedal("pedal_2")
        if not emit_all_events and pedal_2.key_code is not None and pedal_2.key_code in active_keys:
            raise FootPedalError(
                "Pedal 2 is held down while arming. Release the middle pedal "
                "and restart; a held/stale press is never allowed to start Freedrive."
            )
        self._debouncer.reset()
        self._armed = True

    def rearm_after_stop(
        self,
        *,
        release_timeout_s: float = 3.0,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        """Discard events during Stop and wait for the physical key-release edge."""
        if self._closed:
            raise FootPedalError("cannot rearm a closed XInput foot pedal")
        self._armed = False
        pedal_2 = self.config.pedal("pedal_2")
        deadline = self._monotonic() + release_timeout_s
        while True:
            with self._active_lock:
                self._drain_raw_events_locked()
                active_keys = set(self._active_keys)
            if pedal_2.key_code is None or pedal_2.key_code not in active_keys:
                self._debouncer.reset()
                self._armed = True
                return
            if self._monotonic() >= deadline:
                raise FootPedalError(
                    "Pedal 2 remained held after Stop. Freedrive stays stopped; "
                    "release the middle pedal and restart the program."
                )
            sleep(0.02)

    def poll(self) -> list[PedalEvent]:
        """Return mapped events from only the selected physical XInput device."""
        self._poll_terminal_controls()
        if not self._armed:
            raise FootPedalError(
                "foot pedal is not armed; arm it only after the robot is locked/stopped"
            )
        process = self._process
        if process is None or process.poll() is not None:
            raise FootPedalError(
                "XInput pedal listener stopped; keep the local X11 session active"
            )
        events: list[PedalEvent] = []
        while True:
            try:
                edge, key_code = self._raw_events.get_nowait()
            except queue.Empty:
                break
            if edge == KEY_UP:
                self._debouncer.note_key_up(key_code)
                continue
            if edge != KEY_DOWN or not self._debouncer.accept_key_down(key_code):
                continue
            binding = self._bindings.get(key_code)
            if binding is None:
                if bool(getattr(self, "_emit_unmapped_events", False)):
                    events.append(
                        PedalEvent(
                            pedal_id="unmapped",
                            key_code=key_code,
                            action="unmapped",
                            intended_function=None,
                            timestamp_s=self._monotonic(),
                        )
                    )
                continue
            event = PedalEvent(
                pedal_id=binding.pedal_id,
                key_code=key_code,
                action=binding.action,
                intended_function=binding.intended_function,
                timestamp_s=self._monotonic(),
            )
            if self._emit_all_events or binding.action == "freedrive_toggle":
                events.append(event)
            else:
                self._handler.on_key_down(event, binding)
        return events

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self._armed = False
        process = self._process
        self._process = None
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=1.0)
        thread = self._thread
        self._thread = None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        if self._stdin_fd is not None and self._saved_terminal is not None:
            try:
                termios.tcflush(self._stdin_fd, termios.TCIFLUSH)
                termios.tcsetattr(
                    self._stdin_fd, termios.TCSANOW, self._saved_terminal
                )
            except (OSError, termios.error):
                pass
        self._stdin_fd = None
        self._saved_terminal = None

    def __enter__(self) -> XInputFootPedalReader:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()


def dispatch_extension_actions(
    events: Iterable[PedalEvent],
    *,
    handler: PedalActionHandler | None = None,
) -> list[PedalEvent]:
    """Split Freedrive-toggle events from stub-handled extension events.

    Controllers typically only need freedrive_toggle events. Extension actions
    are already handled inside FootPedalReader.poll(); this helper is for tests
    and custom pipelines that construct PedalEvent lists manually.
    """
    stub = handler or StubPedalActionHandler()
    toggles: list[PedalEvent] = []
    for event in events:
        if event.action == "freedrive_toggle":
            toggles.append(event)
        else:
            binding = PedalBinding(
                pedal_id=event.pedal_id,
                key_code=event.key_code,
                action=event.action,
                intended_function=event.intended_function,
            )
            stub.on_key_down(event, binding)
    return toggles
