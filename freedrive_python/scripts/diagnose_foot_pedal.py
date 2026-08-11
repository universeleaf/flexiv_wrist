#!/usr/bin/env python3
"""Diagnose iKKEGOL USB foot pedal Linux input devices and key codes.

Lists candidate input devices (preferring /dev/input/by-id/ when present) and
prints key-down / key-up / key-code information while you press each pedal.

This script never connects to the robot.
"""

from __future__ import annotations

import argparse
import select
import sys
import time
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.foot_pedal_configuration import (  # noqa: E402
    DEFAULT_FOOT_PEDAL_CONFIG,
    load_foot_pedal_configuration,
)
from src.freedrive_configuration import ConfigurationError  # noqa: E402
from src.foot_pedal_input import (  # noqa: E402
    FootPedalError,
    find_foot_pedal_devices,
    list_input_devices,
    resolve_foot_pedal_device,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "List Linux input devices and print EV_KEY events from the iKKEGOL "
            "foot pedal so you can fill config/foot_pedal.yaml key_code values."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_FOOT_PEDAL_CONFIG,
        help="Foot pedal YAML (default: config/foot_pedal.yaml)",
    )
    parser.add_argument(
        "--list-all",
        action="store_true",
        help="List every readable input device, not only name/USB matches",
    )
    parser.add_argument(
        "--path",
        type=str,
        default=None,
        help="Open this device path directly (prefer /dev/input/by-id/...)",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=60.0,
        help="How long to listen for key events (0 = until Ctrl+C)",
    )
    parser.add_argument(
        "--no-grab",
        action="store_true",
        help="Do not exclusive-grab the device (keys may also type into the terminal)",
    )
    return parser


def _print_device(info: object, prefix: str = "  ") -> None:
    by_id = getattr(info, "by_id_path", None) or "(none)"
    vendor = getattr(info, "vendor_id", None)
    product = getattr(info, "product_id", None)
    vendor_s = f"0x{vendor:04x}" if isinstance(vendor, int) else "unknown"
    product_s = f"0x{product:04x}" if isinstance(product, int) else "unknown"
    print(f"{prefix}name:     {getattr(info, 'name', '')!r}")
    print(f"{prefix}path:     {getattr(info, 'path', '')}")
    print(f"{prefix}by-id:    {by_id}")
    print(f"{prefix}vendor:   {vendor_s} ({vendor})")
    print(f"{prefix}product:  {product_s} ({product})")
    print(f"{prefix}phys:     {getattr(info, 'phys', None)}")
    print(f"{prefix}caps:     {getattr(info, 'capabilities_summary', '')}")


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_foot_pedal_configuration(args.config)
    except ConfigurationError as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2

    if args.path:
        from dataclasses import replace

        from src.foot_pedal_configuration import FootPedalDeviceConfig

        config = replace(
            config,
            device=FootPedalDeviceConfig(
                name_contains=config.device.name_contains,
                path=args.path,
                vendor_id=config.device.vendor_id,
                product_id=config.device.product_id,
            ),
        )

    print("=== Candidate / all input devices ===")
    try:
        all_devices = list_input_devices()
    except FootPedalError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    if args.list_all:
        for info in all_devices:
            print("-")
            _print_device(info)
        matched = find_foot_pedal_devices(config, devices=all_devices)
        if not matched and not args.path:
            print(
                "\nNo devices matched foot_pedal.device filters yet. "
                "Plug in the iKKEGOL pedal, or pass --path /dev/input/by-id/... "
                "to listen on a specific node."
            )
            return 0
    else:
        matched = find_foot_pedal_devices(config, devices=all_devices)
        if not matched:
            print(
                "No devices matched foot_pedal.device filters. "
                "Re-run with --list-all, or plug in the pedal and check "
                "name_contains / vendor_id / product_id."
            )
            print(
                "\nHint: looking for name containing "
                f"{config.device.name_contains!r}"
            )
            return 1
        for info in matched:
            print("-")
            _print_device(info)

    try:
        selected = resolve_foot_pedal_device(config)
    except FootPedalError as exc:
        print(f"\nCould not resolve a single device: {exc}", file=sys.stderr)
        return 1

    print("\n=== Listening on selected device ===")
    _print_device(selected)
    print(
        "\nPress Pedal 1, Pedal 2, and Pedal 3 one at a time.\n"
        "Record the KEY_DOWN key_code integers into config/foot_pedal.yaml.\n"
        "KEY_UP and KEY_REPEAT are shown but must be ignored by the controller.\n"
        "Ctrl+C to stop.\n"
    )

    try:
        import evdev
        from evdev import categorize, ecodes
    except ImportError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 1

    try:
        device = evdev.InputDevice(selected.path)
    except PermissionError:
        print(
            f"permission denied opening {selected.path}.\n"
            "Install udev/99-ikkegol-foot-pedal.rules as root, then:\n"
            "  sudo cp udev/99-ikkegol-foot-pedal.rules /etc/udev/rules.d/\n"
            "  sudo udevadm control --reload-rules && sudo udevadm trigger\n"
            "  # replug the pedal; add yourself to group 'input' if needed:\n"
            "  sudo usermod -aG input \"$USER\"  # then log out/in",
            file=sys.stderr,
        )
        return 1

    grab = not args.no_grab
    if grab:
        try:
            device.grab()
            print("(device grabbed exclusively so presses do not type into the shell)")
        except OSError as exc:
            print(f"warning: grab failed ({exc}); continuing without grab")
            grab = False

    deadline = None if args.seconds <= 0 else time.monotonic() + args.seconds
    seen_codes: dict[int, str] = {}
    try:
        while True:
            now = time.monotonic()
            if deadline is not None and now >= deadline:
                print("\nListen timeout reached.")
                break
            timeout = 0.5 if deadline is None else min(0.5, deadline - now)
            ready, _, _ = select.select([device.fd], [], [], max(0.0, timeout))
            if not ready:
                continue
            for raw in device.read():
                if raw.type != ecodes.EV_KEY:
                    continue
                key_event = categorize(raw)
                code = int(raw.code)
                value = int(raw.value)
                if value == 0:
                    edge = "KEY_UP"
                elif value == 1:
                    edge = "KEY_DOWN"
                elif value == 2:
                    edge = "KEY_REPEAT"
                else:
                    edge = f"VALUE_{value}"
                key_name = ecodes.KEY.get(code, key_event.keycode)
                if isinstance(key_name, (list, tuple)):
                    key_name = key_name[0]
                print(
                    f"{edge:11} key_code={code:<4} name={key_name} "
                    f"scancode={getattr(key_event, 'scancode', code)}"
                )
                if value == 1:
                    seen_codes[code] = str(key_name)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        if grab:
            try:
                device.ungrab()
            except Exception:
                pass
        device.close()

    if seen_codes:
        print("\n=== Key codes observed on KEY_DOWN (fill into YAML) ===")
        for code, name in sorted(seen_codes.items()):
            print(f"  key_code: {code}  # {name}")
        sample = next(iter(seen_codes))
        print(
            "\nExample after identifying Pedal 2:\n"
            "  pedals:\n"
            "    pedal_2:\n"
            f"      key_code: {sample}\n"
            "      action: freedrive_toggle"
        )
        print(
            "\nTo implement Pedal 1 / Pedal 3 later:\n"
            "  1. Set their key_code from this listing.\n"
            "  2. Change action from 'none' to 'portable_device' or "
            "'pre_programmed'.\n"
            "  3. Register a PedalActionHandler (see src/foot_pedal_input.py) "
            "or extend FreedriveController._on_pedal_extension()."
        )
    else:
        print("\nNo KEY_DOWN events observed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
