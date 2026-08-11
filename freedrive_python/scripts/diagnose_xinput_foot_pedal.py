#!/usr/bin/env python3
"""Test the physical foot pedal through XInput without /dev/input access.

This script never connects to the robot.
"""

from __future__ import annotations

import argparse
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
    XInputFootPedalReader,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Listen only to the configured physical PCsensor/iKKEGOL XInput "
            "device. No robot connection and no /dev/input permission are used."
        )
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_FOOT_PEDAL_CONFIG,
        help="Foot pedal YAML (default: config/foot_pedal.yaml)",
    )
    parser.add_argument(
        "--seconds",
        type=float,
        default=30.0,
        help="Test duration (0 = until Ctrl+C)",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        config = load_foot_pedal_configuration(args.config)
        reader = XInputFootPedalReader(
            config,
            output=print,
            emit_all_events=True,
            emit_unmapped_events=True,
        )
    except (ConfigurationError, FootPedalError) as exc:
        print(f"pedal setup error: {exc}", file=sys.stderr)
        return 1

    count = 0
    try:
        reader.arm()
        info = reader.device_info
        vendor = f"{info.vendor_id:#06x}" if info.vendor_id is not None else "unknown"
        product = (
            f"{info.product_id:#06x}" if info.product_id is not None else "unknown"
        )
        print("=== XInput physical-pedal test (robot is NOT connected) ===")
        print(f"selected device: {info.name!r}")
        print(f"source: {info.path}")
        print(f"USB: {vendor}:{product}")
        print(
            "依次短踩并松开 Pedal 1、2、3；现在会显示该物理设备发出的所有 KEY_DOWN，\n"
            "包括尚未写入 YAML 的键值。实体键盘事件不会被接收。Ctrl+C 或键盘 q 退出。"
        )
        deadline = None if args.seconds <= 0 else time.monotonic() + args.seconds
        while deadline is None or time.monotonic() < deadline:
            for event in reader.poll():
                count += 1
                print(
                    f"PHYSICAL_PEDAL_KEY_DOWN count={count} "
                    f"physical_device={info.name!r} key_code={event.key_code} "
                    f"mapped_pedal={event.pedal_id} action={event.action}"
                )
            if reader.quit_requested:
                break
            time.sleep(0.02)
    except KeyboardInterrupt:
        print("\nStopped.")
    except FootPedalError as exc:
        print(f"\npedal read error: {exc}", file=sys.stderr)
        return 1
    finally:
        reader.close()

    print(f"test complete; physical KEY_DOWN events observed: {count}")
    return 0 if count > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
