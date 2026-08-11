#!/usr/bin/env python3
"""Read-only Flexiv RDK connection/version/fault check.

This script does not call Enable, Stop, ClearFault, SwitchMode, Home, or any
motion/primitive API.
"""

from __future__ import annotations

import argparse
import ipaddress
import sys


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Connect to the robot through one Ethernet interface and print "
            "identity/fault information without sending robot commands."
        )
    )
    parser.add_argument("--robot-sn", default="Rizon4s-123456")
    parser.add_argument("--network-interface-ip", default="127.0.0.1")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        interface_ip = ipaddress.ip_address(args.network_interface_ip)
    except ValueError:
        print(
            f"invalid interface IPv4 address: {args.network_interface_ip!r}",
            file=sys.stderr,
        )
        return 2
    if interface_ip.version != 4 or interface_ip.is_unspecified:
        print("network interface must be a usable IPv4 address", file=sys.stderr)
        return 2

    try:
        import flexivrdk
    except ImportError as exc:
        print(f"Flexiv RDK import failed: {exc}", file=sys.stderr)
        return 2

    print("READ-ONLY CHECK: no Enable/Stop/ClearFault/mode/motion calls will be made.")
    print(f"flexivrdk={flexivrdk.__version__}")
    print(f"robot_sn={args.robot_sn}")
    print(f"network_interface_whitelist={[str(interface_ip)]}")

    try:
        robot = flexivrdk.Robot(args.robot_sn, [str(interface_ip)])
        info = robot.info()
        print("connection=OK")
        print(f"model={info.model_name}")
        print(f"serial={getattr(info, 'serial_num', args.robot_sn)}")
        print(f"robot_software={info.software_ver}")
        print(f"license={info.license_type}")
        print(f"connected={robot.connected()}")
        print(f"fault={robot.fault()}")
        print(f"operational={robot.operational()}")
        print(f"operational_status={robot.operational_status()}")
        try:
            events = list(robot.event_log())
        except Exception as exc:
            print(f"event_log_unavailable={exc}")
        else:
            print(f"event_count={len(events)}")
            for event in events[-20:]:
                print(f"event={event}")
        return 3 if robot.fault() else 0
    except Exception as exc:
        print(f"connection=FAILED: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
