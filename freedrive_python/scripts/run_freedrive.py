#!/usr/bin/env python3
"""Operator CLI for Flexiv Rizon 4S Cartesian freedrive."""

from __future__ import annotations

import argparse
import ipaddress
import json
import sys
from datetime import datetime
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.foot_pedal_configuration import (  # noqa: E402
    DEFAULT_FOOT_PEDAL_CONFIG,
    load_foot_pedal_configuration,
)
from src.foot_pedal_input import (  # noqa: E402
    FootPedalError,
    FootPedalReader,
    XInputFootPedalReader,
)
from src.freedrive_configuration import (  # noqa: E402
    ConfigurationError,
    FreedriveConfiguration,
)
from src.freedrive_controller import FreedriveController, FreedriveError, SCHEMA_NOTE  # noqa: E402
from src.freedrive_csv_logger import FreedriveCSVLogger  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Flexiv Rizon 4S six-axis Cartesian freedrive. Default mode never commands the robot. "
            "With --confirm-motion, press/release enabling button repeatedly, or use Pedal 2 "
            "toggle when --foot-pedal is configured; "
            "only Ctrl+C or q+Enter terminates the program."
        )
    )
    parser.add_argument("robot_sn", help="Robot serial number, e.g. Rizon4s-123456")
    parser.add_argument(
        "--network-interface-ip",
        default=None,
        help=(
            "IPv4 address of the PC Ethernet interface connected to the robot. "
            "When set, Flexiv RDK searches only this interface, avoiding Wi-Fi/DDS "
            "cross-talk on multi-network computers."
        ),
    )
    parser.add_argument("--rate", type=float, default=20.0, help="Sample rate Hz")
    parser.add_argument("--debounce-ms", type=float, default=80.0)
    parser.add_argument("--startup-timeout", type=float, default=30.0)
    parser.add_argument("--stop-timeout", type=float, default=10.0)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--print-command-only",
        action="store_true",
        help="Validate and print the primitive command without connecting",
    )
    parser.add_argument(
        "--diagnose-only",
        action="store_true",
        help="Connect and log state/tool/wrench without starting freedrive",
    )
    parser.add_argument(
        "--confirm-motion",
        action="store_true",
        help="Allow hardware connection and freedrive",
    )
    parser.add_argument(
        "--foot-pedal",
        action="store_true",
        help=(
            "Hands-free Pedal 2 control from --foot-pedal-config. When Pedal 2 is "
            "armed, skips typing EXECUTE FREEDRIVE and uses KEY_DOWN to enter/exit "
            "Freedrive (no keyboard required after launch)."
        ),
    )
    parser.add_argument(
        "--foot-pedal-backend",
        choices=("xinput", "evdev"),
        default="xinput",
        help=(
            "Physical pedal event source. xinput (default) identifies the PCsensor "
            "device through the local X11 session and needs no /dev/input permission; "
            "evdev opens the configured /dev/input path."
        ),
    )
    parser.add_argument(
        "--foot-pedal-config",
        type=Path,
        default=DEFAULT_FOOT_PEDAL_CONFIG,
        help="YAML foot-pedal configuration (default: config/foot_pedal.yaml)",
    )
    return parser


def build_config(args: argparse.Namespace) -> FreedriveConfiguration:
    if args.rate <= 0:
        raise ConfigurationError("rate must be positive")
    return FreedriveConfiguration(
        sample_period_s=1.0 / args.rate,
        debounce_s=args.debounce_ms / 1000.0,
        startup_timeout_s=args.startup_timeout,
        stop_timeout_s=args.stop_timeout,
        diagnose_only=args.diagnose_only,
        print_command_only=args.print_command_only,
        confirm_motion=args.confirm_motion,
    )


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    foot_pedal_config = None
    try:
        if not args.robot_sn.lower().startswith("rizon4s-") or " " in args.robot_sn:
            raise ConfigurationError(
                "robot_sn must be the Rizon 4S serial without spaces, "
                "for example Rizon4s-123456"
            )
        if args.network_interface_ip is not None:
            try:
                interface_ip = ipaddress.ip_address(args.network_interface_ip)
            except ValueError as exc:
                raise ConfigurationError(
                    f"invalid --network-interface-ip: {args.network_interface_ip!r}"
                ) from exc
            if interface_ip.version != 4 or interface_ip.is_unspecified:
                raise ConfigurationError(
                    "--network-interface-ip must be a usable IPv4 address"
                )
        config = build_config(args)
        report = config.command_report()
        if args.foot_pedal:
            foot_pedal_config = load_foot_pedal_configuration(args.foot_pedal_config)
            if not foot_pedal_config.freedrive_toggle_ready():
                raise ConfigurationError(
                    "--foot-pedal was requested but Pedal 2 is not armed; run "
                    "scripts/diagnose_foot_pedal.py and set pedals.pedal_2.key_code"
                )
    except (ConfigurationError, argparse.ArgumentTypeError) as exc:
        print(f"configuration error: {exc}", file=sys.stderr)
        return 2

    print(
        "Actual freedrive command and effective configuration:\n"
        + json.dumps(
            {
                "robot_sn": args.robot_sn,
                "network_interface_whitelist": (
                    [args.network_interface_ip]
                    if args.network_interface_ip is not None
                    else []
                ),
                **report,
            },
            indent=2,
            default=str,
        )
    )
    if foot_pedal_config is not None:
        pedal2 = foot_pedal_config.pedal("pedal_2")
        print(
            "\nFoot pedal config: "
            f"enabled={foot_pedal_config.enabled}, "
            f"backend={args.foot_pedal_backend}, "
            f"pedal_2.key_code={pedal2.key_code}, "
            f"pedal_2.action={pedal2.action}, "
            f"toggle_ready={foot_pedal_config.freedrive_toggle_ready()}"
        )
    print(f"\nWARNING: {SCHEMA_NOTE}")
    print(
        "\nWARNING: unexpected self-motion is a safety issue. If the arm drifts "
        "with nobody touching it, release the enabling button or press Pedal 2 "
        "again to exit Freedrive; use E-stop if needed, and stop testing until "
        "tool payload, mounting, "
        "dynamics calibration, and sensor offsets are verified in Flexiv Elements."
    )

    pedal_toggle = (
        foot_pedal_config is not None and foot_pedal_config.freedrive_toggle_ready()
    )
    motion_allowed = bool(args.confirm_motion)

    if args.print_command_only or (not motion_allowed and not args.diagnose_only):
        print(
            "\nNo robot connection attempted "
            "(print-command-only / validation-only mode)."
        )
        return 0

    if args.diagnose_only and not args.confirm_motion:
        # Diagnose connects but never starts freedrive; still require confirm.
        print(
            "refusing diagnose connection without --confirm-motion "
            "(diagnose connects but does not start freedrive).",
            file=sys.stderr,
        )
        return 2

    if pedal_toggle and not args.diagnose_only:
        print(
            "\nPedal 2 toggle mode: --confirm-motion accepted; no typed confirmation. "
            "Connecting, entering a locked/stopped state (no Home move), then "
            "waiting for Pedal 2 KEY_DOWN "
            "to enter/exit Freedrive. Use E-stop for emergency; Ctrl+C stops "
            "the whole program."
        )
    else:
        prompt = (
            "\nType exactly EXECUTE FREEDRIVE to connect"
            + (
                " for diagnose-only logging (no Home)"
                if args.diagnose_only
                else " and wait in the current pose for the enabling button"
            )
            + ": "
        )
        if input(prompt) != "EXECUTE FREEDRIVE":
            print("confirmation mismatch; robot was not connected.")
            return 1

    robot = None
    foot_pedal_reader = None
    try:
        try:
            import flexivrdk
        except ImportError as exc:
            raise FreedriveError(
                "Flexiv RDK import failed. Use the project .venv and install "
                "requirements.txt (NumPy is required by flexivrdk): "
                f"{exc}"
            ) from exc

        if getattr(flexivrdk, "__version__", None) != "1.9.0":
            raise FreedriveError(
                "flexivrdk 1.9.0 required for Robot Software 3.11.x; "
                f"current={getattr(flexivrdk, '__version__', 'unknown')}"
            )
        csv_path = args.output or (
            PROJECT_ROOT
            / "output"
            / (
                f"{'diagnose' if args.diagnose_only else 'freedrive'}_"
                f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            )
        )

        if (
            foot_pedal_config is not None
            and foot_pedal_config.freedrive_toggle_ready()
            and not args.diagnose_only
        ):
            try:
                if args.foot_pedal_backend == "xinput":
                    foot_pedal_reader = XInputFootPedalReader(
                        foot_pedal_config,
                        output=print,
                    )
                else:
                    foot_pedal_reader = FootPedalReader(
                        foot_pedal_config,
                        grab=True,
                        output=print,
                    )
            except FootPedalError as exc:
                print(f"foot pedal error: {exc}", file=sys.stderr)
                return 1

        with FreedriveCSVLogger(csv_path) as sink:
            try:
                network_interfaces = (
                    [args.network_interface_ip]
                    if args.network_interface_ip is not None
                    else []
                )
                robot = flexivrdk.Robot(args.robot_sn, network_interfaces)
            except Exception as exc:
                raise FreedriveError(
                    f"robot connection initialization failed for {args.robot_sn}: "
                    f"{exc}. Verify the exact serial in Elements, RDK/Robot Software "
                    "compatibility, Ethernet connection/interface whitelist, and that "
                    "no other RDK client controls this robot."
                ) from exc
            try:
                tool = flexivrdk.Tool(robot)
            except Exception as exc:
                raise FreedriveError(f"Tool API initialization failed: {exc}") from exc
            controller = FreedriveController(
                robot=robot,
                primitive_mode=flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION,
                config=config,
                sink=sink,
                coord_factory=flexivrdk.Coord,
                tool_api=tool,
                mode_names=getattr(flexivrdk, "kModeNames", None),
                foot_pedal_config=foot_pedal_config,
                foot_pedal_reader=foot_pedal_reader,
            )
            controller.run()
        print(f"program finished; CSV: {csv_path}")
        return 0
    except KeyboardInterrupt:
        print("\nCtrl+C received; Stop cleanup entered.", file=sys.stderr)
        return 130
    except Exception as exc:
        print(f"execution failed: {exc}", file=sys.stderr)
        return 1
    finally:
        if foot_pedal_reader is not None:
            try:
                foot_pedal_reader.close()
            except Exception:
                pass
        if robot is not None:
            try:
                robot.Stop()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
