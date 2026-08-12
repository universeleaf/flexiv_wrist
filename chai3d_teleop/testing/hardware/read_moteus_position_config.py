#!/usr/bin/env python3
"""Internal targeted moteus config reader; emits JSON and never enables motion."""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import sys
import time

import moteus


CONFIG_KEYS = [
    "id.id",
    "motor.invert",
    "motor.phase_invert",
    "motor.unwrapped_position_scale",
    "motor_position.commutation_source",
    "motor_position.output.source",
    "motor_position.output.sign",
    "motor_position.output.offset",
    "motor_position.output.reference_source",
    "motor_position.rotor_to_output_ratio",
    "motor_position.rotor_to_output_override",
    "servo.pid_position.kp",
    "servo.pid_position.ki",
    "servo.pid_position.kd",
    "servo.pid_position.ilimit",
    "servo.pid_position.iratelimit",
    "servo.pid_position.max_desired_rate",
    "servo.default_velocity_limit",
    "servo.default_accel_limit",
    "servo.max_position_slip",
    "servo.max_velocity_slip",
    "servo.max_velocity",
    "servo.fixed_voltage_mode",
    "servo.voltage_mode_control",
    "servo.max_current_A",
    "servopos.position_min",
    "servopos.position_max",
]
for source in range(3):
    CONFIG_KEYS.extend(
        [
            f"motor_position.sources.{source}.type",
            f"motor_position.sources.{source}.aux_number",
            f"motor_position.sources.{source}.reference",
            f"motor_position.sources.{source}.sign",
            f"motor_position.sources.{source}.cpr",
            f"motor_position.sources.{source}.pll_filter_hz",
        ]
    )


def _numeric_response(text: str) -> str | None:
    """Return a scalar config value, rejecting stale `conf enumerate` rows."""
    stripped = text.strip()
    if not stripped or any(character.isspace() for character in stripped):
        return None
    try:
        value = float(stripped)
    except ValueError:
        return None
    if math.isfinite(value) or stripped.lower() in {"nan", "+nan", "-nan"}:
        return stripped
    return stripped if stripped.lower() in {"inf", "+inf", "-inf"} else None


async def _read_one_config(stream: moteus.Stream, key: str) -> tuple[str, str]:
    """Query one numeric key while discarding stale diagnostic stream rows."""
    await stream.write_message(f"conf get {key}".encode("latin1"))
    deadline = time.monotonic() + 8.0
    discarded = 0
    while True:
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError(
                f"conf get {key} timed out after discarding {discarded} stale rows"
            )
        try:
            response = await asyncio.wait_for(stream.readline(), timeout=remaining)
        except asyncio.TimeoutError as error:
            raise TimeoutError(
                f"conf get {key} received no scalar response after discarding "
                f"{discarded} stale rows"
            ) from error
        decoded = response.decode("latin1").strip()
        if decoded.startswith("ERR"):
            return "unsupported", decoded
        numeric = _numeric_response(decoded)
        if numeric is not None:
            return "value", numeric
        discarded += 1


async def _drain_diagnostic_stream(
    stream: moteus.Stream, *, maximum_s: float = 30.0, quiet_s: float = 0.5
) -> int:
    """Drain old stream data using the same flow-control path as later reads."""
    started = time.monotonic()
    quiet_since: float | None = None
    drained_bytes = len(stream._read_data)
    stream._read_data = b""
    while time.monotonic() - started < maximum_s:
        try:
            async with stream.lock:
                chunk = await asyncio.wait_for(
                    stream._do_diagnostic_read(stream._maxlen), timeout=1.0
                )
        except asyncio.TimeoutError:
            chunk = b""
        if chunk:
            drained_bytes += len(chunk)
            quiet_since = None
            continue
        now = time.monotonic()
        if quiet_since is None:
            quiet_since = now
        elif now - quiet_since >= quiet_s:
            stream._read_data = b""
            return drained_bytes
        await asyncio.sleep(0.01)
    raise TimeoutError(
        f"diagnostic stream did not become quiet after {maximum_s:g}s "
        f"(drained {drained_bytes} bytes)"
    )


async def read_config(args: argparse.Namespace) -> dict[str, object]:
    transport = moteus.get_singleton_transport(args)
    controller = moteus.Controller(
        id=args.target, transport=transport, can_prefix=args.can_prefix
    )
    stream = moteus.Stream(controller, channel=args.diagnostic_channel)
    values: dict[str, str] = {}
    unsupported: dict[str, str] = {}
    try:
        try:
            await asyncio.wait_for(controller.set_stop(), timeout=2.0)
        except asyncio.TimeoutError as error:
            raise TimeoutError("STOP register command timed out") from error
        # Stop periodic telemetry before issuing many diagnostic requests.
        try:
            await asyncio.wait_for(stream.write_message(b"tel stop"), timeout=2.0)
        except asyncio.TimeoutError as error:
            raise TimeoutError("tel stop diagnostic write timed out") from error
        # A previously interrupted `conf enumerate` can leave hundreds of
        # flow-controlled "key value" rows queued in the device. The library's
        # regular flush path intentionally bypasses flow control, so use the
        # same acknowledged path as normal reads here.
        drained_bytes = await _drain_diagnostic_stream(stream)
        for key in CONFIG_KEYS:
            kind, decoded = await _read_one_config(stream, key)
            if kind == "unsupported":
                unsupported[key] = decoded
            else:
                values[key] = decoded
        return {
            "target": args.target,
            "drained_stale_bytes": drained_bytes,
            "values": values,
            "unsupported": unsupported,
        }
    finally:
        try:
            await asyncio.wait_for(controller.set_stop(), timeout=2.0)
        finally:
            if hasattr(transport, "close"):
                transport.close()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--target", type=int, required=True)
    # This belongs to moteus.Controller rather than the transport factory, so
    # moteus.make_transport_args() does not add it for us.
    parser.add_argument("--can-prefix", type=lambda value: int(value, 0), default=0)
    parser.add_argument("--diagnostic-channel", type=int, default=1)
    moteus.make_transport_args(parser)
    args = parser.parse_args()
    try:
        print(json.dumps(asyncio.run(read_config(args)), sort_keys=True))
        return 0
    except Exception as error:
        print(f"{type(error).__name__}: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
