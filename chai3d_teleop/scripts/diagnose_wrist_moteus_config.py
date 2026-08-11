#!/usr/bin/env python3
"""STOP both wrist servos and report position-loop configuration read-only."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import re
import subprocess
import sys
import tomllib
from typing import Any


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = PROJECT_ROOT / "config" / "nine_dof_teleop.toml"
DEFAULT_OUTPUT_DIR = Path("/tmp")

KEY_PATTERN = re.compile(r"^[A-Za-z][A-Za-z0-9_.]*$")
REPORT_PREFIXES = (
    "id.id",
    "motor.invert",
    "motor.phase_invert",
    "motor.unwrapped_position_scale",
    "motor_position.commutation_source",
    "motor_position.output.",
    "motor_position.rotor_to_output_",
    "motor_position.sources.",
    "servo.pid_position.",
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
)


def parse_config_dump(text: str) -> dict[str, str]:
    result: dict[str, str] = {}
    for raw_line in text.splitlines():
        fields = raw_line.strip().split(maxsplit=1)
        if len(fields) != 2 or KEY_PATTERN.fullmatch(fields[0]) is None:
            continue
        result[fields[0]] = fields[1].strip()
    return result


def _number(config: dict[str, str], key: str) -> float | None:
    try:
        value = float(config[key])
    except (KeyError, ValueError):
        return None
    return value


def analyze_position_config(
    config: dict[str, str], *, expected_reduction: float
) -> list[str]:
    findings: list[str] = []
    output_source_value = _number(config, "motor_position.output.source")
    ratio = _number(config, "motor_position.rotor_to_output_ratio")
    output_sign = _number(config, "motor_position.output.sign")
    fixed_voltage = _number(config, "servo.fixed_voltage_mode")
    kp = _number(config, "servo.pid_position.kp")
    kd = _number(config, "servo.pid_position.kd")

    if output_source_value is None:
        findings.append("ERROR: missing motor_position.output.source")
    else:
        source = int(output_source_value)
        reference = _number(config, f"motor_position.sources.{source}.reference")
        source_sign = _number(config, f"motor_position.sources.{source}.sign")
        findings.append(
            "INFO: output source={} reference={} sign={}".format(
                source,
                "missing" if reference is None else f"{reference:g}",
                "missing" if source_sign is None else f"{source_sign:g}",
            )
        )

    expected_ratio = 1.0 / expected_reduction
    if ratio is None:
        findings.append("ERROR: missing motor_position.rotor_to_output_ratio")
    elif not math.isclose(ratio, expected_ratio, rel_tol=0.05, abs_tol=1e-6):
        findings.append(
            "WARN: rotor_to_output_ratio={:.9g}, nominal reducer-only value is {:.9g} for {:.1f}:1; verify any belt ratio".format(
                ratio, expected_ratio, expected_reduction
            )
        )
    else:
        findings.append(
            f"OK: rotor_to_output_ratio={ratio:.9g} matches {expected_reduction:.1f}:1 reducer"
        )

    if output_sign is None:
        findings.append("ERROR: missing motor_position.output.sign")
    elif output_sign != 1.0:
        findings.append(
            "ERROR: motor_position.output.sign is not +1; this is unsafe on the installed 2024 firmware"
        )
    else:
        findings.append("OK: motor_position.output.sign=+1")

    if fixed_voltage not in (None, 0.0):
        findings.append("ERROR: servo.fixed_voltage_mode is enabled")
    if kp is None or kd is None:
        findings.append("ERROR: position PID kp/kd is missing")
    else:
        findings.append(f"INFO: position PID kp={kp:g}, kd={kd:g}")
        if kp <= 0.0:
            findings.append("ERROR: position PID kp must be positive")
        if kd < 0.0:
            findings.append("ERROR: position PID kd must not be negative")
        elif kd == 0.0:
            findings.append("WARN: position PID kd is zero; the loaded reducer has no active damping")
    return findings


def _run(command: list[str], *, timeout_s: float = 180.0) -> str:
    try:
        completed = subprocess.run(
            command,
            cwd=PROJECT_ROOT,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=timeout_s,
            check=False,
        )
    except subprocess.TimeoutExpired as error:
        partial = error.stdout or ""
        if isinstance(partial, bytes):
            partial = partial.decode("utf-8", errors="replace")
        raise TimeoutError(
            f"read-only moteus query exceeded {timeout_s:g}s\n{partial}"
        ) from error
    if completed.returncode != 0:
        raise RuntimeError(
            "command failed (code {}): {}\n{}".format(
                completed.returncode, " ".join(command), completed.stdout
            )
        )
    return completed.stdout


def _load(path: Path) -> tuple[list[int], list[float], str]:
    with path.open("rb") as stream:
        document: dict[str, Any] = tomllib.load(stream)
    wrist = document["wrist"]
    ids = [int(value) for value in wrist["ids"]]
    reductions = [float(value) for value in wrist["reduction_ratio"]]
    device = str(wrist["fdcanusb"])
    if len(ids) != 2 or len(reductions) != 2:
        raise ValueError("wrist ids/reduction_ratio must each contain two values")
    return ids, reductions, device


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    args = parser.parse_args()
    try:
        config_path = args.config.expanduser().resolve()
        ids, reductions, device = _load(config_path)
        python = PROJECT_ROOT / ".venv_moteus" / "bin" / "python"
        reader = PROJECT_ROOT / "scripts" / "read_moteus_position_config.py"
        if not python.is_file():
            raise FileNotFoundError(f"missing moteus Python: {python}")
        args.output_dir.mkdir(parents=True, exist_ok=True)
        print("只读腕部位置环诊断：发送 STOP 后读取配置；不会使能或移动电机/Flexiv。")
        for motor_id, reduction in zip(ids, reductions):
            print(f"正在读取 ID {motor_id} 的位置环关键参数...", flush=True)
            command = [
                str(python),
                str(reader),
                "--target",
                str(motor_id),
                "--fdcanusb",
                device,
            ]
            raw_report = _run(command)
            report = json.loads(raw_report)
            parsed = {
                str(key): str(value)
                for key, value in dict(report.get("values", {})).items()
            }
            output_path = args.output_dir / f"moteus_id{motor_id}_config.txt"
            output_path.write_text(
                json.dumps(report, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            print(f"\n=== ID {motor_id} ({reduction:g}:1 reducer) ===")
            print(f"drained_stale_bytes {int(report.get('drained_stale_bytes', 0))}")
            for key in sorted(parsed):
                if any(key == prefix or key.startswith(prefix) for prefix in REPORT_PREFIXES):
                    print(f"{key} {parsed[key]}")
            print("-- analysis --")
            for finding in analyze_position_config(
                parsed, expected_reduction=reduction
            ):
                print(finding)
            print(f"targeted_config={output_path}")
        print("\n诊断完成；两台电机保持 STOP。请把以上完整终端输出发回来。")
        return 0
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C。", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"错误: {type(error).__name__}: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
