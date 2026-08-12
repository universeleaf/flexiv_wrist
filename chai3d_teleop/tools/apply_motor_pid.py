#!/usr/bin/env python3
"""Apply and persist the two moteus position PID triplets from TOML."""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys
import tempfile
import tomllib

PROJECT_ROOT = Path(__file__).resolve().parents[1]
CONFIG = PROJECT_ROOT / "config" / "nine_dof_teleop.toml"
MOTEUS_PYTHON = PROJECT_ROOT / ".venv_moteus" / "bin" / "python"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--confirm", default="")
    args = parser.parse_args()
    try:
        document = tomllib.loads(CONFIG.read_text(encoding="utf-8"))
        wrist = document["wrist"]
        pid = document["motor_pid"]
        ids = [int(value) for value in wrist["ids"]]
        gains = [
            (float(pid["joint8_kp"]), float(pid["joint8_ki"]), float(pid["joint8_kd"])),
            (float(pid["joint9_kp"]), float(pid["joint9_ki"]), float(pid["joint9_kd"])),
        ]
        for values in gains:
            kp, ki, kd = values
            if not (0.0 < kp <= 5000.0 and 0.0 <= ki <= 100.0 and 0.0 < kd <= 200.0):
                raise ValueError(f"PID outside supported commissioning range: {values}")
        print(f"PID preview: ID{ids[0]}={gains[0]}, ID{ids[1]}={gains[1]}")
        if args.confirm != "APPLY_MOTOR_PID":
            print("Preview only. Use the UI Apply PID task or python run.py apply-pid.")
            return 0
        for motor_id, (kp, ki, kd) in zip(ids, gains):
            with tempfile.NamedTemporaryFile("w", suffix=".cfg", dir="/tmp") as stream:
                stream.write(f"conf set servo.pid_position.kp {kp}\n")
                stream.write(f"conf set servo.pid_position.ki {ki}\n")
                stream.write(f"conf set servo.pid_position.kd {kd}\n")
                stream.write("conf write\n")
                stream.flush()
                command = [
                    str(MOTEUS_PYTHON), "-m", "moteus.moteus_tool",
                    "-t", str(motor_id), "--fdcanusb", str(wrist["fdcanusb"]),
                    "--no-tel-stop", "--write-config", stream.name,
                ]
                completed = subprocess.run(command, cwd=PROJECT_ROOT, check=False)
                if completed.returncode:
                    raise RuntimeError(f"moteus ID {motor_id} PID write failed")
        print("MOTOR_PID_APPLIED and saved to persistent moteus storage")
        return 0
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
