#!/usr/bin/env python3
"""Short command dispatcher for every mission, tool, UI, and test."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys
import tomllib

ROOT = Path(__file__).resolve().parent
FLEXIV_PY = ROOT.parent / "freedrive_python" / ".venv" / "bin" / "python"
MOTEUS_PY = ROOT / ".venv_moteus" / "bin" / "python"


def _run(command: list[str]) -> int:
    print("RUN:", " ".join(command), flush=True)
    return subprocess.run(command, cwd=ROOT, check=False).returncode


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Flexiv wrist control: python run.py <command>"
    )
    parser.add_argument(
        "command",
        choices=(
            "demo7", "demo9", "teleop", "set-zero", "go-zero", "home",
            "identify-inertia", "dynamic-inertia", "check-inertia", "check-tool", "apply-pid",
            "ui", "build", "test",
        ),
    )
    args = parser.parse_args()
    py = str(FLEXIV_PY)
    with (ROOT / "config" / "nine_dof_teleop.toml").open("rb") as stream:
        saved = tomllib.load(stream)
    demo = saved["demo"]
    common_demo = [
        "--trajectory", str(demo["trajectory"]),
        "--duration-s", str(demo["period_s"]),
        "--radius-m", str(demo["radius_m"]),
        "--orientation-deg", str(demo["orientation_amplitude_deg"]),
        "--endpoint", str(demo["endpoint"]),
        "--cpu-affinity", str(demo["cpu_affinity"]),
        "--real",
    ]
    commands = {
        "demo7": [py, "-m", "scripts.demo_7dof", *common_demo, "--confirm", "RUN_7DOF_TORQUE_OSC"],
        "demo9": [py, "-m", "scripts.demo_9dof", *common_demo, "--confirm", "RUN_9DOF_TORQUE_OSC"],
        "teleop": [py, "-m", "scripts.teleoperate"],
        "set-zero": [str(MOTEUS_PY), "-m", "tools.set_wrist_zero", "--confirm-set-zero", "SET_CURRENT_WRIST_ZERO"],
        "go-zero": [py, "-m", "tools.move_wrist_zero"],
        "home": [py, "-m", "tools.home_system"],
        "identify-inertia": [py, "-m", "tools.auto_identify_inertia"],
        "dynamic-inertia": [py, "-m", "tools.dynamic_inertia", "--duration-s", "45", "--period-s", "8", "--amplitude-deg", "20", "45", "--position-kp-scale", "0.35", "0.35", "--sample-hz", "100", "--print-hz", "2", "--output", "/tmp/wrist_dynamic_inertia.csv", "--confirm-move", "RUN_WRIST_DYNAMIC_INERTIA"],
        "check-inertia": [py, "-m", "tools.check_inertia"],
        "check-tool": [py, "-m", "tools.check_flexiv_tool"],
        "apply-pid": [str(MOTEUS_PY), "-m", "tools.apply_motor_pid", "--confirm", "APPLY_MOTOR_PID"],
        "ui": [py, "-m", "ui.control_panel"],
        "build": ["bash", str(ROOT / "tools" / "build_rt_osc.sh")],
        "test": [py, "-m", "pytest", "-q", "testing/unit"],
    }
    command = commands[args.command]
    if args.command in {"demo7", "demo9", "teleop", "go-zero", "home", "identify-inertia", "dynamic-inertia", "apply-pid"}:
        print("REAL HARDWARE TASK: workspace clear, active Tool correct, E-stop reachable.")
    return _run(command)


if __name__ == "__main__":
    raise SystemExit(main())
