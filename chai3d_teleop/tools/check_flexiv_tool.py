#!/usr/bin/env python3
"""Read the active Flexiv Tool and wrench without commanding robot motion."""

from __future__ import annotations

import os
from pathlib import Path
import sys
import tomllib

SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parents[1]
VENV_ROOT = PROJECT_ROOT.parent / "freedrive_python" / ".venv"
VENV_PYTHON = VENV_ROOT / "bin" / "python"
CONFIG_PATH = PROJECT_ROOT / "config" / "nine_dof_teleop.toml"


def ensure_project_python() -> None:
    if Path(sys.prefix).resolve() == VENV_ROOT.resolve():
        return
    if not VENV_PYTHON.is_file():
        raise FileNotFoundError(f"找不到 Flexiv Python 环境: {VENV_PYTHON}")
    os.execv(str(VENV_PYTHON), [str(VENV_PYTHON), str(SCRIPT_PATH)])


def vector_text(values: object, precision: int = 4) -> str:
    return ", ".join(f"{float(value):+.{precision}f}" for value in values)


def main() -> int:
    ensure_project_python()
    import flexivrdk
    import numpy as np

    with CONFIG_PATH.open("rb") as stream:
        robot_config = tomllib.load(stream)["robot"]
    robot_sn = str(robot_config["robot_sn"])
    network_interface_ip = str(robot_config["network_interface_ip"])

    print("只读检查：不会切换控制模式，也不会发送运动命令。")
    robot = flexivrdk.Robot(robot_sn, [network_interface_ip])
    tool = flexivrdk.Tool(robot)
    params = tool.params()
    states = robot.states()
    compensated = np.asarray(states.ext_wrench_in_world, dtype=float)
    unfiltered = np.asarray(states.ext_wrench_in_world_raw, dtype=float)

    print(f"robot_sn={robot_sn}")
    print(f"active_tool={tool.name()}")
    print(f"mass_kg={float(params.mass):.6f}")
    print(f"CoM_m=[{vector_text(params.CoM)}]")
    print(f"inertia=[{vector_text(params.inertia, precision=6)}]")
    print(f"tcp_location=[{vector_text(params.tcp_location)}]")
    print(
        "compensated_world_wrench=[{}] norm_force_N={:.3f}".format(
            vector_text(compensated), float(np.linalg.norm(compensated[:3]))
        )
    )
    print(
        "unfiltered_world_wrench=[{}] norm_force_N={:.3f}".format(
            vector_text(unfiltered), float(np.linalg.norm(unfiltered[:3]))
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
