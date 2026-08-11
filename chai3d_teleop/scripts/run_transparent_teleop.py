#!/usr/bin/env python3
"""Run the saved omega.7/Flexiv transparent teleoperation profile."""

from __future__ import annotations

import os
from pathlib import Path
import sys


SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parents[1]
VENV_ROOT = PROJECT_ROOT.parent / "freedrive_python" / ".venv"
VENV_PYTHON = VENV_ROOT / "bin" / "python"
CONFIG_PATH = PROJECT_ROOT / "config" / "transparent_omega7.toml"


def ensure_project_python() -> None:
    if Path(sys.prefix).resolve() == VENV_ROOT.resolve():
        return
    if not VENV_PYTHON.is_file():
        raise FileNotFoundError(f"找不到 Flexiv Python 环境: {VENV_PYTHON}")
    os.execv(str(VENV_PYTHON), [str(VENV_PYTHON), str(SCRIPT_PATH)])


def main() -> int:
    ensure_project_python()
    from run_teleop import config_to_argv, main as run_teleop_main

    print(f"加载保存的透明遥操作配置: {CONFIG_PATH}")
    print(
        "该配置会连接真实 Flexiv，并启用 2:1 位移和保守的 "
        "0.25x filtered 力反馈；15 N 后渐进反弹，设备输出只在 12 N 额定值饱和。"
    )
    return run_teleop_main(config_to_argv(CONFIG_PATH))


if __name__ == "__main__":
    raise SystemExit(main())
