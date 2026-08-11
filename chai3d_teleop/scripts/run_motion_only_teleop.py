#!/usr/bin/env python3
"""Run the saved real Flexiv profile with haptic force feedback disabled."""

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

    argv = config_to_argv(CONFIG_PATH)
    argv = [argument for argument in argv if argument != "--enable-force-feedback"]
    # Keep the full workspace mapping, but use reduced generator rates for
    # this diagnostic real-robot run only. The normal saved profile is not
    # changed by these appended overrides.
    argv.extend(
        [
            "--max-linear-velocity=0.1",
            "--max-angular-velocity=0.3",
            "--max-linear-acceleration=0.5",
            "--max-angular-acceleration=1.0",
        ]
    )
    print("真实 Flexiv 运动隔离测试：保留全空间位姿映射，但完全关闭触觉力反馈。")
    print("仅此诊断入口使用较低速度/加速度；正常保存配置没有被降低。")
    print("omega.7 只输出自身重力补偿；如果仍抖动，问题不在 Flexiv wrench 回传。")
    return run_teleop_main(argv)


if __name__ == "__main__":
    raise SystemExit(main())
