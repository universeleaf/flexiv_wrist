#!/usr/bin/env python3
"""Initialize omega.7 when needed, then test gravity compensation and DRD hold."""

from __future__ import annotations

import os
from pathlib import Path
import sys


SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parents[1]
VENV_ROOT = PROJECT_ROOT.parent / "freedrive_python" / ".venv"
VENV_PYTHON = VENV_ROOT / "bin" / "python"


def ensure_project_python() -> None:
    if Path(sys.prefix).resolve() == VENV_ROOT.resolve():
        return
    if not VENV_PYTHON.is_file():
        raise FileNotFoundError(f"找不到项目 Python 环境: {VENV_PYTHON}")
    os.execv(str(VENV_PYTHON), [str(VENV_PYTHON), str(SCRIPT_PATH)])


def main() -> int:
    ensure_project_python()
    from run_teleop import main as run_teleop_main

    print("omega.7 单机初始化/保持测试：不会连接或移动 Flexiv。")
    print("程序会检查初始化状态；仅在本次上电尚未初始化时自动校准。")
    print("若出现 CHAI3D_CALIBRATION starting：不要触碰手柄，等待完成。")
    print("完成后：无论 gripper 是否按下都保持重力平衡；手推时应能自由移动。")
    print("松开 gripper 只停止遥操作，不启用 DRD 刚性位置锁定。")
    return run_teleop_main(
        [
            "--device",
            "0",
            "--switch",
            "0",
            "--device-rate",
            "1000",
            "--gravity-compensation",
            "--scale",
            "2.0",
            "--axis-map=-x,-y,z",
            "--unlimited-translation",
            "--unlimited-step",
            "--enable-rotation",
            "--unlimited-rotation",
            "--unlimited-angular-step",
        ]
    )


if __name__ == "__main__":
    raise SystemExit(main())
