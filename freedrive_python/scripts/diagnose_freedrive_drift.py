#!/usr/bin/env python3
"""Record robot/tool/sensor state without starting a floating primitive.

This is a thin wrapper around run_freedrive.py --diagnose-only.
"""

from __future__ import annotations

import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.run_freedrive import main as run_main  # noqa: E402


def main(argv: list[str] | None = None) -> int:
    args = list(argv if argv is not None else sys.argv[1:])
    if "--diagnose-only" not in args:
        args.append("--diagnose-only")
    return run_main(args)


if __name__ == "__main__":
    raise SystemExit(main())
