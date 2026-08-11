#!/usr/bin/env python3
"""Move joints 8/9 to calibrated zero and hold them for Tool calibration.

This helper starts only the two-moteus wrist bridge. It never connects to or
commands Flexiv or CHAI3D. Keep it running while calibrating the active Tool in
Elements; Ctrl-C sends STOP to both wrist drives.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys
import time

import numpy as np

SCRIPT_PATH = Path(__file__).resolve()
PROJECT_ROOT = SCRIPT_PATH.parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from scripts.run_9dof_teleop import (  # noqa: E402
    DEFAULT_CONFIG,
    WristBridge,
    load_profile,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    profile = load_profile(args.config.resolve())
    tolerance_deg = float(profile.wrist["zero_tolerance_deg"])
    print("腕部 Tool 标定零位保持：只控制 moteus ID2/ID1，不连接 Flexiv 或 CHAI3D。")
    print("正在以保存的绝对零位回零；清空大探针运动路径，Ctrl-C 停止并发送 STOP。")

    try:
        # Tool TCP calibration requires both joints at their calibrated zero,
        # even though normal teleoperation leaves ID1/joint 9 at startup pose.
        with WristBridge(
            profile, zero_hold_mask_override=(True, True)
        ) as wrist:
            wrist.wait_ready(timeout_s=float(profile.wrist["zero_timeout_s"]) + 5.0)
            wrist.wait_first_sample(2.0)
            next_print = 0.0
            while True:
                wrist.command_position(np.zeros(2))
                sample = wrist.latest()
                q_deg = np.rad2deg(sample.q_rad)
                if time.monotonic() >= next_print:
                    print(
                        "WRIST_CALIBRATION_HOLD q8_deg={:+.3f} q9_deg={:+.3f} "
                        "dq8_deg_s={:+.3f} dq9_deg_s={:+.3f} mode={}".format(
                            q_deg[0],
                            q_deg[1],
                            math.degrees(sample.dq_rad_s[0]),
                            math.degrees(sample.dq_rad_s[1]),
                            sample.mode,
                        ),
                        flush=True,
                    )
                    next_print = time.monotonic() + 0.5
                if np.any(np.abs(q_deg) > tolerance_deg + 0.5):
                    raise RuntimeError(
                        f"腕部离开零位容差: q8={q_deg[0]:+.2f}°, q9={q_deg[1]:+.2f}°"
                    )
                time.sleep(0.02)
    except KeyboardInterrupt:
        print("\n收到 Ctrl-C：两台腕部电机已发送 STOP。")
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
