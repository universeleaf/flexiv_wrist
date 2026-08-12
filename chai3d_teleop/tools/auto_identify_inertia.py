#!/usr/bin/env python3
"""Collect wrist excitation data, fit dynamics, and save calibration in one run."""

from __future__ import annotations

from pathlib import Path
import sys

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.teleoperate import DEFAULT_CONFIG, load_profile  # noqa: E402
from tools.identify_wrist_inertia import analyze, collect  # noqa: E402


def main() -> int:
    raw = Path("/tmp/wrist_inertia_samples.csv")
    output = PROJECT_ROOT / "config" / "wrist_inertia_calibration.json"
    try:
        profile = load_profile(DEFAULT_CONFIG)
        print("Automatic wrist identification: saved zero is unchanged.")
        print("Moving q8/q9 with smooth multi-sine excitation; Ctrl-C sends STOP.")
        collect(
            profile,
            raw,
            duration_s=90.0,
            amplitudes_deg=np.asarray([15.0, 30.0]),
            position_kp_scale=np.asarray([0.35, 0.35]),
            position_kd_scale=np.asarray([1.0, 1.0]),
        )
        result = analyze(profile, raw, output)
        if result.get("calibration_status") != "PASS":
            raise RuntimeError(
                "惯量辨识质量检查未通过，结果已保存但不会用于 9DoF："
                + "; ".join(result.get("calibration_failure_reasons", []))
            )
        print(f"INERTIA_IDENTIFICATION_COMPLETE raw={raw} calibration={output}")
        return 0
    except KeyboardInterrupt:
        print("Ctrl-C: wrist STOP requested.", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
