#!/usr/bin/env python3
"""Move joint 8 and joint 9 to the saved application zero, then STOP."""

from __future__ import annotations

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.teleoperate import DEFAULT_CONFIG, WristBridge, load_profile  # noqa: E402


def main() -> int:
    try:
        profile = load_profile(DEFAULT_CONFIG)
        print("Moving q8 and q9 to the saved zero. Keep the wrist workspace clear.")
        with WristBridge(profile, zero_hold_mask_override=(True, True)) as wrist:
            wrist.wait_ready(float(profile.wrist["zero_timeout_s"]) + 5.0)
            sample = wrist.wait_first_sample(2.0)
            print(
                "WRIST_ZERO_REACHED q8={:+.3f}deg q9={:+.3f}deg; sending STOP".format(
                    *(__import__("numpy").rad2deg(sample.q_rad))
                )
            )
        return 0
    except KeyboardInterrupt:
        print("Ctrl-C: wrist STOP requested.", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
