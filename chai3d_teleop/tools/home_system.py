#!/usr/bin/env python3
"""Move both wrist axes to zero and execute the Flexiv Home primitive."""

from __future__ import annotations

from pathlib import Path
import sys
import time

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.teleoperate import DEFAULT_CONFIG, WristBridge, load_profile  # noqa: E402


def _primitive_finished(states: dict[str, object]) -> bool:
    for key in ("reachedTarget", "terminated", "done"):
        value = states.get(key)
        if value is not None and str(value).lower() in {"1", "true", "yes"}:
            return True
    return False


def main() -> int:
    robot = None
    try:
        import flexivrdk

        profile = load_profile(DEFAULT_CONFIG)
        print("Whole-system Home: wrist zero first, then Flexiv Home.")
        print("Keep the complete robot and wrist workspace clear; keep E-stop reachable.")
        with WristBridge(profile, zero_hold_mask_override=(True, True)) as wrist:
            wrist.wait_ready(float(profile.wrist["zero_timeout_s"]) + 5.0)
            wrist.wait_first_sample(2.0)
            wrist.finish_startup()
            robot = flexivrdk.Robot(
                str(profile.robot["robot_sn"]),
                [str(profile.robot["network_interface_ip"])],
            )
            if robot.fault():
                raise RuntimeError("Flexiv is faulted; clear the fault before Home")
            if not robot.operational():
                robot.Enable()
                deadline = time.monotonic() + 20.0
                while not robot.operational():
                    if time.monotonic() >= deadline:
                        raise TimeoutError("Flexiv did not become operational")
                    time.sleep(0.05)
            robot.SwitchMode(flexivrdk.Mode.NRT_PRIMITIVE_EXECUTION)
            robot.ExecutePrimitive("Home", {})
            deadline = time.monotonic() + 120.0
            while True:
                wrist.command_position(np.zeros(2))
                if robot.fault() or not robot.connected():
                    raise RuntimeError("Flexiv fault/disconnect during Home")
                primitive = dict(robot.primitive_states())
                if _primitive_finished(primitive):
                    break
                if time.monotonic() >= deadline:
                    raise TimeoutError("Flexiv Home did not finish within 120 seconds")
                time.sleep(0.02)
            robot.Stop()
            print("HOME_REACHED arm=1 q8=0 q9=0")
        return 0
    except KeyboardInterrupt:
        print("Ctrl-C: stopping Flexiv and wrist.", file=sys.stderr)
        return 130
    except Exception as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 1
    finally:
        if robot is not None:
            try:
                robot.Stop()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
