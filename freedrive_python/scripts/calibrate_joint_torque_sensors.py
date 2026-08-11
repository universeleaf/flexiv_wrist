#!/usr/bin/env python3
"""One-time Flexiv joint-torque-sensor calibration utility.

This is intentionally separate from freedrive. Calibration moves the robot to
the controller-recommended upright posture and requires a reboot before the
result takes effect. It must never run automatically at freedrive startup.
"""

from __future__ import annotations

import argparse
import sys


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run one-time joint torque sensor calibration. The robot moves to "
            "its recommended calibration posture and must be rebooted afterward."
        )
    )
    parser.add_argument("robot_sn")
    parser.add_argument("--confirm-calibration", action="store_true")
    args = parser.parse_args()

    if not args.confirm_calibration:
        print(
            "Refusing to connect. Re-run with --confirm-calibration only after "
            "clearing the full workspace and verifying no tool/contact.",
            file=sys.stderr,
        )
        return 2

    if input(
        "The robot WILL MOVE to its calibration posture. Verify active tool is "
        "Flange, no payload/contact, E-stop reachable, and workspace clear. "
        "Type exactly CONNECT FOR TORQUE CALIBRATION: "
    ) != "CONNECT FOR TORQUE CALIBRATION":
        print("Confirmation mismatch; robot was not connected.")
        return 1

    import flexivrdk

    robot = None
    try:
        if flexivrdk.__version__ != "1.9.0":
            raise RuntimeError(
                "flexivrdk 1.9.0 required for Robot Software 3.11.x, "
                f"got {flexivrdk.__version__}"
            )
        robot = flexivrdk.Robot(args.robot_sn)
        if robot.fault():
            raise RuntimeError(
                "Robot is faulted. Diagnose in Elements; this script will not ClearFault."
            )
        if not robot.operational():
            raise RuntimeError(
                "Robot is not operational. Prepare/enable it in Elements first; "
                "this script will not Enable automatically."
            )

        robot.SwitchMode(flexivrdk.Mode.IDLE)
        tool = flexivrdk.Tool(robot)
        tool_name = tool.name()
        tool_params = tool.params()
        print(
            f"Active tool={tool_name}, mass={tool_params.mass} kg, "
            f"CoM={list(tool_params.CoM)}"
        )
        if tool_name != "Flange" or abs(float(tool_params.mass)) > 1e-6:
            raise RuntimeError(
                "Calibration requires the current no-tool setup: active tool must "
                "be Flange with zero mass. Nothing was changed automatically."
            )

        if input(
            "FINAL WARNING: calibration will move the arm and save persistent "
            "sensor offsets. Type exactly CALIBRATE JOINT TORQUE SENSORS: "
        ) != "CALIBRATE JOINT TORQUE SENSORS":
            print("Final confirmation mismatch; calibration not started.")
            return 1

        maintenance = flexivrdk.Maintenance(robot)
        # Empty list selects the controller-recommended upright posture.
        maintenance.CalibrateJointTorqueSensors([])
        print(
            "Calibration completed. STOP HERE and reboot the robot before any "
            "freedrive test; the new result is not active until reboot."
        )
        return 0
    except Exception as exc:
        print(f"Calibration failed/refused: {exc}", file=sys.stderr)
        return 1
    finally:
        if robot is not None:
            try:
                robot.Stop()
            except Exception as exc:
                print(f"WARNING: Stop() failed: {exc}", file=sys.stderr)


if __name__ == "__main__":
    raise SystemExit(main())
