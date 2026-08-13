#!/usr/bin/env python3
"""Move directly to a Cartesian destination 200 mm above Flexiv Home."""

from __future__ import annotations

from pathlib import Path
import sys
import time

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from scripts.teleoperate import DEFAULT_CONFIG, WristBridge, load_profile  # noqa: E402
from controllers.nine_dof import pose_to_transform, transform_to_pose  # noqa: E402


def lifted_tcp_pose(current_pose: np.ndarray, world_z_lift_m: float) -> np.ndarray:
    """Return the same active-TCP pose translated along world +Z only."""
    pose = np.asarray(current_pose, dtype=float)
    lift = float(world_z_lift_m)
    if pose.shape != (7,) or not np.all(np.isfinite(pose)):
        raise ValueError("Flexiv active TCP pose must be a finite 7-vector")
    if not np.isfinite(lift) or lift <= 0.0:
        raise ValueError("Home final lift distance must be a finite positive value")
    target = pose.copy()
    target[2] += lift
    return target


def elevated_home_tcp_pose(
    world_to_home_flange: np.ndarray,
    flange_to_active_tcp_pose: np.ndarray,
    world_z_lift_m: float,
) -> np.ndarray:
    """Compose the live-model Home TCP, then offset its world-Z destination."""
    world_to_flange = np.asarray(world_to_home_flange, dtype=float)
    if world_to_flange.shape != (4, 4) or not np.all(np.isfinite(world_to_flange)):
        raise ValueError("Home flange transform must be a finite 4x4 matrix")
    flange_to_tcp = pose_to_transform(
        np.asarray(flange_to_active_tcp_pose, dtype=float)
    )
    home_tcp_pose = transform_to_pose(world_to_flange @ flange_to_tcp)
    return lifted_tcp_pose(home_tcp_pose, world_z_lift_m)


def _compute_safe_home_target(
    robot: object, flexivrdk: object, profile: object
) -> tuple[np.ndarray, np.ndarray]:
    """Use live FK/IK to compute Home TCP + world-Z offset without moving."""
    home_q_rad = np.deg2rad(
        np.asarray(profile.home["reference_joint_position_deg"], dtype=float)
    )
    model = flexivrdk.Model(robot)
    model.Update(home_q_rad.tolist(), np.zeros(7).tolist())
    world_to_home_flange = np.asarray(model.T("flange"), dtype=float)
    tool_params = flexivrdk.Tool(robot).params()
    target_pose = elevated_home_tcp_pose(
        world_to_home_flange,
        np.asarray(tool_params.tcp_location, dtype=float),
        float(profile.home["final_lift_world_z_m"]),
    )
    reachable, ik_solution = model.reachable(
        target_pose.tolist(), home_q_rad.tolist(), False
    )
    if not reachable:
        raise RuntimeError(
            "Home + world-Z offset is not reachable with the current active Tool"
        )
    ik_q_rad = np.asarray(ik_solution, dtype=float)
    if ik_q_rad.shape != (7,) or not np.all(np.isfinite(ik_q_rad)):
        raise RuntimeError("RDK returned an invalid IK solution for elevated Home")
    print(
        "HOME_TARGET reference_q_deg=[{}] target_tcp_m=[{}] ik_q_deg=[{}]".format(
            ", ".join(f"{value:+.1f}" for value in np.rad2deg(home_q_rad)),
            ", ".join(f"{value:+.4f}" for value in target_pose[:3]),
            ", ".join(f"{value:+.1f}" for value in np.rad2deg(ik_q_rad)),
        ),
        flush=True,
    )
    return target_pose, ik_q_rad


def _move_directly_to_safe_home(
    robot: object,
    flexivrdk: object,
    profile: object,
    wrist: WristBridge,
    target_pose: np.ndarray,
    ik_q_rad: np.ndarray,
) -> np.ndarray:
    """Move directly to elevated Home; never visit the original low Home."""
    home = profile.home
    robot_cfg = profile.robot
    start_pose = np.asarray(robot.states().tcp_pose, dtype=float)
    period_s = 1.0 / float(home["command_rate_hz"])
    tolerance_m = float(home["final_lift_position_tolerance_m"])
    settle_s = float(home["final_lift_settle_s"])
    deadline = time.monotonic() + float(home["final_lift_timeout_s"])
    settled_since: float | None = None
    next_status_s = 0.0

    robot.SwitchMode(flexivrdk.Mode.NRT_CARTESIAN_MOTION_FORCE)
    # Explicitly disable all force axes so this clearance step is pure Cartesian
    # position motion even if a previous process used Mode 3 force control.
    robot.SetForceControlAxis([False] * 6)
    # Keep the redundant arm on the IK branch closest to the documented Home
    # posture while the primary Cartesian task goes directly to Home+Z.
    robot.SetNullSpacePosture(np.asarray(ik_q_rad, dtype=float).tolist())
    robot.SetNullSpaceObjectives(0.0, 0.0, 1.0)
    print(
        "SAFE_HOME_DIRECT start_tcp_m=[{:+.4f}, {:+.4f}, {:+.4f}] "
        "target_tcp_m=[{:+.4f}, {:+.4f}, {:+.4f}] home_z_offset_m={:.3f}".format(
            *start_pose[:3], *target_pose[:3], float(home["final_lift_world_z_m"])
        ),
        flush=True,
    )
    while True:
        if robot.fault() or not robot.connected():
            raise RuntimeError("Flexiv fault/disconnect during direct safe Home")
        wrist.command_position(np.zeros(2))
        robot.SendCartesianMotionForce(
            target_pose.tolist(),
            max_linear_vel=float(home["final_lift_max_linear_velocity_m_s"]),
            max_angular_vel=float(robot_cfg["max_angular_velocity_rad_s"]),
            max_linear_acc=float(home["final_lift_max_linear_acceleration_m_s2"]),
            max_angular_acc=float(robot_cfg["max_angular_acceleration_rad_s2"]),
        )
        actual_pose = np.asarray(robot.states().tcp_pose, dtype=float)
        error_m = float(np.linalg.norm(actual_pose[:3] - target_pose[:3]))
        now = time.monotonic()
        if error_m <= tolerance_m:
            if settled_since is None:
                settled_since = now
            elif now - settled_since >= settle_s:
                print(
                    f"SAFE_HOME_REACHED position_error_mm={error_m * 1000.0:.2f}",
                    flush=True,
                )
                return target_pose
        else:
            settled_since = None
        if now >= next_status_s:
            print(
                "SAFE_HOME_STATUS actual_z_m={:+.4f} target_z_m={:+.4f} "
                "position_error_mm={:.2f}".format(
                    actual_pose[2], target_pose[2], error_m * 1000.0
                ),
                flush=True,
            )
            next_status_s = now + 1.0
        if now >= deadline:
            raise TimeoutError(
                "Direct elevated Home did not reach the target within "
                f"{float(home['final_lift_timeout_s']):g} seconds; "
                f"remaining position error={error_m * 1000.0:.1f} mm"
            )
        time.sleep(period_s)


def main() -> int:
    robot = None
    try:
        import flexivrdk

        profile = load_profile(DEFAULT_CONFIG)
        print(
            "Whole-system Home: wrist zero, then move directly to the modeled "
            "Flexiv Home TCP + world Z offset; the low Home point is bypassed."
        )
        print("Keep the complete robot and wrist workspace clear; keep E-stop reachable.")
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

        with WristBridge(profile, zero_hold_mask_override=(True, True)) as wrist:
            wrist.wait_ready(float(profile.wrist["zero_timeout_s"]) + 5.0)
            wrist.wait_first_sample(2.0)
            wrist.finish_startup()
            target_pose, ik_q_rad = _compute_safe_home_target(
                robot, flexivrdk, profile
            )
            final_pose = _move_directly_to_safe_home(
                robot, flexivrdk, profile, wrist, target_pose, ik_q_rad
            )
            robot.Stop()
            print(
                "HOME_REACHED arm=1 q8=0 q9=0 "
                "final_tcp_m=[{:+.4f}, {:+.4f}, {:+.4f}] "
                "final_lift_world_z_m={:.3f}".format(
                    *final_pose[:3], float(profile.home["final_lift_world_z_m"])
                )
            )
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
