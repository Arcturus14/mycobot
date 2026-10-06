#!/usr/bin/env python3
"""Send a velocity target through the articulation drive."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=50)
    parser.add_argument("--print-every", type=int, default=5)
    parser.add_argument(
        "--target-deg-s",
        type=float,
        nargs=6,
        default=[10.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
    )
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})
    try:
        from controllers import VelocityTargetController
        from scripts.robot_runtime import IsaacMyCobot, open_default_scene

        open_default_scene(app)
        robot = IsaacMyCobot(app)
        drive = robot.configure_velocity_drive()
        controller = VelocityTargetController(np.deg2rad(args.target_deg_s))
        print(f"drive_mode={drive.mode} stiffness={drive.stiffness} damping={drive.damping}")

        for step in range(args.steps):
            state = robot.read_state()
            dq_cmd = robot.set_velocity_target(controller.compute(state))
            robot.step()
            if step % max(args.print_every, 1) == 0 or step == args.steps - 1:
                measured = robot.read_state()
                print(
                    f"step={step:04d} q={np.rad2deg(measured.q).round(2)} deg "
                    f"dq={np.rad2deg(measured.dq).round(2)} deg/s "
                    f"dq_target={np.rad2deg(dq_cmd).round(2)} deg/s"
                )

        robot.set_velocity_target(np.zeros(6))
        robot.step(25)
        robot.stop()
    finally:
        app.close()


if __name__ == "__main__":
    main()
