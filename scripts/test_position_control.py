#!/usr/bin/env python3
"""Send a position target through the articulation drive (never teleport)."""

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
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--print-every", type=int, default=10)
    parser.add_argument(
        "--target-deg",
        type=float,
        nargs=6,
        default=[20.0, 15.0, 10.0, -15.0, -10.0, 20.0],
        metavar=("J1", "J2", "J3", "J4", "J5", "J6"),
    )
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})
    try:
        from controllers import PositionTargetController
        from scripts.robot_runtime import IsaacMyCobot, open_default_scene

        open_default_scene(app)
        robot = IsaacMyCobot(app)
        drive = robot.configure_position_drive()
        controller = PositionTargetController(np.deg2rad(args.target_deg))
        print(f"drive_mode={drive.mode} stiffness={drive.stiffness} damping={drive.damping}")

        for step in range(args.steps):
            state = robot.read_state()
            q_cmd = robot.set_position_target(controller.compute(state))
            robot.step()
            if step % max(args.print_every, 1) == 0 or step == args.steps - 1:
                measured = robot.read_state()
                error = q_cmd - measured.q
                print(
                    f"step={step:04d} q={np.rad2deg(measured.q).round(2)} deg "
                    f"q_target={np.rad2deg(q_cmd).round(2)} deg error_norm={np.linalg.norm(error):.5f} rad"
                )
        robot.stop()
    finally:
        app.close()


if __name__ == "__main__":
    main()
