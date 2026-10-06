#!/usr/bin/env python3
"""Read MyCobot arm joint positions and velocities from the saved scene."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=250)
    parser.add_argument("--print-every", type=int, default=50)
    args, _ = parser.parse_known_args()
    return args


def main() -> None:
    args = parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})
    try:
        from scripts.robot_runtime import IsaacMyCobot, open_default_scene

        open_default_scene(app)
        robot = IsaacMyCobot(app)
        drive = robot.verify_drive("position")
        print(f"articulation_root={robot.articulation_root_path}")
        print(f"joint_names={robot.joint_names}")
        print(f"drive_mode={drive.mode} stiffness={drive.stiffness} damping={drive.damping}")
        for step in range(args.steps):
            robot.step()
            if step % max(args.print_every, 1) == 0 or step == args.steps - 1:
                state = robot.read_state()
                print(f"step={step:04d} q={state.q.round(5)} rad dq={state.dq.round(5)} rad/s")
        robot.stop()
    finally:
        app.close()


if __name__ == "__main__":
    main()
