#!/usr/bin/env python3
"""Run the saved Isaac Sim scene through a complete MPPI experiment."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--steps", type=int, default=750)
    parser.add_argument("--goal-mm", type=float, nargs=3, default=[194.36, 56.79, 309.06])
    parser.add_argument("--horizon", type=int, default=10)
    parser.add_argument("--samples", type=int, default=136)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--render-every", type=int, default=1)
    parser.add_argument("--print-every", type=int, default=10)
    args, _ = parser.parse_known_args()
    if args.steps <= 0:
        parser.error("--steps must be positive")
    if args.render_every <= 0:
        parser.error("--render-every must be positive")
    return args


def main() -> None:
    args = parse_args()
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": args.headless})
    robot = None
    marker_cleanup = None
    try:
        from isaacsim.core.simulation_manager import SimulationManager

        from controllers.kinematics import end_effector_position
        from controllers.mppi import CONTROL_DT, MPPIConfig
        from controllers.torch_mppi import TorchKinematicMPPI
        from runtime.isaac_robot import MPPI_POSITION_DAMPING, MPPI_POSITION_STIFFNESS
        from runtime.goal_marker import hide_goal_marker, set_goal_marker
        from runtime.mppi_results import save_mppi_results
        from scripts.robot_runtime import IsaacMyCobot, open_default_scene

        open_default_scene(app)
        robot = IsaacMyCobot(app)
        robot.articulation.set_dof_gains(
            stiffnesses=MPPI_POSITION_STIFFNESS[None, :],
            dampings=MPPI_POSITION_DAMPING[None, :],
            dof_indices=robot.arm_dof_indices,
        )
        drive = robot.configure_position_drive()
        goal = np.asarray(args.goal_mm, dtype=float) / 1000.0
        config = MPPIConfig(horizon=args.horizon, num_samples=args.samples)
        controller = TorchKinematicMPPI(config, seed=args.seed)
        backend = controller.backend_info

        state = robot.read_state()
        print(f"[mycobot MPPI] warming CUDA kernels on {backend.device} ({backend.gpu_name})...")
        warmup_sec = controller.warm_up(state.q, goal)
        print(f"[mycobot MPPI] CUDA kernels ready in {warmup_sec:.3f} s")
        set_goal_marker(goal)
        marker_cleanup = hide_goal_marker

        records = {name: [] for name in (
            "time_s", "error_mm", "preprocess_ms", "solve_ms", "postprocess_ms", "command_ms", "cycle_ms",
            "minimum_cost", "effective_sample_size", "predicted_terminal_error_mm",
            "q", "dq", "dq_command", "ee_position_mm",
        )}
        loop_started = time.perf_counter()
        for step in range(1, args.steps + 1):
            cycle_started = time.perf_counter()
            state = robot.read_state()
            dq_command = controller.command(state.q, goal)
            q_command = state.q + CONTROL_DT * dq_command
            robot.set_position_target(q_command)
            render_now = step == 1 or step % args.render_every == 0
            if render_now:
                app.update()
            else:
                SimulationManager.step()

            state = robot.read_state()
            cycle_ms = (time.perf_counter() - cycle_started) * 1000.0
            ee = end_effector_position(state.q)
            error_mm = float(np.linalg.norm(goal - ee) * 1000.0)
            diag = controller.last_diagnostics
            records["time_s"].append(step * CONTROL_DT)
            records["error_mm"].append(error_mm)
            records["preprocess_ms"].append(controller.last_preprocess_time_sec * 1000.0)
            records["solve_ms"].append(controller.last_solve_time_sec * 1000.0)
            records["postprocess_ms"].append(controller.last_postprocess_time_sec * 1000.0)
            records["command_ms"].append(controller.last_command_time_sec * 1000.0)
            records["cycle_ms"].append(cycle_ms)
            records["minimum_cost"].append(np.nan if diag is None else diag.minimum_cost)
            records["effective_sample_size"].append(np.nan if diag is None else diag.effective_sample_size)
            records["predicted_terminal_error_mm"].append(
                np.nan if diag is None else diag.predicted_terminal_error_m * 1000.0
            )
            records["q"].append(state.q.copy())
            records["dq"].append(state.dq.copy())
            records["dq_command"].append(dq_command.copy())
            records["ee_position_mm"].append(ee * 1000.0)

            if args.print_every > 0 and (
                step == 1 or step % args.print_every == 0 or step == args.steps
            ):
                wall_sec = time.perf_counter() - loop_started
                print(
                    f"[mycobot MPPI] step={step}/{args.steps}, "
                    f"sim={step * CONTROL_DT:.2f}s, wall={wall_sec:.2f}s, "
                    f"solve={controller.last_solve_time_sec * 1000.0:.1f}ms, "
                    f"command={controller.last_command_time_sec * 1000.0:.1f}ms, "
                    f"cycle={cycle_ms:.1f}ms, error={error_mm:.1f}mm"
                )

        result = save_mppi_results(
            records,
            config=config,
            metadata={
                "p_goal_mm": np.asarray(args.goal_mm, dtype=float).tolist(),
                "steps": args.steps,
                "control_dt_sec": CONTROL_DT,
                "seed": args.seed,
                "render_every": args.render_every,
                "warmup_sec": warmup_sec,
                "backend": {
                    "device": backend.device,
                    "gpu_name": backend.gpu_name,
                    "torch_version": backend.torch_version,
                    "pytorch_mppi_version": backend.library_version,
                },
                "position_drive": {
                    "stiffness": drive.stiffness.tolist(),
                    "damping": drive.damping.tolist(),
                },
            },
        )
        timing = result.timing
        print(f"[mycobot MPPI summary] steps={args.steps}, final_error={records['error_mm'][-1]:.3f} mm")
        print(
            f"[mycobot MPPI timing] WorstT={timing['WorstT_ms']:.3f} ms "
            f"(step {timing['WorstT_step']}), meanT={timing['meanT_ms']:.3f} ms, "
            f"p95T={timing['p95T_ms']:.3f} ms, stdT={timing['stdT_ms']:.3f} ms, "
            f"deadline_misses={timing['deadline_misses']}/{args.steps}"
        )
        print(f"[mycobot MPPI results] figure={result.figure_path}")
        print(f"[mycobot MPPI results] data={result.data_path}")
        print(f"[mycobot MPPI results] metadata={result.metadata_path}")
    finally:
        if marker_cleanup is not None:
            marker_cleanup()
        if robot is not None:
            robot.stop()
        app.close()


if __name__ == "__main__":
    main()
