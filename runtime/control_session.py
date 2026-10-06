"""Persistent asynchronous control jobs for Isaac Sim's Python Server."""

from __future__ import annotations

import asyncio
import time
from typing import Any

import numpy as np

from runtime.isaac_robot import (
    MPPI_POSITION_DAMPING,
    MPPI_POSITION_STIFFNESS,
    PHYSICS_DT,
    IsaacMyCobot,
)


_session: dict[str, Any] = {
    "robot": None,
    "task": None,
    "mode": "idle",
    "state": "idle",
    "step": 0,
    "total_steps": 0,
    "physics_dt_sec": PHYSICS_DT,
    "control_dt_sec": None,
    "physics_steps_per_control": None,
    "duration_sec": 0.0,
    "command": None,
    "q": None,
    "dq": None,
    "error": None,
    "ee_position_mm": None,
    "goal_mm": None,
    "goal_marker_path": None,
    "mppi": None,
    "result_figure": None,
    "result_data": None,
    "result_metadata": None,
    "timing": None,
    "message": "",
}


def _list(value) -> list[float] | None:
    return None if value is None else np.asarray(value, dtype=float).round(6).tolist()


def resolve_control_steps(
    *,
    steps: int | None,
    duration_sec: float | None,
    default_steps: int,
) -> int:
    """Resolve either a step count or seconds to a positive physics-step count."""
    if steps is not None and duration_sec is not None:
        raise ValueError("provide only one of steps or duration_sec")
    if duration_sec is not None:
        duration_sec = float(duration_sec)
        if duration_sec <= 0.0:
            raise ValueError(f"duration_sec must be positive, got {duration_sec}")
        return max(1, int(round(duration_sec / PHYSICS_DT)))
    resolved = default_steps if steps is None else int(steps)
    if resolved <= 0:
        raise ValueError(f"steps must be positive, got {resolved}")
    return resolved


async def get_robot() -> IsaacMyCobot:
    robot = _session["robot"]
    if robot is None or not robot.articulation.is_physics_tensor_entity_valid():
        robot = await IsaacMyCobot.create()
        _session["robot"] = robot
    return robot


def invalidate_robot() -> None:
    """Discard handles after a stage reload."""
    _session["robot"] = None


def _controller_task_finished(task: asyncio.Task) -> None:
    """Retrieve background failures and expose them through control_status."""
    if task.cancelled():
        return
    try:
        exception = task.exception()
    except asyncio.CancelledError:
        return
    if exception is None:
        return

    detail = f"{type(exception).__name__}: {exception}"
    if (
        isinstance(exception, AssertionError)
        and "physics tensor entity is not valid" in str(exception)
    ):
        detail = (
            "physics articulation became invalid because the timeline stopped or the stage "
            "was reloaded; run remote/open_scene.py and restart the controller"
        )
    _session.update(state="error", message=detail)
    print(f"[mycobot controller] stopped: {detail}")


def _start_controller_task(coroutine) -> asyncio.Task:
    task = asyncio.create_task(coroutine)
    task.add_done_callback(_controller_task_finished)
    _session["task"] = task
    return task


async def stop_controller() -> dict[str, Any]:
    previous_mode = _session["mode"]
    task = _session["task"]
    if task is not None and not task.done():
        task.cancel()
        try:
            await task
        except asyncio.CancelledError:
            pass

    robot = _session["robot"]
    if robot is not None and robot.articulation.is_physics_tensor_entity_valid():
        if previous_mode in {"position", "mppi_position"}:
            current = robot.read_state()
            robot.set_position_target(current.q)
        else:
            robot.zero_velocity_target()

    _session.update(task=None, mode="idle", state="stopped", message="controller stopped")
    return status()


async def _prepare(mode: str, command, steps: int):
    if steps <= 0:
        raise ValueError(f"steps must be positive, got {steps}")
    robot = await get_robot()
    _session.update(
        mode=mode,
        state="starting",
        step=0,
        total_steps=int(steps),
        physics_dt_sec=PHYSICS_DT,
        control_dt_sec=None,
        physics_steps_per_control=None,
        duration_sec=round(int(steps) * PHYSICS_DT, 6),
        command=_list(command),
        q=None,
        dq=None,
        error=None,
        ee_position_mm=None,
        goal_mm=None,
        goal_marker_path=None,
        mppi=None,
        result_figure=None,
        result_data=None,
        result_metadata=None,
        timing=None,
        message="",
    )
    return robot


async def _position_loop(q_target, steps: int, print_every: int) -> None:
    import omni.kit.app

    robot = await _prepare("position", q_target, steps)
    drive = robot.configure_position_drive()
    command = robot.set_position_target(q_target)
    _session.update(
        state="running",
        command=_list(command),
        message=f"position drive: stiffness={_list(drive.stiffness)}, damping={_list(drive.damping)}",
    )
    try:
        for step in range(1, steps + 1):
            robot.set_position_target(command)
            await omni.kit.app.get_app().next_update_async()
            state = robot.read_state()
            error = command - state.q
            _session.update(step=step, q=_list(state.q), dq=_list(state.dq), error=_list(error))
            if print_every > 0 and (step == 1 or step % print_every == 0 or step == steps):
                print(f"[mycobot position] step={step}/{steps}, error_norm={np.linalg.norm(error):.6f} rad")
        _session.update(state="completed", message="position target remains active")
    except asyncio.CancelledError:
        _session.update(state="stopped", message="position controller cancelled")
        raise
    except Exception as exc:
        _session.update(state="error", message=f"{type(exc).__name__}: {exc}")
        raise


async def _velocity_loop(dq_target, steps: int, print_every: int) -> None:
    import omni.kit.app

    robot = await _prepare("velocity", dq_target, steps)
    drive = robot.configure_velocity_drive()
    command = robot.set_velocity_target(dq_target)
    _session.update(
        state="running",
        command=_list(command),
        message=f"velocity drive: stiffness={_list(drive.stiffness)}, damping={_list(drive.damping)}",
    )
    try:
        for step in range(1, steps + 1):
            robot.set_velocity_target(command)
            await omni.kit.app.get_app().next_update_async()
            state = robot.read_state()
            error = command - state.dq
            _session.update(step=step, q=_list(state.q), dq=_list(state.dq), error=_list(error))
            if print_every > 0 and (step == 1 or step % print_every == 0 or step == steps):
                print(f"[mycobot velocity] step={step}/{steps}, error_norm={np.linalg.norm(error):.6f} rad/s")
        _session.update(state="completed", message="velocity command completed and was reset to zero")
    except asyncio.CancelledError:
        _session.update(state="stopped", message="velocity controller cancelled")
        raise
    except Exception as exc:
        _session.update(state="error", message=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        if robot.articulation.is_physics_tensor_entity_valid():
            robot.zero_velocity_target()


async def _mppi_loop(p_goal_m, duration_sec: float | None, print_every: int, seed: int,
                     horizon: int, num_samples: int, render_every: int,
                     steps: int | None) -> None:
    import omni.kit.app
    from isaacsim.core.simulation_manager import SimulationManager

    from controllers.kinematics import end_effector_position
    from controllers.mppi import CONTROL_DT, MPPIConfig
    from controllers.torch_mppi import TorchKinematicMPPI
    from runtime.goal_marker import hide_goal_marker, set_goal_marker

    if steps is not None:
        control_steps = int(steps)
        if control_steps <= 0:
            raise ValueError("steps must be positive")
    else:
        if duration_sec is None or duration_sec <= 0.0:
            raise ValueError("duration_sec must be positive when steps is not set")
        control_steps = max(1, int(round(duration_sec / CONTROL_DT)))
    if render_every <= 0:
        raise ValueError("render_every must be positive")
    # This kinematic phase uses one 20 ms PhysX step per 20 ms MPPI command.
    # The stage itself is configured at 50 Hz, so one normal Kit update both
    # advances physics and refreshes WebRTC without manual simulate/fetch calls.
    physics_steps_per_control = 1
    goal = np.asarray(p_goal_m, dtype=float)
    if goal.shape != (3,):
        raise ValueError("p_goal_m must have shape (3,)")

    robot = await _prepare("mppi_position", goal, control_steps)
    drive = robot.configure_position_drive(
        stiffness=MPPI_POSITION_STIFFNESS,
        damping=MPPI_POSITION_DAMPING,
    )
    config = MPPIConfig(horizon=int(horizon), num_samples=int(num_samples))
    controller = TorchKinematicMPPI(config, seed=int(seed))
    backend = controller.backend_info
    initial_state = robot.read_state()
    _session.update(state="warming_up", message="compiling/warming pytorch_mppi CUDA kernels")
    print(f"[mycobot MPPI] warming CUDA kernels on {backend.device} ({backend.gpu_name})...")
    warmup_sec = controller.warm_up(initial_state.q, goal)
    print(f"[mycobot MPPI] CUDA kernels ready in {warmup_sec:.3f} s")
    goal_marker_path = set_goal_marker(goal)
    _session.update(
        state="running", physics_dt_sec=CONTROL_DT, control_dt_sec=CONTROL_DT,
        physics_steps_per_control=physics_steps_per_control,
        duration_sec=control_steps * CONTROL_DT,
        command=None, goal_mm=_list(goal * 1000.0), goal_marker_path=goal_marker_path,
        message=(f"pytorch_mppi {backend.library_version} on {backend.device} "
                 f"(torch {backend.torch_version}); 50 Hz, H={horizon}, K={num_samples}; position drive "
                 f"render every {render_every} control cycles; goal marker={goal_marker_path}; "
                 f"stiffness={_list(drive.stiffness)}, damping={_list(drive.damping)}"),
    )
    records: dict[str, list[Any]] = {
        "time_s": [],
        "error_mm": [],
        "preprocess_ms": [],
        "solve_ms": [],
        "postprocess_ms": [],
        "command_ms": [],
        "cycle_ms": [],
        "minimum_cost": [],
        "effective_sample_size": [],
        "predicted_terminal_error_mm": [],
        "q": [],
        "dq": [],
        "dq_command": [],
        "ee_position_mm": [],
    }
    loop_started = time.perf_counter()
    cycle_time_sum = 0.0
    try:
        for step in range(1, control_steps + 1):
            cycle_started = time.perf_counter()
            state = robot.read_state()
            dq_command = controller.command(state.q, goal)
            q_command = state.q + CONTROL_DT * dq_command
            q_command = robot.set_position_target(q_command)

            render_now = step == 1 or step % render_every == 0
            if render_now:
                await omni.kit.app.get_app().next_update_async()
            else:
                # Official physics-only step; the default render_every=1 does
                # not take this branch and displays every control step.
                SimulationManager.step()

            state = robot.read_state()
            cycle_sec = time.perf_counter() - cycle_started
            cycle_time_sum += cycle_sec
            wall_elapsed_sec = time.perf_counter() - loop_started
            sim_elapsed_sec = step * CONTROL_DT
            real_time_factor = sim_elapsed_sec / max(wall_elapsed_sec, 1e-9)
            ee = end_effector_position(state.q)
            error_m = float(np.linalg.norm(goal - ee))
            diag = controller.last_diagnostics
            records["time_s"].append(sim_elapsed_sec)
            records["error_mm"].append(error_m * 1000.0)
            records["preprocess_ms"].append(controller.last_preprocess_time_sec * 1000.0)
            records["solve_ms"].append(controller.last_solve_time_sec * 1000.0)
            records["postprocess_ms"].append(controller.last_postprocess_time_sec * 1000.0)
            records["command_ms"].append(controller.last_command_time_sec * 1000.0)
            records["cycle_ms"].append(cycle_sec * 1000.0)
            records["minimum_cost"].append(np.nan if diag is None else diag.minimum_cost)
            records["effective_sample_size"].append(
                np.nan if diag is None else diag.effective_sample_size
            )
            records["predicted_terminal_error_mm"].append(
                np.nan if diag is None else diag.predicted_terminal_error_m * 1000.0
            )
            records["q"].append(state.q.copy())
            records["dq"].append(state.dq.copy())
            records["dq_command"].append(dq_command.copy())
            records["ee_position_mm"].append(ee * 1000.0)
            _session.update(
                step=step, q=_list(state.q), dq=_list(state.dq), command=_list(dq_command),
                error=round(error_m * 1000.0, 3), ee_position_mm=_list(ee * 1000.0),
                mppi=None if diag is None else {
                    "minimum_cost": round(diag.minimum_cost, 6),
                    "effective_sample_size": round(diag.effective_sample_size, 3),
                    "predicted_terminal_error_mm": round(diag.predicted_terminal_error_m * 1000.0, 3),
                    "backend": backend.device,
                    "gpu_name": backend.gpu_name,
                    "cuda_memory_allocated_mb": round(controller.cuda_memory_allocated_mb, 2),
                    "warmup_sec": round(warmup_sec, 4),
                    "preprocess_ms": round(controller.last_preprocess_time_sec * 1000.0, 3),
                    "solve_ms": round(controller.last_solve_time_sec * 1000.0, 3),
                    "postprocess_ms": round(controller.last_postprocess_time_sec * 1000.0, 3),
                    "command_ms": round(controller.last_command_time_sec * 1000.0, 3),
                    "cycle_ms": round(cycle_sec * 1000.0, 3),
                    "mean_cycle_ms": round(cycle_time_sum * 1000.0 / step, 3),
                    "wall_elapsed_sec": round(wall_elapsed_sec, 3),
                    "sim_elapsed_sec": round(sim_elapsed_sec, 3),
                    "real_time_factor": round(real_time_factor, 3),
                    "deadline_missed": cycle_sec > CONTROL_DT,
                    "render_every": render_every,
                    "rendered_this_cycle": render_now,
                },
            )
            if print_every > 0 and (step == 1 or step % print_every == 0 or step == control_steps):
                print(f"[mycobot MPPI] step={step}/{control_steps}, "
                      f"sim={sim_elapsed_sec:.2f}s, wall={wall_elapsed_sec:.2f}s, "
                      f"solve={controller.last_solve_time_sec * 1000.0:.1f}ms, "
                      f"command={controller.last_command_time_sec * 1000.0:.1f}ms, "
                      f"cycle={cycle_sec * 1000.0:.1f}ms, RTF={real_time_factor:.2f}, "
                      f"ee={np.round(ee * 1000.0, 1)}, error={error_m * 1000.0:.1f}mm")
        from runtime.mppi_results import save_mppi_results

        result = save_mppi_results(
            records,
            config=config,
            metadata={
                "p_goal_mm": _list(goal * 1000.0),
                "steps": control_steps,
                "control_dt_sec": CONTROL_DT,
                "seed": int(seed),
                "render_every": int(render_every),
                "warmup_sec": float(warmup_sec),
                "backend": {
                    "device": backend.device,
                    "gpu_name": backend.gpu_name,
                    "torch_version": backend.torch_version,
                    "pytorch_mppi_version": backend.library_version,
                },
                "position_drive": {
                    "stiffness": _list(drive.stiffness),
                    "damping": _list(drive.damping),
                },
            },
        )
        timing = result.timing
        _session.update(
            state="completed",
            result_figure=str(result.figure_path),
            result_data=str(result.data_path),
            result_metadata=str(result.metadata_path),
            timing=timing,
            message="MPPI steps completed; results saved and final position target held",
        )
        print(f"[mycobot MPPI summary] steps={control_steps}, final_error={records['error_mm'][-1]:.3f} mm")
        print(
            f"[mycobot MPPI timing] WorstT={timing['WorstT_ms']:.3f} ms "
            f"(step {timing['WorstT_step']}), meanT={timing['meanT_ms']:.3f} ms, "
            f"p95T={timing['p95T_ms']:.3f} ms, stdT={timing['stdT_ms']:.3f} ms, "
            f"deadline_misses={timing['deadline_misses']}/{control_steps}"
        )
        print(f"[mycobot MPPI results] figure={result.figure_path}")
        print(f"[mycobot MPPI results] data={result.data_path}")
        print(f"[mycobot MPPI results] metadata={result.metadata_path}")
    except asyncio.CancelledError:
        _session.update(state="stopped", message="MPPI controller cancelled")
        raise
    except Exception as exc:
        _session.update(state="error", message=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        hide_goal_marker()


async def start_position_control(q_target, steps: int = 100, print_every: int = 10) -> dict[str, Any]:
    await stop_controller()
    _start_controller_task(_position_loop(q_target, int(steps), int(print_every)))
    await asyncio.sleep(0)
    return status()


async def start_velocity_control(dq_target, steps: int = 50, print_every: int = 5) -> dict[str, Any]:
    await stop_controller()
    _start_controller_task(_velocity_loop(dq_target, int(steps), int(print_every)))
    await asyncio.sleep(0)
    return status()


async def start_mppi_control(p_goal_m, duration_sec: float | None = 15.0, print_every: int = 10,
                             seed: int = 7, horizon: int = 10,
                             num_samples: int = 136, render_every: int = 1,
                             steps: int | None = None) -> dict[str, Any]:
    await stop_controller()
    resolved_duration = None if duration_sec is None else float(duration_sec)
    _start_controller_task(_mppi_loop(p_goal_m, resolved_duration, int(print_every),
                                      int(seed), int(horizon), int(num_samples),
                                      int(render_every), None if steps is None else int(steps)))
    await asyncio.sleep(0)
    return status()


def status() -> dict[str, Any]:
    task = _session["task"]
    result = {key: value for key, value in _session.items() if key not in {"robot", "task"}}
    result["task_active"] = task is not None and not task.done()
    return result
