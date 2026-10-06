"""Save reproducible MPPI run data, configuration, plots, and timing statistics."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from datetime import datetime
import json
from pathlib import Path
from typing import Any, Mapping, Sequence

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
RESULTS_DIR = PROJECT_ROOT / "results"


@dataclass(frozen=True)
class MPPIRunResult:
    figure_path: Path
    data_path: Path
    metadata_path: Path
    timing: dict[str, float | int]


def _arrays(records: Mapping[str, Sequence[Any]]) -> dict[str, np.ndarray]:
    arrays = {name: np.asarray(values) for name, values in records.items()}
    lengths = {len(value) for value in arrays.values()}
    if not arrays or lengths != {len(arrays["time_s"])} or not lengths or next(iter(lengths)) == 0:
        raise ValueError("MPPI result records must be non-empty and have equal lengths")
    return arrays


def _timing_statistics(command_ms: np.ndarray, control_dt: float) -> dict[str, float | int]:
    worst_index = int(np.argmax(command_ms))
    budget_ms = float(control_dt * 1000.0)
    return {
        "WorstT_ms": float(command_ms[worst_index]),
        "WorstT_step": worst_index + 1,
        "meanT_ms": float(np.mean(command_ms)),
        "p95T_ms": float(np.percentile(command_ms, 95.0)),
        "stdT_ms": float(np.std(command_ms)),
        "deadline_misses": int(np.count_nonzero(command_ms > budget_ms)),
        "control_budget_ms": budget_ms,
    }


def _save_figure(
    arrays: Mapping[str, np.ndarray],
    metadata: Mapping[str, Any],
    timing: Mapping[str, float | int],
    output_path: Path,
) -> None:
    from matplotlib.backends.backend_agg import FigureCanvasAgg
    from matplotlib.figure import Figure

    time_s = arrays["time_s"]
    error_mm = arrays["error_mm"]
    command_ms = arrays["command_ms"]
    solve_ms = arrays["solve_ms"]
    config = metadata["mppi_config"]
    goal = metadata["p_goal_mm"]

    figure = Figure(figsize=(11, 8), layout="constrained")
    FigureCanvasAgg(figure)
    error_axis, timing_axis = figure.subplots(2, 1, sharex=True)

    error_axis.plot(time_s, error_mm, color="tab:red", linewidth=1.6, label="Cartesian error")
    error_axis.axhline(
        float(config["goal_tolerance"]) * 1000.0,
        color="tab:green",
        linestyle="--",
        linewidth=1.0,
        label="Goal tolerance",
    )
    error_axis.set_ylabel("Error [mm]")
    error_axis.grid(alpha=0.3)
    error_axis.legend(loc="best")

    timing_axis.plot(time_s, command_ms, color="tab:blue", linewidth=1.2, label="MPPI command")
    timing_axis.plot(time_s, solve_ms, color="tab:orange", linewidth=1.0, alpha=0.8, label="Optimizer solve")
    timing_axis.axhline(
        float(timing["control_budget_ms"]),
        color="black",
        linestyle="--",
        linewidth=1.0,
        label="Control budget",
    )
    timing_axis.axhline(
        float(timing["meanT_ms"]),
        color="tab:blue",
        linestyle=":",
        linewidth=1.0,
        label=f"meanT={float(timing['meanT_ms']):.2f} ms",
    )
    timing_axis.set_xlabel("Simulation time [s]")
    timing_axis.set_ylabel("Computation time [ms]")
    timing_axis.grid(alpha=0.3)
    timing_axis.legend(loc="best")

    figure.suptitle(
        "MyCobot Cartesian MPPI\n"
        f"goal={goal} mm, H={config['horizon']}, K={config['num_samples']}, "
        f"lambda={config['temperature']}, sigma={config['noise_sigma']}, "
        f"steps={metadata['steps']}"
    )
    figure.savefig(output_path, dpi=160)


def save_mppi_results(
    records: Mapping[str, Sequence[Any]],
    *,
    config: Any,
    metadata: Mapping[str, Any],
) -> MPPIRunResult:
    """Persist one successfully completed MPPI run and return its summary."""
    arrays = _arrays(records)
    config_values = asdict(config)
    full_metadata = {**metadata, "mppi_config": config_values}
    timing = _timing_statistics(arrays["command_ms"], float(config.dt))
    full_metadata["timing"] = timing

    RESULTS_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    stem = f"mppi_{timestamp}_H{config.horizon}_K{config.num_samples}_N{full_metadata['steps']}"
    data_path = RESULTS_DIR / f"{stem}.npz"
    metadata_path = RESULTS_DIR / f"{stem}.json"
    figure_path = RESULTS_DIR / f"{stem}.png"

    np.savez_compressed(data_path, **arrays)
    metadata_path.write_text(json.dumps(full_metadata, indent=2, ensure_ascii=False) + "\n")
    _save_figure(arrays, full_metadata, timing, figure_path)
    return MPPIRunResult(figure_path, data_path, metadata_path, timing)
