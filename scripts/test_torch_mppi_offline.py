"""Kinematic closed-loop test for the production pytorch_mppi backend."""

from pathlib import Path
import argparse
import sys
import time
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
from controllers.kinematics import end_effector_position
from controllers.mppi import CONTROL_DT, MAX_JOINT_VELOCITY, MPPIConfig
from controllers.torch_mppi import TorchKinematicMPPI

parser = argparse.ArgumentParser()
parser.add_argument("--horizon", type=int, default=10)
parser.add_argument("--samples", type=int, default=136)
parser.add_argument("--steps", type=int, default=500)
parser.add_argument("--temperature", type=float, default=1.0)
args = parser.parse_args()
goal = np.array([0.19436, 0.05679, 0.30906])
q = np.zeros(6)
controller = TorchKinematicMPPI(
    MPPIConfig(horizon=args.horizon, num_samples=args.samples, temperature=args.temperature), seed=7
)
initial_error = np.linalg.norm(end_effector_position(q) - goal)
warmup_sec = controller.warm_up(q, goal)
started = time.perf_counter()
for _ in range(args.steps):
    dq = controller.command(q, goal)
    assert np.all(np.abs(dq) <= MAX_JOINT_VELOCITY + 1e-5)
    q = q + CONTROL_DT * dq
elapsed = time.perf_counter() - started
final_error = np.linalg.norm(end_effector_position(q) - goal)
print(f"backend:       {controller.backend_info}")
print(f"warm-up:       {warmup_sec:.2f} s")
print(f"initial error: {initial_error * 1000.0:.2f} mm")
print(f"final error:   {final_error * 1000.0:.2f} mm")
print(f"mean command:  {elapsed * 1000.0 / args.steps:.2f} ms")
print(f"q: {np.round(q, 4)} rad")
print(f"diagnostics:   {controller.last_diagnostics}")
if final_error > controller.config.goal_tolerance:
    raise RuntimeError("pytorch_mppi did not reach the configured goal tolerance")
