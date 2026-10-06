"""Fast kinematic-only smoke test; does not start Isaac Sim."""

from pathlib import Path
import sys
import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))
from controllers.kinematics import end_effector_position
from controllers.mppi import CONTROL_DT, KinematicMPPI, MAX_JOINT_VELOCITY

goal = np.array([0.19436, 0.05679, 0.30906])
q = np.zeros(6)
controller = KinematicMPPI(seed=7)
initial_error = np.linalg.norm(end_effector_position(q) - goal)
for _ in range(200):
    dq = controller.command(q, goal)
    assert np.all(np.abs(dq) <= MAX_JOINT_VELOCITY + 1e-12)
    q = q + CONTROL_DT * dq
final_error = np.linalg.norm(end_effector_position(q) - goal)
print(f"initial error: {initial_error * 1000.0:.2f} mm")
print(f"final error:   {final_error * 1000.0:.2f} mm")
print(f"q: {np.round(q, 4)} rad")
if not final_error < initial_error:
    raise RuntimeError("MPPI did not reduce the Cartesian goal error")
