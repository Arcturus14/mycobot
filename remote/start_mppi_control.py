"""Start Cartesian point-goal kinematic MPPI in the running Isaac Sim."""

from pathlib import Path
import sys
import numpy as np

PROJECT_ROOT = Path("/home/arclab/workspace/urop2_manipulator/mycobot")
sys.path.insert(0, str(PROJECT_ROOT))
from runtime.control_session import start_mppi_control

try:
    p_goal_mm
except NameError:
    # FK of q_goal_deg=[30,-20,-30,0,20,0], safely inside the joint limits.
    p_goal_mm = [194.36, 56.79, 309.06]
try:
    duration_sec
except NameError:
    duration_sec = 15.0
try:
    steps
except NameError:
    steps = None
try:
    print_every
except NameError:
    print_every = 10
try:
    seed
except NameError:
    seed = 7
try:
    horizon
except NameError:
    horizon = 10
try:
    num_samples
except NameError:
    num_samples = 136
try:
    render_every
except NameError:
    render_every = 1

goal = np.asarray(p_goal_mm, dtype=float) / 1000.0
result = await start_mppi_control(
    goal, duration_sec, print_every, seed, horizon, num_samples, render_every, steps
)
print(result)
