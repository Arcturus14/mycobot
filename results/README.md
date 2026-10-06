# MPPI simulation results

Each successfully completed MPPI run writes three files with the same timestamped stem:

- `.png`: Cartesian error history above MPPI computation time.
- `.npz`: per-step raw arrays used to produce the figure.
- `.json`: goal, controller configuration, backend, timing statistics, and run settings.

`WorstT` and `meanT` use the full MPPI `command()` time. The figure also shows the optimizer-only solve time and the 20 ms control budget.
