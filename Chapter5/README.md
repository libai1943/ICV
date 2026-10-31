# Chapter 5 — Cooperative intersection planning

An independently reconstructed MATLAB implementation of safe travel corridors (STCs), relative STCs, Social Force Model (SFM) proposals, and the lightweight iterative optimization method (LIOM). Drawings use the book's Times New Roman, color palette, and triangle heading glyph.

## Requirements and launch

Tested with MATLAB R2021b and CasADi 3.7.2, using IPOPT/MUMPS. Set `CASADI_MATLAB_PATH` to the extracted compatible MATLAB package containing `+casadi` (https://web.casadi.org/get/). No software is installed by these scripts.

For the reported configuration, launch MATLAB with `-singleCompThread`; set `OMP_NUM_THREADS=1` and `MKL_NUM_THREADS=1` before launch. Timings depend on hardware and concurrent workloads.

```matlab
cd Chapter5
self_test
run_experiments([4, 8, 12]);
recover_subset; % separate initializer-sensitivity experiment
export_results;
make_figures;
```

## Method and interpretation

The six motion states are x, y, heading, velocity, steering, and acceleration. Controls are jerk and steering rate. Two enlarged optimization rows represent the disc center. The tasks use 161 nodes over 80 s, forward motion, passenger-car dimensions, and 0.25 m nominal clearance.

`make_scenario`, `simulate_sfm`, and `prepare_reference` define initialization. SFM rollout is a heuristic and can miss endpoints or stagnate. The deterministic route-reservation safeguard is separate from SFM; reported performance is not an SFM-only reproduction. Its independently named MATLAB routines do not invoke the research Python optimizer.

`construct_corridors` and `polygon_box_distance` check whole candidate boxes, including their interiors. `optimize_trajectories` uses linear hard constraints and nonlinear RK4/center-linking penalties. Five penalty levels precede optional nonlinear restoration. Solver failures remain in the history.

The independent eight-vehicle initialization failed the motion tolerance; `case_08` preserves that result. `recover_subset` takes the same eight vehicles from the validated twelve-vehicle plan and refines them. `case_08_recovery` is a separately identified initializer study, not an independent fleet-size success. The twelve-vehicle run supplies the book's demonstration figures.

`validate_trajectories` forward-integrates controls, checks every vehicle and polygon, and applies a speed-based intersample clearance correction. `audit_control_bounds` checks analytic extrema of velocity, acceleration, and steering under piecewise-constant controls. These are numerical checks, not formal interval integration or closed-loop tracking certification.

## Outputs

MAT files retain references, trajectories, solver histories, and validation. `trajectory_*.csv` uses SI units and explicit columns. JSON reports include failures. `output/figures` contains vector PDF and PNG previews. `reports/experiment_report.json` records this revision; re-running writes fresh results to `output`. The plot generator requires a validated demonstration and never substitutes an invented experimental trajectory.

These finite stop-to-stop tasks do not measure online throughput or establish real-time operation. Method references: Bu et al., IEEE TITS (2025), DOI 10.1109/TITS.2025.3597472; Li et al., IEEE TITS (2022), DOI 10.1109/TITS.2021.3109011.
