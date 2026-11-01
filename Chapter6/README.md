# Chapter 6: Cooperative Planning in a Cluttered Environment

This chapter presents one complete ASCO demonstration. Ten ICVs move between interwoven initial and terminal poses through the same central region, negotiating six circular obstacles and one another. The example illustrates the method's operation and a validated cooperative maneuver.

## Environment

The recorded implementation uses MATLAB R2021b, CasADi 3.7.2, IPOPT 3.14.11, and MA57. All motion equations use first-order explicit forward Euler. Install compatible CasADi and licensed HSL/MA57 libraries separately; no HSL binaries or font files are distributed.

```matlab
setenv('CASADI_MATLAB_PATH','/absolute/path/to/casadi-matlab-package');
setenv('IPOPT_HSL_LIBRARY','/absolute/path/to/libcoinhsl');
cd Chapter6
self_test;
run_example;
```

Use the actual platform-specific HSL filename. On Windows, add its runtime DLL directory to PATH before starting MATLAB. Launch MATLAB with `-singleCompThread` and set `OMP_NUM_THREADS=1` and `MKL_NUM_THREADS=1` for the recorded threading convention. The solver explicitly selects MA57; no automatic fallback is used.

## Task and method

`cases/case_01.json` defines C001. Each rear-axle path stays within 3.5 m of its initial-to-terminal chord. These overlapping transit bands preserve shared-region interaction; collision avoidance is still handled by ASCO and the complete geometric validator. Every vehicle center visits the radius-8-m central disk.

The model uses 81 nodes, a free 5–75 s horizon, two covering discs per ICV, and a 0.15 m nominal sample clearance. Reverse motion is allowed. Limits are 2.5 m/s velocity, 0.5 m/s² acceleration, 0.2 m/s³ jerk, 0.7 rad steering, and 0.5 rad/s steering rate. Endpoints are stationary.

`prepare_reference` optimizes individual obstacle-aware routes and selects temporal offsets for initialization. The cooperative solve enforces the native Euler equations. ASCO adapts its collision-row set using signed gaps, initially between -4 and 8 m. The implementation uses a 480 s planning budget, at most 32 outer updates, and up to 180 s per IPOPT call.

## Saved demonstration

`reports/trajectory_01_asco.csv` and `reports/case_01_asco.json` contain the recorded trajectory and measured quantities. `reports/independent-audit.json` independently reconstructs the CSV motion equations, endpoints, control bounds, transit bands, complete disc geometry, and central participation. The output illustrates feasibility under the declared model rather than a globally optimal solution.

`run_example` performs the complete computation and produces the task diagram, six snapshots with trails, and motion/clearance profiles. `export_results` and `make_figures` regenerate the data and figures from the saved MAT result in `output`. Numeric timing describes this recorded run, not an isolated-machine performance claim.

Reference: Li et al., IEEE Robotics and Automation Letters 6(2), 1511–1518 (2021).
