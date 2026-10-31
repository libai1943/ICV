# Chapter 6 — Cooperative maneuvers in cluttered environments

Independent MATLAB reconstruction of adaptive-scaling constrained optimization (ASCO) and three comparison implementations. All methods share the same task data, six-state bicycle model, RK4 discretization, cost, initialization, and independent validator. No legacy AMPL, compiled optimizer, or Python solver is invoked.

## Requirements and launch

Tested with MATLAB R2021b and CasADi 3.7.2, using IPOPT/MUMPS. Set `CASADI_MATLAB_PATH` to the extracted compatible MATLAB package containing `+casadi` (https://web.casadi.org/get/). No software is installed by these scripts.

For the reported configuration, launch MATLAB with `-singleCompThread`; set `OMP_NUM_THREADS=1` and `MKL_NUM_THREADS=1` before launch. Timings depend on hardware and concurrent workloads.

```matlab
cd Chapter6
self_test
run_experiments(1:8, ...
    {'asco','full','progressive','linearized'},41,360);
export_results;
make_figures;
```

## Cases and comparison protocol

`cases/case_01.json` through `case_08.json` preserve the first eight A100 boundary/obstacle datasets from the author's RA-L archive. Each contains ten vehicles and five circular obstacles. The solver and descriptive variable names are newly implemented. The task data, not the old optimization routines, are reused.

All cases use 41 nodes, a free 5–75 s horizon, two-disc vehicle covers, and 0.15 m nominal clearance. Reverse motion is allowed. Jerk is limited to 0.2 m/s^3, as stored in the archived data; this differs from the original paper's parameter table. These results are not claimed as exact reproductions of its historical numerical table.

ASCO rebuilds selected disc-pair/time rows using **signed** surface gaps. Initial thresholds are -4 and 2 m; failed solves raise the lower threshold by 3 m. Successful but invalid candidates lower it by 1.3 m (floor -4 m) and expand the upper by 0.5 m. Crossing thresholds triggers a declared reset/expansion safeguard. Final validation always uses all interactions.

`full` loads every collision row. `progressive` accumulates rows near conflict. `linearized` first-order linearizes every collision row while retaining nonlinear RK4 dynamics; it is not a fully convex SCP implementation. This distinction and its failures are explicitly reported. The four methods are not all of the comparators in the original paper.

The case budget is 360 s; iterative methods cap each NLP at 75 s, while full NLP uses the case budget in one call. An iteration cap of 16 also applies. Construction and validation can cause elapsed time to exceed the target. Reports use measured wall time, not claimed CPU time, and exclude the shared independent-vehicle initialization.

`prepare_reference` optimizes each vehicle with static obstacles, then synchronizes the cached reference. A failed independent solve retains its geometric seed and reports the failure. Every comparator gets the same reference, including difficult initializations.

## Validation and output

`validate_plan` forward-integrates each returned control sequence with 40 substeps per interval and checks all vehicle/obstacle interactions. Its intersample correction applies to the numerical reconstruction, not a formal ODE enclosure. `audit_control_bounds` analytically checks longitudinal and steering extrema. Failed runs retain their candidates and diagnostics but are never displayed as validated solutions.

`output/case_*.mat` contains full results; `trajectory_*.csv` uses the same field names as Chapter 5. JSON reports include solver status, wall time, selected-row histories, and independent validation. `reports/experiment_report.json` contains the recorded comparison. `make_figures` regenerates vector PDF and PNG figures and requires the complete comparison set.

Eight fixed cases support a reproducible regression study, not a statistical claim over arbitrary environments. Reference: Li et al., IEEE Robotics and Automation Letters 6(2), 1511–1518 (2021). The book explains the algorithm and implementation differences.

## Continuous-bound repair

The common finalizer checks analytic velocity extrema inside every interval. If an otherwise accepted stopped-to-stopped plan exceeds a scalable bound, it applies one common time-dilation factor to the entire fleet. Positions, orientations, steering angles, paths, and relative timing remain unchanged; velocity scales by 1/s, acceleration by 1/s², jerk by 1/s³, and steering rate by 1/s. It rechecks full geometry and dynamics and rejects horizons exceeding 75 s. This does not repair steering-angle violations or an unsuccessful optimization. Reports retain the original horizon, pre-repair violation, scale, postprocessing time, and final cost. Final completion times refer to the repaired plan.
