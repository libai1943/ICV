# Chapter 6: Cooperative Maneuvers in Cluttered Environments

Independent MATLAB reconstruction of adaptive-scaling constrained optimization (ASCO) and three comparison implementations. All methods share task data, the six-state bicycle model, forward Euler transcription, cost, initialization, and independent validation.

## Numerical environment

The recorded runs use MATLAB R2021b, CasADi 3.7.2, IPOPT 3.14.11, and **MA57**. Every vehicle-motion transcription and initial-guess propagation uses **first-order explicit forward Euler**. No automatic linear-solver fallback is allowed.

Install a compatible CasADi MATLAB package and obtain an HSL library containing MA57 under an appropriate license. The HSL library and any required runtime libraries must match MATLAB's architecture. Licensed binaries and Times New Roman font files are not distributed with this source.

```matlab
setenv('CASADI_MATLAB_PATH','/absolute/path/to/casadi-matlab-package');
setenv('IPOPT_HSL_LIBRARY','/absolute/path/to/libcoinhsl');
```

Use the actual platform-specific filename, including its extension. On Windows, add the directory containing dependent HSL runtime DLLs to `PATH` before launching MATLAB. A missing or incompatible MA57 library is an error to resolve, not a reason to substitute another solver. Launch MATLAB with `-singleCompThread` and set `OMP_NUM_THREADS=1` and `MKL_NUM_THREADS=1` before launch for the recorded threading configuration. Timings are single-run wall measurements and may be affected by concurrent workloads.

## Reproduce

```matlab
cd Chapter6
self_test;
run_experiments(1:8, ...
    {'asco','full','progressive','linearized'},81,240);
export_results;
make_figures;
```

MA57 uses its internal AMD ordering (`ma57_pivot_order=2`) with workspace allocation factor 3. The linear solver remains MA57 for every comparison.

## Cases and comparison protocol

`cases/case_01.json` through `case_08.json` preserve the first eight A100 boundary/obstacle datasets from the author's RA-L archive: ten vehicles and five circular obstacles per case. Only task data are reused; the optimization routines and descriptive variable names are newly implemented.

The common mesh has 81 nodes, a free 5–75 s horizon, two-disc vehicle covers, and a 0.15 m nominal clearance. Reverse motion is allowed. The archived jerk limit is 0.2 m/s^3; this differs from the original paper's parameter table. These results are not exact reproductions of its historical table.

ASCO rebuilds disc-pair/time rows using signed surface gaps. Initial thresholds are -4 and 2 m; failed solves raise the lower threshold by 3 m. Successful but invalid candidates lower it by 1.3 m (floor -4 m) and expand the upper threshold by 0.5 m. Crossing thresholds triggers the documented reset/expansion safeguard. Final validation always checks every interaction.

`full` loads every collision row. `progressive` accumulates rows near conflict. `linearized` linearizes collision rows while retaining nonlinear forward Euler dynamics; it is not a fully convex sequential-convex-programming implementation. These four implementations are not all the historical comparators in the original paper.

The target case budget is 240 s. Iterative methods cap each NLP at 75 s; the full NLP uses the case budget in one call. Construction, validation, and an ongoing solve can exceed the target. A 16-iteration guard also applies. Reported wall times exclude shared independent-vehicle initialization and are descriptive rather than hardware-independent performance bounds.

`prepare_reference` plans each vehicle with static obstacles and synchronizes the results. A failed independent solve retains its geometric seed and reports the failure. Every comparator receives the identical cached reference, including difficult initializations.

## Validation and outputs

`validate_plan` checks native Euler residuals, endpoints, bounds, and all nominal vehicle/obstacle interactions. Between nodes it evaluates the piecewise-linear rear-axle/heading reconstruction at 40 subdivisions, with a speed-based intersample correction. `audit_control_bounds` checks the interpolated states and piecewise-constant controls. No time-dilation repair is applied. A failed solver call or failed validation remains a failed experiment.

`output/case_*.mat` stores complete candidates and references. `trajectory_*.csv` uses the same field names as Chapter 5. JSON retains statuses, timings, selection histories, and validation. Failed candidates are preserved for diagnosis but are never plotted as accepted demonstrations. `reports` holds the recorded comparison; fresh runs write to `output`.

Eight fixed cases support a reproducible regression study, not a statistical claim over arbitrary environments. Reference: Li et al., IEEE Robotics and Automation Letters 6(2), 1511–1518 (2021).
