# Chapter 4: Two-Stage Cooperative Lane Changes

This companion folder provides editable MATLAB/CasADi equations and actual simulation results. It implements the two-stage structure of B. Li et al., *Balancing Computation Speed and Quality: A Decentralized Motion Planning Method for Cooperative Lane Changes of Connected and Automated Vehicles*, IEEE TIV 3(3), 340–350, 2018, doi:10.1109/TIV.2018.2843159.

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
cd Chapter4
run_experiments;
make_figures;
```

After a completed run, `make_figures` alone regenerates the vector PDF figures from the saved trajectories. Install Times New Roman for matching typography.

## Inputs and conventions

`state=[x;y;theta;v;phi]`, `control=[a;omega]`, rear-axle coordinates, SI units and radians. Acceleration samples are rate-limited by their forward differences. The explicit jerk bound is imposed on `diff(a)/dt`; jerk rate is not constrained. Positive speed is forward, positive steering turns left.

The public upstream task generation, physical dimensions, bounds, descending-x priority, and 0.5 m backward translation increment are retained from libai1943/MultiCavLaneChange, commit 9831df03d1f78ff913fc686ff8b9c49c480feacc. The implementation is editable and uses first-order explicit forward Euler transcription. It uses rectangular-footprint SAT screening and four-corner road bounds rather than a two-disc surrogate. This numerical variant is stated in the chapter.

The default `run_experiments` runs three cases, each with ten ICVs: Baseline (jerk limit 0.2); Tighter jerk bound (the same task with limit 0.1); and Alternate lane assignment (jerk limit 0.2). The native meshes have 300 and 100 nodes. `run_ch04_readable(config)` accepts `seed`, `vehicle_count`, `jerk_limit`, `K1`, `K2`, and `output_directory`. The solver must report success and every validation assertion must pass before a result is saved.

## Results

Total durations were 12.900645, 15.471526, and 14.494063 seconds. These are maneuver durations, not run times. Actual solver statuses, iteration counts, single-run planning times, parameters, and checks are saved under `output`.

Each case contains `book_result.mat` (task, both stages, and full trajectory), `trajectory.csv`, and `book_validation.json`. The combined `experiment_report.json` contains concise measurements. `make_figures` creates the scene, covering-disc, actual conflict/translation, actual priority, six-snapshot, state/control, and jerk-variation figures.

## Validation interpretation

The validation checks forward Euler residuals, endpoints, stage continuity, speed/steering/acceleration/rate/jerk bounds, road containment, and every rectangular vehicle pair. The between-node clearance certificate applies to piecewise-linear rear-axle positions and unwrapped headings. A separate native-mesh forward Euler rollout checks accumulated propagation error. Interpolation for geometric checking is not a different dynamics integrator.

These three completed feasibility demonstrations do not establish global optimality or broad statistical performance. No comparison against other planners is claimed.

## Figure conventions

The vehicle body uses a centered triangular forward marker. Covering-disc boundaries are drawn above the body; subfigure captions are centered below panels. `make_figures` loads the actual saved results, including the conflict pair before translation and the six stages of the baseline maneuver. Case seeds remain in configuration/metadata, not experimental case labels. Jerk is lowercase in equations and plot labels. Generated MAT/PDF/PNG files are not required in the source checkout: the two commands above regenerate them.
