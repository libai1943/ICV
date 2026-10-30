# Chapter 4: Two-Stage Cooperative Lane Changes

This companion folder provides editable MATLAB/CasADi equations and actual simulation results. It implements the two-stage structure of B. Li et al., *Balancing Computation Speed and Quality: A Decentralized Motion Planning Method for Cooperative Lane Changes of Connected and Automated Vehicles*, IEEE TIV 3(3), 340–350, 2018, doi:10.1109/TIV.2018.2843159.

## Reproduce

1. Install a CasADi MATLAB package appropriate to your operating system. The executed runs used MATLAB R2021b and CasADi 3.7.2 with Ipopt/MUMPS. No Optimization Toolbox, AMPL, HSL DLL, or protected `.p` file is required by this entry point.
2. Set its path and run:

```matlab
setenv('CASADI_MATLAB_PATH','/absolute/path/to/casadi-matlab-package');
run_experiments;
make_figures;
```

After one successful execution, `make_figures` alone regenerates the vector PDF figures. Install Times New Roman for matching typography. Solver and font binaries are not bundled.

## Inputs and conventions

`state=[x;y;theta;v;phi]`, `control=[a;omega]`, rear-axle coordinates, SI units and radians. Acceleration is piecewise linear, so its interval derivative is jerk. The explicit jerk bound is imposed on `diff(a)/dt`; jerk rate is not constrained. Positive speed is forward, positive steering turns left.

The public upstream task generation, physical dimensions, bounds, descending-x priority, and 0.5 m backward translation increment are retained from libai1943/MultiCavLaneChange, commit 9831df03d1f78ff913fc686ff8b9c49c480feacc. The implementation is editable and uses trapezoidal direct transcription. It uses rectangular-footprint SAT screening and four-corner road bounds rather than a two-disc surrogate. This numerical variant is stated in the chapter.

The default `run_experiments` runs three cases, each with ten ICVs: Baseline (jerk limit 0.2); Tighter jerk bound (the same task with limit 0.1); and Alternate lane assignment (jerk limit 0.2). The native meshes have 300 and 100 nodes. `run_ch04_readable(config)` accepts `seed`, `vehicle_count`, `jerk_limit`, `K1`, `K2`, and `output_directory`. The solver must report success and every validation assertion must pass before a result is saved.

## Results

Total durations were 12.900626, 15.471504, and 14.494044 seconds. These are maneuver durations, not run times. Actual solver statuses, iteration counts, single-run planning times, parameters, and checks are saved under `output`.

Each case contains `book_result.mat` (task, both stages, and full trajectory), `trajectory.csv`, and `book_validation.json`. The combined `experiment_report.json` contains concise measurements. `make_figures` creates the scene, covering-disc, actual conflict/translation, actual priority, six-snapshot, state/control, and jerk-variation figures.

## Validation interpretation

The validation checks trapezoidal rate residuals, endpoints, stage continuity, speed/steering/acceleration/rate/jerk bounds, road containment, and every rectangle pair. The between-node clearance certificate applies to piecewise-linear rear-axle positions and unwrapped headings. Separately, RK4 integration with five substeps per native interval and linearly interpolated controls measures the discrepancy from the transcribed trajectory. It is not the same reconstruction as the geometric certificate.

These three completed feasibility demonstrations do not establish global optimality or broad statistical performance. No comparison against other planners is claimed.

## Figure conventions

The vehicle body uses a centered triangular forward marker. Covering-disc boundaries are drawn above the body; subfigure captions are centered below panels. `make_figures` loads the actual saved results, including the conflict pair before translation and the six stages of the baseline maneuver. Case seeds remain in configuration/metadata, not experimental case labels. Jerk is lowercase in equations and plot labels. Generated MAT/PDF/PNG files are not required in the source checkout: the two commands above regenerate them.
