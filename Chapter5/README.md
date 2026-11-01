# Chapter 5: Cooperative Planning at Irregular Intersections

Independent MATLAB implementation of safe travel corridors (STCs), relative STCs, Social Force Model (SFM) proposals, and the lightweight iterative optimization method (LIOM). The five curved approaches have unequal widths, oblique junction angles, and polygonal land blocks.

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
cd Chapter5
self_test;
run_experiments([9,12,15]);
export_results;
make_figures;
```

## Formulation and initialization

The physical states are `[x;y;theta;v;phi;a]`, with controls `[jerk;omega]`, rear-axle position, SI units, and radians. Eight auxiliary rows locate four overlapping covering discs. A 4.689 m by 1.942 m vehicle is covered by four discs of radius approximately 1.134 m. Each task uses 241 nodes over 120 s, forward motion, and a 0.12 m nominal geometric margin.

All five curved branches carry opposing traffic. Their widths are `[5.6,5.8,5.5,5.9,5.7]` m; the central region has elliptical semiaxes 12 m and 9 m. Each road is only moderately wider than two vehicle bodies (3.884 m together). Fixed scenario seeds perturb branch angles while retaining unequal widths, strong bends, oblique approaches, and nonconvex roadside land blocks. Both arrivals and departures use every branch.

`simulate_sfm` provides a diagnostic social-force rollout. All reported optimization runs use references independently constructed by `prepare_reference` from curved routes and deterministic temporal reservations. LIOM refines these reservation references. The MATLAB implementation uses a fixed 1 s scaling time when combining acceleration-scaled virtual forces with desired velocity to obtain a steering direction.

`construct_corridors` checks complete candidate boxes against polygonal obstacles, including their interiors. It starts with a positive box around each center and refines the expansion increment near oblique boundaries. Relative corridors are formed for individual covering-disc pairs.

Node feasibility alone is insufficient. Additional separating-plane constraints cover every intervening interval: a fixed heading-increment bound gives a rotational chord-deviation allowance, and both ends of each relative-center chord must lie on the safe side of its plane. Pairs may be screened only when their complete endpoint boxes and this allowance guarantee separation. These constraints retain an affine form within each outer iteration.

`optimize_trajectories` retains these hard constraints and penalizes nonlinear forward Euler and center-linking residuals with increasing weights. A final nonlinear feasibility-restoration step retains the same physical constraints. IPOPT convergence and numerical feasibility are recorded separately: a finite, hard-feasible intermediate iterate may update the reference even if IPOPT reaches an iteration limit. An experimental trajectory is accepted only after the independent checks pass.

## Validation and recorded results

The recorded 9-, 12-, and 15-ICV tasks pass the independent validator. See `reports/experiment_report.json` for actual residuals, clearances, runtimes, initialization diagnostics, and all intermediate solver statuses. The results describe the feasibility and computational effort of these three tasks.

`validate_trajectories` checks every native Euler update, boundary state, control bound, covering-disc pair, and polygon. Between nodes it checks the piecewise-linear rear-axle/heading reconstruction with 80 subdivisions and a speed-based conservative clearance correction. `audit_control_bounds` checks that reconstruction and its interval controls. The geometric check uses the declared piecewise-linear reconstruction. An independent NumPy implementation also verifies the exported trajectory files.

MAT files retain initial references, trajectories, solver histories, and validation. CSV files use explicit SI-unit column names. JSON retains the intermediate solver diagnostics. The plot generator uses the validated saved trajectories. `output/figures` contains vector PDFs and PNG previews, while `reports` preserves the recorded numerical data independently of subsequent runs.

The vehicle body contains a triangular forward marker; colors, Times New Roman labels, and subfigure captions follow the companion chapters. Figure 5.1 is a credited real photograph in the manuscript, not a simulation output.

References: Bu et al., IEEE TITS (2025), DOI 10.1109/TITS.2025.3597472; Li et al., IEEE TITS (2022), DOI 10.1109/TITS.2021.3109011.

For the optional final print-size treatment of subfigure labels, run `python ../tools/typeset_figures.py` from this directory after installing PyMuPDF and specifying an installed Times New Roman font if necessary. The geometry remains vector content; this step is independent of optimization.

`resume_experiment(N)` resumes a saved, hard-feasible checkpoint that already contains the whole-interval guards. A fresh `run_experiments` run remains the canonical reproduction command.
