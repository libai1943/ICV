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

The states are `[x;y;theta;v;phi;a]`, with controls `[jerk;omega]`, rear-axle position, SI units, and radians. Two auxiliary rows represent the enclosing-disc center. The tasks use 241 nodes over 120 s, forward motion, passenger-car dimensions, and a 0.25 m nominal separation margin. Approaches range from 6.8 to 8.0 m in width. Scenario seeds change their angles slightly while retaining the same five-arm construction.

`simulate_sfm` produces a heuristic proposal; it is not guaranteed to reach the assigned endpoints. The recorded SFM proposals did not do so. `prepare_reference` therefore constructs curved geometric routes with deterministic temporal reservations. The accepted experiments evaluate this complete safeguarded pipeline, not SFM alone. The MATLAB routines are independently implemented and do not invoke the research Python optimizer.

`construct_corridors` checks complete candidate boxes against polygonal obstacles, including their interiors. It decreases the expansion increment near oblique boundaries. Relative corridors are formed in relative-center space. `optimize_trajectories` retains the corridor constraints and penalizes nonlinear forward Euler and center-linking residuals with increasing weights; optional nonlinear feasibility restoration retains the same physical constraints. Failed intermediate solver calls remain in the history.

## Validation and recorded results

All three recorded fleet sizes passed the independent validator. Maximum Euler defects were below 2.4e-6, and the conservative interval clearance bounds exceeded 0.13 m. These are numerical feasibility demonstrations, not global-optimality or real-time claims.

`validate_trajectories` checks every native Euler update, boundary state, control bound, vehicle pair, and polygon. Between nodes it checks the piecewise-linear rear-axle/heading reconstruction with 20 subdivisions and a speed-based clearance correction. `audit_control_bounds` checks that reconstruction and the interval controls. This geometric interpolation does not replace the forward Euler dynamics and is not a closed-loop tracking certificate.

MAT files retain initial references, trajectories, solver histories, and validation. CSV files use explicit SI-unit column names. JSON includes failed intermediate iterations. The plot generator requires an accepted saved trajectory; it never manufactures an experimental solution. `output/figures` contains vector PDFs and PNG previews, while `reports` preserves the recorded numerical data independently of subsequent runs.

The vehicle body contains a triangular forward marker; colors, Times New Roman labels, and subfigure captions follow the companion chapters. Figure 5.1 is a credited real photograph in the manuscript, not a simulation output.

References: Bu et al., IEEE TITS (2025), DOI 10.1109/TITS.2025.3597472; Li et al., IEEE TITS (2022), DOI 10.1109/TITS.2021.3109011.
