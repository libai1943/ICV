# Chapter 7: Fifty-Vehicle Formation Reconfiguration

Editable MATLAB/CasADi reconstruction of receding-horizon coupled planning. The benchmark uses the first fifty assignments of source formation case 4, with position coordinates scaled by 0.3 and physical vehicle dimensions unchanged. `formation_case04.csv` preserves the complete source case in columns `[center_x0,center_y0,heading0,center_x_goal,center_y_goal,heading_goal]`. The goal heading is free in this experiment.

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
cd Chapter7
run_experiment(50);
export_results;
make_figures;
```

The implementation has five states `[x;y;theta;v;phi]`, two controls `[a;omega]`, and two auxiliary disc-center coordinates. The rear-axle reference is used in the motion equations; input and target formation coordinates are vehicle centers. Forward Euler uses 0.5 s steps and a 10 s window. Each committed segment is 5 s. End speed and continuation controls are zero, so the retained tail admits a stationary extension.

Trust boxes around center references widen along each window. Collision rows are screened separately at nodes and interval midpoints. Retained pairs use conservative supporting half-spaces with normals fixed from the current reference. Midpoint screening and separation include the bounded difference between a midpoint center and the average endpoint centers. The final validator checks every physical disc pair independently of those affine rows.

The local solver uses the residual-refinement structure of Chapter5: hard affine constraints and a cost comprising squared forward Euler/center-linking residuals plus the progress cost divided by a growing penalty weight. The sequence is 1e2, 1e4, 1e6, 1e8, 1e10. Each stage permits 700 IPOPT iterations and 45 s of solver CPU time; MA57 uses AMD ordering and workspace allocation factor 3. A Damped Gauss–Newton residual Hessian is supplied. This is a stated numerical variant, not an exact reproduction of the original local NLP implementation.

Each window permits two local refinements. Unsuccessful intermediate iterates can seed later penalty stages, and only independent full feasibility validation permits commitment. Solver convergence is recorded separately. The full stage history remains in the results. Failure is never converted to success merely because a plotted trajectory appears plausible.

## Completion and validation

The validator checks all native Euler equations, component bounds, and every vehicle pair, independent of screened rows. Piecewise-linear rear-axle positions and unwrapped headings are checked at twenty subdivisions with a center-speed correction. The enclosing disc covers the entire rectangular body. No physical tracking guarantee follows from this numerical check.

When terminal center errors fall below 0.5 m, a final solve adds exact target-center equalities. Completion requires maximum goal error below 1e-4 m, stopped endpoints, and the full validator. Failure or the window limit is reported explicitly; a partial trajectory is never labeled a completed formation.

`output/checkpoint_50.mat` permits resumption after an interruption. To start a fresh run, move the checkpoint and old outputs to a separate archive first. Do not resume a checkpoint after changing task geometry, time step, constraints, or vehicle count. `partial_50.mat` and `latest_window.mat` are diagnostic files, not completed results. A completed run writes `formation_50.mat`, trajectory CSV, experiment JSON, and vector figures.

The solver may return different local solutions on another numerical platform. Recorded timings are descriptive wall measurements, not real-time performance claims. The graph-screening and rolling-window design have no general guarantee of avoiding local deadlock or reaching every assigned formation.

The Gauss–Newton Hessian includes a 1e-6 diagonal damping term. A rejected second local refinement retains the already validated first refinement; a failed first refinement stops the run without claiming completion. The benchmark CSV uses 17 significant digits to preserve the archived double-precision inputs.

Acceptance is based on independently verified feasibility, not the optimizer termination flag alone. A feasible iterate at an iteration limit can be committed; `solver_success` and every stage status are retained, and no local-optimality claim is made for it. Infeasible candidates are rejected.
