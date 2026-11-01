# Chapter 3: Numerical Methods and Initial Guesses

## Run

## Numerical environment

The recorded runs use MATLAB R2021b, CasADi 3.7.2, IPOPT 3.14.11, and **MA57**. Every vehicle-motion transcription and initial-guess propagation uses **first-order explicit forward Euler**. No automatic linear-solver fallback is allowed.

Install a compatible CasADi MATLAB package and obtain an HSL library containing MA57 under an appropriate license. The HSL library and any required runtime libraries must match MATLAB's architecture. Licensed binaries and Times New Roman font files are not distributed with this source.

```matlab
setenv('CASADI_MATLAB_PATH','/absolute/path/to/casadi-matlab-package');
setenv('IPOPT_HSL_LIBRARY','/absolute/path/to/libcoinhsl');
```

Use the actual platform-specific filename, including its extension. On Windows, add the directory containing dependent HSL runtime DLLs to `PATH` before launching MATLAB. A missing or incompatible MA57 library is an error to resolve, not a reason to substitute another solver. Launch MATLAB with `-singleCompThread` and set `OMP_NUM_THREADS=1` and `MKL_NUM_THREADS=1` before launch for the recorded threading configuration. Timings are single-run wall measurements and may be affected by concurrent workloads.

The graph searches require base MATLAB; the scalar optimization additionally requires the numerical environment above. Run:

```matlab
cd Chapter3
report = run_examples;
```

The command writes `output/validation.json`, `gradient_results.csv`, and the vector PDF/PNG connectivity diagram. Times New Roman is used if installed; its font files are not distributed.

## Included implementations

- `gradient_example`: IPOPT/MA57 local solutions from the 12 initial guesses used in the chapter's nonconvex scalar example.
- `astar_grid`: four- or eight-connected spatial A*. Coordinates are `[row,column]`, `true` cells are blocked. Cardinal moves cost 1 and diagonals cost sqrt(2). A diagonal is rejected if either incident cardinal cell is blocked. Manhattan and octile heuristics match the respective costs. Improved closed nodes may reopen.
- `space_time_astar`: four-connected unit-duration moves plus waiting, minimizing arrival time within a finite horizon. `C.vertex` rows are `[row,column,time]`. `C.edge` rows are `[r0,c0,r1,c1,departureTime]`. Time starts at zero. An agent stays at its goal after arrival, so future goal constraints must also be satisfied.
- `first_conflict`: earliest vertex or edge-swap conflict, with paths extended by staying at their goals.
- `cbs_grid`: best-first constraint-tree search minimizing sum of arrival times, with exact low-level search on the bounded graph. Each conflict creates two children; each child replans only one agent.
- `draw_connectivity`: reproducible four/eight-neighbor vector illustration.

For readability, open lists use arrays rather than a heap. This does not change the search result, but is not intended for large maps. CBS has a 10,000-expansion guard; exceeding it is reported as failure, not as proof that the task is infeasible. The finite low-level horizon can likewise exclude solutions that arrive later.

## Executed checks

The sample spatial grid has A* costs 15 (four-connected) and 12.6568542495 (eight-connected), each matched against Dijkstra's algorithm. Other checks reject diagonal corner cutting, enforce a future goal restriction, detect a pure edge swap, and resolve a two-agent swap with CBS sum of costs 6. An independent exhaustive enumeration rules out every candidate pair with smaller cost for this small example.

The 12 IPOPT/MA57 runs on the bounded interval [-8,8] reproduce the book table. First-order KKT conditions are checked, including one-sided conditions at a bound. The outputs include terminal values and iteration counts, not only a plot.

## Model boundary

These searches model discrete point agents. They do not impose vehicle steering, acceleration, finite footprints, or swept-body clearance. Four-connected vertex and edge-swap conflict checks are complete for the stated discrete model; they must not be reused as a vehicle collision certificate. Use the resulting paths as coarse initial guesses when building a continuous cooperative trajectory planner.

## References

P. E. Hart, N. J. Nilsson, and B. Raphael, *A Formal Basis for the Heuristic Determination of Minimum Cost Paths*, IEEE TSSC, 1968.

G. Sharon, R. Stern, A. Felner, and N. R. Sturtevant, *Conflict-Based Search for Optimal Multi-Agent Pathfinding*, Artificial Intelligence, 219, 40–66, 2015.

## Illustrated results

`run_examples` calls `draw_search_examples` to regenerate Figures 3.3-3.5 from computed paths. `illustrated_examples.json` records four/eight-connected A* costs 15 and 12.6568542495 (47/33 expansions), the corridor path with two waits and arrival time 6, and CBS sum of costs 6 (8 high-level expansions). The CBS root and both first children are explicitly checked for the illustrated vertex/edge conflicts. Subfigure captions are centered below panels.
