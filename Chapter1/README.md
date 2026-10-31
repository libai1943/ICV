# Chapter 1: Introductory Cooperative-Motion Illustrations

The 24-ICV illustration combines independently optimized bicycle-model maneuvers, collision-checked priority scheduling, moving incoming queues, and continuing outgoing traffic. It is an introductory example, not a jointly time-optimal LIOM benchmark. Road width is 7.0 m; each physical body is 4.689 m long and 1.942 m wide.

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
cd Chapter1
run_intro_intersection;
make_intersection_figure;
```

Every maneuver and queue update uses explicit forward Euler and IPOPT/MA57. The drawing routine checks all vehicles and corner obstacles, including between-node geometry of the declared linear pose reconstruction. It selects a snapshot with incoming queues, turning traffic, and at least four outgoing vehicles. The recorded frame contains eighteen visible ICVs. All axes use a fixed physical scale; no coordinate ticks are drawn.

The upper historical panel is a credited scan in the manuscript. The MATLAB lower panel preserves its aspect ratio; the manuscript assembly uses equal-size viewports without stretching. That historical image is not distributed as program-generated data.

After running Chapter4, `make_lane_change_figure` reads its new baseline and shows the two stages, including two intermediate lane changes. The Chapter7 figure generator supplies the fifty-vehicle formation illustration from its completed result. These introductory figures therefore share the experiment data used later in the book.
