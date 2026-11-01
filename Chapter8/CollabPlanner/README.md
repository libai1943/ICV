# Chapter 8: cooperative-planning ROS package

This directory contains the demonstration planner introduced in Chapter 8. The initial implementation was developed by Yueshuo Sun and is now maintained with the book's companion code.

## Provenance

- Original repository: https://github.com/YueshuoSun/CollabPlanner
- Original branch: `main`
- Exact imported commit: `2f567625bfb7fac27d17316de3bb8f0e3021705c`
- Snapshot: https://github.com/YueshuoSun/CollabPlanner/tree/2f567625bfb7fac27d17316de3bb8f0e3021705c
- Book repository: https://github.com/libai1943/ICV

## Editorial changes

- Translated Chinese comments and user-facing messages into English.
- Consolidated Yueshuo Sun's first-party contribution headers in this README.
- Preserved mathematical expressions, identifiers, ROS interfaces, configuration values, and third-party notices, except for the specific changes listed below.

- Selected IPOPT's `ma57` linear solver to match the book. A compatible, separately installed HSL library providing MA57 is required.
- Replaced the developer-specific absolute log-file prefix with `logfile_` in the process working directory. With a standard ROS launch this is usually the ROS runtime directory; direct execution uses the shell's current directory.
- Updated the `collab_planner` package maintainer to Bai Li, whose existing package author contact is `libai@zju.edu.cn`. Yueshuo Sun's original implementation credit is retained above and in the contribution table.
- Transliterated the utility author name as Yakun Ouyang while retaining the original creation dates and attribution. Other existing author and copyright notices remain in their source files.

## Build-metadata correction

Removed the stale `util` build-export dependency from `package.xml`. The package uses its own `common/util` sources, already compiled by its CMake file; no separate `util` ROS package is required.

## Verification scope

The import was checked for remaining Chinese text, copied-file completeness, preservation of third-party files, and changes to executable tokens. The ROS packages have not been compiled or run on a vehicle in the import environment.

## Original per-file contributions

The original first-party name/date headers have been consolidated here.

| File | Contribution | Original date |
|---|---|---|
| `collab_node.cpp` | Created | 2023/12/30 |
| `common/environment.h` | Created | 2024/01/02 |
| `common/trajectory_point.h` | Created | 2023/11/2 |
| `common/util/point_cloud.h` | Modified | 2024/01/02 |
| `common/util/time.h` | Modified | 2023/11/03 |
| `common/vehicle_param.h` | Created | 2024/01/02 |
| `trajectory_nlp/collab_traj_nlp.cc` | Created | 2024/01/01 |
| `trajectory_nlp/collab_traj_nlp.h` | Created | 2024/01/01 |
| `trajectory_nlp/collab_traj_optimizer.cpp` | Created | 2023/12/31 |
| `trajectory_nlp/collab_traj_optimizer.h` | Created | 2023/12/31 |
| `trajectory_nlp/collab_traj_optimizer_parallel.cpp` | Created | 2023/12/31 |
| `trajectory_nlp/trajectory_nlp_config.h` | Modified | 2023/11/2 |
| `visualization_plot.cpp` | Modified | 1/26/24 |
