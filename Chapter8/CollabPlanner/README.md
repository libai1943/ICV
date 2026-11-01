# Chapter 8: cooperative-planning ROS package

This directory contains the demonstration planner introduced in Chapter 8. The initial implementation was developed by Yueshuo Sun and is now maintained with the book's companion code.

## Provenance

- Original repository: https://github.com/YueshuoSun/CollabPlanner
- Original branch: `main`
- Source commit: `2f567625bfb7fac27d17316de3bb8f0e3021705c`
- Snapshot: https://github.com/YueshuoSun/CollabPlanner/tree/2f567625bfb7fac27d17316de3bb8f0e3021705c
- Book repository: https://github.com/libai1943/ICV

## Numerical and runtime conventions

Vehicle dynamics use first-order explicit forward Euler. Nonlinear optimization selects IPOPT with MA57. CSV log paths are relative to the process working directory. Package dependencies are declared in the supplied manifests and CMake files.

The eight-vehicle target assignment retains 50 candidates with the smallest distance variance, then maximizes their distance sum. Lexicographic assignment indices resolve ties. The result is a fixed-size target array.

See the parent chapter README for build and launch instructions. A catkin workspace contains one copy of each package name.

## Source contributions


The following files credit the original implementation by Yueshuo Sun.

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
