# Chapter 11: integrated experimental ROS sources

This directory contains the experimental ROS source. Yueshuo Sun developed the initial cooperative-planner implementation and supplied this experimental integration. The existing contributions and licenses of the supporting packages remain in their files.

## Provenance

- Original repository: https://github.com/YueshuoSun/CollabPlanner
- Original branch: `experiment`
- Source commit: `81774359b95ae1a45bfd9a0c06e93a04197694f8`
- Snapshot: https://github.com/YueshuoSun/CollabPlanner/tree/81774359b95ae1a45bfd9a0c06e93a04197694f8
- Book repository: https://github.com/libai1943/ICV

## Numerical and runtime conventions

Vehicle dynamics use first-order explicit forward Euler. Nonlinear optimization selects IPOPT with MA57. CSV log paths are relative to the process working directory. Package dependencies are declared in the supplied manifests and CMake files.

The eight-vehicle target assignment retains 50 candidates with the smallest distance variance, then maximizes their distance sum. Lexicographic assignment indices resolve ties. The result is a fixed-size target array.

See the parent chapter README for build and launch instructions. A catkin workspace contains one copy of each package name.

## Source contributions


The following files credit the original implementation by Yueshuo Sun.

| File | Contribution | Original date |
|---|---|---|
| `collab_planner/collab_node.cpp` | Created | 2023/12/30 |
| `collab_planner/common/environment.h` | Created | 2024/01/02 |
| `collab_planner/common/trajectory_point.h` | Created | 2023/11/2 |
| `collab_planner/common/util/point_cloud.h` | Modified | 2024/01/02 |
| `collab_planner/common/util/time.h` | Modified | 2023/11/03 |
| `collab_planner/common/vehicle_param.h` | Created | 2024/01/02 |
| `collab_planner/trajectory_nlp/collab_traj_nlp.cc` | Created | 2024/01/01 |
| `collab_planner/trajectory_nlp/collab_traj_nlp.h` | Created | 2024/01/01 |
| `collab_planner/trajectory_nlp/collab_traj_optimizer.cpp` | Created | 2023/12/31 |
| `collab_planner/trajectory_nlp/collab_traj_optimizer.h` | Created | 2023/12/31 |
| `collab_planner/trajectory_nlp/collab_traj_optimizer_parallel.cpp` | Created | 2023/12/31 |
| `collab_planner/trajectory_nlp/trajectory_nlp_config.h` | Modified | 2023/11/2 |
| `collab_planner/visualization_plot.cpp` | Modified | 1/26/24 |

