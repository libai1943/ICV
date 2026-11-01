# Chapter 11: integrated experimental ROS sources

This directory contains the complete tracked source snapshot of the experimental branch. Yueshuo Sun developed the initial cooperative-planner implementation and supplied this experimental integration. The existing contributions and licenses of the supporting packages remain in their files.

## Provenance

- Original repository: https://github.com/YueshuoSun/CollabPlanner
- Original branch: `experiment`
- Exact imported commit: `81774359b95ae1a45bfd9a0c06e93a04197694f8`
- Snapshot: https://github.com/YueshuoSun/CollabPlanner/tree/81774359b95ae1a45bfd9a0c06e93a04197694f8
- Book repository: https://github.com/libai1943/ICV

## Editorial changes

- Translated Chinese comments and user-facing messages into English.
- Consolidated Yueshuo Sun's first-party contribution headers in this README.
- Preserved mathematical expressions, identifiers, ROS interfaces, configuration values, and third-party notices, except for the specific changes listed below.

- Selected IPOPT's `ma57` linear solver to match the book. A compatible, separately installed HSL library providing MA57 is required.
- Replaced the developer-specific absolute log-file prefix with `logfile_` in the process working directory. With a standard ROS launch this is usually the ROS runtime directory; direct execution uses the shell's current directory.
- Updated the `collab_planner` package maintainer to Bai Li, whose existing package author contact is `libai@zju.edu.cn`. Yueshuo Sun's original implementation credit is retained above and in the contribution table.
- Transliterated the utility author name as Yakun Ouyang while retaining the original creation dates and attribution. Other existing author and copyright notices remain in their source files.

## Build-metadata corrections

- Removed the stale `util` build-export dependency from `collab_planner/package.xml`; the package builds its own `common/util` sources.
- Declared `sandbox_msgs` in the planner package manifest and its catkin component/export lists, matching the existing `sandbox_msgs/Trajectory.h` include.
- Corrected the localization dependency from the executable name `vrpn_client_node` to its package name `vrpn_client_ros`.

These changes repair dependency declarations; the planning, tracking, localization, and communication algorithms are unchanged.

## Verification scope

The import was checked for remaining Chinese text, copied-file completeness, preservation of third-party files, and changes to executable tokens. The ROS packages have not been compiled or run on a vehicle in the import environment.

## Package layout

The imported tree contains `collab_planner`, `pure_pursuit`, `sandbox_msgs`, `chassis`, `nokov_localization`, `vrpn_client_ros`, and `third_party`. Chapter 10 also contains copies of `pure_pursuit` and `sandbox_msgs` for chapter-local reading. Place only one copy of each ROS package in a catkin workspace.

## Original per-file contributions

The original first-party name/date headers have been consolidated here.

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

## Book revision: explicit forward Euler integral

The tracking node now uses the previous sample error in its incremental PI integral term. This is the documented change after the initial import. The same edit is maintained in both Chapters 10 and 11. Default zero-integral-gain behavior is unchanged; Chapter 10 includes an algebraic regression with nonzero gains. This revision has not been tested on a vehicle.
