# Chapter 10: trajectory-tracking ROS packages

These are chapter-local copies of `pure_pursuit` and `sandbox_msgs` from the experimental integration supplied by Yueshuo Sun. The original per-file authorship and package metadata remain unchanged. The initial cooperative-planner implementation and the source integration are credited to Yueshuo Sun.

## Provenance

- Original repository: https://github.com/YueshuoSun/CollabPlanner
- Original branch: `experiment`
- Exact imported commit: `81774359b95ae1a45bfd9a0c06e93a04197694f8`
- Original paths: `pure_pursuit/` and `sandbox_msgs/`
- Snapshot: https://github.com/YueshuoSun/CollabPlanner/tree/81774359b95ae1a45bfd9a0c06e93a04197694f8
- Book repository: https://github.com/libai1943/ICV

These packages are copied without algorithmic changes. The Chapter 11 experimental tree includes the same packages. Put only one copy of each package in a catkin workspace. The existing Chapter 10 MATLAB PID example has been preserved. ROS build and vehicle execution were not performed in the import environment.

## Book revision: explicit forward Euler integral

The tracking node now uses the previous sample error in its incremental PI integral term. This is the documented change after the initial import. The same edit is maintained in both Chapters 10 and 11. Default zero-integral-gain behavior is unchanged; Chapter 10 includes an algebraic regression with nonzero gains. This revision has not been tested on a vehicle.
