# Chapter 10: trajectory-tracking ROS packages

These are chapter-local copies of `pure_pursuit` and `sandbox_msgs` from the experimental integration supplied by Yueshuo Sun. The original per-file authorship and package metadata remain unchanged. The initial cooperative-planner implementation and the source integration are credited to Yueshuo Sun.

## Provenance

- Original repository: https://github.com/YueshuoSun/CollabPlanner
- Original branch: `experiment`
- Source commit: `81774359b95ae1a45bfd9a0c06e93a04197694f8`
- Original paths: `pure_pursuit/` and `sandbox_msgs/`
- Snapshot: https://github.com/YueshuoSun/CollabPlanner/tree/81774359b95ae1a45bfd9a0c06e93a04197694f8
- Book repository: https://github.com/libai1943/ICV

The tracking and message packages match Chapter 11. Put one copy of each package in a catkin workspace. The incremental PI integral uses the previous-sample error, consistent with first-order explicit forward Euler. The tracking node validates candidate poses and half-open trajectory ranges before accessing them and publishes a neutral command when the maneuver ends.
