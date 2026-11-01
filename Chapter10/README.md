# Chapter 10: trajectory tracking and discrete PID conventions

This chapter provides the MATLAB convention example and chapter-local copies of the ROS tracking and message packages.

## MATLAB explicit-Euler illustration

`pid_euler_step` computes the command using the stored integral, then advances that integral by first-order explicit forward Euler. The equivalent incremental expression therefore uses the **previous error** in its integral increment. `self_test` compares both expressions over 1,000 input samples.

Run from the Chapter 10 folder in MATLAB:

```matlab
self_test
```

This is an unsaturated algebraic illustration. Actuator limits, anti-windup, derivative filtering, and physical tracking performance are addressed separately in a complete controller. The measured-error derivative uses a causal difference.

## Imported ROS controller

`ros_packages/pure_pursuit` contains the tracking node, and `ros_packages/sandbox_msgs` contains the required messages. The ROS source preserves the original incremental longitudinal controller, whose integral increment uses the **current error**. It therefore uses a different discrete integral convention from the explicit-Euler MATLAB illustration. Its recorded default integral gain is zero, so the choice of current or previous error contributes zero to that default integral term.

Both chapter-local ROS packages are the same source snapshots included in Chapter 11. Their initial integration and exact upstream commit are recorded in [ros_packages/README.md](ros_packages/README.md). The existing per-file authors and package metadata are retained.

## Build the tracking packages in a separate workspace

Keep the central ICV checkout outside any catkin `src` directory. In a fresh terminal:

```bash
source /opt/ros/noetic/setup.bash
export ICV_BOOK_ROOT="$HOME/src/ICV"
mkdir -p "$HOME/tracking_ws/src"
ln -sT "$ICV_BOOK_ROOT/Chapter10/ros_packages/pure_pursuit" \
  "$HOME/tracking_ws/src/pure_pursuit"
ln -sT "$ICV_BOOK_ROOT/Chapter10/ros_packages/sandbox_msgs" \
  "$HOME/tracking_ws/src/sandbox_msgs"
cd "$HOME/tracking_ws"
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
catkin_make
source devel/setup.bash
```

This workspace contains the two packages needed to study the tracking interface. Chapter 11 supplies the localization, chassis, planner, and integrated launch procedure. Its workspace already contains its own tracking and message packages, so use one copy of each package name in any workspace.

## Source archive and verification

`Chapter10-source.zip` contains the MATLAB files, the two ROS packages, and their documentation. The migration included Python/XML syntax inspection and a static inventory of the package layout. The ROS controller was not compiled, launched, or tested on a vehicle in the review environment.
