# Chapter 8: numerical-stack example and cooperative-planning software

This chapter contains the standalone CasADi program used in the book and the complete simulation ROS package.

| Path | Purpose |
|---|---|
| `casadi_example.cc` | Primary book example, including solution, cost, residual, and solver status |
| `CollabPlanner/` | Simulation package named `collab_planner` |
| `Chapter8-source.zip` | Chapter-local source and documentation archive |

The C++ example and the planner select IPOPT with MA57. The analytic solution of the example is `(x,y,z)=(0,1,0)`, with zero objective value and zero equality residual.

## Recorded environment

The chapter's Linux baseline is Ubuntu 20.04.6, ROS Noetic, ThirdParty-HSL 2.2.5, IPOPT 3.14.17, and CasADi 3.7.0. The book provides the installation walkthrough and platform-support context. The MATLAB experiments use the separately recorded environment described in the repository's main README.

The commands below assume ROS, compatible CasADi headers/shared libraries, IPOPT, and HSL with MA57 have been installed. HSL binaries are obtained and installed separately. When HSL uses a nondefault library filename, set the solver's `ipopt.hsllib` option to the installed library's absolute path.

## Obtain the central source

Clone the repository outside every catkin `src` directory:

```bash
mkdir -p "$HOME/src"
git clone https://github.com/libai1943/ICV.git "$HOME/src/ICV"
export ICV_BOOK_ROOT="$HOME/src/ICV"
git -C "$ICV_BOOK_ROOT" rev-parse HEAD
```

For an existing checkout, set `ICV_BOOK_ROOT` to that checkout's absolute path. Record its commit with the experiment.

## Build the primary book example

```bash
export ICV_BOOK_ROOT="$HOME/src/ICV"
mkdir -p "$ICV_BOOK_ROOT/Chapter8/build"
g++ -std=c++11 "$ICV_BOOK_ROOT/Chapter8/casadi_example.cc" \
  -I/usr/local/include -L/usr/local/lib -Wl,-rpath,/usr/local/lib \
  -lcasadi -o "$ICV_BOOK_ROOT/Chapter8/build/casadi_example"
"$ICV_BOOK_ROOT/Chapter8/build/casadi_example"
```

Confirm MA57 in the solver startup output, then inspect the termination status, solution, objective, and equality residual. The expected numerical solution approaches the analytic values given above within the selected tolerance.

## Build the simulation ROS workspace

Use a fresh terminal and a workspace containing a single `collab_planner` package. The `ln -sT` command reports an existing destination, so inspect an existing workspace before reusing it.

```bash
source /opt/ros/noetic/setup.bash
export ICV_BOOK_ROOT="$HOME/src/ICV"
mkdir -p "$HOME/collab_ws/src"
ln -sT "$ICV_BOOK_ROOT/Chapter8/CollabPlanner" \
  "$HOME/collab_ws/src/collab_planner"
sudo apt update
sudo apt install ros-noetic-jsk-rviz-plugins
cd "$HOME/collab_ws"
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
catkin_make -DWITH_HSL=ON
source devel/setup.bash
```

`CollabPlanner` is itself a ROS package. Chapter 11 has a different package tree and uses its own workspace. The MA57 setting is already present in `trajectory_nlp/collab_traj_nlp.cc`.

## Run the demonstration

Open three terminals. Source the same simulation workspace in each one.

Terminal 1:

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/collab_ws/devel/setup.bash"
roscore
```

Terminal 2:

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/collab_ws/devel/setup.bash"
rosrun collab_planner keyboard_input
```

Terminal 3:

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/collab_ws/devel/setup.bash"
roslaunch collab_planner test.launch
```

Follow the English prompts in the keyboard terminal to select the initial and target formations. Available characters are printed by the program. The planner's CSV log filenames are relative to its process working directory, usually the ROS runtime directory for a standard launch.

## Attribution

The CasADi example follows the three-variable demonstration supplied by Yueshuo Sun in `YueshuoSun/casadi_example` (source commit `6314f25f7608dd149cbce2780fc79dc0087023ed`). [CollabPlanner/README.md](CollabPlanner/README.md) records planner authorship and supporting contributions. Third-party notices remain in their files.
