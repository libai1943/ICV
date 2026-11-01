# Chapter 11: integrated cooperative-driving experiment

`CollabPlanner` contains the complete experimental source snapshot, now maintained inside the book repository. It is a container for six ROS packages:

| Package | Role |
|---|---|
| `collab_planner` | Cooperative planning and keyboard interface |
| `pure_pursuit` | Trajectory tracking |
| `sandbox_msgs` | Shared trajectory and control messages |
| `chassis` | Host-side vehicle communication |
| `nokov_localization` | Localization processing |
| `vrpn_client_ros` | Motion-capture stream interface |

The `third_party` directory contains the host Tinyproto dependency archive. [CollabPlanner/README.md](CollabPlanner/README.md) records the exact upstream commit, Yueshuo Sun's initial implementation and integration, preserved third-party contributions, and the import changes. `Chapter11-source.zip` includes the complete chapter tree and the binary Tinyproto archive.

## Central checkout and prerequisites

The following commands assume the Ubuntu/ROS and numerical-library setup documented in Chapter 8. Keep the ICV checkout outside the catkin workspace:

```bash
export ICV_BOOK_ROOT="$HOME/src/ICV"
git -C "$ICV_BOOK_ROOT" rev-parse HEAD
```

Record this central-repository commit with experimental results. The planner already selects MA57. Install a compatible HSL library providing MA57 as described in Chapter 8.

## Build the supplied host Tinyproto snapshot

The dependency is `CollabPlanner/third_party/tinyproto.zip`. Its SHA-256 is:

```text
81020d2ae4d151511bb528610420572cdcccddffcd2385b6df34a6a531982eae
```

Its version metadata differs between `library.properties` and `Makefile.common`; the hash identifies the supplied host snapshot unambiguously. The firmware baseline in Chapter 9 is recorded separately.

Use the archive's Makefile: the `chassis` package links `tinyprotocol`, and this build produces `libtinyprotocol.a` with the corresponding installation layout.

```bash
export ICV_BOOK_ROOT="$HOME/src/ICV"
sha256sum "$ICV_BOOK_ROOT/Chapter11/CollabPlanner/third_party/tinyproto.zip"
mkdir -p "$HOME/opt-src"
unzip "$ICV_BOOK_ROOT/Chapter11/CollabPlanner/third_party/tinyproto.zip" \
  -d "$HOME/opt-src"
cd "$HOME/opt-src/tinyproto-stable"
make -j4
sudo make install DESTDIR=/usr/local
sudo ldconfig
```

Use a clean extraction directory for this snapshot. The host and firmware must use the compatible framing mode and checksum settings described in the communication section of the book.

## Build the experimental workspace

Use a fresh terminal and a separate `exp_ws`. Link the six packages explicitly:

```bash
source /opt/ros/noetic/setup.bash
export ICV_BOOK_ROOT="$HOME/src/ICV"
mkdir -p "$HOME/exp_ws/src"
for icv_package in chassis collab_planner nokov_localization \
  pure_pursuit sandbox_msgs vrpn_client_ros; do
  ln -sT "$ICV_BOOK_ROOT/Chapter11/CollabPlanner/$icv_package" \
    "$HOME/exp_ws/src/$icv_package"
done
sudo apt update
sudo apt install python3-scipy python3-numpy \
  ros-noetic-jsk-rviz-plugins ros-noetic-jsk-recognition-msgs
cd "$HOME/exp_ws"
rosdep install --from-paths src --ignore-src -r -y --rosdistro noetic
catkin_make -DWITH_HSL=ON
source devel/setup.bash
```

The explicit dependency list includes NumPy and the JSK messages imported by the localization script. `rosdep` installs the declared package dependencies, including VRPN. Each package name appears once in this workspace. The Chapter 8 simulation workspace and Chapter 10 tracking workspace are used independently.

## Configure and run the experiment

Complete the physical setup, motion-capture calibration, vehicle identifiers, serial permissions, and controller calibration described in Chapters 9–11. Set `server` to the actual motion-capture server address; `10.1.1.198` is the recorded example address.

Terminal 1:

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/exp_ws/devel/setup.bash"
roslaunch collab_planner exp.launch server:=10.1.1.198
```

Terminal 2:

```bash
source /opt/ros/noetic/setup.bash
source "$HOME/exp_ws/devel/setup.bash"
rosrun collab_planner keyboard_input
```

Use the English keyboard prompts to select the formation task. The planner writes relative CSV log filenames in its process working directory. The chapter-local tracking node uses the explicit forward Euler integral convention explained in [Chapter10/README.md](../Chapter10/README.md).

## Verification scope

The migration checked source differences, Python syntax, XML syntax, the Tinyproto archive hash, and an isolated inventory containing exactly the six package names listed above. Small package-manifest corrections are documented in `CollabPlanner/README.md`. ROS compilation, communication, Arduino upload, and vehicle execution were not performed in the review environment. The tracking integral increment has subsequently been aligned with the book's explicit forward Euler convention. Other illustrative robustness refinements in the book are identified separately from the imported runtime implementation.
