# Cooperative Trajectory Planning for Intelligent Connected Vehicles

[Download ZIP](https://github.com/libai1943/ICV/archive/refs/heads/main.zip) · [Getting started](#start-with-one-vehicle-then-plan-for-ten) · [Guide](#outputs-and-verification) · [Citation](#cite-the-book-and-the-software)

The companion code follows an intelligent connected vehicle from a single bicycle model to a fleet negotiating shared road space. The eleven chapters cover geometry, cooperative trajectory optimization, tracking and the ROS and onboard programs used to put a plan into motion.

The planning models use **first-order explicit forward Euler** for vehicle-motion discretization. Gradient-based nonlinear optimization explicitly selects **IPOPT with MA57**. The chapter descriptions distinguish trajectory interpolation, optimization, and the discrete conventions used by the tracking controllers.

| Folder | Content | Main entry point |
|---|---|---|
| [Chapter1](Chapter1) | Introductory traffic illustration and snapshot generation | `run_intro_intersection` |
| [Chapter2](Chapter2) | Vehicle geometry, collision avoidance, safe travel corridors | `run_examples` |
| [Chapter3](Chapter3) | IPOPT initial-guess example, A*, space-time A*, CBS | `run_examples` |
| [Chapter4](Chapter4) | Two-stage ten-vehicle cooperative lane changes | `run_experiments` |
| [Chapter5](Chapter5) | Irregular five-arm intersections, STCs/RSTCs, SFM, LIOM | `run_experiments` |
| [Chapter6](Chapter6) | One validated ten-vehicle ASCO maneuver through a shared conflict region | `run_example` |
| [Chapter7](Chapter7) | Fifty-vehicle receding-horizon formation reconfiguration | `run_experiment(50)` |
| [Chapter8](Chapter8) | Numerical-stack example and complete simulation ROS package | [Chapter README](Chapter8/README.md) |
| [Chapter9](Chapter9) | Vehicle firmware and supplied SSD1306 library | `Firmware/main/main.ino` |
| [Chapter10](Chapter10) | MATLAB PID convention example and tracking/message ROS packages | `self_test`; [ROS instructions](Chapter10/README.md) |
| [Chapter11](Chapter11) | Complete experimental ROS package tree and host Tinyproto snapshot | [Chapter README](Chapter11/README.md) |

## Start with one vehicle, then plan for ten

For a first look at the numerical model, run the small Chapter 2 experiment from the repository root in MATLAB. It integrates one six-state bicycle over eight seconds, compares the errors at two time steps, and writes a trajectory table and a figure. This individual function uses MATLAB directly.

```matlab
cd Chapter2
out = fullfile(pwd,'output');
if ~isfolder(out), mkdir(out); end
report = vehicle_model_demo(out);
disp(report.refinement_error_ratio)
```

Read `output/bicycle_trajectory.csv` to compare the commanded jerk and steering rate with the resulting acceleration and steering angle. The function checks that halving the time step roughly halves the acceleration error, as expected for forward Euler.

Next, try Chapter 6 once the MATLAB solver stack below is configured. Ten vehicles must negotiate six obstacles and one another in a shared conflict region. Follow its `self_test` and `run_example` sequence to inspect the input case, selected collision constraints and final validation. [Chapter 6 setup and example](Chapter6/README.md)

Readers working on ROS or firmware can start at Chapter 8 for simulation, Chapter 9 for onboard code and Chapters 10–11 for tracking and integration. For illustrations from the recorded experiments, open the vector figures distributed with the book. [Chapter 1 illustrations](Chapter1/reports/figures)

## Environments and source packages

The recorded MATLAB environment is R2021b, CasADi 3.7.2, IPOPT 3.14.11, and a compatible HSL library containing MA57. The Linux/ROS baseline and its numerical-library versions are documented in Chapter 8. HSL binaries and font files require separate installation.

Chapters 8–11 contain their source locally, together with provenance, source attribution, and English instructions. Each of these chapters also provides a `ChapterN-source.zip` archive. Extracting an archive produces its `ChapterN` directory. The Chapter 11 archive includes the binary `third_party/tinyproto.zip` dependency archive.

Clone the book repository **outside a catkin `src` directory**. Each chapter README shows how to link only its required packages into a separate workspace. Chapter 8 and Chapter 11 contain different snapshots of `collab_planner`; Chapter 10 and Chapter 11 both contain `pure_pursuit` and `sandbox_msgs`. A workspace uses one copy of each package name.

```bash
mkdir -p "$HOME/src"
git clone https://github.com/libai1943/ICV.git "$HOME/src/ICV"
export ICV_BOOK_ROOT="$HOME/src/ICV"
git -C "$ICV_BOOK_ROOT" rev-parse HEAD
```

For an existing checkout, set `ICV_BOOK_ROOT` to its absolute path. Record the printed commit alongside experiment results. Use a fresh terminal for each ROS workspace and source only the workspace selected for that task.

## Outputs and verification

The MATLAB chapters write new outputs to their chapter-local `output` directories; `reports` preserves the recorded demonstrations. The ROS planner writes relative CSV log filenames in its process working directory. Chapter READMEs identify package dependencies, launch commands, and controller conventions.

Core localization, tracking, target-assignment, and communication lifecycle tests are provided in `tests`. They exercise actual source functions using portable interface fixtures. Build and commission the ROS/firmware system on the documented platform before operating vehicles. Third-party authorship and license notices remain with the corresponding files.

## Cite the book and the software

Bai Li, Zhuyan Yin, Yongxiao Tian, Shengjie Lou and Youmin Zhang. *Cooperative Trajectory Planning for Intelligent Connected Vehicles*. For a reproducible software reference, include the chapter, entry point and commit from your checkout. [Companion repository](https://github.com/libai1943/ICV)
