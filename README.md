# Cooperative Trajectory Planning for Intelligent Connected Vehicles

Companion to the book by Bai Li, Zhuyan Yin, Yongxiao Tian, Shengjie Lou, and Youmin Zhang.

All vehicle-motion discretizations use **first-order explicit forward Euler**. Every gradient-based nonlinear optimization uses **IPOPT with MA57**. Geometric interpolation used for drawing and collision checking is identified separately. Solver selection is explicit; no fallback is permitted.

| Folder | Content | Main entry point |
|---|---|---|
| Chapter1 | Introductory traffic illustration and snapshot generation | `run_intro_intersection` |
| Chapter2 | Vehicle geometry, collision avoidance, safe travel corridors | `run_examples` |
| Chapter3 | IPOPT initial-guess example, A*, space-time A*, CBS | `run_examples` |
| Chapter4 | Two-stage ten-vehicle cooperative lane changes | `run_experiments` |
| Chapter5 | Irregular five-arm intersections, STCs/RSTCs, SFM, LIOM | `run_experiments` |
| Chapter6 | ASCO and three comparisons on eight ten-vehicle cases | `run_experiments` |
| Chapter7 | Fifty-vehicle receding-horizon formation reconfiguration | `run_experiment(50)` |
| Chapter8 | Linux installation tutorial and CasADi C++ example | See chapter README |
| Chapter10 | Explicit-Euler PID integral and equivalence test | `self_test` |

The recorded MATLAB environment is R2021b, CasADi 3.7.2, IPOPT 3.14.11, and a compatible HSL library containing MA57. Chapter READMEs explain configuration and validation. Geometry and graph search do not call the optimizer. Licensed HSL binaries and font files are not included.

Fresh outputs are written within each chapter's `output` directory; `reports` preserves the recorded runs, including failures. Vehicle dimensions, SI-unit field names, Times New Roman labels, color palette, and triangular forward markers follow common conventions. These scripts do not install an operating system or deploy ROS.
