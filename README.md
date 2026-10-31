# Cooperative Trajectory Planning for Intelligent Connected Vehicles

Companion to the book by Bai Li, Zhuyan Yin, Yongxiao Tian, Shengjie Lou, and Youmin Zhang.

MATLAB companion code. Each chapter contains its own README, numerical checks, and figure generator.

| Folder | Content | Main entry point |
|---|---|---|
| [Chapter2](Chapter2) | Vehicle geometry, collision avoidance, safe travel corridors | `run_examples` |
| [Chapter3](Chapter3) | A*, space-time A*, conflict-based search, initial guesses | `run_examples` |
| [Chapter4](Chapter4) | Two-stage ten-vehicle cooperative lane changes | `run_experiments` |
| [Chapter5](Chapter5) | Intersection STCs/RSTCs, SFM proposals, LIOM, initializer study | `run_experiments` |
| [Chapter6](Chapter6) | ASCO and three comparison methods on eight ten-vehicle tasks | `run_experiments` |

Chapters 2 and 3 use MATLAB without a third-party optimizer. Chapters 4–6 require the compatible CasADi MATLAB package described in their READMEs. Tested with MATLAB R2021b. All generated results stay in each chapter's `output` directory.

Vehicle geometry, variable naming, trajectory CSV fields, colors, Times New Roman labeling, and triangle heading glyphs follow a common convention. Geometry illustrations, actual numerical experiments, and failed runs are distinguished explicitly. No hardware or ROS deployment is performed by these scripts.
