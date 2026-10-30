# Cooperative Trajectory Planning for Intelligent Connected Vehicles

MATLAB companion examples for the book. Each chapter is self-contained and has its own README.

- [Chapter2](Chapter2): vehicle geometry, bicycle-model propagation, collision avoidance formulations, rounded obstacle inflation, and safe travel corridors.
- [Chapter4](Chapter4): ten-ICV two-stage cooperative lane changes, three validated cases, and the complete color/vector plotting pipeline (MATLAB + CasADi).
- [Chapter3](Chapter3): gradient-descent initial guesses, four/eight-connected A*, space-time A*, and conflict-based search.

Open the desired chapter folder in MATLAB and run `run_examples`. The examples were executed with MATLAB R2021b. No third-party solver or additional toolbox is required for these two chapters. Generated results are written to the chapter's `output` directory.

These examples distinguish geometric checks, discretized search models, and continuous vehicle models. Consult the chapter README before interpreting a result.

For Chapter4, follow its README: configure CasADi, run `run_experiments`, then `make_figures`.
