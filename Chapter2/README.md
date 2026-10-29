# Chapter 2: Problem Formulation

## Run

Tested with MATLAB R2021b, using base MATLAB only:

```matlab
cd Chapter2
report = run_examples;
```

The command creates `output`, runs geometric and mathematical checks, integrates a jerk-extended bicycle model, and exports vector PDF figures with PNG previews. Figure labels use Times New Roman; install this font for the same appearance. The font is not bundled.

## Files and mathematical scope

- `rectangle_vertices`: physical rectangular footprint from rear-axle pose `[x;y;theta]` and geometry `LW, LF, LR, LB`.
- `sat_gap`: signed projection gap over all polygon edge normals. Positive means separated; zero permits contact; negative means overlapping interiors. A positive projection gap is a lower bound, not generally the exact Euclidean distance.
- `polygon_distance`: complete convex-polygon intersection test followed by bidirectional vertex-to-edge distances. Inputs must be convex, cyclically ordered, nondegenerate polygons.
- `area_residual`: triangle-area sum minus polygon area. This is a point-membership test. Bidirectional vertex tests alone do not rule out edge crossings.
- `rounded_offset`: vector drawing boundary of a convex polygon plus a disc. Corner arcs are sampled only for plotting. They are not used to decide clearance.
- `grow_corridor`: cyclic axis-aligned box growth. Every candidate box is checked against each original polygon with exact polygon distance and the specified disc-inflation radius. The algorithm is order-dependent and does not claim a maximum-area box.
- `collision_examples`: independently checks the crossing-rectangle counterexample, an exact distance, an OBCA dual witness, a separating-line witness, and the conservative half-space inequality.
- `vehicle_model_demo`: integrates state `[x;y;theta;v;phi;a]` with inputs `[JERK;omega]`; writes `bicycle_trajectory.csv`. No states are clipped.
- `draw_figures`: rebuilds the dimensional diagrams, disc covers, triangle-area panels, rounded STC construction, corridor sequence, and separating-line diagram.

Lengths are in meters, time in seconds, and angles in radians. The vehicle reference is always the rear-axle center. Geometric length/radius labels are upright; running indices and state symbols are italic. The three-disc cover divides the physical body into three longitudinal rectangles and circumscribes each; it covers every corner.

## Checks and outputs

`output/validation.json` records the results. The polygon example has distance 3 m, and both its separating-line and OBCA certificates attain 3 m. A crossing pair has every vertex outside the other rectangle but still overlaps. The half-space identity is checked at 2,000 points. The bicycle demo checks the integrated steering and acceleration against their analytic expressions.

The figures are explanatory geometry, not a trajectory-planner benchmark. Corridor checks are spatial. Time-node constraints alone are not continuous-time collision guarantees. The OBCA example verifies a certificate for fixed polygons; it does not claim to solve a complete OBCA trajectory NLP.

## References

X. Zhang, A. Liniger, and F. Borrelli, *Optimization-Based Collision Avoidance*, IEEE TCST, 29(3), 972–983. Author manuscript: https://arxiv.org/abs/1711.03449

S. Boyd and L. Vandenberghe, *Convex Optimization*, Cambridge University Press, 2004, Chapter 8. https://web.stanford.edu/~boyd/cvxbook/

S. Gottschalk, M. C. Lin, and D. Manocha, *OBBTree: A Hierarchical Structure for Rapid Interference Detection*, SIGGRAPH, 1996.
