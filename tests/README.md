# Companion-source regression tests

Requirements: Python 3 with NumPy and a C++14 compiler (GCC, Clang, or Zig).

```bash
python tests/test_runtime.py --compiler g++
python tests/test_chassis.py --compiler g++
```

The tests load the localization classes and extract the tracking/assignment/communication functions from the supplied source files. Interface fixtures provide deterministic messages, time, and serial events. Checks cover missing and rotated markers, valid zero coordinates, millimeter-to-meter conversion, rejected measurements, half-open range endpoints, maneuver replacement, neutral commands at completion, heading interpolation, all retained assignment candidates, fixed-size arrays, and receiver lifetime.

These are portable function tests. ROS deployment and vehicle commissioning follow the chapter instructions. Test outputs are written to a temporary directory or a directory selected with `--output`.
