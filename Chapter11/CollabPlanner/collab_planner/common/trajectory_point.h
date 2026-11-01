//
//

#pragma once

#include <vector>

#include "math/pose.h"

struct TrajectoryPoint {
    double x = 0.0, y = 0.0, theta = 0.0, v = 0.0, phi = 0.0, a = 0.0, omega = 0.0, kappa = 0.0, t = 0.0, xc = 0.0,
           yc = 0.0;
    TrajectoryPoint() = default;
    explicit TrajectoryPoint(const common::math::Pose &pose) : x(pose.x()), y(pose.y()), theta(pose.theta()) {}

    explicit TrajectoryPoint(const common::math::Vec2d &vec_2d) : x(vec_2d.x()), y(vec_2d.y()) {}

    explicit operator common::math::Pose() const { return {x, y, theta}; }

    explicit operator common::math::Vec2d() const { return {x, y}; }
};

using Trajectory = std::vector<TrajectoryPoint>;

using Trajectorys = std::array<Trajectory, 8>;
