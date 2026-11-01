//
//

#pragma once

#include <tuple>

#include "common/math/box2d.h"
#include "common/math/circle_2d.h"
#include "common/math/pose.h"

class VehicleParam {
   public:
    // Car center point is car reference point, i.e., center of rear axle.
    double back_edge_to_center = 0.032;
    // the distance between the front and back wheels
    double wheel_base = 0.143;
    double length = 0.229;
    double width = 0.191;
    double front_edge_to_center = length - back_edge_to_center;
    double center_to_rear = front_edge_to_center - length * 0.5;

    double max_acceleration = 0.05;
    double max_deceleration = -0.05;

    double max_velocity = 0.2;
    // set to zero is reverse is disabled
    double max_reverse_velocity = -0.2;

    // vehicle max steer angle
    double max_steer_angle = 0.3491;
    double max_backward_steer_angle = 0.3491;
    // vehicle max steer rate; how fast can the steering wheel turn.
    double max_steer_angle_rate = 0.10;

    // ratio between the turn of steering wheel and the turn of wheels
    double steer_ratio = 1;

    double radius = hypot(0.5 * length, 0.5 * width);

    template <class T>
    std::tuple<T, T> GetDiscPositions(const T &x, const T &y, const T &theta) const {
        auto xc = x + center_to_rear * cos(theta);
        auto yc = y + center_to_rear * sin(theta);
        return std::make_tuple(xc, yc);
    }

    common::math::Circle2d GetDiscs(double x, double y, double theta) const {
        double xc, yc;
        std::tie(xc, yc) = GetDiscPositions(x, y, theta);
        return common::math::Circle2d(xc, yc, radius);
    }

    common::math::Box2d GenerateBox(const common::math::Pose &pose) const {
        double distance = length / 2 - back_edge_to_center;
        return {pose.extend(distance), pose.theta(), length, width};
    }

    common::math::Box2d GenerateBoxBuffer(const common::math::Pose &pose) const {
        double distance = length / 2 - back_edge_to_center;
        return {pose.extend(distance), pose.theta(), length + 0.5, width + 0.5};
    }
};
