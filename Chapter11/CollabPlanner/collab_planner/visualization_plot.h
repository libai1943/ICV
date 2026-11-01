#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <mutex>

#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
#include "common/util/color.h"
#include "common/vehicle_param.h"

using common::math::Polygon2d;
using common::math::Vec2d;
using common::util::Color;

namespace VisualizationPlot {
using Vector = std::vector<double>;

void Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic);

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color(1, 1, 1), int id = -1,
          const std::string &ns = "");

void Plot(const Vector &xs, const Vector &ys, double width = 0.1, const std::vector<Color> &color = {}, int id = -1,
          const std::string &ns = "");

void PlotPolygon(const Vector &xs, const Vector &ys, double width = 0.1, Color color = Color::White, int id = -1,
                 const std::string &ns = "");

void PlotPolygon(const Polygon2d &polygon, double width = 0.1, Color color = Color::White, int id = -1,
                 const std::string &ns = "");

void PlotPath(const Vector &xs, const Vector &ys, const Vector &thetas, double bias = 120.0, double width = 0.1,
              int id = -1, const std::string &ns = "");

void PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, const Color &color,
                    double max_velocity = 10.0, double width = 0.1, int id = -1, const std::string &ns = "");

void PlotPoints(const double &xs, const double &ys, int id = -1, const std::string &ns = "",
                const Color &color = Color::White, double width = 0.05);

void Trigger();

void Clear();
}    // namespace VisualizationPlot
