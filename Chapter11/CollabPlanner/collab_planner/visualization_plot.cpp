//
// Created by yenkn on 4/13/21.
//

#include "visualization_plot.h"

#include <common/math/math_utils.h>

#include <cmath>

namespace VisualizationPlot {
namespace {
std::string frame_ = "map";
std::mutex mutex_;

ros::Publisher publisher_;
visualization_msgs::MarkerArray arr_;
}    // namespace
}    // namespace VisualizationPlot

void VisualizationPlot::Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic) {
    frame_ = frame;
    publisher_ = node.advertise<visualization_msgs::MarkerArray>(topic, 1000, true);
}

void VisualizationPlot::Plot(const Vector &xs, const Vector &ys, double width, Color color, int id,
                             const std::string &ns) {
    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_;
    msg.header.stamp = ros::Time();
    msg.ns = ns;
    msg.id = id >= 0 ? id : arr_.markers.size();

    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = width;
    msg.color = color.toColorRGBA();

    for (size_t i = 0; i < xs.size(); i++) {
        geometry_msgs::Point pt;
        pt.x = xs[i];
        pt.y = ys[i];
        pt.z = 0.1 * id;
        msg.points.push_back(pt);
    }

    mutex_.lock();
    arr_.markers.push_back(msg);
    mutex_.unlock();
}

void VisualizationPlot::Plot(const VisualizationPlot::Vector &xs, const VisualizationPlot::Vector &ys, double width,
                             const std::vector<Color> &color, int id, const std::string &ns) {
    assert(xs.size() == color.size());

    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_;
    msg.header.stamp = ros::Time();
    msg.ns = ns;
    msg.id = id >= 0 ? id : arr_.markers.size();

    msg.action = visualization_msgs::Marker::ADD;
    msg.type = visualization_msgs::Marker::LINE_STRIP;
    msg.pose.orientation.w = 1.0;
    msg.scale.x = width;

    for (size_t i = 0; i < xs.size(); i++) {
        geometry_msgs::Point pt;
        pt.x = xs[i];
        pt.y = ys[i];
        msg.points.push_back(pt);
        msg.colors.push_back(color[i].toColorRGBA());
    }

    mutex_.lock();
    arr_.markers.push_back(msg);
    mutex_.unlock();
}

void VisualizationPlot::PlotPolygon(const Vector &xs, const Vector &ys, double width, Color color, int id,
                                    const std::string &ns) {
    auto xxs = xs;
    auto yys = ys;
    xxs.push_back(xxs[0]);
    yys.push_back(yys[0]);
    Plot(xxs, yys, width, color, id, ns);
}

void VisualizationPlot::PlotPolygon(const Polygon2d &polygon, double width, Color color, int id,
                                    const std::string &ns) {
    std::vector<double> xs, ys;
    for (auto &pt : polygon.points()) {
        xs.push_back(pt.x());
        ys.push_back(pt.y());
    }
    PlotPolygon(xs, ys, width, color, id, ns);
}

void VisualizationPlot::PlotPath(const Vector &xs, const Vector &ys, const Vector &thetas, double bias, double width,
                                 int id, const std::string &ns) {
    std::vector<Color> colors(xs.size());
    auto gears = common::math::GetPathGears(xs, ys, thetas);

    for (size_t i = 0; i < xs.size() - 1; i++) {
        colors[i] = Color::fromHSV(2 * bias - (gears[i] ? 1.0 : -1.0) * bias, 1.0, 1.0);
        colors[i].set_a(std::min(1.0, width / 0.02));
    }
    colors[colors.size() - 1] = colors.back();

    Plot(xs, ys, width, colors, id, ns);
}

void VisualizationPlot::PlotTrajectory(const Vector &xs, const Vector &ys, const Vector &vs, const Color &color,
                                       double max_velocity, double width, int id, const std::string &ns) {
    Plot(xs, ys, width, color, id, ns);
}

void VisualizationPlot::PlotPoints(const double &xs, const double &ys, int id, const std::string &ns,
                                   const Color &color, double width) {
    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_;
    msg.header.stamp = ros::Time();
    msg.ns = ns.empty() ? "Points" : ns;
    msg.id = id >= 0 ? id : arr_.markers.size();

    msg.action = visualization_msgs::Marker::ADD;

    msg.type = visualization_msgs::Marker::CYLINDER;

    msg.pose.position.x = xs;
    msg.pose.position.y = ys;
    msg.pose.position.z = 0;
    msg.pose.orientation.w = 1.0;

    msg.scale.x = width;
    msg.scale.y = width;
    msg.scale.z = 0.01;

    msg.color = color.toColorRGBA();

    mutex_.lock();
    arr_.markers.push_back(msg);
    mutex_.unlock();
}

void VisualizationPlot::Trigger() {
    mutex_.lock();
    publisher_.publish(arr_);
    arr_.markers.clear();
    ros::spinOnce();
    mutex_.unlock();
}

void VisualizationPlot::Clear() {
    mutex_.lock();
    arr_.markers.clear();

    visualization_msgs::MarkerArray arr;
    visualization_msgs::Marker msg;
    msg.header.frame_id = frame_;
    msg.ns = "Markers";

    msg.action = visualization_msgs::Marker::DELETEALL;
    arr.markers.push_back(msg);
    publisher_.publish(arr);
    mutex_.unlock();
}
