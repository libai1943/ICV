#include <algorithm>
#include <cmath>
#include <cfloat>
#include <sstream>
#include <vector>
#include <ros/ros.h>
#include <sandbox_msgs/AprilObject.h>       
#include <sandbox_msgs/ChassisCommand.h>    
#include <sandbox_msgs/Trajectory.h>       
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

const double lfw_ = 0.0;             
const double lrv_ = 0.0;            
const double wheel_base_ = 0.143;    
const double center_to_rear_ = 0.0825;
const double approaching_tolerance_ = 0.05;   

using namespace sandbox_msgs;

class PurePursuitVisualization {
   public:
    explicit PurePursuitVisualization(const ros::NodeHandle &nh) : node_handle_(nh) {
        marker_publisher_ = node_handle_.advertise<visualization_msgs::Marker>("/pure_pursuit", 1, true);
    }
    void set_target(int target) {
        target_ = target;
        init_marker();
    }
    void visualize_goal(double goal_x, double goal_y, double goal_yaw) {
        goal_.header.stamp = ros::Time::now();
        goal_.pose.position.x = goal_x;
        goal_.pose.position.y = goal_y;
        goal_.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);

        marker_publisher_.publish(goal_);
    }
    void visualize(double pos_x, double pos_y) {
        points_.header.stamp = ros::Time::now();
        line_strip_.header.stamp = ros::Time::now();
        points_.points.clear();

        points_.points.push_back(make_point(pos_x, pos_y));
        line_strip_.points.push_back(make_point(pos_x, pos_y));
        marker_publisher_.publish(line_strip_);
    }
    void visualize_pose(double x, double y, double theta) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "world";
        marker.ns = "Truth";
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.id = 1;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.scale.x = 0.05;
        marker.scale.y = 0.01;
        marker.scale.z = 0.03;
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker_publisher_.publish(marker);
    }
    visualization_msgs::Marker line_strip_;

   private:
    static inline geometry_msgs::Point make_point(double x, double y, double z = 0.0) {
        geometry_msgs::Point pt;
        pt.x = x;
        pt.y = y;
        pt.z = z;
        return pt;
    }
    void init_marker() {
        points_.header.frame_id = line_strip_.header.frame_id = goal_.header.frame_id = "world";
        std::stringstream ss;
        ss << "Markers_" << target_;
        points_.ns = line_strip_.ns = goal_.ns = ss.str();
        points_.action = line_strip_.action = goal_.action = visualization_msgs::Marker::ADD;
        points_.pose.orientation.w = line_strip_.pose.orientation.w = goal_.pose.orientation.w = 1.0;
        points_.id = 0;
        line_strip_.id = 1;
        goal_.id = 2;
        points_.type = visualization_msgs::Marker::POINTS;
        line_strip_.type = visualization_msgs::Marker::LINE_STRIP;
        goal_.type = visualization_msgs::Marker::CYLINDER;
        points_.scale.x = 0.02;
        points_.scale.y = 0.02;

        line_strip_.scale.x = 0.015;

        goal_.scale.x = approaching_tolerance_;
        goal_.scale.y = approaching_tolerance_;
        goal_.scale.z = 0.1;
        points_.color.g = 1.0f;
        points_.color.a = 1.0;

        line_strip_.color.r = 0.5;
        line_strip_.color.g = 0.5;
        line_strip_.color.b = 0.5;
        line_strip_.color.a = 1.0;

        goal_.color.r = 1.0;
        goal_.color.g = 1.0;
        goal_.color.b = 0.0;
        goal_.color.a = 0.5;
    }

    int target_ = 0;
    ros::NodeHandle node_handle_;
    ros::Publisher marker_publisher_;
    visualization_msgs::Marker points_, goal_;
};
TrajectoryPoint Interpolate(const TrajectoryPoint p0, TrajectoryPoint p1, double weight) {
    TrajectoryPoint pt;
    pt.time = (1 - weight) * p0.time + weight * p1.time;
    pt.x = (1 - weight) * p0.x + weight * p1.x;
    pt.y = (1 - weight) * p0.y + weight * p1.y;
    const double delta_yaw = std::atan2(std::sin(p1.yaw - p0.yaw),
                                         std::cos(p1.yaw - p0.yaw));
    pt.yaw = p0.yaw + weight * delta_yaw;
    pt.velocity = (1 - weight) * p0.velocity + weight * p1.velocity;
    pt.acceleration = (1 - weight) * p0.acceleration + weight * p1.acceleration;
    return pt;
}
TrajectoryPoint InterpolateWithTime(const TrajectoryPoint p0, TrajectoryPoint p1, double time) {
    double weight = (time - p0.time) / (p1.time - p0.time);
    return Interpolate(p0, p1, weight);
}
class PurePursuit {
   public:
    PurePursuit(const ros::NodeHandle &nh) : node_handle_(nh), visualization_(nh) {
        tracking_object_ = node_handle_.param<int>("target", 0);
        visualization_.set_target(tracking_object_);
        traj_subscriber_ =
            node_handle_.subscribe<Trajectory>("/traj", 10, &PurePursuit::TrajCallback, this, ros::TransportHints().tcpNoDelay());
        object_subscriber_ =
            node_handle_.subscribe<AprilObject>("/object", 10, &PurePursuit::ObjectCallback, this, ros::TransportHints().tcpNoDelay());
        command_publisher_ = node_handle_.advertise<ChassisCommand>("/chassis", 1, true);
        reached_publisher_ = node_handle_.advertise<std_msgs::Int32>("/reached", 1, false);
        timer_ = node_handle_.createTimer(ros::Duration(0.05), &PurePursuit::ControlCallback, this);
    }

   private:
    ros::NodeHandle node_handle_;                             
    ros::Publisher command_publisher_, reached_publisher_;    
    ros::Subscriber traj_subscriber_, object_subscriber_, truth_subscriber_,
        execute_subscriber_;                    
    ros::Timer timer_;                          
    double start_time_ = 0.0;                   
    PurePursuitVisualization visualization_;     
    Trajectory trajectory_;                      
    AprilObject object_;                         
    std::vector<std::pair<int, int>> ranges_;    

    int current_range_ = 0;      
    int tracking_object_ = 0;   

    double lookahead_distance_ = 0.18, lookahead_gain_ = 1.0;    
    double longitude_kp_ = 1.8, longitude_ki_ = 0.0, longitude_output_ = 0.0, longitude_perror_ = 0.0;   
    int car_index_ = 0, lookahead_index_ = 0;    

    bool pose_initialized_ = false;
    bool approached_ = false;          
    bool final_approached_ = false;    

    double lookahead_distance_min_ = 0.08;
    double lookahead_distance_max_ = 0.16;
    double lookahead_distance_ratio_ = 0.5;

    double v_lookahead_lb_ = 0.1;
    double v_lookahead_ub_ = 0.2;

    double v_min_ = 0.0;
    double v_max_ = 0.25;

    double max_steering_ = 0.59559375;
    double min_steering_ = -0.59559375;
    void ObjectCallback(const AprilObjectConstPtr& msg) {
        if (msg->id != tracking_object_ ||
            msg->type != AprilObject::VEHICLE) return;
        AprilObject candidate = *msg;
        candidate.x *= 0.001;
        candidate.y *= 0.001;
        if (!std::isfinite(candidate.x) ||
            !std::isfinite(candidate.y) ||
            !std::isfinite(candidate.yaw)) return;
        if (pose_initialized_) {
            double jump = std::hypot(candidate.x - object_.x,
                                     candidate.y - object_.y);
            if (jump > 0.5) return;
        }
        object_ = candidate;
        pose_initialized_ = true;
    }
    void TrajCallback(const TrajectoryConstPtr& msg) {
        if (msg->target != tracking_object_ || msg->points.size() < 2) return;
        for (std::size_t i = 0; i < msg->points.size(); ++i) {
            const auto& p = msg->points[i];
            if (!std::isfinite(p.time) || !std::isfinite(p.x) ||
                !std::isfinite(p.y) || !std::isfinite(p.yaw) ||
                !std::isfinite(p.velocity) || !std::isfinite(p.acceleration) ||
                p.time < 0.0) return;
            if (i > 0 && p.time <= msg->points[i - 1].time) return;
        }
        std::vector<std::pair<int, int>> new_ranges;
        int start = 0;
        for (std::size_t i = 1; i < msg->points.size(); ++i) {
            if (std::copysign(1.0, msg->points[i].velocity) !=
                std::copysign(1.0, msg->points[i - 1].velocity)) {
                new_ranges.emplace_back(start, static_cast<int>(i));
                start = static_cast<int>(i);
            }
        }
        new_ranges.emplace_back(start, static_cast<int>(msg->points.size()));
        trajectory_ = *msg;
        ranges_ = std::move(new_ranges);
        current_range_ = 0;
        car_index_ = lookahead_index_ = ranges_.front().first;
        start_time_ = ros::Time::now().toSec();
        longitude_output_ = longitude_perror_ = 0.0;
        lookahead_gain_ = 1.0;
        approached_ = final_approached_ = false;
        visualization_.line_strip_.points.clear();
    }

    bool ValidRange() const {
        if (current_range_ < 0 ||
            static_cast<std::size_t>(current_range_) >= ranges_.size()) return false;
        const auto range = ranges_[current_range_];
        return range.first >= 0 && range.first < range.second &&
            static_cast<std::size_t>(range.second) <= trajectory_.points.size();
    }

    void PublishNeutralCommand() {
        ChassisCommand command;
        command.target = tracking_object_;
        command.throttle = 0.0;
        command.steering = 0.0;
        command_publisher_.publish(command);
    }

    inline double DistanceToTraj(int i, double x, double y) { return hypot(trajectory_.points[i].y - y, trajectory_.points[i].x - x); }

    void UpdateLookahead() {
        if (!ValidRange()) return;    

        bool found_lookahead = false;   
        int start = ranges_[current_range_].first, end = ranges_[current_range_].second;
        car_index_ = std::max(start, std::min(car_index_, end - 1));
        lookahead_index_ = std::max(start, std::min(lookahead_index_, end - 1));

        double min_distance = FLT_MAX;   
        for (int i = car_index_; i < end; i++) {
            double distance = DistanceToTraj(i, object_.x, object_.y);    
            if (distance < min_distance) {
                car_index_ = i;
                min_distance = distance;
            }
        }
        double v_cmd = fabs(trajectory_.points[car_index_].velocity);   
        if (v_cmd < 0.1) {
            lookahead_distance_ = 0.08;    
        } else if (v_cmd > 0.2) {
            lookahead_distance_ = 0.16;   
        } else {
            lookahead_distance_ = v_cmd * 0.6;   
        }
        lookahead_distance_ *= lookahead_gain_;    
        for (int i = std::max(car_index_, lookahead_index_); i < end; i++) {
            double distance = DistanceToTraj(i, object_.x, object_.y);

            if (distance >= lookahead_distance_) {
                lookahead_index_ = i;
                found_lookahead = true;
                break;
            }
        }
        auto goal = trajectory_.points[end - 1];
        if (!found_lookahead) {
            lookahead_index_ = end - 1;
        }
        visualization_.visualize(object_.x + center_to_rear_ * cos(object_.yaw), object_.y + center_to_rear_ * sin(object_.yaw));
    }

    void SwitchRange() {
        if (!ValidRange()) { approached_ = true; return; }
        int end = ranges_[current_range_].second;
        double approaching_distance = DistanceToTraj(end - 1, object_.x, object_.y);
        double tolerance = current_range_ == ranges_.size() - 1 ? 0.05 : approaching_tolerance_;
        if (approaching_distance <= tolerance) {
            if (current_range_ == ranges_.size() - 1) {
                approached_ = true;
                final_approached_ = true;
                std_msgs::Int32 msg;
                msg.data = tracking_object_;
                reached_publisher_.publish(msg);
                longitude_output_ = longitude_perror_ = 0.0;
                std::cout << tracking_object_ << " - Goal reached" << std::endl;
            } else {
                current_range_++;
                car_index_ = lookahead_index_ = ranges_[current_range_].first;
                end = ranges_[current_range_].second;
                auto goal = trajectory_.points[end - 1];
                double traj_length = DistanceToTraj(ranges_[current_range_].first, goal.x, goal.y);
                lookahead_gain_ = std::max(0.1, std::min(traj_length / lookahead_distance_, 1.0));
                std::cout << tracking_object_ << " - Approached, changing goal to: (" << goal.x << ", " << goal.y << ')' << std::endl;
            }
        }
    }

    double LongitudeController() {
        double time = ros::Time::now().toSec() - start_time_;
        TrajectoryPoint truth = trajectory_.points.back();
        auto upper_point = std::lower_bound(trajectory_.points.begin(), trajectory_.points.end(), time,
                                            [](const TrajectoryPoint &tp, double t) { return tp.time < t; });
        if (upper_point != trajectory_.points.end()) {
            truth = *upper_point;
            if (upper_point > trajectory_.points.begin()) {
                truth = InterpolateWithTime(*std::prev(upper_point), *upper_point, time);
            }
        }
        double sinx = sin(truth.yaw), cosx = cos(truth.yaw);
        double relative_x = cosx * (truth.x - object_.x) + sinx * (truth.y - object_.y);
        // Explicit forward Euler: integrate the previous sample error.
        longitude_output_ += longitude_kp_ * (relative_x - longitude_perror_) + longitude_ki_ * longitude_perror_;
        longitude_perror_ = relative_x;
        longitude_output_ = std::min(std::max(longitude_output_, -0.25), 0.25);
        return longitude_output_;
    }

    void Controller() {
        if (!ValidRange() || lookahead_index_ < ranges_[current_range_].first ||
            lookahead_index_ >= ranges_[current_range_].second) {
            PublishNeutralCommand();
            return;
        }
        TrajectoryPoint &lookahead = trajectory_.points[lookahead_index_];
        auto lookahead_distance = DistanceToTraj(lookahead_index_, object_.x, object_.y);
        if (lookahead_distance <= 1e-9) { PublishNeutralCommand(); return; }
        double sinx = sin(object_.yaw), cosx = cos(object_.yaw);
        double relative_x = cosx * (lookahead.x - object_.x) + sinx * (lookahead.y - object_.y);
        double relative_y = -sinx * (lookahead.x - object_.x) + cosx * (lookahead.y - object_.y);
        double alpha = atan2(relative_y, relative_x);

        double delta = atan2(wheel_base_ * sin(alpha), lookahead_distance / 2 + (lookahead.velocity > 0 ? lfw_ : lrv_) * cos(alpha));
        double velocity = LongitudeController();

        ChassisCommand command;
        command.target = tracking_object_;
        command.throttle = velocity;
        command.steering = delta;

        command_publisher_.publish(command);
    }

    void ControlCallback(const ros::TimerEvent&) {
        if (!pose_initialized_ || !ValidRange() || approached_) {
            PublishNeutralCommand();
            return;
        }
        SwitchRange();
        if (approached_ || !ValidRange()) {
            PublishNeutralCommand();
            return;
        }
        UpdateLookahead();
        Controller();
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh("~");

    PurePursuit pp(nh);
    ros::spin();

    return 0;
}
