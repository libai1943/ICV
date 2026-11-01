//
//

#pragma once

#include <geometry_msgs/Point.h>
#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <condition_variable>
#include <numeric>
#include <queue>
#include <thread>

#include "collab_traj_nlp.h"
#include "common/environment.h"
#include "common/math/pose.h"
#include "common/trajectory_point.h"
#include "nlohmann/json.hpp"
#include "trajectory_nlp_config.h"

using namespace common::math;
using json = nlohmann::json;

namespace common {
namespace math {

inline void to_json(json& j, const Vec2d& p) { j = json{p.x(), p.y()}; }
inline void from_json(const json& j, Vec2d& p) {
    p.set_x(j[0]);
    p.set_y(j[1]);
}

inline void to_json(json& j, const Pose& p) { j = json{p.x(), p.y(), p.theta()}; }
inline void from_json(const json& j, Pose& p) {
    p.set_x(j[0]);
    p.set_y(j[1]);
    p.set_theta(j[2]);
}

inline void to_json(json& j, const Polygon2d& p) { j = p.points(); }
inline void from_json(const json& j, Polygon2d& p) { p = Polygon2d(j.get<std::vector<Vec2d>>()); }

}    // namespace math
}    // namespace common

inline std::vector<common::math::Vec2d> PointsToVec(const std::vector<geometry_msgs::Point>& points) {
    std::vector<common::math::Vec2d> tmp(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        tmp[i].set_x(points[i].x);
        tmp[i].set_y(points[i].y);
    }
    return tmp;
}

namespace collab_traj_nlp {

class TrajectoryOptimizer {
   public:
    TrajectoryOptimizer(const TrajectoryNLPConfig& config);
    TrajectoryOptimizer(const TrajectoryNLPConfig& config, ros::Publisher* pub);
    TrajectoryOptimizer(const TrajectoryOptimizer&) = delete;
    TrajectoryOptimizer& operator=(const TrajectoryOptimizer&) = delete;

    void PlanTrajOnline(States& guess, States& result, const Poses& starts, const Poses& goals);

    const States& get_states() const { return accumulated_results_; }
    // Clear all vectors in the State structure
    void ClearState(State& state) {
        state.x.clear();
        state.y.clear();
        state.theta.clear();
        state.v.clear();
        state.phi.clear();
        state.a.clear();
        state.omega.clear();
        state.xc.clear();
        state.yc.clear();
    }

    // Clear accumulated_results
    void ClearAccumulatedResults(States& accumulated_results) {
        for (State& state : accumulated_results) {
            ClearState(state);
        }
    }
    States accumulated_results_;    // Accumulated result sequences

    void ReadConfig();

    /************************Multi-vehicle interface********************************/
    // Set the state sequence for one vehicle
    void set_vehicle_state(size_t vehicle_id, double x, double y, double theta, double v, double phi = 0, double a = 0,
                           double omega = 0) {
        if (vehicle_id < vehicles_states_.size()) {
            vehicles_states_[vehicle_id].x = x;
            vehicles_states_[vehicle_id].y = y;
            vehicles_states_[vehicle_id].theta = theta;
            vehicles_states_[vehicle_id].v = v;
            vehicles_states_[vehicle_id].phi = phi;
            vehicles_states_[vehicle_id].a = a;
            vehicles_states_[vehicle_id].omega = omega;
        } else {
            std::cerr << "Invalid vehicle_id: " << vehicle_id << std::endl;
        }
    }

    // Get the state sequence for one vehicle
    const TrajectoryPoint& get_vehicle_state(size_t vehicle_id) const {
        if (vehicle_id < vehicles_states_.size()) {
            return vehicles_states_[vehicle_id];
        } else {
            std::cerr << "Invalid vehicle_id: " << vehicle_id << std::endl;
            throw std::out_of_range("Invalid vehicle_id");
        }
    }

    // Set the initial poses
    void set_start_pose(size_t vehicle_id, double x, double y, double theta) {
        if (vehicle_id < start_poses_.size()) {
            start_poses_[vehicle_id] = common::math::Pose(x, y, theta);
        } else {
            std::cerr << "Invalid vehicle_id: " << vehicle_id << std::endl;
        }
    }

    // Get the initial poses
    const common::math::Pose& get_start_pose(size_t vehicle_id) const {
        if (vehicle_id < start_poses_.size()) {
            return start_poses_[vehicle_id];
        } else {
            std::cerr << "Invalid vehicle_id: " << vehicle_id << std::endl;
            throw std::out_of_range("Invalid vehicle_id");
        }
    }

    const Poses& get_start_pose() const { return start_poses_; }

    // Set the target poses
    void set_goal_pose(size_t vehicle_id, double x, double y, double theta = 0.0) {
        if (vehicle_id < goal_poses_.size()) {
            goal_poses_[vehicle_id] = common::math::Pose(x, y, theta);
        } else {
            std::cerr << "Invalid vehicle_id: " << vehicle_id << std::endl;
        }
    }

    // Get the target poses
    const common::math::Pose& get_goal_pose(size_t vehicle_id) const {
        if (vehicle_id < goal_poses_.size()) {
            return goal_poses_[vehicle_id];
        } else {
            std::cerr << "Invalid vehicle_id: " << vehicle_id << std::endl;
            throw std::out_of_range("Invalid vehicle_id");
        }
    }

    const Poses& get_goal_pose() const { return goal_poses_; }

    void set_all_vehicle_states(const std::array<TrajectoryPoint, kVechicleNum>& states) {
        for (size_t i = 0; i < states.size(); ++i) {
            vehicles_states_[i] = states[i];
        }
    }

    /************************Multi-vehicle interface********************************/
    bool GetCars(ros::NodeHandle& nh, const std::string& character, CharacterCars& cars);
    bool GetAllCars(ros::NodeHandle& nh, CharacterCarsMap& all_cars);

    bool last_send_ = false;

   private:
    std::string log_filename_;

    std::vector<double> history_log;
    int counter_ = 0;
    bool optimization_done_;

    States current_result_;
    int max_iteration_ = 50;

    int final_index_ = 0;
    std::array<TrajectoryPoint, kVechicleNum> vehicles_states_;
    Poses start_poses_;
    Poses goal_poses_;
    double total_time_ = 0.0;

    ros::Publisher* result_pub_;
    double start_time_ = 0.0;
    common::math::Pose current_pose_;
    TrajectoryNLPConfig nlp_config_;
    VehicleParam vehicle_param_;
    CollabTrajNLP collab_nlp_;
    std::array<Trajectory, kVechicleNum> vehicle_traj_;

    // parallel
    std::queue<States> result_queue_;
    std::mutex queue_mutex_;
    std::condition_variable queue_cv_;
    std::atomic<bool> optimization_is_done_{false};
    std::thread optimization_thread_;

    void GenerateInitialGuessForFirstTimeNlp(const Poses& starts, States& states);

    std::vector<std::tuple<int, int, int>> GenerateCollisionAvoidanceConstraints(const States& states);

    void GenerateInitialGuessForNlp2(States& states);

    void CollabOptimization(States& guess, const Poses& starts, const Poses& goals);

    void SendResults();

    void LogCurrentResult(double time);

    bool CheckVehicleCollisionAtEachPoint(const collab_traj_nlp::States& colli_states);
};

}    // namespace collab_traj_nlp
