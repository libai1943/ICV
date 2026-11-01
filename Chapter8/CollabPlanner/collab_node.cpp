//
//
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <queue>
#include <vector>
#include <stdexcept>

#include "collab_planner/KeyboardInput.h"
#include "common/backward.hpp"
#include "trajectory_nlp/collab_traj_optimizer.h"
#include "visualization_plot.h"

using namespace common::math;
using json = nlohmann::json;

namespace backward {
backward::SignalHandling sh;
}

class GlobalPlanner {
   public:
    GlobalPlanner() : nh_("~/collab_planner") {
        VisualizationPlot::Init(nh_, "world", "/collab_planner_markers");
        ReadConfig();

        keyboard_service_ = nh_.advertiseService("/keyboard_input", &GlobalPlanner::HandleKeyboardInput, this);
        planner_ = std::make_shared<collab_traj_nlp::TrajectoryOptimizer>(nlp_config_);
        goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &GlobalPlanner::GetGoalPoseCallback, this);

        VisualizationPlot::PlotPolygon(Polygon2d(Box2d(AABox2d(Vec2d(Environment::Instance().x_min, Environment::Instance().y_min),
                                                               Vec2d(Environment::Instance().x_max, Environment::Instance().y_max)))),
                                       0.006, Color::Black, 66, "Bounds");
        VisualizationPlot::Trigger();
    }

   private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_sub_;
    ros::ServiceServer keyboard_service_;
    std::shared_ptr<collab_traj_nlp::TrajectoryOptimizer> planner_;
    TrajectoryNLPConfig nlp_config_;

    Poses current_poses_;
    bool current_pose_received_ = false;

    bool start_receive_ = false;
    bool goal_receive_ = false;
    bool first_exeu_ = true;

    void ReadConfig() {
        nh_.param("back_edge_to_center", Environment::Instance().vehicle_param.back_edge_to_center, 0.032);
        nh_.param("wheel_base", Environment::Instance().vehicle_param.wheel_base, 0.143);
        nh_.param("length", Environment::Instance().vehicle_param.length, 0.229);
        nh_.param("width", Environment::Instance().vehicle_param.width, 0.191);
        nh_.param("max_acceleration", Environment::Instance().vehicle_param.max_acceleration, 0.05);
        nh_.param("max_deceleration", Environment::Instance().vehicle_param.max_deceleration, -0.05);
        nh_.param("max_velocity", Environment::Instance().vehicle_param.max_velocity, 0.2);
        nh_.param("max_reverse_velocity", Environment::Instance().vehicle_param.max_reverse_velocity, -0.2);
        nh_.param("max_steer_angle", Environment::Instance().vehicle_param.max_steer_angle, 0.3491);
        nh_.param("max_backward_steer_angle", Environment::Instance().vehicle_param.max_backward_steer_angle, 0.3491);
        nh_.param("max_steer_angle_rate", Environment::Instance().vehicle_param.max_steer_angle_rate, 0.1);
        nh_.param("steer_ratio", Environment::Instance().vehicle_param.steer_ratio, 1.0);

        nh_.param("x_max", Environment::Instance().x_max, 3.4);
        nh_.param("x_min", Environment::Instance().x_min, 0.0);
        nh_.param("y_max", Environment::Instance().y_max, 2.35);
        nh_.param("y_min", Environment::Instance().y_min, 0.0);

        nh_.param("nfe", nlp_config_.nfe, 100);
        nh_.param("nlp_fe_half_id", nlp_config_.nlp_fe_half_id, 25);
        nh_.param("tf_max", nlp_config_.tf_max, 500.0);
        nh_.param("opti_omega", nlp_config_.opti_omega, 0.01);
        nh_.param("opti_a", nlp_config_.opti_a, 0.01);
        nh_.param("opti_v", nlp_config_.opti_v, 0.01);
        nh_.param("opti_distance", nlp_config_.opti_distance, 1.00);
    }

    // Service callback
    bool HandleKeyboardInput(collab_planner::KeyboardInput::Request& req, collab_planner::KeyboardInput::Response& res) {
        CharacterCars cars;

        if (first_exeu_) {
            if (planner_->GetCars(nh_, req.data, cars)) {
                for (int i = 0; i < Environment::Instance().vehicle_num; ++i) {
                    current_poses_[i] = Pose(cars[i].x(), cars[i].y(), 0.0);
                    Polygon2d vec;
                    vec = Polygon2d(Environment::Instance().vehicle_param.GenerateBox(
                        Pose(current_poses_[i].x(), current_poses_[i].y(), current_poses_[i].theta())));
                    VisualizationPlot::PlotPolygon(vec, 0.01, Color::colors[i + 1], i, "vehicle");
                }
                VisualizationPlot::Trigger();
                std::cout << "Initial vehicle positions acquired successfully." << std::endl;
                std::cout << "Enter the target formation character in the keyboard_input terminal." << std::endl;
                std::cout << "Supported characters: 1 2 3 7 C D F H I J K L N O S T U V X Y" << std::endl;
                start_receive_ = true;
                first_exeu_ = false;
                current_pose_received_ = true;
                return true;
            }
        }

        // Pass the received keyboard input to GetCars
        if (planner_->GetCars(nh_, req.data, cars)) {
            // Set the success flag in the response
            res.success = true;
            planner_->ClearAccumulatedResults(planner_->accumulated_results_);
            for (int i = 0; i < Environment::Instance().vehicle_num; ++i) {
                if (!current_pose_received_) {
                    ROS_ERROR("Nokov !");
                }

                planner_->set_start_pose(i, current_poses_[i].x(), current_poses_[i].y(), current_poses_[i].theta());
                planner_->set_goal_pose(i, cars[i].x(), cars[i].y());
            }
            auto goal_poses = FindOptimalPairing(planner_->get_start_pose(), planner_->get_goal_pose());
            for (int i = 0; i < Environment::Instance().vehicle_num; ++i) {
                planner_->set_goal_pose(i, goal_poses[i].x(), goal_poses[i].y());
                VisualizationPlot::PlotPoints(goal_poses[i].x(), goal_poses[i].y(), (i + 1) * 4, "vehicle final", Color::colors[i + 1]);
            }
            VisualizationPlot::Trigger();
            collab_traj_nlp::States states, guesses;
            planner_->PlanTrajOnline(states, guesses, planner_->get_start_pose(), planner_->get_goal_pose());
            start_receive_ = false;
            goal_receive_ = true;
            auto traj_states = planner_->get_states();

            for (size_t i = 0; i < Environment::Instance().vehicle_num; ++i) {
                current_poses_[i] = Pose(traj_states[i].x.back(), traj_states[i].y.back(), traj_states[i].theta.back());
                auto accu_state = traj_states[i];
                if (!accu_state.x.empty()) {
                    auto plot_x = accu_state.xc;
                    auto plot_y = accu_state.yc;
                    auto plot_v = accu_state.v;

                    VisualizationPlot::PlotTrajectory(plot_x, plot_y, plot_v, Color::colors[i + 1],
                                                      Environment::Instance().vehicle_param.max_velocity, 0.01, i + 1, "Final Trajectory");
                }
            }

            VisualizationPlot::Trigger();
            std::cout << "Displaying the vehicle trajectories." << std::endl;
            for (int j = 0; j < traj_states.front().x.size(); ++j) {
                for (int i = 0; i < Environment::Instance().vehicle_num; ++i) {
                    PlotVehicle(i, {traj_states[i].x[j], traj_states[i].y[j], traj_states[i].theta[j]}, traj_states[i].phi[j]);
                }
                ros::Duration(nlp_config_.tf_max / (nlp_config_.nfe - 1)).sleep();
            }
            std::cout << "Enter the target formation character in the keyboard_input terminal." << std::endl;
            std::cout << "Supported characters: 1 2 3 7 C D F H I J K L N O S T U V X Y" << std::endl;
        } else {
            ROS_ERROR("Failed to retrieve cars information.");
            res.success = false;
        }
        return true;    // Return true to indicate that the service request was handled
    }

    // Compute the Euclidean distance between two points
    double EuclideanDistance(const Pose& a, const Pose& b) { return hypot(a.x() - b.x(), a.y() - b.y()); }

    // Compute the variance
    double CalculateVariance(const std::vector<double>& distances) {
        double mean = std::accumulate(distances.begin(), distances.end(), 0.0) / distances.size();
        double variance = 0.0;
        for (double distance : distances) {
            variance += std::pow(distance - mean, 2);
        }
        return variance / distances.size();
    }

    // Record used by the priority queue
    struct VariancePairing {
        double variance;
        double distance_sum;
        std::vector<int> indices;

        bool operator<(const VariancePairing& other) const { return variance < other.variance; }
    };

    Poses FindOptimalPairing(const Poses& start_points, const Poses& end_points) {
        std::vector<int> indices(start_points.size());
        iota(indices.begin(), indices.end(), 0);

        std::priority_queue<VariancePairing, std::vector<VariancePairing>> top_pairings;

        do {
            std::vector<double> distances;
            for (size_t i = 0; i < start_points.size(); ++i) {
                distances.push_back(EuclideanDistance(start_points[i], end_points[indices[i]]));
            }
            double variance = CalculateVariance(distances);
            double distance_sum = std::accumulate(distances.begin(), distances.end(), 0.0);
            top_pairings.push({variance, distance_sum, indices});
            if (top_pairings.size() > 50) {
                top_pairings.pop();
            }
        } while (next_permutation(indices.begin(), indices.end()));

        if (top_pairings.empty())
            throw std::runtime_error("No assignment candidates");
        VariancePairing best = top_pairings.top();
        top_pairings.pop();
        while (!top_pairings.empty()) {
            const auto candidate = top_pairings.top();
            top_pairings.pop();
            if (candidate.distanceSum > best.distanceSum ||
                (candidate.distanceSum == best.distanceSum &&
                 candidate.indices < best.indices))
                best = candidate;
        }
        Poses reordered{};
        for (std::size_t i = 0; i < end_points.size(); ++i)
            reordered.at(i) = end_points.at(best.indices.at(i));
        return reordered;
    }

    void GetGoalPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
        CharacterCars cars;
        if (first_exeu_) {
            if (planner_->GetCars(nh_, "1", cars)) {
                for (int i = 0; i < Environment::Instance().vehicle_num; ++i) {
                    ROS_INFO("Car position for '1': x=%lf, y=%lf", cars[i].x(), cars[i].y());
                    planner_->set_start_pose(i, cars[i].x(), cars[i].y(), 0);
                    Polygon2d vec;
                    vec = Polygon2d(Environment::Instance().vehicle_param.GenerateBox(Pose(cars[i].x(), cars[i].y(), 0)));
                    VisualizationPlot::PlotPolygon(vec, 0.01, Color::Red, (i + 1) * 3, "vehicle initial");
                }
                VisualizationPlot::Trigger();
                start_receive_ = true;
                first_exeu_ = false;
            }
        } else {
            auto cars_start = planner_->get_states();
            for (int i = 0; i < Environment::Instance().vehicle_num; ++i) {
                auto car_start = cars_start[i];
                ROS_INFO("Car position start: x=%lf, y=%lf", car_start.x.back(), car_start.y.back());
                planner_->set_start_pose(i, car_start.x.back(), car_start.y.back(), car_start.theta.back());
                Polygon2d vec;
                vec = Polygon2d(Environment::Instance().vehicle_param.GenerateBox(
                    Pose(car_start.x.back(), car_start.y.back(), car_start.theta.back())));
                VisualizationPlot::PlotPolygon(vec, 0.01, Color::Red, (i + 1) * 3, "vehicle initial");
            }
            VisualizationPlot::Trigger();
            start_receive_ = true;
        }
    }

    void PlotVehicle(int id, const common::math::Pose& pt, double phi) {
        VisualizationPlot::PlotPolygon(
            common::math::Polygon2d(Environment::Instance().vehicle_param.GenerateBox({pt.x(), pt.y(), pt.theta()})), 0.01,
            Color::colors[id + 1], id, "vehicle");

        VisualizationPlot::Trigger();
    }
};

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "collab_planner_node");
    GlobalPlanner global_planner;
    std::cout << "Enter the initial formation character in the keyboard_input terminal." << std::endl;
    std::cout << "Supported characters: 1 2 3 7 C D F H I J K L N O S T U V X Y" << std::endl;
    ros::AsyncSpinner spinner(0);
    spinner.start();
    ros::waitForShutdown();
    return 0;
}
