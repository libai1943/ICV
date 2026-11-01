//
//

#include "collab_traj_optimizer.h"

#include "common/util/time.h"
#include "visualization_plot.h"

namespace collab_traj_nlp {

TrajectoryOptimizer::TrajectoryOptimizer(const TrajectoryNLPConfig& config) : nlp_config_(config), collab_nlp_(config) {
    vehicle_param_ = Environment::Instance().vehicle_param;
    // Use the current date and time to create the log filename
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%H-%M-%S");
    log_filename_ = "logfile_" + ss.str() + ".csv";
}

TrajectoryOptimizer::TrajectoryOptimizer(const TrajectoryNLPConfig& config, ros::Publisher* pub)
    : nlp_config_(config), collab_nlp_(config) {
    vehicle_param_ = Environment::Instance().vehicle_param;
    result_pub_ = pub;
}

void TrajectoryOptimizer::PlanTrajOnline(States& guess, States& result, const Poses& starts, const Poses& goals) {
    GenerateInitialGuessForFirstTimeNlp(starts, guess);

    // Reset the working variables
    counter_ = 0;
    optimization_done_ = false;    // Mark optimization as incomplete
    last_send_ = false;

    CollabOptimization(guess, starts, goals);
    result = accumulated_results_;
}

void TrajectoryOptimizer::CollabOptimization(States& guess, const Poses& starts, const Poses& goals) {
    Constraints constraints;

    for (int i = 0; i < Environment::Instance().vehicle_num; ++i) {
        constraints.at(i).start.x = starts.at(i).x();
        constraints.at(i).start.y = starts.at(i).y();
        constraints.at(i).start.theta = starts.at(i).theta();
        constraints.at(i).start.v = 0;
        constraints.at(i).start.omega = 0;
        constraints.at(i).start.a = 0;
        constraints.at(i).start.phi = 0;

        constraints.at(i).goal.x = goals.at(i).x();
        constraints.at(i).goal.y = goals.at(i).y();
        constraints.at(i).goal.theta = 0;
        constraints.at(i).goal.v = 0;
        constraints.at(i).goal.omega = 0;
        constraints.at(i).goal.a = 0;
        constraints.at(i).goal.phi = 0;
    }
    collab_nlp_.buildCommon(constraints);
    double start_time = common::util::GetCurrentTimestamp();
    total_time_ = 0;
    while (true) {
        ++counter_;
        // Run the optimization
        collab_nlp_.Solve(constraints, guess, current_result_);
        double now_time = common::util::GetCurrentTimestamp();
        std::vector<double> summer;
        summer.clear();
        for (int i = 0; i < current_result_.size(); ++i) {
            double dist =
                hypot(current_result_[i].xc.back() - goals[i].x(), current_result_[i].yc.back() - goals[i].y());
            summer.push_back(dist);
        }

        double sum = std::accumulate(summer.begin(), summer.end(), 0.0);
        double mean = sum / summer.size();

        history_log.push_back(mean);

        if (mean < 0.01 && *std::max_element(summer.begin(), summer.end()) < 0.025) {
            optimization_done_ = true;
            SendResults();
            std::cout << "Computation time: " << now_time - start_time << " s" << std::endl;

            break;
        }

        if ((counter_ > max_iteration_) && (history_log[history_log.size() - 2] == history_log.back())) {
            ROS_ERROR("Optimization Failed!");
            optimization_done_ = true;
            break;
        }

        SendResults();

        GenerateInitialGuessForNlp2(current_result_);

        for (int i = 0; i < Environment::Instance().vehicle_num; ++i) {
            constraints.at(i).start.x = current_result_.at(i).x.front();
            constraints.at(i).start.y = current_result_.at(i).y.front();
            constraints.at(i).start.theta = current_result_.at(i).theta.front();
            constraints.at(i).start.v = current_result_.at(i).v.front();
            constraints.at(i).start.phi = current_result_.at(i).phi.front();
            constraints.at(i).start.a = current_result_.at(i).a.front();
            constraints.at(i).start.omega = current_result_.at(i).omega.front();
        }
        guess = current_result_;
    }
}

void TrajectoryOptimizer::LogCurrentResult(double time) {
    std::ofstream file;

    // Check if the file already exists
    std::ifstream infile(log_filename_);

    // Open the file in append mode
    file.open(log_filename_, std::ios::app);

    if (!file.is_open()) {
        ROS_ERROR("Unable to open file for logging");
        return;
    }

    file << time << "\n";
    total_time_ += time;
    // Close the file
    file.close();
}

void TrajectoryOptimizer::SendResults() {
    bool is_last_iteration = (optimization_done_ == true);
    for (auto& state : current_result_) {
        size_t data_size = is_last_iteration ? state.x.size() : nlp_config_.nlp_fe_half_id;
        if (state.x.size() > nlp_config_.nlp_fe_half_id) {
            std::copy_n(state.x.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].x));
            std::copy_n(state.y.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].y));
            std::copy_n(state.theta.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].theta));
            std::copy_n(state.v.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].v));
            std::copy_n(state.phi.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].phi));
            std::copy_n(state.a.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].a));
            std::copy_n(state.omega.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].omega));
            std::copy_n(state.xc.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].xc));
            std::copy_n(state.yc.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].yc));
        }
        final_index_ = (final_index_ + 1) % accumulated_results_.size();
    }
    for (size_t i = 0; i < Environment::Instance().vehicle_num; ++i) {
        auto accu_state = accumulated_results_[i];
        if (!accu_state.x.empty()) {
            auto plot_x = accu_state.xc;
            auto plot_y = accu_state.yc;
            auto plot_v = accu_state.v;

            VisualizationPlot::PlotTrajectory(plot_x, plot_y, plot_v, Color::colors[i + 1],
                                              Environment::Instance().vehicle_param.max_velocity, 0.01, i + 1,
                                              "Final Trajectory");
        }
    }
    VisualizationPlot::Trigger();
    if (optimization_done_) {
        last_send_ = true;
    }
}

void TrajectoryOptimizer::GenerateInitialGuessForFirstTimeNlp(const Poses& starts, States& states) {
    for (int i = 0; i < Environment::Instance().vehicle_num; ++i) {
        states[i].x.resize(nlp_config_.nfe, 0);
        states[i].y.resize(nlp_config_.nfe, 0);
        states[i].theta.resize(nlp_config_.nfe, 0);
        states[i].v.resize(nlp_config_.nfe, 0);
        states[i].a.resize(nlp_config_.nfe, 0);
        states[i].phi.resize(nlp_config_.nfe, 0);
        states[i].omega.resize(nlp_config_.nfe, 0);
        states[i].xc.resize(nlp_config_.nfe, 0);
        states[i].yc.resize(nlp_config_.nfe, 0);

        for (int j = 0; j < nlp_config_.nfe; ++j) {
            double xr = starts[i].x();
            double yr = starts[i].y();
            double theta = starts[i].theta();

            states[i].x.at(j) = xr;
            states[i].y.at(j) = yr;
            states[i].theta.at(j) = theta;
            states[i].v.at(j) = 0;
            states[i].a.at(j) = 0;
            states[i].phi.at(j) = 0;
            states[i].omega.at(j) = 0;
            states[i].xc.at(j) = xr + vehicle_param_.center_to_rear * cos(theta);
            states[i].yc.at(j) = yr + vehicle_param_.center_to_rear * sin(theta);
        }
    }
}

void TrajectoryOptimizer::GenerateInitialGuessForNlp2(States& states) {
    int num_suffix = nlp_config_.nlp_fe_half_id;

    for (auto& state : states) {
        int size = state.x.size();
        if (size > num_suffix) {
            // Shift the final 75 samples to the first 75 positions
            for (int i = 0; i < size - num_suffix; ++i) {
                state.x[i] = state.x[i + num_suffix];
                state.y[i] = state.y[i + num_suffix];
                state.theta[i] = state.theta[i + num_suffix];
                state.v[i] = state.v[i + num_suffix];
                state.a[i] = state.a[i + num_suffix];
                state.phi[i] = state.phi[i + num_suffix];
                state.omega[i] = state.omega[i + num_suffix];
                state.xc[i] = state.xc[i + num_suffix];
                state.yc[i] = state.yc[i + num_suffix];
            }

            // Read the final sample
            auto x_fill = state.x.back();
            auto y_fill = state.y.back();
            auto theta_fill = state.theta.back();
            auto v_fill = state.v.back();
            auto a_fill = state.a.back();
            auto phi_fill = state.phi.back();
            auto omega_fill = state.omega.back();
            auto xc_fill = state.xc.back();
            auto yc_fill = state.yc.back();

            // Fill the final 25 positions with the last sample
            for (int i = size - num_suffix; i < size; ++i) {
                state.x[i] = x_fill;
                state.y[i] = y_fill;
                state.theta[i] = theta_fill;
                state.v[i] = 0;
                state.a[i] = 0;
                state.phi[i] = 0;
                state.omega[i] = 0;
                state.xc[i] = xc_fill;
                state.yc[i] = yc_fill;
            }
        }
    }
}

bool TrajectoryOptimizer::GetCars(ros::NodeHandle& nh, const std::string& character, CharacterCars& cars) {
    XmlRpc::XmlRpcValue char_info;
    if (!nh.getParam("characters/" + character, char_info)) {
        ROS_ERROR("Failed to get character info");
        return false;
    }

    for (int i = 0; i < char_info.size(); ++i) {
        Vec2d car;
        car.set_x(static_cast<int>(char_info[i]["x"]) / 100.0);
        car.set_y(static_cast<int>(char_info[i]["y"]) / 100.0);
        cars.at(i) = car;
    }
    return true;
}

bool TrajectoryOptimizer::GetAllCars(ros::NodeHandle& nh, CharacterCarsMap& all_cars) {
    XmlRpc::XmlRpcValue chars_info;
    if (!nh.getParam("characters", chars_info)) {
        ROS_ERROR("Failed to get characters info");
        return false;
    }

    all_cars.clear();
    for (auto it = chars_info.begin(); it != chars_info.end(); ++it) {
        const std::string& character = it->first;
        XmlRpc::XmlRpcValue& char_info = it->second;

        CharacterCars cars;
        for (int i = 0; i < char_info.size(); ++i) {
            Vec2d car;
            car.set_x(static_cast<int>(char_info[i]["x"]) / 100.0);
            car.set_y(static_cast<int>(char_info[i]["y"]) / 100.0);
            cars.at(i) = car;
        }
        all_cars[character] = cars;
    }
    return true;
}

bool TrajectoryOptimizer::CheckVehicleCollisionAtEachPoint(const collab_traj_nlp::States& colli_states) {
    for (size_t time = 0; time < colli_states[0].x.size(); ++time) {
        for (size_t i = 0; i < colli_states.size(); ++i) {
            const auto& state1_x = colli_states[i].x.at(time);
            const auto& state1_y = colli_states[i].y.at(time);
            const auto& state1_theta = colli_states[i].theta.at(time);
            Polygon2d vec1 =
                Polygon2d(Environment::Instance().vehicle_param.GenerateBox(Pose(state1_x, state1_y, state1_theta)));
            for (size_t j = i + 1; j < colli_states.size(); ++j) {
                const auto& state2_x = colli_states[j].x.at(time);
                const auto& state2_y = colli_states[j].y.at(time);
                const auto& state2_theta = colli_states[j].theta.at(time);
                Polygon2d vec2 = Polygon2d(
                    Environment::Instance().vehicle_param.GenerateBox(Pose(state2_x, state2_y, state2_theta)));

                if (vec1.HasOverlap(vec2)) {
                    return true;
                }
            }
        }
    }

    return false;    // No collision was found at any sampled time
}

}    // namespace collab_traj_nlp
