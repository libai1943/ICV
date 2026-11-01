//
//

// TODO

#include "collab_traj_optimizer.h"
#include "common/util/time.h"
#include "visualization_plot.h"

/*debug*/
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

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

    counter_ = 0;
    // Reset the control flags
    optimization_is_done_ = false;    // Mark optimization as incomplete
    // new_result_available_ = false;    // Mark the new result as unavailable
    // last_send_ = false;
    // Start the optimization thread
    // This thread runs the optimizer and updates the result after each iteration
    optimization_thread_ = std::thread(&TrajectoryOptimizer::CollabOptimization, this, std::ref(guess), starts, goals);

    // Start the result-publication thread
    // This thread waits for each new result, then processes and publishes it
    std::thread result_thread(&TrajectoryOptimizer::SendResults, this);

    // Wait for the optimization thread
    // The main thread waits here until all optimization iterations finish
    if (optimization_thread_.joinable()) {
        optimization_thread_.join();
    }

    // Wait for the result-publication thread
    // Wait until all queued intermediate results have been processed and published
    if (result_thread.joinable()) {
        result_thread.join();
    }

    // Copy the final result into result
    // Return the final optimization result to the caller
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
        // constraints.at(i).goal.x = goals.at(i).x()-vehicle_param_.center_to_rear*cos(0);
        // constraints.at(i).goal.y = goals.at(i).y()-vehicle_param_.center_to_rear*sin(0);

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

        // Enqueue the current result
        {
            std::lock_guard<std::mutex> lock(queue_mutex_);
            result_queue_.push(current_result_);    // Store a copy to avoid a data race
        }
        queue_cv_.notify_one();    // Notify the result-publication thread

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
            optimization_is_done_.store(true, std::memory_order_release);
            queue_cv_.notify_one();    // Send the final notification

            // bool is_colli = CheckVehicleCollisionAtEachPoint(get_states());

            std::cout << "Computation time: " << now_time - start_time << " s" << std::endl;

            break;
        }

        if ((counter_ > max_iteration_) && (history_log[history_log.size() - 2] == history_log.back())) {
            ROS_ERROR("Optimization Failed!");
            optimization_is_done_.store(true, std::memory_order_release);
            queue_cv_.notify_one();    // Send the final notification
            break;
        }

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
    // ros::Publisher result_pub = nh_.advertise<sandbox_msgs::Trajectory>("topic_name", 10);
    while (true) {
        std::unique_lock<std::mutex> lock(queue_mutex_);
        // Wait until the queue contains data or optimization has finished
        queue_cv_.wait(
            lock, [this] { return !result_queue_.empty() || optimization_is_done_.load(std::memory_order_acquire); });

        while (!result_queue_.empty()) {
            States result = result_queue_.front();
            result_queue_.pop();

            // Release the lock before processing data so the producer can continue
            lock.unlock();
            // ------ Data processing and visualization without holding the queue lock ------
            for (auto& state : result) {
                size_t data_size = optimization_done_ ? state.x.size() : nlp_config_.nlp_fe_half_id;
                // Copy the data into accumulated_results_
                std::copy_n(state.x.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].x));
                std::copy_n(state.y.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].y));
                std::copy_n(state.theta.begin(), data_size,
                            std::back_inserter(accumulated_results_[final_index_].theta));
                std::copy_n(state.v.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].v));
                std::copy_n(state.phi.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].phi));
                std::copy_n(state.a.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].a));
                std::copy_n(state.omega.begin(), data_size,
                            std::back_inserter(accumulated_results_[final_index_].omega));
                std::copy_n(state.xc.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].xc));
                std::copy_n(state.yc.begin(), data_size, std::back_inserter(accumulated_results_[final_index_].yc));

                final_index_ = (final_index_ + 1) % accumulated_results_.size();
            }
            for (size_t i = 0; i < Environment::Instance().vehicle_num; ++i) {
                auto accu_state = accumulated_results_[i];
                if (!accu_state.x.empty()) {
                    auto plot_x = accu_state.xc;
                    auto plot_y = accu_state.yc;
                    auto plot_v = accu_state.v;

                    VisualizationPlot::PlotTrajectory(plot_x, plot_y, plot_v, Color::colors_y[i + 1],
                                                      Environment::Instance().vehicle_param.max_velocity, 0.01, i + 1,
                                                      "Final Trajectory");
                }
            }
            VisualizationPlot::Trigger();
            lock.lock();    // Lock again before inspecting the queue
        }
        // Determine whether to exit the loop
        if (optimization_is_done_.load(std::memory_order_acquire)) {
            break;
        }
    }
    // last_send_ = true;
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
                // state.v[i] = 0;
                // state.a[i] = 0;
                // state.phi[i] = 0;
                // state.omega[i] = 0;
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
                // state.v[i] = v_fill;
                // state.a[i] = a_fill;
                // state.phi[i] = phi_fill;
                // state.omega[i] = omega_fill;
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
    // Each element of accu_states contains one vehicle's state sequence
    // Iterate over all sampled times
    for (size_t time = 0; time < colli_states[0].x.size(); ++time) {
        // At each sampled time, iterate over all vehicles
        for (size_t i = 0; i < colli_states.size(); ++i) {
            // Get vehicle i's state at the current time
            const auto& state1_x = colli_states[i].x.at(time);
            const auto& state1_y = colli_states[i].y.at(time);
            const auto& state1_theta = colli_states[i].theta.at(time);
            Polygon2d vec1 =
                Polygon2d(Environment::Instance().vehicle_param.GenerateBox(Pose(state1_x, state1_y, state1_theta)));
            // Compare against the remaining vehicles
            for (size_t j = i + 1; j < colli_states.size(); ++j) {
                // Get vehicle j's state at the current time
                const auto& state2_x = colli_states[j].x.at(time);
                const auto& state2_y = colli_states[j].y.at(time);
                const auto& state2_theta = colli_states[j].theta.at(time);
                Polygon2d vec2 = Polygon2d(
                    Environment::Instance().vehicle_param.GenerateBox(Pose(state2_x, state2_y, state2_theta)));

                if (vec1.HasOverlap(vec2)) {
                    return true;    // Collision found
                }
            }
        }
    }

    return false;    // No collision was found at any sampled time
}

}    // namespace collab_traj_nlp
