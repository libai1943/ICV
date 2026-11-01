//
//

#include "collab_traj_nlp.h"

#include <ros/ros.h>

namespace collab_traj_nlp {

CollabTrajNLP::CollabTrajNLP(const TrajectoryNLPConfig &config) : config_(config) {
    nlp_config_ = {{"ipopt", Dict({
                                 {"linear_solver", "ma57"},
                                 {"print_level", 3},
                             })}};
}

void CollabTrajNLP::buildCommon(const Constraints &cars_goal_pos) {
    int num_vehicles = Environment::Instance().vehicle_num;
    double tf = config_.tf_max;
    vehicles_opti_vars_.clear();
    for (int i = 0; i < num_vehicles; ++i) {
        VehicleOptiVars vars;

        vars.var_x_ = SX::sym("x_" + std::to_string(i), config_.nfe);
        vars.var_y_ = SX::sym("y_" + std::to_string(i), config_.nfe);
        vars.var_theta_ = SX::sym("theta_" + std::to_string(i), config_.nfe);
        vars.var_v_ = SX::sym("v_" + std::to_string(i), config_.nfe);
        vars.var_phi_ = SX::sym("phi_" + std::to_string(i), config_.nfe);
        vars.var_a_ = SX::sym("a_" + std::to_string(i), config_.nfe);
        vars.var_omega_ = SX::sym("omega_" + std::to_string(i), config_.nfe);
        vars.var_xc_ = SX::sym("xc_" + std::to_string(i), config_.nfe);
        vars.var_yc_ = SX::sym("yc_" + std::to_string(i), config_.nfe);
        vars.opti_var_ = SX::vertcat({vars.var_x_, vars.var_y_, vars.var_theta_, vars.var_v_, vars.var_phi_,
                                      vars.var_a_, vars.var_omega_, vars.var_xc_, vars.var_yc_});
        vehicles_opti_vars_.emplace_back(std::move(vars));
    }
    double max_omega =
        Environment::Instance().vehicle_param.max_steer_angle_rate / Environment::Instance().vehicle_param.steer_ratio;
    double max_a = Environment::Instance().vehicle_param.max_acceleration;
    double max_v = Environment::Instance().vehicle_param.max_velocity;
    SX total_objective = 0;
    std::vector<SX> g_kin_vector;

    for (int i = 0; i < num_vehicles; ++i) {
        auto &vehicle_vars = vehicles_opti_vars_[i];
        SX objective = 0;
        objective += config_.opti_a * sumsqr(vehicle_vars.var_a_) / num_vehicles / config_.nfe;
        objective += config_.opti_omega * sumsqr(vehicle_vars.var_omega_) / num_vehicles / config_.nfe;
        objective += config_.opti_v * sumsqr(vehicle_vars.var_v_) / num_vehicles / config_.nfe;
        int end = config_.nfe - 1;
        SX end_dist_to_goal = pow(vehicle_vars.var_xc_(end) - cars_goal_pos[i].goal.x, 2) +
                              pow(vehicle_vars.var_yc_(end) - cars_goal_pos[i].goal.y, 2);

        SX total_dist_to_goal = 0;

        for (int j = 1; j < config_.nfe - 1; j++) {
            SX dist_to_goal = pow(vehicle_vars.var_xc_(j) - cars_goal_pos[i].goal.x, 2) +
                              pow(vehicle_vars.var_yc_(j) - cars_goal_pos[i].goal.y, 2);
            total_dist_to_goal += dist_to_goal;
        }

        objective += config_.opti_distance * total_dist_to_goal;

        objective += end_dist_to_goal;

        total_objective += objective;

        auto hi = tf / (config_.nfe - 1);
        auto prev = Slice(0, config_.nfe - 1);
        auto all = Slice(0, config_.nfe);
        auto next = Slice(1, config_.nfe);
        SX g_x_kin = vehicle_vars.var_x_(next) -
                     (vehicle_vars.var_x_(prev) + hi * vehicle_vars.var_v_(prev) * cos(vehicle_vars.var_theta_(prev)));
        SX g_y_kin = vehicle_vars.var_y_(next) -
                     (vehicle_vars.var_y_(prev) + hi * vehicle_vars.var_v_(prev) * sin(vehicle_vars.var_theta_(prev)));
        SX g_theta_kin =
            vehicle_vars.var_theta_(next) -
            (vehicle_vars.var_theta_(prev) + hi * vehicle_vars.var_v_(prev) * tan(vehicle_vars.var_phi_(prev)) /
                                                 Environment::Instance().vehicle_param.wheel_base);
        SX g_v_kin = vehicle_vars.var_v_(next) - (vehicle_vars.var_v_(prev) + hi * vehicle_vars.var_a_(prev));
        SX g_phi_kin = vehicle_vars.var_phi_(next) - (vehicle_vars.var_phi_(prev) + hi * vehicle_vars.var_omega_(prev));

        SX xc = vehicle_vars.var_x_(all) +
                Environment::Instance().vehicle_param.center_to_rear * cos(vehicle_vars.var_theta_(all));
        SX yc = vehicle_vars.var_y_(all) +
                Environment::Instance().vehicle_param.center_to_rear * sin(vehicle_vars.var_theta_(all));
        SX g_kin = SX::vertcat({g_x_kin, g_y_kin, g_theta_kin, g_v_kin, g_phi_kin, vehicle_vars.var_xc_ - xc,
                                vehicle_vars.var_yc_ - yc});    // problem
        g_kin_vector.emplace_back(g_kin);
    }

    objective_ = total_objective;

    g_kin_ = SX::vertcat(g_kin_vector);
}

Function CollabTrajNLP::buildNLP() {
    std::vector<SX> all_vehicles_opti_vars;

    for (const auto &vehicle_vars : vehicles_opti_vars_) {
        all_vehicles_opti_vars.push_back(vehicle_vars.opti_var_);
    }
    SX x = SX::vertcat(all_vehicles_opti_vars);

    SX g = SX::vertcat({g_kin_, g_collision_});

    SXDict nlp = {{"x", x}, {"f", objective_}, {"g", g}};

    auto config = nlp_config_;
    if (config_.use_iterative) {
        config["ipopt"] =
            combine(nlp_config_["ipopt"], {{"max_iter", config_.iterative_direct_iterations * config_.nfe / 100}});
    }

    return nlpsol("solver", "ipopt", nlp, config);
}

std::pair<DM, DM> CollabTrajNLP::GetVariableBounds(const Constraints &constraints) const {
    DM lb_all, ub_all;
    for (int i = 0; i < Environment::Instance().vehicle_num; ++i) {
        DM identity = DM::ones(config_.nfe, 1);

        DM lb_x = Environment::Instance().x_min * identity, ub_x = Environment::Instance().x_max * identity;
        DM lb_y = Environment::Instance().y_min * identity, ub_y = Environment::Instance().y_max * identity;
        DM lb_theta = -DM::inf(config_.nfe), ub_theta = DM::inf(config_.nfe);
        DM lb_v = Environment::Instance().vehicle_param.max_reverse_velocity * identity,
           ub_v = Environment::Instance().vehicle_param.max_velocity * identity;
        DM lb_phi = -Environment::Instance().vehicle_param.max_steer_angle * identity,
           ub_phi = Environment::Instance().vehicle_param.max_steer_angle * identity;
        DM lb_a = Environment::Instance().vehicle_param.max_deceleration * identity,
           ub_a = Environment::Instance().vehicle_param.max_acceleration * identity;
        DM lb_omega = -Environment::Instance().vehicle_param.max_steer_angle_rate * identity,
           ub_omega = Environment::Instance().vehicle_param.max_steer_angle_rate * identity;
        DM lb_xc = Environment::Instance().x_min * identity, ub_xc = Environment::Instance().x_max * identity;
        DM lb_yc = Environment::Instance().y_min * identity, ub_yc = Environment::Instance().y_max * identity;

        lb_x(0) = ub_x(0) = constraints[i].start.x;
        lb_y(0) = ub_y(0) = constraints[i].start.y;
        lb_theta(0) = ub_theta(0) = constraints[i].start.theta;
        lb_v(0) = ub_v(0) = constraints[i].start.v;
        lb_phi(0) = ub_phi(0) = constraints[i].start.phi;
        lb_a(0) = ub_a(0) = constraints[i].start.a;
        lb_omega(0) = ub_omega(0) = constraints[i].start.omega;

        int end = config_.nfe - 1;
        lb_v(end) = ub_v(end) = constraints[i].goal.v;
        lb_a(end) = ub_a(end) = constraints[i].goal.a;
        lb_omega(end) = ub_omega(end) = constraints[i].goal.omega;

        DM lb = DM::vertcat({lb_x, lb_y, lb_theta, lb_v, lb_phi, lb_a, lb_omega, lb_xc, lb_yc});
        DM ub = DM::vertcat({ub_x, ub_y, ub_theta, ub_v, ub_phi, ub_a, ub_omega, ub_xc, ub_yc});

        lb_all = lb_all.is_empty() ? lb : DM::vertcat({lb_all, lb});
        ub_all = ub_all.is_empty() ? ub : DM::vertcat({ub_all, ub});
    }

    return {lb_all, ub_all};
}

std::pair<std::vector<SX>, size_t> CollabTrajNLP::GetCollisionBounds(const States &states) const {
    std::vector<std::tuple<int, int, int>> potential_collisions;

    for (int i = 0; i < config_.nfe; ++i) {
        for (size_t j = 0; j < Environment::Instance().vehicle_num - 1; ++j) {
            for (size_t k = j + 1; k < Environment::Instance().vehicle_num; ++k) {
                const auto &p1_xc = states[j].x.at(i) +
                                    Environment::Instance().vehicle_param.center_to_rear * cos(states[j].theta.at(i));
                const auto &p1_yc = states[j].y.at(i) +
                                    Environment::Instance().vehicle_param.center_to_rear * sin(states[j].theta.at(i));
                const auto &p2_xc = states[k].x.at(i) +
                                    Environment::Instance().vehicle_param.center_to_rear * cos(states[k].theta.at(i));
                const auto &p2_yc = states[k].y.at(i) +
                                    Environment::Instance().vehicle_param.center_to_rear * sin(states[k].theta.at(i));

                if (hypot(p1_xc - p2_xc, p1_yc - p2_yc) < 8 * config_.threshold_distance) {
                    potential_collisions.emplace_back(i, j, k);
                }
            }
        }
    }
    std::vector<SX> collision_constraints;
    for (const auto &collision : potential_collisions) {
        int i = std::get<0>(collision);
        int j = std::get<1>(collision);
        int k = std::get<2>(collision);

        SX dist_squared = pow(vehicles_opti_vars_[j].var_xc_(i) - vehicles_opti_vars_[k].var_xc_(i), 2) +
                          pow(vehicles_opti_vars_[j].var_yc_(i) - vehicles_opti_vars_[k].var_yc_(i), 2);
        double threshold =
            4 * pow(Environment::Instance().vehicle_param.radius, 2) + 0.03 * (static_cast<double>(i) / config_.nfe);

        collision_constraints.push_back(dist_squared - threshold);
    }

    return {collision_constraints, collision_constraints.size()};
}

States CollabTrajNLP::GetStatesFromSolution(const DM &solution) const {
    States results;

    for (size_t i = 0; i < Environment::Instance().vehicle_num; ++i) {
        int base = i * config_.nfe * num_vars_per_vehicle_;
        State result;
        result.x.resize(config_.nfe, 0);
        result.y.resize(config_.nfe, 0);
        result.theta.resize(config_.nfe, 0);
        result.v.resize(config_.nfe, 0);
        result.phi.resize(config_.nfe, 0);
        result.a.resize(config_.nfe, 0);
        result.omega.resize(config_.nfe, 0);
        result.xc.resize(config_.nfe, 0);
        result.yc.resize(config_.nfe, 0);
        for (size_t j = 0; j < config_.nfe; ++j) {
            result.x[j] = double(solution(base + j, 0));
            result.y[j] = double(solution(base + config_.nfe + j, 0));
            result.theta[j] = double(solution(base + 2 * config_.nfe + j, 0));
            result.v[j] = double(solution(base + 3 * config_.nfe + j, 0));
            result.phi[j] = double(solution(base + 4 * config_.nfe + j, 0));
            result.a[j] = double(solution(base + 5 * config_.nfe + j, 0));
            result.omega[j] = double(solution(base + 6 * config_.nfe + j, 0));
            result.xc[j] = double(solution(base + 7 * config_.nfe + j, 0));
            result.yc[j] = double(solution(base + 8 * config_.nfe + j, 0));
        }
        results.at(i) = result;
    }

    return results;
}

bool CollabTrajNLP::Solve(Constraints &profiles, States &guesses, States &results) {
    if (guesses.size() != profiles.size()) {
        std::cerr << "Error: guesses size (" << guesses.size() << ") does not match profiles size (" << profiles.size()
                  << ")." << std::endl;
        return false;
    }
    DM eq_bound = DM::zeros(((config_.nfe - 1) * 7 + 2) * Environment::Instance().vehicle_num);
    std::vector<DM> lbx_all, ubx_all, x0_all;
    auto gcoll = GetCollisionBounds(guesses);
    DM lbg = DM::vertcat({eq_bound, DM::zeros(gcoll.second)});
    DM ubg = DM::vertcat({eq_bound, DM::inf(gcoll.second)});

    for (size_t i = 0; i < profiles.size(); ++i) {
        DM guess = DM::vertcat({guesses[i].x, guesses[i].y, guesses[i].theta, guesses[i].v, guesses[i].phi,
                                guesses[i].a, guesses[i].omega, guesses[i].xc, guesses[i].yc});
        x0_all.push_back(guess);
    }

    auto var_bounds = GetVariableBounds(profiles);

    lbx_all.push_back(var_bounds.first);
    ubx_all.push_back(var_bounds.second);

    DM lbx = DM::vertcat(lbx_all);
    DM ubx = DM::vertcat(ubx_all);
    DM x0 = DM::vertcat(x0_all);

    DMDict arg;
    arg["lbx"] = lbx;
    arg["ubx"] = ubx;
    arg["lbg"] = lbg;
    arg["ubg"] = ubg;
    arg["x0"] = x0;

    g_collision_ = SX::vertcat(gcoll.first);

    auto solver = buildNLP();
    auto res = solver(arg);
    bool is_success = solver.stats()["success"];
    DM opt = res.at("x");
    results = GetStatesFromSolution(opt);

    return is_success;
}

}    // namespace collab_traj_nlp
