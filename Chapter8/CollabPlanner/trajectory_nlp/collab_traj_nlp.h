//
//

#pragma once

#include <casadi/casadi.hpp>
#include <tuple>

#include "common/environment.h"
#include "common/math/vec2d.h"
#include "common/trajectory_point.h"
#include "trajectory_nlp_config.h"

using common::math::Vec2d;

namespace collab_traj_nlp {

using namespace casadi;

constexpr size_t kVechicleNum = 8;

struct State {
    std::vector<double> x, y, theta, v, phi, a, omega, xc, yc;
};

struct Constraint {
    TrajectoryPoint start, goal;
    std::vector<bool> gears;
};

typedef std::array<State, kVechicleNum> States;
typedef std::array<Constraint, kVechicleNum> Constraints;

struct VehicleData {
    State state;
    Constraint constraint;
};

class CollabTrajNLP {
   public:
    explicit CollabTrajNLP(const TrajectoryNLPConfig &);

    State ConvertToState(const CharacterCars &cars) {
        State state;
        for (const common::math::Vec2d &car : cars) {
            state.x.push_back(car.x());
            state.y.push_back(car.y());
        }
        return state;
    }

    CharacterCars ConvertToCharacterCars(const State &state) {
        CharacterCars cars;
        for (size_t i = 0; i < cars.size(); ++i) {
            Vec2d car;
            car.set_x(state.x[i]);
            car.set_y(state.y[i]);
            cars[i] = car;
        }
        return cars;
    }

    CharacterCars ConstraintsToCharacterCars(const Constraints &constraints) {
        CharacterCars cars;
        for (size_t i = 0; i < kVechicleNum; ++i) {
            cars[i] = Vec2d(constraints[i].goal);    // or constraints[i].goals
        }
        return cars;
    }

    Constraints CharacterCarsToConstraints(const CharacterCars &cars) {
        std::array<Constraint, kVechicleNum> constraints;
        for (size_t i = 0; i < kVechicleNum; ++i) {
            constraints[i].goal = TrajectoryPoint(cars[i]);
        }
        return constraints;
    }

    void buildCommon(const Constraints &cars_goal_pos);

    bool Solve(Constraints &profiles, States &guesses, States &results);

   private:
    TrajectoryNLPConfig config_;

    struct VehicleOptiVars {
        SX var_x_, var_y_, var_theta_, var_v_, var_phi_, var_a_, var_omega_, var_xc_, var_yc_;
        SX opti_var_;
    };
    std::vector<VehicleOptiVars> vehicles_opti_vars_;

    Dict nlp_config_;
    SX objective_;

    SX g_kin_;
    SX g_collision_;
    int num_vars_per_vehicle_ = 9;

    Function buildNLP();
    std::pair<DM, DM> GetVariableBounds(const Constraints &constraints) const;
    std::pair<std::vector<SX>, size_t> GetCollisionBounds(const States &states) const;
    States GetStatesFromSolution(const DM &solution) const;
};

}    // namespace collab_traj_nlp