//
// Created by yenkn on 3/1/22.
//
#pragma once

#include "common/environment.h"

struct TrajectoryNLPConfig {
    int nfe = 100;

    int nlp_fe_half_id = 25;

    double threshold_distance = Environment::Instance().vehicle_param.radius;

    double tf_max = 5.0;

    /**
     * penalty factor to |phi|_2 cost, resulting in smoother trajectory
     */
    double opti_v = 1e-4;

    /**
     * penalty factor to |omega|_2 cost
     */
    double opti_omega = 1e-2;

    /**
     * penalty factor to |a|_2 cost
     */
    double opti_a = 1e-2;

    double opti_distance = 0.001;

    int iterative_direct_iterations = 1000;
    int iterative_first_iterations = 400;
    int iterative_rest_iterations = 1000;

    bool use_iterative = true;
};