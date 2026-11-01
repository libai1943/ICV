/**
 * @file casadi_example.cc
 * @brief Solve a basic nonlinear program (NLP) with the CasADi C++ API.
 *
 * This example defines decision variables, an objective, and constraints,
 * then solves the optimization problem with IPOPT.
 *
 * Optimization problem:
 * minimize(x,y,z)   f(x, z) = x^2 + 100*z^2
 * subject to        g(x, y, z) = z + (1-x)^2 - y = 0
 */

#include <casadi/casadi.hpp>
#include <iostream>

using namespace casadi;

int main() {
    // --------------------------------------------------------------------------
    // 1. Define the problem: create symbolic variables
    // --------------------------------------------------------------------------
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX z = MX::sym("z");

    // --------------------------------------------------------------------------
    // 2. Construct the objective and constraint expressions
    // --------------------------------------------------------------------------
    // Objective: f = x^2 + 100*z^2
    MX f = pow(x, 2) + 100 * pow(z, 2);
    // Constraint: g = z + (1-x)^2 - y
    MX g = z + pow(1 - x, 2) - y;

    // --------------------------------------------------------------------------
    // 3. Assemble the nonlinear program (NLP)
    // --------------------------------------------------------------------------
    MXDict nlp_problem;
    // "x": Decision variables.
    nlp_problem["x"] = vertcat(x, y, z);
    // "f": Objective function.
    nlp_problem["f"] = f;
    // "g": Constraint function.
    nlp_problem["g"] = g;

    // --------------------------------------------------------------------------
    // 4. Configure and create the solver
    // --------------------------------------------------------------------------
    // Store the IPOPT options in a dictionary.
    Dict solver_options;
    // 'ipopt.linear_solver' selects the linear solver used internally by IPOPT.
    solver_options["ipopt.linear_solver"] = "ma57";
    // Console verbosity.
    solver_options["ipopt.print_level"] = 5;
    // `nlpsol` creates an NLP solver instance.
    Function solver = nlpsol("solver", "ipopt", nlp_problem, solver_options);

    // --------------------------------------------------------------------------
    // 5. Solve the problem
    // --------------------------------------------------------------------------
    // Provide numerical inputs to the solver.
    DMDict solver_arguments;
    // Set the initial decision-variable guess (x0), which can strongly affect convergence.
    solver_arguments["x0"] = DM({2.5, 3.0, 0.75});
    // Set the lower and upper bounds (lbg and ubg) for g(x,y,z).
    // For the equality g = 0, both bounds are zero.
    solver_arguments["lbg"] = 0;
    solver_arguments["ubg"] = 0;
    // Call the solver with these inputs and receive the result dictionary.
    DMDict solution = solver(solver_arguments);

    // --------------------------------------------------------------------------
    // 6. Extract and print the result
    // --------------------------------------------------------------------------
    // Extract the returned decision vector and objective value.
    DM optimal_x = solution.at("x");
    DM optimal_f = solution.at("f");
    std::cout << "--------------------------------" << std::endl;
    std::cout << "Solve completed." << std::endl;
    std::cout << "Returned solution (x, y, z): " << optimal_x << std::endl;
    std::cout << "Returned objective value: " << optimal_f << std::endl;
    std::cout << "--------------------------------" << std::endl;

    return 0;
}
