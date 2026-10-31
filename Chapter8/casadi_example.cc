#include <casadi/casadi.hpp>
#include <iostream>
using namespace casadi;

int main() {
  MX x = MX::sym("x");
  MX y = MX::sym("y");
  MX z = MX::sym("z");
  MXDict problem;
  problem["x"] = MX::vertcat({x, y, z});
  problem["f"] = x*x + 100*z*z;
  problem["g"] = z + (1-x)*(1-x) - y;
  Dict options;
  options["ipopt.linear_solver"] = "ma57";
  options["ipopt.print_level"] = 5;
  options["ipopt.tol"] = 1e-9;
  Function solver = nlpsol("solver", "ipopt", problem, options);
  DMDict arguments;
  arguments["x0"] = DM(std::vector<double>{2.5, 3.0, 0.75});
  arguments["lbg"] = 0.0;
  arguments["ubg"] = 0.0;
  DMDict solution = solver(arguments);
  std::cout << "solution: " << solution.at("x") << '\n';
  std::cout << "cost: " << solution.at("f") << '\n';
  std::cout << "constraint residual: " << solution.at("g") << '\n';
  std::cout << "status: " << solver.stats().at("return_status")
            << std::endl;
  return 0;
}
