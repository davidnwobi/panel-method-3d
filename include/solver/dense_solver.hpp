#pragma once
#include "solver/isolver.hpp"
#include <Eigen/Core>
#include <iostream>

struct DenseSolver : ISolver {
  Eigen::VectorXd solve(const Eigen::MatrixXd &lhs,
                        const Eigen::VectorXd &rhs) override {
    std::cout << "Solving...\n";
    return lhs.fullPivLu().solve(rhs);
  }
};
