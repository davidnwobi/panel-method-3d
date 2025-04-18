#pragma once
#include "solver/isolver.hpp"
#include <Eigen/Core>
#include <iostream>

struct DenseSolver : ISolver {
  Eigen::VectorXd solve(const Eigen::MatrixXd &lhs,
                        const Eigen::VectorXd &rhs, double tol=1e-6, std::size_t maxit = 10) override {
    std::cout << "Solving...\n";
    return lhs.fullPivLu().solve(rhs);
  }
};
