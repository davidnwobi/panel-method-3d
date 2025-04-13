#pragma once
#include "solver/isolver.hpp"
#include <Eigen/Core>
#include <iostream>

struct DenseSolver : ISolver {
  Eigen::VectorXf solve(const Eigen::MatrixXf &lhs,
                        const Eigen::VectorXf &rhs, float tol=1e-6, std::size_t maxit = 10) override {
    std::cout << "Solving...\n";
    return lhs.fullPivLu().solve(rhs);
  }
};
