#pragma once
#include <Eigen/Core>

struct ISolver {
  virtual ~ISolver() = default;
  virtual Eigen::VectorXf solve(const Eigen::MatrixXf &lhs,
                                const Eigen::VectorXf &rhs, float tol=1e-6, std::size_t maxit = 10) = 0;
};
