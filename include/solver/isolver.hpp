#pragma once
#include <Eigen/Core>

struct ISolver {
  virtual ~ISolver() = default;
  virtual Eigen::VectorXd solve(const Eigen::MatrixXd &lhs,
                                const Eigen::VectorXd &rhs, double tol=1e-6, std::size_t maxit = 10) = 0;
};
