#pragma once
#include <Eigen/Core>

struct ISolver {
  virtual ~ISolver() = default;
  virtual Eigen::VectorXd solve(const Eigen::MatrixXd &lhs,
                                const Eigen::VectorXd &rhs) = 0;
};
