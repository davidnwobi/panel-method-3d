#pragma once
#include "solver/isolver.hpp"
#include <Eigen/Core>
#include <iostream>

struct DenseSolver : ISolver<DenseSolver> {

  DenseSolver() : ISolver<DenseSolver>() {}
  template <typename MatrixType, typename VecType>
  static Eigen::VectorXd solveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
                                   const Eigen::MatrixBase<VecType> &rhs,
                                   double tol = 1e-6, std::size_t maxit = 10) {
    return lhs.partialPivLu().solve(rhs);
  }
};
