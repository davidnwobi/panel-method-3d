#pragma once
#include <Eigen/Core>

template <typename Derived> struct ISolver {

  template <typename MatrixType, typename VecType>
  Eigen::VectorXd solve(const Eigen::MatrixBase<MatrixType> &lhs,
                        const Eigen::MatrixBase<VecType> &rhs,
                        double tol = 1e-6, std::size_t maxit = 10) {

    return Derived::solveImpl(lhs, rhs, tol, maxit);
  }
};
