#pragma once
#include <Eigen/Core>

template <typename Derived> struct ISolver {

  template <typename MatrixType, typename VecType>
  Eigen::VectorXf solve(const Eigen::MatrixBase<MatrixType> &lhs,
                        const Eigen::MatrixBase<VecType> &rhs, float tol = 1e-6,
                        std::size_t maxit = 1000) {

    return Derived::solveImpl(lhs, rhs, tol, maxit);
  }
};
