#pragma once
#include <Eigen/Core>
#include <hodlr_eigK.hpp>

template <typename MatrixType, typename VecType>
Eigen::VectorXd hodlrSolveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
                               const Eigen::MatrixBase<VecType> &rhs,
                               double tol) {
  const int leaf = 128;

  // ------------------------------------------------------------------
  //  HODLR build  +  factor
  // ------------------------------------------------------------------
  HodlrWrapper hodlr_wrapper(lhs, leaf, tol);
  auto &T = hodlr_wrapper.hodlr();
  auto &K = hodlr_wrapper.kernel();

  T.factorize();

  // ------------------------------------------------------------------
  //  HODLR solve
  // ------------------------------------------------------------------
  return T.solve(rhs);
}

struct HODLRSolver {

  template <typename MatrixType, typename VecType>
  Eigen::VectorXd solve(const Eigen::MatrixBase<MatrixType> &lhs,
                        const Eigen::MatrixBase<VecType> &rhs,
                        double tol = 1e-6) {
    return hodlrSolveImpl(lhs, rhs, tol);
  }
};
