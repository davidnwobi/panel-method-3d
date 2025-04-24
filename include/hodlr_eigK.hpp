#pragma once
#include <Eigen/Core>
#include <HODLR.hpp>
namespace hodlr_solver_internal {
template <typename MatrixType> class EigenKernel : public HODLR_Matrix {
  const MatrixType &A_;

public:
  explicit EigenKernel(const MatrixType &A) : HODLR_Matrix(A.rows()), A_(A) {}
  double getMatrixEntry(int i, int j) override { return A_(i, j); }

  Eigen::Index rows() { return A_.rows(); }
  const Eigen::Index rows() const { return A_.rows(); }

  Eigen::Index cols() { return A_.cols(); }
  const Eigen::Index cols() const { return A_.cols(); }
};
} // namespace hodlr_solver_internal
template <typename MatrixType> class HodlrWrapper {

  hodlr_solver_internal::EigenKernel<MatrixType> K;
  HODLR T;
  bool assembled;

public:
  explicit HodlrWrapper(const MatrixType &A, int leaf, double tol)
      : K(A), T(A.rows(), leaf, tol) {

    T.assemble(&K, "rookPivoting", 0, 0);
  }
  template <typename Derived>
  Derived operator*(const Eigen::MatrixBase<Derived> &rhs) const {
    Mat x(rhs.rows(), rhs.cols());
    x.middleCols(0, rhs.cols()) = rhs;
    Mat y = const_cast<HODLR &>(T).matmatProduct(rhs);
    Derived out = y;
    return out;
  }
  void factorize() const { const_cast<HODLR &>(T).factorize(); }
  template <typename Derived>
  Derived solve(const Eigen::MatrixBase<Derived> &rhs) const {
    return const_cast<HODLR &>(T).solve(rhs);
  }
  Eigen::Index rows() { return K.rows(); }
  const Eigen::Index rows() const { return K.rows(); }

  Eigen::Index cols() { return K.cols(); }
  const Eigen::Index cols() const { return K.cols(); }

  HODLR &hodlr() { return T; }
  hodlr_solver_internal::EigenKernel<MatrixType> &kernel() { return K; }
};
// explicit HodlrWrapper(const MatrixType &A, int leaf, double tol)
//     : K(A), T(A.rows(), leaf, tol) {}
// Eigen::VectorXd operator*(const Eigen::VectorXd &rhs) const {
//   using Derived = Eigen::VectorXd;
//   if constexpr (Eigen::MatrixBase<Derived>::ColsAtCompileTime == 1) {
//     printf("\n 1, Rows %ld, Cols, %ld, rhs Rows: %ld\n", rows(), cols(),
//            rhs.rows());
//     const_cast<HODLR &>(T).assemble(
//         const_cast<hodlr_solver_internal::EigenKernel<MatrixType> *>(&K),
//         "rookPivoting", 0, 0);
//     Mat x(rhs.rows(), 1);
//     x.col(0) = rhs;
//     Mat y = const_cast<HODLR &>(T).matmatProduct(x);
//     Derived out = y.col(0);
//     return out;
//   }
//   printf("\n, 2Rows %ld, Cols, %ld, rhs Rows: %ld\n", rows(), cols(),
//          rhs.rows());
//   return const_cast<HODLR &>(T).matmatProduct(rhs);
// }
