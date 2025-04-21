#pragma once
#include "solver/isolver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <iostream>
#include <unsupported/Eigen/IterativeSolvers>
#include <vector>
// #define ANALYSIS_DIR
// "D:/PortableDev/projects/panel_methods_3d/python/out_cpp"

struct GMRESILUSolver : ISolver<GMRESILUSolver> {
  inline static double spTol = 1e-6;
  inline static double dropTol = 1e-6;

public:
  GMRESILUSolver() : ISolver<GMRESILUSolver>() {}
  auto setspTol(double spTol_) {
    spTol = spTol_;
    return *this;
  }
  auto setdropTol(double dropTol_) {
    dropTol = dropTol_;
    return *this;
  }
  template <typename MatrixType, typename VecType>
  static Eigen::VectorXd solveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
                                   const Eigen::MatrixBase<VecType> &rhs,
                                   double tol = 1e-6,
                                   std::size_t maxit = 1000) {

    typedef Eigen::SparseMatrix<double> SpMat;
    typedef Eigen::Triplet<double> T;

    SpMat A(lhs.rows(), lhs.cols());
    A = lhs.sparseView(spTol, 1);
    A.makeCompressed();
    Eigen::SparseMatrix<double> precond_mat(lhs.rows(), lhs.cols());
    precond_mat = A.pruned(dropTol, 1);
    precond_mat.makeCompressed();

    const Eigen::IncompleteLUT<double> preconditioner(
        precond_mat, Eigen::NumTraits<double>::dummy_precision(), 1);
    // printf("\n");
    // printf("%1.10f\n", spTol);
    // printf("%1.6f\n",
    //        ((double)A.nonZeros()) / ((double)(lhs.rows() * lhs.cols())));
    Eigen::VectorXd x(rhs.rows());
    x.setZero();
    Eigen::Index iters = 1000;
    Eigen::internal::gmres(lhs, rhs, x, preconditioner, iters, maxit, tol);
    // std::cout << "#iterations:     " << solver.iterations() << std::endl;
    // std::cout << "estimated error: " << solver.error() << std::endl;

    return x;
  }
};
