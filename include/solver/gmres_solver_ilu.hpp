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
// "D:/PortableDev/projects/panel_methods_3f/python/out_cpp"

struct GMRESILUSolver : ISolver<GMRESILUSolver> {
  inline static float spTol = 1e-6;
  inline static float dropTol = 1e-6;

public:
  GMRESILUSolver() : ISolver<GMRESILUSolver>() {}
  auto setspTol(float spTol_) {
    spTol = spTol_;
    return *this;
  }
  auto setdropTol(float dropTol_) {
    dropTol = dropTol_;
    return *this;
  }
  template <typename MatrixType, typename VecType>
  static Eigen::VectorXf solveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
                                   const Eigen::MatrixBase<VecType> &rhs,
                                   float tol = 1e-6,
                                   std::size_t maxit = 1000) {

    typedef Eigen::SparseMatrix<float> SpMat;
    typedef Eigen::Triplet<float> T;

    SpMat A(lhs.rows(), lhs.cols());
    A = lhs.sparseView(spTol, 1);
    A.makeCompressed();
    Eigen::SparseMatrix<float> precond_mat(lhs.rows(), lhs.cols());
    precond_mat = A.pruned(dropTol, 1);
    precond_mat.makeCompressed();

    const Eigen::IncompleteLUT<float> preconditioner(
        precond_mat, Eigen::NumTraits<float>::dummy_precision(), 1);
    // printf("\n");
    // printf("%1.10f\n", spTol);
    // printf("%1.6f\n",
    //        ((float)A.nonZeros()) / ((float)(lhs.rows() * lhs.cols())));
    Eigen::VectorXf x(rhs.rows());
    x.setZero();
    Eigen::Index iters = 1000;
    Eigen::internal::gmres(lhs, rhs, x, preconditioner, iters, maxit, tol);
    // std::cout << "#iterations:     " << iters << std::endl;
    // std::cout << "estimated error: " << tol << std::endl;
    // std::cout << "#iterations:     " << solver.iterations() << std::endl;
    // std::cout << "estimated error: " << solver.error() << std::endl;

    return x;
  }
};
