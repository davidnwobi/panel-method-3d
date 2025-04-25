#pragma once
#include "solver/isolver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <cmath>
#include <iostream>
#include <unsupported/Eigen/IterativeSolvers>
#include <unsupported/Eigen/SparseExtra>
#include <vector>
// #define ANALYSIS_DIR
// "D:/PortableDev/projects/panel_methods_3f/python/out_cpp"

#define SAVE_SYSTEM 0
#define ITER_RES 0
using namespace Eigen;

struct DenseGMRESILUSolver : ISolver<DenseGMRESILUSolver> {
  static inline float dropTol = 1e-6;
  inline static Eigen::Index iters = 1000;
  inline static float errs = 1e-6;

public:
  DenseGMRESILUSolver() : ISolver<DenseGMRESILUSolver>() {}
  auto setdropTol(float dropTol_) {
    dropTol = dropTol_;
    return *this;
  }
  template <typename MatrixType, typename VecType>
  static Eigen::VectorXf solveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
                                   const Eigen::MatrixBase<VecType> &rhs,
                                   float tol = 1e-6, std::size_t maxit = 1000) {
    // std::cout << "Creating...\n";
    using SpMat = Eigen::SparseMatrix<float>;
    Eigen::SparseMatrix<float> precond_mat(lhs.rows(), lhs.cols());
    precond_mat = lhs.sparseView(dropTol, 1);
    precond_mat.makeCompressed();

    Eigen::IncompleteLUT<float> preconditioner(
        precond_mat, Eigen::NumTraits<float>::dummy_precision(), 1);
    preconditioner.setDroptol(dropTol);
    // Eigen::IncompleteLUT<float> preconditioner(
    //     precond_mat, NumTraits<float>::dummy_precision(), 1);
    // print("Sparsity: ",

    // printf("DropTol: %1.6f\n", dropTol);
    // std::cout << "Solving...\n";
    Eigen::VectorXf x(rhs.rows());
    x.setZero();
    iters = maxit;
    errs = tol;
    Eigen::internal::gmres(lhs, rhs, x, preconditioner, iters, maxit, errs);
    // std::cout << "#iterations:     " << iters << std::endl;
    // std::cout << "estimated error: " << tol << std::endl;

    // std::cout << "#iterations:     " << solver.iterations() << std::endl;
    // std::cout << "estimated error: " << solver.error() << std::endl;

#if (SAVE_SYSTEM == 1)
    SpMat r;
    r = lhs.sparseView(1e-7, 1);
    Eigen::saveMarket(r, "lhs.txt");
    Eigen::saveMarketDense(rhs, "rhs.txt");
    Eigen::saveMarketDense(x, "solution.txt");
#endif
    return x;
  }
};
