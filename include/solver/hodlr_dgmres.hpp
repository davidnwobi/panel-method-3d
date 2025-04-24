#pragma once
#include "solver/isolver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <Eigen/SparseCore>
#include <cmath>
#include <hodlr_eigK.hpp>
#include <iostream>
#include <unsupported/Eigen/IterativeSolvers>
#include <unsupported/Eigen/SparseExtra>
// #define ANALYSIS_DIR
// "D:/PortableDev/projects/panel_methods_3d/python/out_cpp"

#define SAVE_SYSTEM 0
#define ITER_RES 0
using namespace Eigen;

struct HodlrDgmres : ISolver<HodlrDgmres> {

  inline static double dropTol = 1e-6;
  inline static double spTol = 1e-6;

public:
  auto setdropTol(double dropTol_) {
    dropTol = dropTol_;
    return *this;
  }
  auto setspTol(double spTol_) {
    spTol = spTol_;
    return *this;
  }
  HodlrDgmres() : ISolver<HodlrDgmres>() {}
  template <typename MatrixType, typename VecType>
  static Eigen::VectorXd solveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
                                   const Eigen::MatrixBase<VecType> &rhs,
                                   double tol = 1e-6,
                                   std::size_t maxit = 1000) {
    // std::cout << "Creating...\n";
    using SpMat = Eigen::SparseMatrix<double>;
    // Eigen::SparseMatrix<double> precond_mat(lhs.rows(), lhs.cols());
    // precond_mat = lhs.sparseView(1e-2, 1);

    Eigen::SparseMatrix<double> precond_mat(lhs.rows(), lhs.cols());
    precond_mat = lhs.sparseView(dropTol, 1);
    precond_mat.makeCompressed();

    Eigen::IncompleteLUT<double> preconditioner(
        precond_mat, NumTraits<double>::dummy_precision(), 1);
    // print("Sparsity: ",

    HodlrWrapper hodlr_wrapper(lhs, 128, spTol);
    hodlr_wrapper.factorize();
    hodlr_wrapper.hodlr().plotTree("c3_rank_matrix.txt");
    // printf("DropTol: %1.6f\n", dropTol);
    // std::cout << "Solving...\n";
    Eigen::VectorXd x(rhs.rows());
    x.setZero();
    Eigen::Index iters = 1000;
    double errs = tol;
    Eigen::internal::gmres(hodlr_wrapper, rhs, x, preconditioner, iters, maxit,
                           errs);
    // std::cout << "#iterations:     " << iters << std::endl;
    // std::cout << "estimated error: " << errs << std::endl;

#if (SAVE_SYSTEM == 1)
    ;
#endif
    return x;
  }
};
// struct HodlrDgmres : ISolver<HodlrDgmres> {
//
//   inline static double dropTol = 1e-6;
//
// public:
//   auto setdropTol(double dropTol_) {
//     dropTol = dropTol_;
//     return *this;
//   }
//   HodlrDgmres() : ISolver<HodlrDgmres>() {}
//   template <typename MatrixType, typename VecType>
//   static Eigen::VectorXd solveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
//                                    const Eigen::MatrixBase<VecType> &rhs,
//                                    double tol = 1e-5,
//                                    std::size_t maxit = 1000) {
//     // std::cout << "Creating...\n";
//     using SpMat = Eigen::SparseMatrix<double>;
//     // Eigen::SparseMatrix<double> precond_mat(lhs.rows(), lhs.cols());
//     // precond_mat = lhs.sparseView(1e-2, 1);
//
//     Eigen::SparseMatrix<double> precond_mat(lhs.rows(), lhs.cols());
//     precond_mat = lhs.sparseView(dropTol, 1);
//
//     Eigen::IncompleteLUT<double> preconditioner(
//         precond_mat, NumTraits<double>::dummy_precision(), 1);
//     // print("Sparsity: ",
//
//     HodlrWrapper hodlr_wrapper(lhs, 64, tol * 0.1);
//     hodlr_wrapper.factorize();
//     // std::cout << "Solving...\n";
//     Eigen::VectorXd x(rhs.rows());
//     x.setZero();
//     Eigen::Index iters = 1000;
//     double errs = tol;
//     Eigen::internal::gmres(hodlr_wrapper, rhs, x, preconditioner, iters,
//     maxit,
//                            errs);
//     // std::cout << "#iterations:     " << iters << std::endl;
//     // std::cout << "estimated error: " << errs << std::endl;
//
// #if (SAVE_SYSTEM == 1)
//     ;
// #endif
//     return x;
//   }
// };
