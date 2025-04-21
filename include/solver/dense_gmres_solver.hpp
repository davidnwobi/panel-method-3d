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
// "D:/PortableDev/projects/panel_methods_3d/python/out_cpp"

#define SAVE_SYSTEM 0
#define ITER_RES 0
using namespace Eigen;

struct DenseGMRESSolver : ISolver<DenseGMRESSolver> {

  inline static double spTol = 1e-6;

public:
  auto setspTol(double pTol_) {
    spTol = pTol_;
    return *this;
  }
  DenseGMRESSolver() : ISolver<DenseGMRESSolver>() {}
  template <typename MatrixType, typename VecType>
  static Eigen::VectorXd solveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
                                   const Eigen::MatrixBase<VecType> &rhs,
                                   double tol = 1e-6,
                                   std::size_t maxit = 1000) {
    // std::cout << "Creating...\n";
    using SpMat = Eigen::SparseMatrix<double>;
    Eigen::SparseMatrix<double> precond_mat(lhs.rows(), lhs.cols());
    precond_mat = lhs.sparseView(1e-2, 1);

    const Eigen::IdentityPreconditioner preconditioner;
    // print("Sparsity: ",

    // printf("DropTol: %1.6f\n", dropTol);
    // std::cout << "Solving...\n";
    Eigen::VectorXd x(rhs.rows());
    x.setZero();
    Eigen::Index iters = 1000;
    Eigen::internal::gmres(lhs, rhs, x, preconditioner, iters, maxit, tol);
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
