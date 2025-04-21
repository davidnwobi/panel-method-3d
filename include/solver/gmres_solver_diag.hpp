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

struct GMRESDiagSolver : ISolver<GMRESDiagSolver> {

  inline static double spTol = 1e-6;

public:
  GMRESDiagSolver() : ISolver<GMRESDiagSolver>() { spTol = 1e-6; }
  auto setspTol(double spTol_) {
    spTol = spTol_;
    return *this;
  }

  template <typename MatrixType, typename VecType>
  static Eigen::VectorXd solveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
                                   const Eigen::MatrixBase<VecType> &rhs,
                                   double tol = 1e-6,
                                   std::size_t maxit = 1000) {
    // std::cout << "Creating...\n";
    typedef Eigen::SparseMatrix<double> SpMat;
    typedef Eigen::Triplet<double> T;

    SpMat A(lhs.rows(), lhs.cols());
    A = lhs.sparseView(spTol, 1);
    A.makeCompressed();

    // printf("\n");
    // printf("%1.10f\n", spTol);
    // printf("%1.6f\n",
    //        ((double)A.nonZeros()) / ((double)(lhs.rows() * lhs.cols())));
    // printf("DropTol: %1.6f\n", dropTol);
    const Eigen::DiagonalPreconditioner<double> preconditioner(A);
    // print("Sparsity: ",

    // printf("DropTol: %1.6f\n", dropTol);
    // std::cout << "Solving...\n";
    Eigen::VectorXd x(rhs.rows());
    x.setZero();
    Eigen::Index iters = 1000;

    Eigen::internal::gmres(A, rhs, x, preconditioner, iters, maxit, tol);
    // std::cout << "#iterations:     " << iters << std::endl;
    // std::cout << "estimated error: " << tol << std::endl;

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
