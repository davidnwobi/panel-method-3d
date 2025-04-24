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

struct GMRESSolver : ISolver<GMRESSolver> {
  inline static float spTol = 1e-6;

public:
  GMRESSolver() : ISolver<GMRESSolver>() {}
  auto setspTol(float spTol_) {
    spTol = spTol_;
    return *this;
  }
  template <typename MatrixType, typename VecType>
  static Eigen::VectorXf solveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
                                   const Eigen::MatrixBase<VecType> &rhs,
                                   float tol = 1e-6,
                                   std::size_t maxit = 1000) {
    // std::cout << "Creating...\n";
    typedef Eigen::SparseMatrix<float> SpMat;
    typedef Eigen::Triplet<float> T;

    SpMat A(lhs.rows(), lhs.cols());
    A = lhs.sparseView(spTol, 1);
    A.makeCompressed();
    // A.setFromTriplets(tripletList.begin(), tripletList.end());
    // print("Sparsity: ",

    // printf("\n");
    // printf("%1.10f\n", spTol);
    // printf("%1.6f\n",
    //        ((float)A.nonZeros()) / ((float)(lhs.rows() * lhs.cols())));

    // printf("DropTol: %1.6f\n", dropTol);
    const Eigen::IdentityPreconditioner preconditioner;
    // print("Sparsity: ",

    // printf("DropTol: %1.6f\n", dropTol);
    // std::cout << "Solving...\n";
    Eigen::VectorXf x(rhs.rows());
    x.setZero();
    Eigen::Index iters = 1000;
    Eigen::internal::gmres(lhs, rhs, x, preconditioner, iters, maxit, tol);
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
