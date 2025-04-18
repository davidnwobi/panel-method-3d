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

#define SAVE_SYSTEM 1
struct GMRESSolver : ISolver {
private:
  float dropTol = 1e-6;

public:
  auto setdropTol(float dropTol_) {
    dropTol = dropTol_;
    return *this;
  }
  Eigen::VectorXf solve(const Eigen::MatrixXf &lhs, const Eigen::VectorXf &rhs,
                        float tol = 1e-6, std::size_t maxit = 1000) override {
    std::cout << "Creating...\n";
    typedef Eigen::SparseMatrix<float> SpMat;
    typedef Eigen::Triplet<float> T;
    // SpMat A = lhs.sparseView(dropTol);
    // Eigen::GMRES<SpMat> solver(A);
    std::vector<T> tripletList;
    tripletList.reserve(lhs.rows() * lhs.cols());
    for (int i = 0; i < lhs.rows(); i++) {
      for (int j = 0; j < lhs.cols(); j++) {
        if (std::abs(lhs(i, j)) > dropTol) {
          tripletList.push_back(T(i, j, lhs(i, j)));
        }
      }
    }

    SpMat A(lhs.rows(), lhs.cols());
    // A = lhs.sparseView(dropTol, dropTol);
    A.setFromTriplets(tripletList.begin(), tripletList.end());
    print("Sparsity: ",
          ((double)A.nonZeros()) / ((double)(lhs.rows() * lhs.cols())));

    printf("DropTol: %1.6f\n", dropTol);
    Eigen::GMRES<SpMat> solver(A);
    solver.setTolerance(tol);
    solver.setMaxIterations(maxit);
    Eigen::VectorXf x = rhs;
    std::cout << "Solving...\n";
    x = solver.solve(rhs);
    std::cout << "#iterations:     " << solver.iterations() << std::endl;
    std::cout << "estimated error: " << solver.error() << std::endl;

#if (SAVE_SYSTEM == 1)
    Eigen::saveMarket(A, "lhs.txt");
    Eigen::saveMarketDense(rhs, "rhs.txt");
    Eigen::saveMarketDense(x, "solution.txt");
#endif
    return x;
  }
};
