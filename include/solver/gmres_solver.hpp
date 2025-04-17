#pragma once
#include "solver/isolver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <cmath>
#include <iostream>
#include <unsupported/Eigen/IterativeSolvers>
#include <unsupported/Eigen/SparseExtra>
#include <vector>
// #define ANALYSIS_DIR
// "D:/PortableDev/projects/panel_methods_3d/python/out_cpp"

#define SAVE_SYSTEM 1
struct GMRESSolver : ISolver {
  Eigen::VectorXf solve(const Eigen::MatrixXf &lhs, const Eigen::VectorXf &rhs,
                        float tol = 1e-6, std::size_t maxit = 1000) override {
    float lim = 1e-6;
    std::cout << "Creating...\n";
    typedef Eigen::SparseMatrix<float> SpMat;

    SpMat A = lhs.sparseView(lim);
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
