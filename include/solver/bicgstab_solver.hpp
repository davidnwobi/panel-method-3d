#pragma once
#include "solver/isolver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <iostream>
#include <unsupported/Eigen/IterativeSolvers>
#include <vector>
// #define ANALYSIS_DIR
// "D:/PortableDev/projects/panel_methods_3d/python/out_cpp"

struct BICGSTABSolver : ISolver {
  Eigen::VectorXd solve(const Eigen::MatrixXd &lhs, const Eigen::VectorXd &rhs,
                        double tol = 1e-6, std::size_t maxit = 10) override {
    double lim = 5e-6;
    std::cout << "Creating...\n";

    typedef Eigen::SparseMatrix<double> SpMat;

    typedef Eigen::SparseMatrix<double> SpMat;
    SpMat A = lhs.sparseView(lim);

    Eigen::BiCGSTAB<SpMat> solver(A);
    Eigen::VectorXd x = rhs;
    std::cout << "Solving...\n";
    x = solver.solve(rhs);
    std::cout << "#iterations:     " << solver.iterations() << std::endl;
    std::cout << "estimated error: " << solver.error() << std::endl;

    return x;
  }
};
