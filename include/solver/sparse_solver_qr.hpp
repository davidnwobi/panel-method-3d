#pragma once
#include "solver/isolver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/OrderingMethods>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include <iostream>
#include <vector>
// #define ANALYSIS_DIR
// "D:/PortableDev/projects/panel_methods_3d/python/out_cpp"
struct SparseSolverQR : ISolver {
  Eigen::VectorXd solve(const Eigen::MatrixXd &lhs, const Eigen::VectorXd &rhs,
                        double tol = 1e-6, std::size_t maxit = 10) override {
    // FileReaderFactory::make_file_reader("dat", " ",
    // true)->save_data(std::string(ANALYSIS_DIR) + "/infMat.dat", lhs);
    double lim = 1e-6;
#if (BENCHMARKING == 0)
    std::cout << "Creating...\n";
#endif

    typedef Eigen::SparseMatrix<double> SpMat;
    SpMat A = lhs.sparseView(lim);

    Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int>> solver;
#if (BENCHMARKING == 0)
    std::cout << "Computing...\n";
#endif
    solver.compute(A);
#if (BENCHMARKING == 0)
    std::cout << "Solving...\n";
#endif
    return solver.solve(rhs);
  }
};
