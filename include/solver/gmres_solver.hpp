#pragma once
#include "solver/isolver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <cmath>
#include <iostream>
#include <unsupported/Eigen/IterativeSolvers>
#include <vector>
// #define ANALYSIS_DIR
// "D:/PortableDev/projects/panel_methods_3d/python/out_cpp"

struct GMRESSolver {
  Eigen::VectorXd solve(const Eigen::Ref<const Eigen::MatrixXd> &lhs,
                        const Eigen::Ref<const Eigen::VectorXd> &rhs,
                        double tol = 1e-6, std::size_t maxit = 1000) {
    double lim = 1e-6;
    std::cout << "Creating...\n";
    typedef Eigen::SparseMatrix<double> SpMat;

    SpMat A = lhs.sparseView(lim);

    Eigen::GMRES<SpMat> solver(A);
    solver.setTolerance(tol);
    solver.setMaxIterations(maxit);
    Eigen::VectorXd x = rhs;
    std::cout << "Solving...\n";
    x = solver.solve(rhs);
    std::cout << "#iterations:     " << solver.iterations() << std::endl;
    std::cout << "estimated error: " << solver.error() << std::endl;

    return x;
  }
};
