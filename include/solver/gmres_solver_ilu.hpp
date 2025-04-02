#pragma once
#include "solver/isolver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <unsupported/Eigen/IterativeSolvers>
#include <iostream>
#include <vector>
// #define ANALYSIS_DIR
// "D:/PortableDev/projects/panel_methods_3d/python/out_cpp"

struct GMRESILUSolver : ISolver {
  Eigen::VectorXd solve(const Eigen::MatrixXd &lhs,
                        const Eigen::VectorXd &rhs, double tol=1e-6, std::size_t maxit = 10) override {
    double lim = 1e-8;
    std::cout << "Creating...\n";

    typedef Eigen::SparseMatrix<double> SpMat;
    typedef Eigen::Triplet<double> T;

    std::vector<T> tripletList;
    tripletList.reserve(lhs.rows() * lhs.cols());
    for (int i = 0; i < lhs.rows(); i++) {
      for (int j = 0; j < lhs.cols(); j++) {
        if (std::abs(lhs(i, j)) > lim) {
          tripletList.push_back(T(i, j, lhs(i, j)));
        }
      }
    }

    SpMat A(lhs.rows(), lhs.cols());
    A.setFromTriplets(tripletList.begin(), tripletList.end());
    Eigen::GMRES<SpMat, Eigen::IncompleteLUT<double>> solver(A);
    Eigen::VectorXd x = rhs;
    std::cout << "Solving...\n";
    x = solver.solve(rhs);
    std::cout << "#iterations:     " << solver.iterations() << std::endl;
    std::cout << "estimated error: " << solver.error()      << std::endl;
     
    return x;
  }
};
