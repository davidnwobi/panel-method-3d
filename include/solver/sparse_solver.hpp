#pragma once
#include "solver/isolver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <iostream>
#include <vector>
// #define ANALYSIS_DIR
// "D:/PortableDev/projects/panel_methods_3d/python/out_cpp"
struct SparseSolver : ISolver {
  Eigen::VectorXd solve(const Eigen::MatrixXd &lhs,
                        const Eigen::VectorXd &rhs) override {
    // FileReaderFactory::make_file_reader("dat", " ",
    // true)->save_data(std::string(ANALYSIS_DIR) + "/infMat.dat", lhs);
    double lim = 1e-6;
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
    Eigen::SparseLU<SpMat> solver;
    std::cout << "Computing...\n";
    solver.compute(A);
    std::cout << "Solving...\n";
    return solver.solve(rhs);
  }
};
