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
  Eigen::VectorXf solve(const Eigen::MatrixXf &lhs,
                        const Eigen::VectorXf &rhs, float tol=1e-6, std::size_t maxit = 10) override {
    // FileReaderFactory::make_file_reader("dat", " ",
    // true)->save_data(std::string(ANALYSIS_DIR) + "/infMat.dat", lhs);
    float lim = 1e-10;
#if (BENCHMARKING == 0)
    std::cout << "Creating...\n";
#endif
    typedef Eigen::SparseMatrix<float> SpMat;
    typedef Eigen::Triplet<float> T;

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
