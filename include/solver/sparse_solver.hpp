#pragma once
#include "solver/isolver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/PardisoSupport>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>
#include <iostream>
#include <vector>
// #define ANALYSIS_DIR
// "D:/PortableDev/projects/panel_methods_3f/python/out_cpp"
struct SparseSolver : ISolver<SparseSolver> {

  inline static float spTol = 1e-6;

public:
  SparseSolver() : ISolver<SparseSolver>() {}
  auto setspTol(float spTol_) {
    spTol = spTol_;
    return *this;
  }
  template <typename MatrixType, typename VecType>
  static Eigen::VectorXf solveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
                                   const Eigen::MatrixBase<VecType> &rhs,
                                   float tol = 1e-6, std::size_t maxit = 10) {
    // FileReaderFactory::make_file_reader("dat", " ",
    // true)->save_data(std::string(ANALYSIS_DIR) + "/infMat.dat", lhs);
    typedef Eigen::SparseMatrix<float> SpMat;
    typedef Eigen::Triplet<float> T;

    SpMat A(lhs.rows(), lhs.cols());
    A = lhs.sparseView(spTol, 1);
    A.makeCompressed();
#ifdef EIGEN_USE_MKL
    Eigen::PardisoLU<SpMat> solver;
#else
    Eigen::SparseLU<SpMat> solver;
#endif
    solver.compute(A);
    return solver.solve(rhs);
  }
};
