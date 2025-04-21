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
// "D:/PortableDev/projects/panel_methods_3d/python/out_cpp"
struct SparseSolver : ISolver<SparseSolver> {

  inline static double spTol = 1e-6;

public:
  SparseSolver() : ISolver<SparseSolver>() {}
  auto setspTol(double spTol_) {
    spTol = spTol_;
    return *this;
  }
  template <typename MatrixType, typename VecType>
  static Eigen::VectorXd solveImpl(const Eigen::MatrixBase<MatrixType> &lhs,
                                   const Eigen::MatrixBase<VecType> &rhs,
                                   double tol = 1e-6, std::size_t maxit = 10) {
    // FileReaderFactory::make_file_reader("dat", " ",
    // true)->save_data(std::string(ANALYSIS_DIR) + "/infMat.dat", lhs);
    typedef Eigen::SparseMatrix<double> SpMat;
    typedef Eigen::Triplet<double> T;

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
