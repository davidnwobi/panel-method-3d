#pragma once
#include "solver/isolver.hpp"
#include <Eigen/Core>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <unsupported/Eigen/IterativeSolvers>
// #define ANALYSIS_DIR
// "D:/PortableDev/projects/panel_methods_3d/python/out_cpp"

struct GMRESILUSolver : ISolver {
  Eigen::VectorXd solve(const Eigen::MatrixXd &lhs, const Eigen::VectorXd &rhs,
                        double tol = 1e-6, std::size_t maxit = 10) override;
};
