// #define EIGEN_USE_MKL_ALL
#include "solver/dense_gmres_solver.hpp"
#include "solver/dense_gmres_solver_ilu.hpp"
#include "solver/dense_solver.hpp"
#include "solver/gmres_solver.hpp"
#include "solver/gmres_solver_ilu.hpp"
#include "solver/hodlr_dgmres.hpp"
#include "solver/hodlr_solver.hpp"
#include "solver/sparse_solver.hpp"
#include "solver/sparse_solver_umfpack.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <array>
#include <benchmark/benchmark.h>
#include <cstdio>
#include <cstdlib>
#include <unsupported/Eigen/SparseExtra>

#define I 0
static const float MULTIPLIER = 10;
using ul = long long;
class MyFixture : public benchmark::Fixture {
public:
  Eigen::MatrixXf lhs;
  Eigen::VectorXf rhs;
  float maxit = 1000;
  float dropTol = 1e-4;
  float spTol = 1e-4;
  float tol = 1e-6;
  // const std::array<float, 7> dropTols = {1e-1, 3e-2, 1e-2, 3e-3,
  //                                         1e-3, 3e-4, 1e-4};
  void SetUp(::benchmark::State &state) {
    float dpTols[] = {1e-2, 1e-2, 3e-3, 1e-3, 3e-3};
    float spTols[] = {1e-5, 1e-5, 1e-6, 3.125e-6, 1e-5};
    std::string mats[] = {"c1", "c2", "c3", "swept_wing", "canardTest"};
    spTol = spTols[I];
    dropTol = dpTols[I];
    // dropTol << 1e-1, 3e-2, 1e-2, 3e-3, 1e-3, 3e-4, 1e-4;
    const std::string base = "../../";
    Eigen::SparseMatrix<float> spmat;
    Eigen::loadMarketDense(lhs, base + mats[I] + "/lhs.txt");
    Eigen::loadMarketDense(rhs, base + mats[I] + "/rhs.txt");
  }

  void TearDown(::benchmark::State &state) {}
};

#ifdef EIGEN_USE_MKL_ALL

BENCHMARK_F(MyFixture, dense_solver)
(benchmark::State &state) {

  Eigen::VectorXf x;
  for (auto _ : state) {

    DenseSolver solver;
    solver.solve(this->lhs, this->rhs);
  }

  state.SetBytesProcessed((ul)state.iterations() *
                          (ul)(lhs.rows() * lhs.cols()) * (ul)16);
}
BENCHMARK_F(MyFixture, sparse_lu)(benchmark::State &st) {
  for (auto _ : st) {
    SparseSolver solver;
    solver.setspTol(this->spTol);
    solver.solve(this->lhs, this->rhs);
  }
  st.SetBytesProcessed((ul)st.iterations() * (ul)(lhs.rows() * lhs.cols()) *
                       (ul)16);
}

BENCHMARK_F(MyFixture, hodlr)(benchmark::State &st) {
  for (auto _ : st) {
    HODLRSolver solver;
    solver.solve(this->lhs, this->rhs, this->spTol);
  }
  st.SetBytesProcessed((ul)st.iterations() * (ul)(lhs.rows() * lhs.cols()) *
                       (ul)16);
}
BENCHMARK_F(MyFixture, hodlr_dmgres)(benchmark::State &st) {
  for (auto _ : st) {
    HodlrDgmres solver;
    solver.setdropTol(this->dropTol);
    solver.setspTol(this->spTol);
    solver.solve(this->lhs, this->rhs, this->tol);
  }
  st.SetBytesProcessed((ul)st.iterations() * (ul)(lhs.rows() * lhs.cols()) *
                       (ul)16);
}
#endif

#ifndef EIGEN_USE_MKL_ALL
BENCHMARK_F(MyFixture, dense_gmres)(benchmark::State &st) {
  for (auto _ : st) {
    DenseGMRESSolver solver;
    solver.solve(this->lhs, this->rhs, this->tol);
  }
  st.SetBytesProcessed((ul)st.iterations() * (ul)(lhs.rows() * lhs.cols()) *
                       (ul)16);
}

BENCHMARK_F(MyFixture, dense_gmres_ilu)(benchmark::State &st) {
  for (auto _ : st) {
    DenseGMRESILUSolver solver;
    solver.setdropTol(this->dropTol);
    solver.solve(this->lhs, this->rhs, this->tol);
  }
  st.SetBytesProcessed((ul)st.iterations() * (ul)(lhs.rows() * lhs.cols()) *
                       (ul)16);
}

BENCHMARK_F(MyFixture, gmres)(benchmark::State &st) {
  for (auto _ : st) {
    GMRESSolver solver;
    solver.setspTol(this->spTol);
    solver.solve(this->lhs, this->rhs, this->tol);
  }
  st.SetBytesProcessed((ul)st.iterations() * (ul)(lhs.rows() * lhs.cols()) *
                       (ul)16);
}
BENCHMARK_F(MyFixture, GMRES_ILU)(benchmark::State &st) {
  for (auto _ : st) {
    GMRESILUSolver solver;
    solver.setspTol(this->spTol);
    solver.setdropTol(this->dropTol);
    solver.solve(this->lhs, this->rhs, this->tol);
  }
  st.SetBytesProcessed((ul)st.iterations() * (ul)(lhs.rows() * lhs.cols()) *
                       (ul)16);
}
#endif

BENCHMARK_MAIN();
