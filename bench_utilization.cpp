
// #define EIGEN_USE_MKL_ALL
#include "solver/dense_gmres_solver.hpp"
#include "solver/dense_gmres_solver_ilu.hpp"
#include "solver/dense_solver.hpp"
#include "solver/gmres_solver.hpp"
#include "solver/gmres_solver_ilu.hpp"
#include "solver/sparse_solver.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/Sparse>
#include <array>
#include <benchmark/benchmark.h>
#include <cstdio>
#include <cstdlib>
#include <unsupported/Eigen/SparseExtra>

#define I 2
static const double MULTIPLIER = 10;
using ul = long long;
class MyFixture : public benchmark::Fixture {
public:
  Eigen::MatrixXd lhs;
  Eigen::VectorXd rhs;
  double tol = 1e-6;
  double maxit = 1000;
  double spTol = 1e-6;
  const std::array<double, 7> dropTols = {1e-1, 3e-2, 1e-2, 3e-3,
                                          1e-3, 3e-4, 1e-4};
  void SetUp(::benchmark::State &state) {
    double spTols[] = {5e-5, 1e-5, 1e-6, 3.125e-6, 1e-5};
    std::string mats[] = {"c1", "c2", "c3", "swept_wing", "canardTest"};
    spTol = spTols[I];
    // dropTol << 1e-1, 3e-2, 1e-2, 3e-3, 1e-3, 3e-4, 1e-4;
    const std::string base = "../../";
    Eigen::SparseMatrix<double> spmat;
    Eigen::loadMarketDense(lhs, base + mats[I] + "/lhs.txt");
    Eigen::loadMarketDense(rhs, base + mats[I] + "/rhs.txt");
  }

  void TearDown(::benchmark::State &state) {}
};

BENCHMARK_DEFINE_F(MyFixture, bench_dense_gmres_ilu)
(benchmark::State &state) {
  using SpMat = Eigen::SparseMatrix<double>;
  Eigen::SparseMatrix<double> precond_mat(lhs.rows(), lhs.cols());
  precond_mat = lhs.sparseView(this->dropTols[state.range(0)], 1);
  precond_mat.makeCompressed();
  const Eigen::IncompleteLUT<double> preconditioner(
      precond_mat, NumTraits<double>::dummy_precision(), 1);
  Eigen::VectorXd x(rhs.rows());
  for (auto _ : state) {
    double err = tol;
    Eigen::Index iters = 1000;
    x.setZero();
    Eigen::internal::gmres(lhs, rhs, x, preconditioner, iters, maxit, err);
  }

  state.SetBytesProcessed((ul)state.iterations() *
                          (ul)(lhs.rows() * lhs.cols()) * (ul)16);
}
// BENCHMARK_F(MyFixture, bench_dense_gmres_ilu)
// (benchmark::State &state) {
//   using SpMat = Eigen::SparseMatrix<double>;
//   Eigen::SparseMatrix<double> precond_mat(lhs.rows(), lhs.cols());
//   precond_mat = lhs.sparseView(3e-3, 1);
//   const Eigen::IncompleteLUT<double> preconditioner(
//       precond_mat, NumTraits<double>::dummy_precision(), 1);
//   Eigen::VectorXd x(rhs.rows());
//   int n = lhs.rows();
//   for (auto _ : state) {
//     VectorXd x(n);
//     double err = tol;
//     Eigen::Index iters = n;
//     x.setZero();
//     Eigen::internal::gmres(lhs, rhs, x, preconditioner, iters, maxit, err);
//   }
//
//   state.SetBytesProcessed((ul)state.iterations() *
//                           (ul)(lhs.rows() * lhs.cols()) * (ul)16);
// }
//
// BENCHMARK_F(MyFixture, dense_lu)
// (benchmark::State &state) {
//
//   Eigen::VectorXd x;
//   for (auto _ : state) {
//     x = lhs.lu().solve(rhs);
//   }
//
//   state.SetBytesProcessed((ul)state.iterations() *
//                           (ul)(lhs.rows() * lhs.cols()) * (ul)16);
// }
//
// BENCHMARK_REGISTER_F(MyFixture, bench_dense_gmres_ilu)->DenseRange(0, 6, 1);
// BENCHMARK_REGISTER_F(MyFixture, bench_sparse_gmres_ilu)->DenseRange(0, 6,
// 1);
BENCHMARK_MAIN();
// int main(){

//     prulf("%f\n", bench_1(65536*20000));
// }
