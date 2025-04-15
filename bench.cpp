#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <benchmark/benchmark.h>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <singularity/internal_functions.hpp>

static const float MULTIPLIER = 10;
using ul = long long;

template <typename Derived>
void convert(Eigen::MatrixBase<Derived> &dest,
             const Eigen::MatrixBase<Derived> &src,
             const Eigen::Isometry3f &conv_mat) {
  dest = (conv_mat * (src.matrix()));
}

void bench(benchmark::State &state) {
  ul no_points = state.range(0);
  Eigen::ArrayXf R12 = Eigen::ArrayXf::Zero(no_points, 1);
  Eigen::ArrayXf Q12 = Eigen::ArrayXf::Zero(no_points, 1);
  Eigen::ArrayXf J12 = Eigen::ArrayXf::Zero(no_points, 1);
  const Eigen::ArrayX3f points = Eigen::ArrayX3f::Random(no_points, 3);
  const Eigen::ArrayXf point1 = Eigen::ArrayXf::Random(3, 1);
  const Eigen::ArrayXf point2 = Eigen::ArrayXf::Random(3, 1);

  for (auto _ : state) {
    for (ul i = 0; i < no_points / 10; i++) {
      R12_Q12_J12(R12, Q12, J12, points, point1, point2);
    }
  }
  state.SetBytesProcessed((ul)state.iterations() * (ul)no_points * (ul)8);
}
void bench_cov(benchmark::State &state) {
  ul no_points = state.range(0);
  Eigen::Isometry3f conv_mat;
  conv_mat.linear() = Eigen::Matrix3f::Random();
  conv_mat.translation() = Eigen::Vector3f::Random();
  const Eigen::Matrix3Xf points = Eigen::Array3Xf::Random(3, no_points);
  Eigen::Matrix3Xf pointDest = Eigen::Array3Xf::Zero(3, no_points);

  const ul iter = 100;
  for (auto _ : state) {
    for (ul i = 0; i < iter; i++) {
      convert(pointDest, points, conv_mat);
    }
  }
  state.SetBytesProcessed((ul)state.iterations() * (ul)no_points * (ul)8 *
                          iter);
}

void fill_1(const Eigen::MatrixXf &lhs) {
  float lim = 5e-5;
  typedef Eigen::SparseMatrix<float> SpMat;
  typedef Eigen::Triplet<float> T;

  std::vector<T> tripletList;
  tripletList.reserve(lhs.rows() * lhs.cols());
  for (int i = 0; i < lhs.rows(); i++) {
    for (int j = 0; j < lhs.cols(); j++) {
      if (!std::isinf(lhs(i, j)) && !std::isnan(lhs(i, j)) &&
          std::abs(lhs(i, j)) > lim) {
        tripletList.push_back(T(i, j, lhs(i, j)));
      }
    }
  }
  SpMat A(lhs.rows(), lhs.cols());
  A.setFromTriplets(tripletList.begin(), tripletList.end());
}

void fill_2(const Eigen::MatrixXf &lhs) {
  float lim = 5e-5;
  typedef Eigen::SparseMatrix<float> SpMat;
  typedef Eigen::Triplet<float> T;

  std::vector<T> tripletList(lhs.rows() * lhs.cols());
  for (int j = 0; j < lhs.cols(); j++) {
    for (int i = 0; i < lhs.rows(); i++) {
      if (std::abs(lhs(i, j)) > lim) {
        tripletList[j * lhs.rows() + i] = T(i, j, lhs(i, j));
      }
    }
  }
  SpMat A(lhs.rows(), lhs.cols());
  A.setFromTriplets(tripletList.begin(), tripletList.end());
}

void fill_3(const Eigen::MatrixXf &lhs) {
  float lim = 5e-3;
  typedef Eigen::SparseMatrix<float> SpMat;
  typedef Eigen::Triplet<float> T;
  SpMat A = lhs.sparseView(lim);
}

void bench_fill(benchmark::State &state) {
  const int N_DIM = state.range(0);
  const Eigen::MatrixXf points =
      (Eigen::ArrayXXf::Ones(N_DIM, N_DIM) * 10) *
          (Eigen::ArrayXXf::Random(N_DIM, N_DIM) * 3) +
      3;

  for (auto _ : state) {
    fill_2(points);
  }
  state.SetBytesProcessed((ul)state.iterations() * (ul)(N_DIM * N_DIM) * (ul)8);
}

BENCHMARK(bench_fill)
    ->RangeMultiplier(4)
    ->Range(16 * 16, 64 * 64)
    ->DisplayAggregatesOnly(true);

BENCHMARK_MAIN();
// int main(){

//     prulf("%f\n", bench_1(65536*20000));
// }
