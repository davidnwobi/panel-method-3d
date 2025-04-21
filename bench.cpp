#include "utils/utils.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Sparse>
#include <benchmark/benchmark.h>
#include <cstdio>
#include <cstdlib>
#include <immintrin.h>
#include <memory>
#include <singularity/internal_functions.hpp>
#include <sleef.h>

static const double MULTIPLIER = 10;
using ul = long long;

// #define _LOAD(src) _mm256_load_ps(src)
// #define _STORE(dest, v1) _mm256_store_ps(dest, v1)
// #define __VECTOR_S __m256
// #define __VECTOR_I __m256i
// #define _ADD(v1, v2) _mm256_add_ps(v1, v2)
// #define _SUB(v1, v2) _mm256_sub_ps(v1, v2)
// #define _MUL(v1, v2) _mm256_mul_ps(v1, v2)
// #define _DIV(v1, v2) _mm256_div_ps(v1, v2)
// #define _CONST_VEC(v1) _mm256_set1_ps(v1)
// #define _COPY_SIGN(v1, v2) Sleef_copysignf8(v1, v2)
// #define _ZERO() _mm256_setzero_ps()
// #define _GT(v1, v2) _mm256_cmp_ps(v1, v2, _CMP_GT_OQ)
// #define _LT(v1, v2) _mm256_cmp_ps(v1, v2, _CMP_LT_OQ)
// #define _FABS(v1) Sleef_fabsf8(v1)
// #define _SELECT_F(false, true, mask) _mm256_blendv_ps(false, true, mask)
// #define _HYPOPT(v1, v2) Sleef_hypotf8_u05avx2(v1, v2)
// #define _SQR(v1) _MUL(v1, v1)
// #define _SQRT(v1) Sleef_sqrtf8_u05avx2(v1)
// #define _LOG(v1) Sleef_logf8_u10avx2(v1)
// #define _ATAN(v1) Sleef_atanf8_u10avx2(v1)
//
// void R12_Q12_J12_AVX2(double *R12_, double *Q12_, double *J12_, double *x,
// double *y,
//                       double *z, double *node1, double *node2, size_t N) {
//   const size_t PACK = 8;
//   // Preliminary Constants
//   const double x1f = node1[0], y1f = node1[1];
//   const double x2d = node2[0], y2d = node2[1];
//   const double m12d = slope<double>(x1f, y1f, x2d, y2d);
//
//   const double dxf = node2[0] - node1[0];
//   const double dyf = node2[1] - node1[1];
//   const double df = std::sqrt(dxf * dxf + dyf * dyf);
//
//   __VECTOR_S x1 = _CONST_VEC(x1f);
//   __VECTOR_S y1 = _CONST_VEC(y1f);
//
//   __VECTOR_S x2 = _CONST_VEC(x2d);
//   __VECTOR_S y2 = _CONST_VEC(y2d);
//
//   // Slope function
//   __VECTOR_S dx = _CONST_VEC(dxf);
//   __VECTOR_S dy = _CONST_VEC(dyf);
//   __VECTOR_S m12 = _CONST_VEC(m12d);
//   __VECTOR_S d = _CONST_VEC(df);
//
//   for (size_t i = 0; i < N / PACK; ++i) {
//     __VECTOR_S px = _LOAD(x);
//     __VECTOR_S py = _LOAD(y);
//     __VECTOR_S pz = _LOAD(z);
//
//     // Compute R12
//     __VECTOR_S r12 =
//         _DIV(_SUB(_MUL(_SUB(px, x1), dy), _MUL(_SUB(py, y1), dx)), d);
//     _STORE(R12_, r12);
//
//     __VECTOR_S dx1 = _SUB(px, x1);
//     __VECTOR_S dy1 = _SUB(py, y1);
//     __VECTOR_S e1 = _ADD(_SQR(dx1), _SQR(pz));
//     __VECTOR_S h1 = _MUL(dx1, dy1);
//     __VECTOR_S r1 = _SQRT(_ADD(_ADD(_SQR(dx1), _SQR(dy1)), _SQR(pz)));
//
//     __VECTOR_S dx2 = _SUB(px, x2);
//     __VECTOR_S dy2 = _SUB(py, y2);
//     __VECTOR_S e2 = _ADD(_SQR(dx2), _SQR(pz));
//     __VECTOR_S h2 = _MUL(dx2, dy2);
//     __VECTOR_S r2 = _SQRT(_ADD(_ADD(_SQR(dx2), _SQR(dy2)), _SQR(pz)));
//
//     __VECTOR_S q12 = _LOG(_DIV(_ADD(_ADD(r1, r2), d), _SUB(_ADD(r1, r2),
//     d))); _STORE(Q12_, q12);
//
//     __VECTOR_S a1 = _DIV(_SUB(_MUL(m12, e1), h1), _MUL(pz, r1));
//     __VECTOR_S a2 = _DIV(_SUB(_MUL(m12, e2), h2), _MUL(pz, r2));
//
//     __VECTOR_S j12 = _SUB(_ATAN(a1), _ATAN(a2));
//     _STORE(J12_, j12);
//
//     x += PACK;
//     y += PACK;
//     z += PACK;
//     R12_ += PACK;
//     Q12_ += PACK;
//     J12_ += PACK;
//   }
//
//   for (size_t i = 0; i < N % PACK; ++i) {
//     const double px = x[i];
//     const double py = y[i];
//     const double pz = z[i];
//
//     R12_[i] = ((px - node1[0]) * dyf - (py - node1[1]) * dxf) / df;
//
//     // For node1:
//     const double dx1 = px - x1f;
//     const double dy1 = py - y1f;
//     const double e1 = dx1 * dx1 + pz * pz; // ek(node1)
//     const double h1 = dx1 * dy1;           // hk(node1)
//     const double r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));
//
//     // For node2:
//     const double dx2 = px - x2d;
//     const double dy2 = py - y2d;
//     const double e2 = dx2 * dx2 + pz * pz; // ek(node2)
//     const double h2 = dx2 * dy2;           // hk(node2)
//     const double r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));
//
//     Q12_[i] = std::log((r1 + r2 + df) / (r1 + r2 - df));
//
//     const double a1 = (m12d * e1 - h1) / (pz * r1);
//     const double a2 = (m12d * e1 - h1) / (pz * r1);
//     J12_[i] = std::atan(a1) - std::atan(a2);
//   }
// }
//
// void R12_Q12_J12_NORM(double *R12_, double *Q12_, double *J12_, double *x,
// double *y,
//                       double *z, double *node1, double *node2, size_t N) {
//   const double x1 = node1[0], y1 = node1[1];
//   const double x2 = node2[0], y2 = node2[1];
//   const double m12 = slope<double>(x1, y1, x2, y2);
//
//   const double dx = node2[0] - node1[0];
//   const double dy = node2[1] - node1[1];
//   const double d = std::sqrt(dx * dx + dy * dy);
//
//   for (Eigen::Index i = 0; i < N; ++i) {
//     const double px = x[i];
//     const double py = y[i];
//     const double pz = z[i];
//
//     R12_[i] = ((px - node1[0]) * dy - (py - node1[1]) * dx) / d;
//
//     // For node1:
//     const double dx1 = px - x1;
//     const double dy1 = py - y1;
//     const double e1 = dx1 * dx1 + pz * pz; // ek(node1)
//     const double h1 = dx1 * dy1;           // hk(node1)
//     const double r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));
//
//     // For node2:
//     const double dx2 = px - x2;
//     const double dy2 = py - y2;
//     const double e2 = dx2 * dx2 + pz * pz; // ek(node2)
//     const double h2 = dx2 * dy2;           // hk(node2)
//     const double r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));
//
//     Q12_[i] = std::log((r1 + r2 + d) / (r1 + r2 - d));
//
//     auto termP = [&](double m, double e, double h, double rr) {
//       // if pz=0, you might want to handle that carefully
//       const double denom = pz * rr;
//       return std::atan((m * e - h) / denom);
//     };
//     J12_[i] = termP(m12, e1, h1, r1) - termP(m12, e2, h2, r2);
//   }
//   // printf("m12 %f\n", m12);
//   // printf("d %f\n", d);
// }
void bench_kernel(benchmark::State &state) {
  const size_t N = state.range(0);
  double *R12, *Q12, *J12, *R12_, *Q12_, *J12_, *x, *y, *z, *node1, *node2;
  const size_t N_BYTES = N * sizeof(double);
  const size_t ALIGNMENT = 32;
  R12 = (double *)aligned_alloc(ALIGNMENT, N_BYTES);
  Q12 = (double *)aligned_alloc(ALIGNMENT, N_BYTES);
  J12 = (double *)aligned_alloc(ALIGNMENT, N_BYTES);
  R12_ = (double *)aligned_alloc(ALIGNMENT, N_BYTES);
  Q12_ = (double *)aligned_alloc(ALIGNMENT, N_BYTES);
  J12_ = (double *)aligned_alloc(ALIGNMENT, N_BYTES);
  x = (double *)aligned_alloc(ALIGNMENT, N_BYTES);
  y = (double *)aligned_alloc(ALIGNMENT, N_BYTES);
  z = (double *)aligned_alloc(ALIGNMENT, N_BYTES);
  node1 = (double *)aligned_alloc(ALIGNMENT, ALIGNMENT);
  node2 = (double *)aligned_alloc(ALIGNMENT, ALIGNMENT);

  srand(time(NULL));
  for (int i = 0; i < N; i++) {
    x[i] = rand();
    y[i] = rand();
    z[i] = rand();
  }
  for (int i = 0; i < 3; i++) {
    node1[i] = rand();
    node2[i] = rand();
  }
  // R12_Q12_J12_NORM(R12, Q12, J12, x, y, z, node1, node2, N);
  // R12_Q12_J12_AVX2(R12_, Q12_, J12_, x, y, z, node1, node2, N);

  for (auto _ : state) {
    R12_Q12_J12_NORM(R12_, Q12_, J12_, x, y, z, node1, node2, N);
  }
  state.SetBytesProcessed((ul)state.iterations() * (ul)N * (ul)8);
}

void bench_kernel2(benchmark::State &state) {
  const size_t N = state.range(0);
  double *node1, *node2;
  const size_t N_BYTES = N * sizeof(double);
  const size_t ALIGNMENT = 32;
  Eigen::ArrayX3d points = Eigen::ArrayX3d::Random(N, 3);
  Eigen::ArrayXd R12_(N);
  Eigen::ArrayXd Q12_(N);
  Eigen::ArrayXd J12_(N);
  Eigen::Array3Xd tPoints = Eigen::Array3Xd::Random(3, 4);

  double *r12 = const_cast<double *>(R12_.data());
  double *q12 = const_cast<double *>(Q12_.data());
  double *j12 = const_cast<double *>(J12_.data());
  double *x = const_cast<double *>(points.data());
  double *y = const_cast<double *>(points.data() + N);
  double *z = const_cast<double *>(points.data() + N * 2);
  node1 = const_cast<double *>(tPoints.data());
  node2 = const_cast<double *>(tPoints.data() + 3);
  srand(time(NULL));
  // R12_Q12_J12_NORM(R12, Q12, J12, x, y, z, node1, node2, N);
  // R12_Q12_J12_AVX2(R12_, Q12_, J12_, x, y, z, node1, node2, N);

  for (auto _ : state) {
    // R12_Q12_J12_NORM(r12, q12, j12, x, y, z, node1, node2, N);
    R12_Q12_J12(R12_, Q12_, J12_, points, tPoints.col(0), tPoints.col(1));
  }
  state.SetBytesProcessed((ul)state.iterations() * (ul)N * (ul)8);
}

BENCHMARK(bench_kernel2)
    ->RangeMultiplier(2)
    ->Range(2 << 4, 2 << 20)
    ->DisplayAggregatesOnly(true);

BENCHMARK_MAIN();
// int main(){

//     prulf("%f\n", bench_1(65536*20000));
// }
