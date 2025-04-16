#include "singularity/internal_functions.hpp"
#include "fast_atan/atan_approx.h"
#include "fast_log/log_approx.h"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <cmath>

#define CLAMP_TO 1e9

using namespace Eigen;
using RowArray3f = Eigen::Array<float, 1, 3, Eigen::RowMajor>;
Eigen::ArrayXf R12(const Eigen::Ref<const Eigen::ArrayX3f> &points,
                   const Eigen::Ref<const Eigen::Array3f> &node1,
                   const Eigen::Ref<const Eigen::Array3f> &node2) {
  const float dx = node2(0) - node1(0);
  const float dy = node2(1) - node1(1);
  // 2D distance between node1 and node2
  const float d = std::sqrt(dx * dx + dy * dy);

  Eigen::ArrayXf out(points.rows());

  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    const float px = points(i, 0);
    const float py = points(i, 1);

    // ( (px - node1.x)*dy - (py - node1.y)*dx ) / d
    out[i] = ((px - node1(0)) * dy - (py - node1(1)) * dx) / d;
  }

  return out;
}

Eigen::ArrayXf Q12(const Eigen::Ref<const Eigen::ArrayX3f> &points,
                   const Eigen::Ref<const Eigen::Array3f> &node1,
                   const Eigen::Ref<const Eigen::Array3f> &node2) {
  const float x1 = node1(0), y1 = node1(1);
  const float x2 = node2(0), y2 = node2(1);

  const float dx = node2(0) - node1(0);
  const float dy = node2(1) - node1(1);
  // 2D distance between node1 and node2
  const float d = std::sqrt(dx * dx + dy * dy);

  Eigen::ArrayXf out(points.rows());
  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    float px = points(i, 0);
    float py = points(i, 1);
    float pz = points(i, 2);

    const float dx1 = px - x1;
    const float dy1 = py - y1;
    const float r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));

    const float dx2 = px - x2;
    const float dy2 = py - y2;
    const float r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));

    out[i] = std::log((r1 + r2 + d) / (r1 + r2 - d));
  }
  return out;
}

Eigen::ArrayXf J12(const Eigen::Ref<const Eigen::ArrayX3f> &points,
                   const Eigen::Ref<const Eigen::Array3f> &node1,
                   const Eigen::Ref<const Eigen::Array3f> &node2) {
  const float x1 = node1(0), y1 = node1(1);
  const float x2 = node2(0), y2 = node2(1);

  // "m" slope function
  auto slope = [&](float xA, float yA, float xB, float yB) {
    const float dx = xB - xA;
    const float dy = yB - yA;
    // handle near‐vertical or near‐horizontal
    if (std::fabs(dx) < 1e-14) {
      return sgn(dy) * std::numeric_limits<float>::infinity();
    }
    if (std::fabs(dy) < 1e-14) {
      return 0.0f;
    }
    return dy / dx;
  };

  // precompute slope for the two nodes
  const float m12 = slope(x1, y1, x2, y2);

  // We'll accumulate the difference
  Eigen::ArrayXf out(points.rows());

  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    const float px = points(i, 0);
    const float py = points(i, 1);
    const float pz = points(i, 2);

    // ek(faceV) = (px - faceV.x)^2 + pz^2
    // hk(faceV) = (px - faceV.x)*(py - faceV.y)
    // r(faceV)  = sqrt( (px-faceV.x)^2 + (py-faceV.y)^2 + (pz-faceV.z)^2 )

    // For node1:
    const float dx1 = px - x1;
    const float dy1 = py - y1;
    const float e1 = dx1 * dx1 + pz * pz; // ek(node1)
    const float h1 = dx1 * dy1;           // hk(node1)
    const float r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));

    // For node2:
    const float dx2 = px - x2;
    const float dy2 = py - y2;
    const float e2 = dx2 * dx2 + pz * pz; // ek(node2)
    const float h2 = dx2 * dy2;           // hk(node2)
    const float r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));

    // termP(m, e, h, r) = atan( (m*e - h) / (pz*r) )
    // watch for pz=0.0?
    auto termP = [&](float m, float e, float h, float rr) {
      // if pz=0, you might want to handle that carefully
      const float denom = pz * rr;
      return std::atan((m * e - h) / denom);
    };

    // difference of termP for node1, node2
    out[i] = termP(m12, e1, h1, r1) - termP(m12, e2, h2, r2);
  }

  return out;
}

float slope(float xA, float yA, float xB, float yB) {
  const float dx = xB - xA;
  const float dy = yB - yA;
  // handle near‐vertical or near‐horizontal
  if (std::fabs(dx) < 1e-14) {
    return std::copysign(CLAMP_TO, dy);
  }
  if (std::fabs(dy) < 1e-14) {
    return 0.0f;
  }
  return dy / dx;
};

#define _LOAD(src) _mm256_load_ps(src)
#define _STORE(dest, v1) _mm256_store_ps(dest, v1)
#define __VECTOR_S __m256
#define __VECTOR_I __m256i
#define _ADD(v1, v2) _mm256_add_ps(v1, v2)
#define _SUB(v1, v2) _mm256_sub_ps(v1, v2)
#define _MUL(v1, v2) _mm256_mul_ps(v1, v2)
#define _DIV(v1, v2) _mm256_div_ps(v1, v2)
#define _CONST_VEC(v1) _mm256_set1_ps(v1)
#define _COPY_SIGN(v1, v2) Sleef_copysignf8(v1, v2)
#define _ZERO() _mm256_setzero_ps()
#define _GT(v1, v2) _mm256_cmp_ps(v1, v2, _CMP_GT_OQ)
#define _LT(v1, v2) _mm256_cmp_ps(v1, v2, _CMP_LT_OQ)
#define _FABS(v1) Sleef_fabsf8(v1)
#define _SELECT_F(false, true, mask) _mm256_blendv_ps(false, true, mask)
#define _HYPOPT(v1, v2) Sleef_hypotf8_u05avx2(v1, v2)
#define _SQR(v1) _MUL(v1, v1)
#define _SQRT(v1) Sleef_sqrtf8_u05avx2(v1)
#define _LOG(v1) Sleef_logf8_u10avx2(v1)
#define _ATAN(v1) Sleef_atanf8_u10avx2(v1)

void R12_Q12_J12_AVX2(float *R12_, float *Q12_, float *J12_, float *x, float *y,
                      float *z, float *node1, float *node2, size_t N) {
  const size_t PACK = 8;
  // Preliminary Constants
  const float x1f = node1[0], y1f = node1[1];
  const float x2f = node2[0], y2f = node2[1];
  const float m12f = slope<float>(x1f, y1f, x2f, y2f);

  const float dxf = node2[0] - node1[0];
  const float dyf = node2[1] - node1[1];
  const float df = std::sqrt(dxf * dxf + dyf * dyf);

  __VECTOR_S x1 = _CONST_VEC(x1f);
  __VECTOR_S y1 = _CONST_VEC(y1f);

  __VECTOR_S x2 = _CONST_VEC(x2f);
  __VECTOR_S y2 = _CONST_VEC(y2f);

  // Slope function
  __VECTOR_S dx = _CONST_VEC(dxf);
  __VECTOR_S dy = _CONST_VEC(dyf);
  __VECTOR_S m12 = _CONST_VEC(m12f);
  __VECTOR_S d = _CONST_VEC(df);

  for (size_t i = 0; i < N / PACK; ++i) {
    __VECTOR_S px = _LOAD(x);
    __VECTOR_S py = _LOAD(y);
    __VECTOR_S pz = _LOAD(z);

    // Compute R12
    __VECTOR_S r12 =
        _DIV(_SUB(_MUL(_SUB(px, x1), dy), _MUL(_SUB(py, y1), dx)), d);
    _STORE(R12_, r12);

    __VECTOR_S dx1 = _SUB(px, x1);
    __VECTOR_S dy1 = _SUB(py, y1);
    __VECTOR_S e1 = _ADD(_SQR(dx1), _SQR(pz));
    __VECTOR_S h1 = _MUL(dx1, dy1);
    __VECTOR_S r1 = _SQRT(_ADD(_ADD(_SQR(dx1), _SQR(dy1)), _SQR(pz)));

    __VECTOR_S dx2 = _SUB(px, x2);
    __VECTOR_S dy2 = _SUB(py, y2);
    __VECTOR_S e2 = _ADD(_SQR(dx2), _SQR(pz));
    __VECTOR_S h2 = _MUL(dx2, dy2);
    __VECTOR_S r2 = _SQRT(_ADD(_ADD(_SQR(dx2), _SQR(dy2)), _SQR(pz)));

    __VECTOR_S q12 = _LOG(_DIV(_ADD(_ADD(r1, r2), d), _SUB(_ADD(r1, r2), d)));
    _STORE(Q12_, q12);

    __VECTOR_S a1 = _DIV(_SUB(_MUL(m12, e1), h1), _MUL(pz, r1));
    __VECTOR_S a2 = _DIV(_SUB(_MUL(m12, e2), h2), _MUL(pz, r2));

    __VECTOR_S j12 = _SUB(_ATAN(a1), _ATAN(a2));
    _STORE(J12_, j12);

    x += PACK;
    y += PACK;
    z += PACK;
    R12_ += PACK;
    Q12_ += PACK;
    J12_ += PACK;
  }

  for (size_t i = 0; i < N % PACK; ++i) {
    const float px = x[i];
    const float py = y[i];
    const float pz = z[i];

    R12_[i] = ((px - node1[0]) * dyf - (py - node1[1]) * dxf) / df;

    // For node1:
    const float dx1 = px - x1f;
    const float dy1 = py - y1f;
    const float e1 = dx1 * dx1 + pz * pz; // ek(node1)
    const float h1 = dx1 * dy1;           // hk(node1)
    const float r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));

    // For node2:
    const float dx2 = px - x2f;
    const float dy2 = py - y2f;
    const float e2 = dx2 * dx2 + pz * pz; // ek(node2)
    const float h2 = dx2 * dy2;           // hk(node2)
    const float r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));

    Q12_[i] = std::log((r1 + r2 + df) / (r1 + r2 - df));

    const float a1 = (m12f * e1 - h1) / (pz * r1);
    const float a2 = (m12f * e1 - h2) / (pz * r2);
    J12_[i] = std::atan(a1) - std::atan(a2);
  }
}

void R12_Q12_J12_NORM(float *R12_, float *Q12_, float *J12_, float *x, float *y,
                      float *z, float *node1, float *node2, size_t N) {
  const float x1 = node1[0], y1 = node1[1];
  const float x2 = node2[0], y2 = node2[1];
  const float m12 = slope<float>(x1, y1, x2, y2);

  const float dx = node2[0] - node1[0];
  const float dy = node2[1] - node1[1];
  const float d = std::sqrt(dx * dx + dy * dy);

  for (Eigen::Index i = 0; i < N; ++i) {
    const float px = x[i];
    const float py = y[i];
    const float pz = z[i];

    R12_[i] = ((px - node1[0]) * dy - (py - node1[1]) * dx) / d;
  }

  for (Eigen::Index i = 0; i < N; ++i) {
    const float px = x[i];
    const float py = y[i];
    const float pz = z[i];
    // For node1:
    const float dx1 = px - x1;
    const float dy1 = py - y1;
    const float e1 = dx1 * dx1 + pz * pz; // ek(node1)
    const float h1 = dx1 * dy1;           // hk(node1)
    const float r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));

    // For node2:
    const float dx2 = px - x2;
    const float dy2 = py - y2;
    const float e2 = dx2 * dx2 + pz * pz; // ek(node2)
    const float h2 = dx2 * dy2;           // hk(node2)
    const float r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));

    const float q12e = (r1 + r2 + d) / (r1 + r2 - d);
    Q12_[i] = fastlog2((double)q12e);

    auto termP = [&](float m, float e, float h, float rr) {
      // if pz=0, you might want to handle that carefully
      const float denom = pz * rr;
      return fast_atan((m * e - h), denom);
    };
    J12_[i] = termP(m12, e1, h1, r1) - termP(m12, e2, h2, r2);
  }
  // printf("m12 %f\n", m12);
  // printf("d %f\n", d);
}

void J12_NORM(float *__restrict J12_, float *__restrict x, float *__restrict y,
              float *__restrict z, float *__restrict node1,
              float *__restrict node2, size_t N) {
  const float x1 = node1[0], y1 = node1[1];
  const float x2 = node2[0], y2 = node2[1];
  const float m12 = slope<float>(x1, y1, x2, y2);

  for (Eigen::Index i = 0; i < N; ++i) {
    const float px = x[i];
    const float py = y[i];
    const float pz = z[i];
    // For node1:
    const float dx1 = px - x1;
    const float dy1 = py - y1;
    const float e1 = dx1 * dx1 + pz * pz; // ek(node1)
    const float h1 = dx1 * dy1;           // hk(node1)
    const float r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));

    // For node2:
    const float dx2 = px - x2;
    const float dy2 = py - y2;
    const float e2 = dx2 * dx2 + pz * pz; // ek(node2)
    const float h2 = dx2 * dy2;           // hk(node2)
    const float r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));

    auto termP = [&](float m, float e, float h, float rr) {
      // if pz=0, you might want to handle that carefully
      const float denom = pz * rr;
      return fast_atan((m * e - h), denom);
    };
    J12_[i] = termP(m12, e1, h1, r1) - termP(m12, e2, h2, r2);
  }
  // printf("m12 %f\n", m12);
  // printf("d %f\n", d);
}
