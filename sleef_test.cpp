#include "utils/utils.hpp"
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <immintrin.h>
#include <sleef.h>
#include <stdio.h>
#include <sys/types.h>

#define PACK_WIDTH 16

#define CLAMP_TO 1e9
void add_float_vec(float *c, float *a, float *b, size_t N) {
  __m512 va, vb, vc;
  for (int i = 0; i < N / PACK_WIDTH; i++) {
    va = _mm512_load_ps(a);
    vb = _mm512_load_ps(b);
    vc = _mm512_add_ps(va, vb);
    _mm512_store_ps(c, vc);
    a += PACK_WIDTH;
    b += PACK_WIDTH;
    c += PACK_WIDTH;
  }

  for (int i = 0; i < N % 8; i++) {
    c[i] = a[i] + b[i];
  }
}
void sqrt_float_vec(float *c, float *a, size_t N) {
  __m512 va, vc;
  for (int i = 0; i < N / PACK_WIDTH; i++) {
    va = _mm512_load_ps(a);
    _mm512_store_ps(c, vc);
    a += PACK_WIDTH;
    c += PACK_WIDTH;
  }
  for (int i = 0; i < N % 8; i++) {
    c[i] = std::sqrt(a[i]);
  }
}

template <typename T> T slope(T xA, T yA, T xB, T yB) {
  const T dx = xB - xA;
  const T dy = yB - yA;
  // handle near‐vertical or near‐horizontal
  if (std::fabs(dx) < 1e-14) {
    return std::copysign(CLAMP_TO, dy);
  }
  if (std::fabs(dy) < 1e-14) {
    return 0.0f;
  }
  return dy / dx;
};

// #define _LOAD(src) _mm256_load_ps(src)
// #define _STORE(dest, v1) _mm256_store_ps(dest, v1)
// #define __VECTOR_S __m256
// #define __VECTOR_I __m256i
// #define _ADD(v1, v2) _mm256_add_ps(v1, v2)
// #define _SUB(v1, v2) _mm256_sub_ps(v1, v2)
// #define _MUL(v1, v2) _mm256_mul_ps(v1, v2)
// #define _DIV(v1, v2) _mm256_div_ps(v1, v2)
// #define _CONST_VEC(v1) _mm256_set1_ps(v1)
// #define _COPY_SIGN(v1, v2) Sleef_copysignf8_avx2(v1, v2)
// #define _ZERO() _mm256_setzero_ps()
// #define _GT(v1, v2) _mm256_cmp_ps(v1, v2, _CMP_GT_OQ)
// #define _LT(v1, v2) _mm256_cmp_ps(v1, v2, _CMP_LT_OQ)
// #define _FABS(v1) Sleef_fabsf8_avx2(v1)
// #define _SELECT_F(false, true, mask) _mm256_blendv_ps(false, true, mask)
// #define _HYPOPT(v1, v2) Sleef_hypotf8_u05avx2(v1, v2)
// #define _SQR(v1) _MUL(v1, v1)
// #define _SQRT(v1) Sleef_sqrtf8_u05avx2(v1)
// #define _LOG(v1) Sleef_logf8_u10avx2(v1)
// #define _ATAN(v1) Sleef_atanf8_u10avx2(v1)
// #define PACK 8

#define _LOAD(src) _mm512_load_ps(src)
#define _STORE(dest, v1) _mm512_store_ps(dest, v1)
#define __VECTOR_S __m512
#define __VECTOR_I __m512i
#define _ADD(v1, v2) _mm512_add_ps(v1, v2)
#define _SUB(v1, v2) _mm512_sub_ps(v1, v2)
#define _MUL(v1, v2) _mm512_mul_ps(v1, v2)
#define _DIV(v1, v2) _mm512_div_ps(v1, v2)
#define _CONST_VEC(v1) _mm512_set1_ps(v1)
#define _COPY_SIGN(v1, v2) Sleef_copysignf16_avx512f(v1, v2)
#define _ZERO() _mm512_setzero_ps()
#define _GT(v1, v2) _mm512_cmp_ps_mask(v1, v2, _CMP_GT_OQ)
#define _LT(v1, v2) _mm512_cmp_ps_mask(v1, v2, _CMP_LT_OQ)
#define _FABS(v1) Sleef_fabsf16_avx512(v1)
#define _SELECT_F(a, b, mask) _mm512_mask_blend_ps(mask, a, b)
#define _HYPOPT(v1, v2) Sleef_hypotf16_u05avx512f(v1, v2)
#define _SQR(v1) _MUL(v1, v1)
#define _SQRT(v1) Sleef_sqrtf16_u05avx512f(v1)
#define _LOG(v1) Sleef_logf16_u10avx512f(v1)
#define _ATAN(v1) Sleef_atanf16_u10avx512f(v1)
#define PACK 16

void R12_Q12_J12_AVX2(float *R12_, float *Q12_, float *J12_, float *x, float *y,
                      float *z, float *node1, float *node2, size_t N) {
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

    Q12_[i] = std::log((r1 + r2 + d) / (r1 + r2 - d));

    auto termP = [&](float m, float e, float h, float rr) {
      // if pz=0, you might want to handle that carefully
      const float denom = pz * rr;
      return std::atan((m * e - h) / denom);
    };
    J12_[i] = termP(m12, e1, h1, r1) - termP(m12, e2, h2, r2);
  }
  // printf("m12 %f\n", m12);
  // printf("d %f\n", d);
}

bool validate(float *a, float *b, size_t N) {
  for (int i = 0; i < N; i++) {
    if (std::abs((((double)a[i] - (double)b[i])) / ((double)a[i])) > 1e-4) {
      printf("Mismatch at %d, a = %1.6f, b = %1.6f, diff = %1.6f\n", i, a[i],
             b[i], (a[i] - b[i]));
      return false;
    }
  }
  return true;
}
int main(int argc, char **argv) {
  const size_t N = 9937;
  float *R12, *Q12, *J12, *R12_, *Q12_, *J12_, *x, *y, *z, *node1, *node2;
  const size_t N_BYTES = N * sizeof(float);
  const size_t ALIGNMENT = 64;
  R12 = (float *)aligned_alloc(ALIGNMENT, N_BYTES);
  Q12 = (float *)aligned_alloc(ALIGNMENT, N_BYTES);
  J12 = (float *)aligned_alloc(ALIGNMENT, N_BYTES);
  R12_ = (float *)aligned_alloc(ALIGNMENT, N_BYTES);
  Q12_ = (float *)aligned_alloc(ALIGNMENT, N_BYTES);
  J12_ = (float *)aligned_alloc(ALIGNMENT, N_BYTES);
  x = (float *)aligned_alloc(ALIGNMENT, N_BYTES);
  y = (float *)aligned_alloc(ALIGNMENT, N_BYTES);
  z = (float *)aligned_alloc(ALIGNMENT, N_BYTES);
  node1 = (float *)aligned_alloc(ALIGNMENT, ALIGNMENT);
  node2 = (float *)aligned_alloc(ALIGNMENT, ALIGNMENT);

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
  R12_Q12_J12_NORM(R12, Q12, J12, x, y, z, node1, node2, N);
  R12_Q12_J12_AVX2(R12_, Q12_, J12_, x, y, z, node1, node2, N);

  print("r12\n");
  validate(R12, R12_, N);
  print("q12\n");
  validate(Q12, Q12_, N);
  print("j12\n");
  validate(J12, J12_, N);
  // __m256 va, vb, vc;
  // va = _mm256_load_ps(a);
  // vb = _mm256_load_ps(b);
  // vc = _mm256_add_ps(va, vb);
  //
  // _mm256_store_ps(c, vc);
}
