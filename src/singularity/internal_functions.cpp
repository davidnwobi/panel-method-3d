#include "singularity/internal_functions.hpp"
#include "fast_atan/atan_approx.h"
#include "fast_log/log_approx.h"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <cmath>

#define CLAMP_TO 1e9

using namespace Eigen;
using RowArray3d = Eigen::Array<double, 1, 3, Eigen::RowMajor>;

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

void R12_Q12_J12_NORM(double *__restrict R12_, double *__restrict Q12_,
                      double *__restrict J12_, double *__restrict x,
                      double *__restrict y, double *__restrict z,
                      double *__restrict node1, double *__restrict node2,
                      size_t N) {
  const double x1 = node1[0], y1 = node1[1];
  const double x2 = node2[0], y2 = node2[1];
  const double m12 = slope<double>(x1, y1, x2, y2);

  const double dx = node2[0] - node1[0];
  const double dy = node2[1] - node1[1];
  const double d = std::sqrt(dx * dx + dy * dy);

  for (Eigen::Index i = 0; i < N; ++i) {
    const double px = x[i];
    const double py = y[i];
    const double pz = z[i];

    R12_[i] = ((px - node1[0]) * dy - (py - node1[1]) * dx) / d;
  }

  for (Eigen::Index i = 0; i < N; ++i) {
    const double px = x[i];
    const double py = y[i];
    const double pz = z[i];
    // For node1:
    const double dx1 = px - x1;
    const double dy1 = py - y1;
    const double e1 = dx1 * dx1 + pz * pz; // ek(node1)
    const double h1 = dx1 * dy1;           // hk(node1)
    const double r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));

    // For node2:
    const double dx2 = px - x2;
    const double dy2 = py - y2;
    const double e2 = dx2 * dx2 + pz * pz; // ek(node2)
    const double h2 = dx2 * dy2;           // hk(node2)
    const double r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));

    const double q12e = (r1 + r2 + d) / (r1 + r2 - d);
    Q12_[i] = std::log((double)q12e);

    auto termP = [&](double m, double e, double h, double rr) {
      // if pz=0, you might want to handle that carefully
      const double denom = pz * rr;
      // return fast_atan((m * e - h), denom);
      return std::atan((m * e - h) / denom);
    };
    J12_[i] = termP(m12, e1, h1, r1) - termP(m12, e2, h2, r2);
  }
  // printf("m12 %f\n", m12);
  // printf("d %f\n", d);
}
void J12_NORM(double *__restrict J12_, double *__restrict x,
              double *__restrict y, double *__restrict z,
              double *__restrict node1, double *__restrict node2, size_t N) {
  const double x1 = node1[0], y1 = node1[1];
  const double x2 = node2[0], y2 = node2[1];
  const double m12 = slope<double>(x1, y1, x2, y2);

  for (Eigen::Index i = 0; i < N; ++i) {
    const double px = x[i];
    const double py = y[i];
    const double pz = z[i];
    // For node1:
    const double dx1 = px - x1;
    const double dy1 = py - y1;
    const double e1 = dx1 * dx1 + pz * pz; // ek(node1)
    const double h1 = dx1 * dy1;           // hk(node1)
    const double r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));

    // For node2:
    const double dx2 = px - x2;
    const double dy2 = py - y2;
    const double e2 = dx2 * dx2 + pz * pz; // ek(node2)
    const double h2 = dx2 * dy2;           // hk(node2)
    const double r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));

    auto termP = [&](double m, double e, double h, double rr) {
      // if pz=0, you might want to handle that carefully
      const double denom = pz * rr;
      return std::atan((m * e - h) / denom);
    };
    J12_[i] = termP(m12, e1, h1, r1) - termP(m12, e2, h2, r2);
  }
  // printf("m12 %f\n", m12);
  // printf("d %f\n", d);
}
