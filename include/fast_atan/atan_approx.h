#pragma once
#include <math.h>

inline double atan_approximation(double x) {
  double c1 = 0.99997726f;
  double c3 = -0.33262347f;
  double c5 = 0.19354346f;
  double c7 = -0.11643287f;
  double c9 = 0.05265332d;
  double c11 = -0.01172120f;

  double x_sq = x * x;
  return x * fmaf(x_sq,
                  fmaf(x_sq,
                       fmaf(x_sq, fmaf(x_sq, fmaf(x_sq, c11, c9), c7), c5), c3),
                  c1);
}

inline double fast_atan(double y, double x) {
  // Ensure input is in [-1, +1]
  bool swap = fabs(x) < fabs(y);
  double atan_input = (swap ? x : y) / (swap ? y : x);

  // Approximate atan
  double res = atan_approximation(atan_input);

  // If swapped, adjust atan output
  return swap ? (atan_input >= 0.0f ? M_PI_2 : -M_PI_2) - res : res;
}
