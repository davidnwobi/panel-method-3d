#pragma once
#include <math.h>

inline float atan_approximation(float x) {
  float c1 = 0.99997726f;
  float c3 = -0.33262347f;
  float c5 = 0.19354346f;
  float c7 = -0.11643287f;
  float c9 = 0.05265332f;
  float c11 = -0.01172120f;

  float x_sq = x * x;
  return x * fmaf(x_sq,
                  fmaf(x_sq,
                       fmaf(x_sq, fmaf(x_sq, fmaf(x_sq, c11, c9), c7), c5), c3),
                  c1);
}

inline float fast_atan(float y, float x) {
  // Ensure input is in [-1, +1]
  bool swap = fabs(x) < fabs(y);
  float atan_input = (swap ? x : y) / (swap ? y : x);

  // Approximate atan
  float res = atan_approximation(atan_input);

  // If swapped, adjust atan output
  return swap ? (atan_input >= 0.0f ? M_PI_2 : -M_PI_2) - res : res;
}
