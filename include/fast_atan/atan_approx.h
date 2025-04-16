#pragma once
#include <math.h>

inline float atan_approximation(float x) {
  float a1 = 0.99997726f;
  float a3 = -0.33262347f;
  float a5 = 0.19354346f;
  float a7 = -0.11643287f;
  float a9 = 0.05265332f;
  float a11 = -0.01172120f;

  float x_sq = x * x;
  return x *
         (a1 +
          x_sq * (a3 + x_sq * (a5 + x_sq * (a7 + x_sq * (a9 + x_sq * a11)))));
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
