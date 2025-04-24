#pragma once
#include <bit>
#include <cinttypes>
#include <cmath>
#include <cstring>
#include <limits>
/// @returns the exponent and a normalized mantissa with the relationship:
/// [a * 2^b] = x

inline float logapprox(float val) {
  union {
    float f;
    int32_t i;
  } valu;
  float exp, addcst, x;
  valu.f = val;
  exp = valu.i >> 23;
  /* -89.970756366f = -127 * log(2) + constant term of polynomial bellow. */
  addcst = val > 0 ? -89.970756366f : -std::numeric_limits<float>::infinity();
  valu.i = (valu.i & 0x7FFFFF) | 0x3F800000;
  x = valu.f;

  /* Generated in Sollya using:
    > f = remez(log(x)-(x-1)*log(2),
            [|1,(x-1)*(x-2), (x-1)*(x-2)*x, (x-1)*(x-2)*x*x,
              (x-1)*(x-2)*x*x*x|], [1,2], 1, 1e-8);
    > plot(f+(x-1)*log(2)-log(x), [1,2]);
    > f+(x-1)*log(2)
 */
  return x * (3.529304993f +
              x * (-2.461222105f +
                   x * (1.130626167f +
                        x * (-0.288739945f + x * 3.110401639e-2f)))) +
         (addcst + 0.6931471805f * exp);
}
