#include "fast_log/log_approx.h"
#include <bit>
#include <cinttypes>
#include <cmath>
#include <cstring>

double __attribute__((always_inline)) fastlog2(double x) {

  /// Extract the fraction, and the power-of-two exponent.

  int pow2;
  uint64_t bits = std::bit_cast<uint64_t, double>(x);
  if (bits == 0) {
    x = 0.;
    pow2 = 0;
  } else {
    // See:
    // https://en.wikipedia.org/wiki/IEEE_754#Basic_and_interchange_formats

    // Extract the 52-bit mantissa field.
    uint64_t mantissa = bits & 0xFFFFFFFFFFFFF;
    bits >>= 52;

    // Extract the 11-bit exponent field, and add the bias.
    int exponent = int(bits & 0x7ff) - 1023;
    bits >>= 11;

    // Extract the sign bit.
    uint64_t sign = bits;
    bits >>= 1;

    // Construct the normalized double;
    uint64_t res = sign;
    res <<= 11;
    res |= 1023 - 1;
    res <<= 52;
    res |= mantissa;

    double frac = std::bit_cast<double, uint64_t>(res);

    x = frac;
    int pow2 = exponent + 1;
  }
  // Use a 4-part polynom to approximate log2(x);
  double c[] = {1.33755322, -4.42852392, 6.30371424, -3.21430967};
  double log2 = 0.6931471805599453;

  // Use Horner's method to evaluate the polynomial.
  double val = c[3] + x * (c[2] + x * (c[1] + x * (c[0])));

  // Compute log2(x), and convert the result to base-e.
  return log2 * (pow2 + val);
}
