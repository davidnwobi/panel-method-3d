#include <bit>
#include <cinttypes>
#include <cmath>
#include <cstring>

/// @returns the exponent and a normalized mantissa with the relationship:
/// [a * 2^b] = x
std::pair<double, int> __attribute__((always_inline)) my_frexp(double x) {
  uint64_t bits = std::bit_cast<uint64_t, double>(x);
  if (bits == 0) {
    return {0., 0};
  }
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
  return {frac, exponent + 1};
}

double __attribute__((always_inline)) fastlog2(double x) {

  /// Extract the fraction, and the power-of-two exponent.

  auto a = my_frexp(x);
  x = a.first;
  int pow2 = a.second;

  // Use a 4-part polynom to approximate log2(x);
  double c[] = {1.33755322, -4.42852392, 6.30371424, -3.21430967};
  double log2 = 0.6931471805599453;

  // Use Horner's method to evaluate the polynomial.
  double val = c[3] + x * (c[2] + x * (c[1] + x * (c[0])));

  // Compute log2(x), and convert the result to base-e.
  return log2 * (pow2 + val);
}
