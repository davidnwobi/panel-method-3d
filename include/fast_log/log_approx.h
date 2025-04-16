#include <utility>

/// @returns the exponent and a normalized mantissa with the relationship:
/// [a * 2^b] = x
std::pair<double, int> __attribute__((always_inline)) my_frexp(double x);

double __attribute__((always_inline)) fastlog2(double x);
