
#include "central_difference.hpp"
#include <Eigen/Core>
#include <gtest/gtest.h>
#include <vector>

TEST(CentralDifferenceTest, BoundaryPoints) {
  Eigen::ArrayXf x = Eigen::ArrayXf::LinSpaced(5, 0, 4); // x = [0, 1, 2, 3, 4]
  Eigen::ArrayXf f = 2 * x + 1;                          // f(x) = 2x + 1

  Eigen::ArrayXXf xMat = x.reshaped(5, 1);
  Eigen::ArrayXXf fMat = f.reshaped(5, 1);

  auto result = centralDifference<true>(xMat, fMat);

  // Check forward/backward difference at boundaries
  EXPECT_NEAR(result(0, 0), 2.0, 1e-6); // First point
  EXPECT_NEAR(result(4, 0), 2.0, 1e-6); // Last point
}

TEST(CentralDifferenceTest, QuadraticFunction) {
  Eigen::ArrayXf x = Eigen::ArrayXf::LinSpaced(10, -5, 5); // x = [-5, ..., 5]
  Eigen::ArrayXf f = x.square();                           // f(x) = x^2

  Eigen::ArrayXXf xMat = x.reshaped(10, 1);
  Eigen::ArrayXXf fMat = f.reshaped(10, 1);

  auto result = centralDifference<true>(xMat, fMat);

  // Derivative should approximate 2x
  for (int i = 1; i < result.rows() - 1; i++) { // Skip boundaries for now
    float expected = 2 * x[i];
    EXPECT_NEAR(result(i, 0), expected, 1e-6);
  }
}

TEST(CentralDifferenceTest, IrregularSpacing) {
  // Define x and f(x) for y = log(x)
  Eigen::ArrayXXf x = Eigen::pow(2, Eigen::ArrayXf::LinSpaced(150, 0, 2));
  Eigen::ArrayXXf f = x.log();

  // Expected derivative is f'(x) = 1/x
  Eigen::ArrayXXf expected = 1 / x;

  Eigen::ArrayXXf result = centralDifference<true>(x, f);
  EXPECT_TRUE(result.isApprox(expected, 1e-2));
}

TEST(CentralDifferenceTest, SinusoidalFunction) {
  // Define x and f(x) for y = sin(x)
  Eigen::ArrayXXf x = Eigen::ArrayXf::LinSpaced(150, 0, M_PI / 2.0);
  Eigen::ArrayXXf f = x.sin();

  // Expected derivative is cos(x)
  Eigen::ArrayXXf expected = x.cos();

  Eigen::ArrayXXf result = centralDifference<true>(x, f);

  EXPECT_TRUE(result.isApprox(expected, 1e-2));
}

TEST(CentralDifferenceTest, MultiColumnTest) {
  // 5 rows, 1 column for x
  Eigen::ArrayXf x = Eigen::ArrayXf::LinSpaced(5, 0, 4);

  // f has 5 rows, 2 columns: [f1, f2]
  // f1(x) = x^2, f2(x) = x^3
  Eigen::ArrayXXf f(5, 2);
  f.col(0) = x.pow(2);
  f.col(1) = x.pow(3);

  // Perform central difference
  Eigen::ArrayXXf deriv = centralDifference<true>(
      x.reshaped(5, 1).replicate(2, 1).reshaped(5, 2).array().eval(), f);
  // deriv should be 5x2

  // We'll just check a couple of known spots
  // For f1 (x^2), index 0 should be 1, index 4 should be 7
  EXPECT_NEAR(deriv(0, 0), 1.0, 1e-12);
  EXPECT_NEAR(deriv(4, 0), 7.0, 1e-12);

  // For f2 (x^3), let's check index=1 and index=4:
  //   derivative of x^3 is 3x^2
  //   but boundary differences won't match exactly 3x^2 at boundaries.
  // We'll just do a forward/backward difference check at 0 and 4
  // to see if it's consistent with your function's formula

  // index 0 => (f2(1) - f2(0)) / (x(1) - x(0)) => (1 - 0)/1 => 1
  EXPECT_NEAR(deriv(0, 1), 1.0, 1e-12);

  // index 4 => (f2(4) - f2(3)) / (x(4)-x(3)) => (64-27)/1 => 37
  EXPECT_NEAR(deriv(4, 1), 37.0, 1e-12);
}
