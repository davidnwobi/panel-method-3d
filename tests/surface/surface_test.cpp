#include "utils/utils.hpp"
#include <Eigen/Core>
#include <filesystem>
#include <gtest/gtest.h>

std::filesystem::path testDataLoc(std::string(SOURCE_DIR) + "/tests/test_data");
using namespace Eigen;

TEST(VMERGE, SimpleMerge) {
  Eigen::MatrixXf matA = MatrixXf::Random(10, 4);
  Eigen::MatrixXf matB = MatrixXf::Random(10, 4);

  auto matC = vMerge(matA, matB);

  EXPECT_TRUE(matC.rows() == matA.rows() + matB.rows());
  EXPECT_TRUE(matC.middleRows(0, matA.rows()).isApprox(matA));
  EXPECT_TRUE(matC.middleRows(matA.rows(), matB.rows()).isApprox(matB));
}
TEST(VMERGE, SimpleMergeIntoSameArray) {
  // Test Merging into the same array;

  Eigen::MatrixXf matA = MatrixXf::Random(4, 4);
  Eigen::MatrixXf matB = MatrixXf::Random(5, 4);
  Eigen::MatrixXf matD = MatrixXf::Random(3, 4);

  Eigen::MatrixXf matC(
      Eigen::Map<Eigen::MatrixXf>(matA.data(), matA.rows(), matA.cols()));
  matC = vMerge(matC, matB);
  matC = vMerge(matC, matD);

  EXPECT_TRUE(matC.rows() == matA.rows() + matB.rows() + matD.rows());
  EXPECT_TRUE(matC.middleRows(0, matA.rows()).isApprox(matA));
  EXPECT_TRUE(matC.middleRows(matA.rows(), matB.rows()).isApprox(matB));
  EXPECT_TRUE(
      matC.middleRows(matA.rows() + matB.rows(), matD.rows()).isApprox(matD));
}

TEST(HMERGE, SimpleMerge) {
  Eigen::MatrixXf matA = MatrixXf::Random(10, 4);
  Eigen::MatrixXf matB = MatrixXf::Random(10, 4);

  auto matC = hMerge(matA, matB);

  EXPECT_TRUE(matC.cols() == matA.cols() + matB.cols());
  EXPECT_TRUE(matC.middleCols(0, matA.cols()).isApprox(matA));
  EXPECT_TRUE(matC.middleCols(matA.cols(), matB.cols()).isApprox(matB));
}
TEST(HMERGE, SimpleMergeIntoSameArray) {
  // Test Merging into the same array;

  Eigen::MatrixXf matA = MatrixXf::Random(4, 4);
  Eigen::MatrixXf matB = MatrixXf::Random(4, 5);
  Eigen::MatrixXf matD = MatrixXf::Random(4, 3);

  Eigen::MatrixXf matC(
      Eigen::Map<Eigen::MatrixXf>(matA.data(), matA.rows(), matA.cols()));
  matC = hMerge(matC, matB);
  matC = hMerge(matC, matD);

  EXPECT_TRUE(matC.cols() == matA.cols() + matB.cols() + matD.cols());
  EXPECT_TRUE(matC.middleCols(0, matA.cols()).isApprox(matA));
  EXPECT_TRUE(matC.middleCols(matA.cols(), matB.cols()).isApprox(matB));
  EXPECT_TRUE(
      matC.middleCols(matA.cols() + matB.cols(), matD.cols()).isApprox(matD));
}
