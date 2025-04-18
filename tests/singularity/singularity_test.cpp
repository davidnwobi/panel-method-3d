#include "compTask.hpp"
#include "evalPoints.hpp"
#include "singularity/const_doublet.hpp"
#include "singularity/const_source.hpp"
#include "surface/surface_reader.hpp"
#include "utils/utils.hpp"
#include <algorithm>
#include <gtest/gtest.h>
#include <numeric>
#include <utility>

#include <ranges>
namespace views = std::views;
#define RANGE(n) views::iota(0, (int)n)

std::filesystem::path testDataLoc(std::string(SOURCE_DIR) + "/tests/test_data");

TEST(APPLY_ADAJACENT_TEST, APPLY_ADJACENT_TEST) {

  std::vector<int> numbers(10, 0);
  std::iota(numbers.begin(), numbers.end(), 1);
  std::vector<int> out(10);
  apply_adjacent(numbers.begin(), numbers.end(), out.begin(), std::plus<int>());

  std::vector<int> ans = {3, 5, 7, 9, 11, 13, 15, 17, 19, 0};
  EXPECT_TRUE(std::equal(out.begin(), out.end(), ans.begin()));
}

TEST(APPLY_ADAJACENT_TEST_CIRCULAR, APPLY_ADJACENT_TEST_CIRCULAR) {

  std::vector<int> numbers(10, 0);
  std::iota(numbers.begin(), numbers.end(), 1);
  std::vector<int> out(10);
  apply_adjacent_circular(numbers.begin(), numbers.end(), out.begin(),
                          std::plus<int>());

  std::vector<int> ans = {3, 5, 7, 9, 11, 13, 15, 17, 19, 11};
  EXPECT_TRUE(std::equal(out.begin(), out.end(), ans.begin()));
}

TEST(RECTANGULAR_SOURCE_TEST_Compute_Task, RECTANGULAR_SOURCE_TEST) {

  std::filesystem::path filePath(testDataLoc.string() + "/singlepanel00.txt");
  std::vector<PanelSet> psets = readConvertedComponentsFromFile(filePath);
  auto panelGeoSurf = PanelGeometry(psets[0].body);
  Eigen::MatrixXd points =
      FileReaderFactory::make_file_reader("dat", " ", true)
          ->read_data(testDataLoc.string() + "/rectangular_source_test.txt");

  EvalPoints<double> evalPoints;
  evalPoints.mEvalPoints = points.topLeftCorner(points.rows(), 3);

  std::vector<std::size_t> idxs(evalPoints.mEvalPoints.rows());
  std::iota(idxs.begin(), idxs.end(), 0);
  auto computeTask =
      createInfluenceComputeTask(panelGeoSurf, evalPoints, 0, idxs);

  computeTask.face.points = psets[0].body.mPoints;
  computeTask.indices = std::move(idxs);
  computeTask.points = evalPoints.mEvalPoints;
  EXPECT_TRUE(
      SourceP<false>::calcInfluence(computeTask).isApprox(points.col(3), 1e-6));
}
TEST(RECTANGULAR_DOUBLET_TEST_Compute_Task, RECTANGULAR_DOUBLET_TEST) {

  std::filesystem::path filePath(testDataLoc.string() + "/singlepanel00.txt");
  std::vector<PanelSet> psets = readConvertedComponentsFromFile(filePath);
  auto panelGeoSurf = PanelGeometry(psets[0].body);
  Eigen::MatrixXd points =
      FileReaderFactory::make_file_reader("dat", " ", true)
          ->read_data(testDataLoc.string() + "/rectangular_doublet_test.txt");

  EvalPoints<double> evalPoints;
  evalPoints.mEvalPoints = points.topLeftCorner(points.rows(), 3);

  std::vector<std::size_t> idxs(evalPoints.mEvalPoints.rows());
  std::iota(idxs.begin(), idxs.end(), 0);
  auto computeTask =
      createInfluenceComputeTask(panelGeoSurf, evalPoints, 0, idxs);

  computeTask.face.points = psets[0].body.mPoints;
  computeTask.indices = std::move(idxs);
  computeTask.points = evalPoints.mEvalPoints;
  EXPECT_TRUE(DoubletP<false>::calcInfluence(computeTask)
                  .isApprox(points.col(3), 1e-6));
}
