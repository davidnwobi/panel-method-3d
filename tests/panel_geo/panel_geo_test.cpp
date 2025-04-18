#include "compTask.hpp"
#include "evalPoints.hpp"
#include "panel_geo/panel_geo.hpp"
#include "surface/surface_reader.hpp"
#include <filesystem>
#include <gtest/gtest.h>
#include <numeric>

std::filesystem::path testDataLoc(std::string(SOURCE_DIR) + "/tests/test_data");

TEST(SinglePanelTest, testCenterPointsandNormals) {

  std::filesystem::path filePath(testDataLoc.string() + "/singlepanel.txt");
  std::vector<PanelSet> psets = readConvertedComponentsFromFile(filePath);
  auto panelGeo = PanelGeometry(psets[0].body);

  EXPECT_TRUE(
      panelGeo.centrePoints.row(0).isApprox(Eigen::RowVector3d(0.5, 0.5, 0)))
      << panelGeo.centrePoints.row(0) << " is not equal to "
      << (Eigen::RowVector3d(0.5, 0.5, 0));
  EXPECT_TRUE(panelGeo.tangentXVectors.row(0).isApprox(
      Eigen::RowVector3d(1.0, 0.0, 0.0)))
      << panelGeo.tangentXVectors.row(0) << " is not equal to "
      << (Eigen::RowVector3d(1.0, 0.0, 0.0));
  EXPECT_TRUE(panelGeo.tangentYVectors.row(0).isApprox(
      Eigen::RowVector3d(0.0, 1.0, 0.0)))
      << panelGeo.tangentYVectors.row(0) << " is not equal to  "
      << (Eigen::RowVector3d(0.0, 1.0, 0.0));
  EXPECT_TRUE(
      panelGeo.normalVectors.row(0).isApprox(Eigen::RowVector3d(0.0, 0.0, 1.0)))
      << panelGeo.normalVectors.row(0) << " is not equal to "
      << (Eigen::RowVector3d(0.0, 0.0, 1.0));
}

TEST(TestLocalConversion, testLocalConversion) {

  std::filesystem::path filePath(testDataLoc.string() + "/naca0010lowres.txt");
  std::vector<PanelSet> psets = readConvertedComponentsFromFile(filePath);
  auto panelGeoSurf = PanelGeometry(psets[0].body);

  EvalPoints evalPoints;
  evalPoints.mEvalPoints =
      panelGeoSurf.centrePoints + panelGeoSurf.normalVectors;

  std::vector<std::size_t> idxs(evalPoints.mEvalPoints.rows());
  std::iota(idxs.begin(), idxs.end(), 0);

  for (int i = 0; i < 5; i++) {
    // Point direct over the influencing panel should be 0,0,1
    std::cout << createInfluenceComputeTask(panelGeoSurf, evalPoints, i, idxs)
                     .points.row(i)
              << "\n";
    EXPECT_TRUE(createInfluenceComputeTask(panelGeoSurf, evalPoints, i, idxs)
                    .points.row(i)
                    .isApprox(Eigen::RowVector3d(0.0, 0.0, 1.0)));
  }
}

TEST(SinglePanelTest, testArea) {

  std::filesystem::path filePath(testDataLoc.string() + "/singlepanel.txt");
  std::vector<PanelSet> psets = readConvertedComponentsFromFile(filePath);
  auto panelGeo = PanelGeometry(psets[0].body);

  EXPECT_TRUE(std::abs(panelGeo.areas(0) - 1) < 1e-6)
      << panelGeo.areas(0) << " is not equal to " << 1;
}
