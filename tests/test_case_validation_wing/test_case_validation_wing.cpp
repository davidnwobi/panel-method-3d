#include "aerocalcs/aerocalcsingle.hpp"
#include "panel_geo/panel_geo.hpp"
#include "solver/sparse_solver.hpp"
#include "surface/surface_reader.hpp"
#include "utils/utils.hpp"
#include "gtest/gtest.h"
#include <Eigen/Core>

#include <Eigen/Core>
#include <filesystem>
#include <memory>
#include <ranges>
#include <string>
#include <unordered_map>
#include <utility>

TEST(VALIDATION_WING, VALIDATION_WING) {
  print("Hello World");
  namespace fs = std::filesystem;

  fs::path testDataLoc(std::string(SOURCE_DIR) + "/tests/test_data");
  fs::path filePath(testDataLoc.string() + "/validation_wing.txt");
  //
  int n = 21;
  auto pset = readConvertedComponentsFromFile(filePath)[0];
  PanelGeometry<SurfacePanel> body(pset.body);
  PanelGeometry<SurfacePanel> wake(pset.wake);
  Eigen::ArrayX3d surfacePanelInfo =
      vMergeRecursive(std::move(body.centrePoints),
                      std::move(body.tangentXVectors), body.tangentYVectors,
                      body.normalVectors, body.areas.replicate<1, 3>().eval())
          .eval();
  Eigen::ArrayX3d wakePanelInfo =
      vMergeRecursive(std::move(wake.centrePoints),
                      std::move(wake.tangentXVectors), wake.tangentYVectors,
                      body.normalVectors, wake.areas.replicate<1, 3>().eval())
          .eval();
  Eigen::ArrayX3d surfacePanelInfoComp =
      FileReaderFactory::make_file_reader("dat", " ", false)
          ->read_data(std::string(testDataLoc) +
                      "/test_case_validation_wing/surfacePanelInfo.txt");

  EXPECT_TRUE(
      std::abs(
          (surfacePanelInfoComp - surfacePanelInfo).rowwise().norm().sum()) <
      1e-10);

  Eigen::ArrayX3d wakePanelInfoComp =
      FileReaderFactory::make_file_reader("dat", " ", false)
          ->read_data(std::string(testDataLoc) +
                      "/test_case_validation_wing/wakePanelInfo.txt");
  //  print();
  EXPECT_TRUE((wakePanelInfoComp - wakePanelInfo).isApproxToConstant(0));
  AeroCalcSingle calc(std::move(readConvertedComponentsFromFile(filePath)[0]),
                      {10.0}, std::make_unique<SparseSolver>());

  Eigen::ArrayX2d polars(n, 2);
  for (const auto i : std::ranges::views::iota(18, n)) {
    calc.run({i * 0.5, 1, 1});
    polars(i, 0) = calc.polars["aoa"];
    polars(i, 1) = calc.polars["CL"];
  }

  Eigen::ArrayX2d polarComp =
      FileReaderFactory::make_file_reader("dat", " ", false)
          ->read_data(std::string(testDataLoc) +
                      "/test_case_validation_wing/polars.dat");
  EXPECT_TRUE((polars - polarComp).bottomRows(3).isApproxToConstant(0));
}
