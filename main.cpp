#include "aerocalcs/aerocalcsingle.hpp"
#include "solver/sparse_solver.hpp"
#include "surface/surface_reader.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <filesystem>
#include <memory>
#include <panel_geo/panel_geo.hpp>
#include <ranges>
#include <string>
#include <unordered_map>

// #define RANGE(n) views::iota(0, (int)n)
//

int main() {

  namespace fs = std::filesystem;

  fs::path testDataLoc(std::string(SOURCE_DIR) + "/tests/test_data");
  fs::path filePath(testDataLoc.string() + "/validation_wing.txt");
  //
  int n = 21;
  AeroCalcSingle calc(std::move(readConvertedComponentsFromFile(filePath)[0]),
                      {10.0}, std::make_unique<SparseSolver>());

  Eigen::ArrayX2d polars(n, 2);
  for (const auto i : std::ranges::views::iota(0, n)) {
    calc.run({i * 0.5, 1, 1});
    polars(i, 0) = calc.polars["aoa"];
    polars(i, 1) = calc.polars["CL"];
  }

  FileReaderFactory::make_file_reader("dat", " ", true)
      ->save_data(std::string(ANALYSIS_DIR) + "/polars.dat", polars);
  return 0;

  //  FileReaderFactory::make_file_reader("dat", " ", true)
  //      ->save_data(std::string(ANALYSIS_DIR) + "/velocities.dat",
  //                  pm.getComputedVelocites());
  //  FileReaderFactory::make_file_reader("dat", " ", true)
  //      ->save_data(std::string(ANALYSIS_DIR) + "/liftCoeff.dat",
  //                  liftCoefficient);
  //  FileReaderFactory::make_file_reader("dat", " ", true)
  //      ->save_data(std::string(ANALYSIS_DIR) + "/xPoints.dat",
  //                  surfacePanelGeo.centrePoints.col(0).reshaped(nXsecs,
  //                  nYsecs));
  //  FileReaderFactory::make_file_reader("dat", " ", true)
  //      ->save_data(std::string(ANALYSIS_DIR) + "/yPoints.dat",
  //                  surfacePanelGeo.centrePoints.col(1).reshaped(nXsecs,
  //                  nYsecs));
  //  FileReaderFactory::make_file_reader("dat", " ", true)
  //      ->save_data(std::string(ANALYSIS_DIR) + "/doubletDist.dat",
  //                  pm.getSolution());
  //  FileReaderFactory::make_file_reader("dat", " ", true)
  //      ->save_data(std::string(ANALYSIS_DIR) + "/sourceStrength.dat",
  //                  pm.getSource());
  //  FileReaderFactory::make_file_reader("dat", " ", true)
  //      ->save_data(std::string(ANALYSIS_DIR) + "/pressure.dat", dCp);
  //
  return 0;
}
