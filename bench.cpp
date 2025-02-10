#include "aerocalcs/aerocalcsingle.hpp"
#include "panel_geo/panel_geo.hpp"
#include "panel_method/source_doublet_single.hpp"
#include "pm_octree_defs.hpp"
#include "solver/sparse_solver.hpp"
#include "surface/surface_panel.hpp"
#include "surface/surface_reader.hpp"
#include "surface/wake_panel.hpp"

#include <Eigen/Core>
#include <benchmark/benchmark.h>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>

template <typename Derived>
Derived rotate_2d_about_origin(const Eigen::MatrixBase<Derived> &points2d,
                               double angle_d) {
  double angle_r = angle_d * M_PI / 180.0;
  return (Eigen::Matrix2d{{std::cos(angle_r), -std::sin(angle_r)},
                          {std::sin(angle_r), std::cos(angle_r)}}) *
         points2d;
}

void rotate_3d_about_origin(Eigen::Ref<Eigen::ArrayX3d> points3d,
                            double angle_d) {
  Eigen::MatrixXd points = rotate_2d_about_origin(
      (Eigen::MatrixXd(2, points3d.rows()) << points3d.col(0).transpose(),
       points3d.col(2).transpose())
          .finished(),
      angle_d);
  points3d.col(0) = points.row(0).transpose();
  points3d.col(2) = points.row(1).transpose();
}

void rotate_points_about_start(Eigen::Ref<Eigen::ArrayX3d> points3d,
                               double angle_d) {
  Eigen::RowVector3d original_loc(points3d(0, 0), 0, points3d(0, 2));
  points3d = points3d.rowwise() - original_loc.array(); // translate to origin
  rotate_3d_about_origin(points3d, angle_d);
  points3d = points3d.rowwise() + original_loc.array(); // translate from origin
}

double bench_panel_impl(int no_panels) {
  namespace fs = std::filesystem;

  fs::path testDataLoc(std::string(SOURCE_DIR) + "/tests/test_data");
  fs::path filePath(testDataLoc.string() + "/0012_" +
                    std::to_string(no_panels) + ".txt");

  double aoa = 10;
  auto pset = readConvertedComponentsFromFile(filePath)[0];
  rotate_points_about_start(pset.wake.mPoints, aoa);
  PanelGeometry<SurfacePanel> body(pset.body);
  PanelGeometry<SurfacePanel> wake(pset.wake);

  EvalPoints<double> evalPoints(body.centrePoints);
  AeroCalcSingle calc(std::move(readConvertedComponentsFromFile(filePath)[0]),
                      {10.0}, std::type_identity<SourceDoubletSingle>{},
                      std::make_unique<SparseSolver>());
  calc.run({aoa, 1, 1});
  return calc.polars["CL"];
}
void bench_panel(benchmark::State &state) {
  int no_panels = state.range(0) * state.range(0);
  double CL;
  for (auto _ : state) {
    CL = bench_panel_impl(no_panels);
  }
  benchmark::DoNotOptimize(CL);
}

BENCHMARK(bench_panel)->DenseRange(10, 100, 10)->MinTime(60);

BENCHMARK_MAIN();
