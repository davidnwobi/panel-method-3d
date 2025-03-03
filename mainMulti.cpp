#include "aerocalcs/aerocalcsingle.hpp"
#include "central_difference.hpp"
#include "compTask.hpp"
#include "infMat.hpp"
#include "panel_geo/panel_geo.hpp"
#include "panel_method/source_doublet_single.hpp"
#include "singularity/const_doublet.hpp"
#include "singularity/const_doublet_far.hpp"
#include "singularity/const_source.hpp"
#include "singularity/const_source_far.hpp"
#include "solver/dense_solver.hpp"
#include "solver/sparse_solver.hpp"
#include "surface/surface_panel.hpp"
#include "surface/surface_reader.hpp"
#include "surface/wake_panel.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iterator>
#include <memory>
#include <numeric>
#include <ranges>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
// #define RANGE(n) views::iota(0, (int)n)
//
using namespace std::ranges;
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
void align_wake_to_flow(std::vector<PanelSet> &panel_sets, double aoa) {
  std::ranges::for_each(
      panel_sets,
      [&aoa](WakePanel &wake) {
        return rotate_points_about_start(wake.mPoints, aoa);
      },
      &PanelSet::wake);
}
using PanelGeometryPair =
    std::pair<PanelGeometry<SurfacePanel>, PanelGeometry<WakePanel>>;
using ComputeTaskPair =
    std::pair<std::vector<ComputeTask>, std::vector<ComputeTask>>;

std::vector<PanelGeometryPair>
calc_panel_geometry(std::vector<PanelSet> &panel_sets) {
  std::vector<PanelGeometryPair> panelGeometryPair;
  panelGeometryPair.reserve(panel_sets.size());
  std::ranges::copy(panel_sets | std::views::transform([](PanelSet &pset) {
                      return std::make_pair(
                          PanelGeometry<SurfacePanel>(pset.body),
                          PanelGeometry<WakePanel>(pset.wake));
                    }),
                    std::back_inserter(panelGeometryPair));
  return panelGeometryPair;
}
void postProcessPanelResults(
    const PanelGeometry<SurfacePanel> surfacePanelGeo,
    const Eigen::Ref<const Eigen::ArrayXXd> &surfaceVelocties,
    AeroResults &out) {

  Eigen::ArrayXd dCp = 1 - (surfaceVelocties.rowwise().squaredNorm());

  double q =
      0.5 * out.lastParams.rho * out.lastParams.Vinf * out.lastParams.Vinf;
  Eigen::ArrayXd dP = q * dCp;
  Eigen::ArrayX3d dF =
      surfacePanelGeo.normalVectors.colwise() * (dP * surfacePanelGeo.areas);

  out.panelResults["dCp"] = dCp;
  out.panelResults["dVx"] = surfaceVelocties.col(0).eval();
  out.panelResults["dVy"] = surfaceVelocties.col(1).eval();
  out.panelResults["dVz"] = surfaceVelocties.col(2).eval();
  out.panelResults["dP"] = dP.eval();
  out.panelResults["dFx"] = dF.col(0).eval();
  out.panelResults["dFy"] = dF.col(1).eval();
  out.panelResults["dFz"] = dF.col(2).eval();
}
void postProcessPolars(AeroResults &out) {
  double q =
      0.5 * out.lastParams.rho * out.lastParams.Vinf * out.lastParams.Vinf;
  Eigen::Array3d F(3);
  F << out.panelResults["dFx"].sum(), out.panelResults["dFy"].sum(),
      out.panelResults["dFz"].sum();
  Eigen::ArrayXd CF = F / (q * out.refGeom.refArea);
  double CL = (-CF(0) * std::sin(out.lastParams.aoa * M_PI / 180) +
               CF(2) * std::cos(out.lastParams.aoa * M_PI / 180));
  double CD = (F(0) * std::cos(out.lastParams.aoa * M_PI / 180) +
               F(2) * std::sin(out.lastParams.aoa * M_PI / 180));

  out.polars["aoa"] = out.lastParams.aoa;
  out.polars["Fx"] = F(0);
  out.polars["Fy"] = F(1);
  out.polars["Fz"] = F(2);
  out.polars["CFx"] = CF(0);
  out.polars["CFy"] = CF(1);
  out.polars["CFz"] = CF(2);
  out.polars["CL"] = CL;
  out.polars["CD"] = CD;
}
AeroResults
post_process_body(const PanelGeometry<SurfacePanel> surfacePanelGeo,
                  const Eigen::Ref<const Eigen::ArrayXXd> &surfaceVelocities,
                  FlowParams &&flowParams, double refArea) {
  AeroResults results;
  results.lastParams = flowParams;
  results.refGeom = {refArea};
  postProcessPanelResults(surfacePanelGeo, surfaceVelocities, results);
  postProcessPolars(results);
  // postProcessSpanResults();
  return results;
}
EvalPoints<double>
create_eval_points(std::span<PanelGeometryPair> panelGeometries) {
  auto totalEvalPoints =
      std::accumulate(panelGeometries.begin(), panelGeometries.end(), 0,
                      [](std::size_t count, const PanelGeometryPair &panelGeo) {
                        return count + panelGeo.first.centrePoints.rows();
                      });

  EvalPoints<double> evalPoints(totalEvalPoints);
  std::size_t iPoints = 0;
  auto pointsView = panelGeometries | std::views::transform([](auto const &g) {
                      return g.first.centrePoints;
                    });
  std::ranges::for_each(pointsView, [&](auto const &pts) {
    const auto rows = pts.rows();
    evalPoints.mEvalPoints.middleRows(iPoints, rows) = pts;
    iPoints += rows;
  });
  return evalPoints;
}
ComputeTaskPair makeComputeTasksPairImpl(const PanelGeometryPair panelGeometry,
                                         const EvalPoints<double> &evalPoints) {
  int surfPanels = panelGeometry.first.centrePoints.rows();
  int wakePanels = panelGeometry.second.centrePoints.rows();
  auto surfaceComputeTaskView =
      RANGE(surfPanels) |
      views::transform([&panelGeometry, &evalPoints](int faceIdx) {
        return createInfluenceComputeTask(panelGeometry.first, evalPoints,
                                          faceIdx);
      });
  auto wakeComputeTaskView =
      RANGE(wakePanels) |
      views::transform([&panelGeometry, &evalPoints](int faceIdx) {
        return createInfluenceComputeTask(panelGeometry.second, evalPoints,
                                          faceIdx);
      });

  ComputeTaskPair compTaskPair;
  compTaskPair.first.reserve(surfPanels);
  compTaskPair.second.reserve(wakePanels);
  std::ranges::copy(surfaceComputeTaskView,
                    std::back_inserter(compTaskPair.first));
  std::ranges::copy(wakeComputeTaskView,
                    std::back_inserter(compTaskPair.second));
  print("Wake Tasks, ", wakeComputeTaskView.size());
  return compTaskPair;
};
std::vector<ComputeTaskPair>
makeComputeTaskPairs(std::span<PanelGeometryPair> panelGeometries,
                     const EvalPoints<double> &evalPoints) {

  int nBodies = panelGeometries.size();
  auto pairs =
      RANGE(nBodies) |
      views::transform([&panelGeometries, &evalPoints](int iBody) {
        return makeComputeTasksPairImpl(panelGeometries[iBody], evalPoints);
      });
  std::vector<ComputeTaskPair> compTaskPairs;
  compTaskPairs.reserve(nBodies);
  std::ranges::copy(pairs, std::back_inserter(compTaskPairs));
  return compTaskPairs;
}
Eigen::MatrixXd
assembleLhsImpl(std::span<const ComputeTask> surfacePanelCompTasks,
                std::span<const ComputeTask> wakePanelCompTasks,
                const PanelGeometry<WakePanel> &wakePanelGeo,
                const EvalPoints<double> &evalPoints) {

  std::size_t evalDims = evalPoints.mEvalPoints.rows();
  std::size_t surfDims = surfacePanelCompTasks.size();
  std::size_t wakeDims = wakePanelCompTasks.size();
  Eigen::MatrixXd surfaceInfluenceMatrix = makeInfluenceMatrix<DoubletP, true>(
      evalDims, surfDims, surfacePanelCompTasks);
  // print(surfaceInfluenceMatrix.topLeftCorner(20, 20)) << "\n";
  Eigen::MatrixXd wakeInfluenceMatrix = makeInfluenceMatrix<DoubletP, false>(
      evalDims, wakeDims, wakePanelCompTasks);

  // combine source and wake
  for (std::size_t iWakeP = 0;
       iWakeP < wakePanelGeo.mSurface.mTrailingEdgeIdx.rows(); iWakeP++) {

    // NOTE: bad for cache?

    int lowerFaceIdx = wakePanelGeo.mSurface.mTrailingEdgeIdx(iWakeP, 0);
    int upperFaceIdx = wakePanelGeo.mSurface.mTrailingEdgeIdx(iWakeP, 1);
    surfaceInfluenceMatrix(Eigen::placeholders::all, lowerFaceIdx) -=
        wakeInfluenceMatrix(Eigen::placeholders::all, iWakeP);
    surfaceInfluenceMatrix(Eigen::placeholders::all, upperFaceIdx) +=
        wakeInfluenceMatrix(Eigen::placeholders::all, iWakeP);
  }
  return surfaceInfluenceMatrix;
}

Eigen::MatrixXd assembleLhs(std::span<const ComputeTaskPair> compTaskPairs,
                            std::span<const PanelGeometryPair> panelGeometries,
                            const EvalPoints<double> &evalPoints) {

  auto lhsView =
      RANGE(compTaskPairs.size()) | views::transform([&](std::size_t idx) {
        return assembleLhsImpl(compTaskPairs[idx].first,
                               compTaskPairs[idx].second,
                               panelGeometries[idx].second, evalPoints);
      });

  std::size_t mDims = evalPoints.mEvalPoints.rows();
  Eigen::MatrixXd lhs(mDims, mDims);
  std::size_t iPoints = 0;
  std::ranges::for_each(lhsView, [&](const Eigen::Ref<const MatrixXd> &pts) {
    const auto cols = pts.cols();
    lhs.middleCols(iPoints, cols) = pts;
    iPoints += cols;
  });
  return lhs;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd>
assembleRhsImpl(std::span<const ComputeTask> surfacePanelCompTasks,
                const PanelGeometry<SurfacePanel> &surfacePanelGeo,
                const EvalPoints<double> &evalPoints,
                const Eigen::Ref<Eigen::Array3d> &freeStream) {

  std::size_t evalDims = evalPoints.mEvalPoints.rows();
  std::size_t surfDims = surfacePanelCompTasks.size();
  Eigen::MatrixXd sourceInfluenceMat = makeInfluenceMatrix<SourceP, true>(
      evalDims, surfDims, surfacePanelCompTasks);
  Eigen::VectorXd sourceStrength =
      rowwiseDotProduct(surfacePanelGeo.normalVectors, freeStream);
  return {-(sourceInfluenceMat * sourceStrength), sourceStrength};
}

std::pair<Eigen::VectorXd, VectorXd>
assembleRhs(std::span<const ComputeTaskPair> compTaskPairs,
            std::span<const PanelGeometryPair> panelGeometries,
            const EvalPoints<double> &evalPoints,
            const Eigen::Ref<Eigen::Array3d> &freeStream) {

  auto rhsView =
      RANGE(compTaskPairs.size()) | views::transform([&](std::size_t idx) {
        return assembleRhsImpl(compTaskPairs[idx].first,
                               panelGeometries[idx].first, evalPoints,
                               freeStream);
      });

  std::size_t mDims = evalPoints.mEvalPoints.rows();
  Eigen::VectorXd rhs(mDims);
  Eigen::VectorXd sourceStrength(mDims);
  std::size_t iPoints = 0;
  std::ranges::for_each(rhsView, [&](const auto &pts) {
    const auto rows = pts.first.rows();
    rhs.middleRows(iPoints, rows) = pts.first;
    sourceStrength.middleRows(iPoints, rows) = pts.second;
    iPoints += rows;
  });
  return {rhs, sourceStrength};
}
Eigen::MatrixXd
calculatePanelVelocities(const PanelGeometryPair &panelGeometryPair,
                         const Eigen::Ref<Eigen::ArrayXd> &doubletStrength,
                         const Eigen::Ref<Eigen::ArrayXd> &sourceStrength,
                         const Eigen::Ref<Eigen::ArrayXd> &freeStream) {
  auto &panel = panelGeometryPair.first;
  std::size_t nYSecs = panel.mSurface.nYsecs;
  std::size_t nXsecs = panel.mSurface.nXsecs;

  // Is this being reshaped properly? yes

  // Convert points to panel coordinate surface chord-wise from trailing edge
  // bottom to trailing edge top
  // is solution solved  in the same order>	depends on if face has the same
  // order, yes compute task <- centerpoints <- face

  // 1): Convert the centrepoints to panel reference frame
  Eigen::ArrayXXd xPoints(nXsecs, nYSecs);
  xPoints.setZero();
  for (int iY = 0; iY < nYSecs; iY++) {
    for (int iX = 1; iX < nXsecs; iX++) {
      xPoints(iX, iY) = (panel.centrePoints.row(iX + iY * nXsecs) -
                         panel.centrePoints.row((iX - 1) + iY * nXsecs))
                            .matrix()
                            .norm() +
                        xPoints(iX - 1, iY);
    }
  }
  Eigen::ArrayXXd yPoints(nXsecs, nYSecs);
  yPoints.setZero();
  for (int iX = 0; iX < nXsecs; iX++) {
    for (int iY = 1; iY < nYSecs; iY++) {
      yPoints(iX, iY) = (panel.centrePoints.row(iX + iY * nXsecs) -
                         panel.centrePoints.row(iX + (iY - 1) * nXsecs))
                            .matrix()
                            .norm() +
                        yPoints(iX, iY - 1);
    }
  }
  Eigen::ArrayXXd zPoints = panel.centrePoints.col(2).reshaped(nXsecs, nYSecs);

  // 2:) u = -d(mu)/d(x_l); v = -d(mu)/d(y_l); w = sigma
  Eigen::ArrayXXd fPoints(nXsecs, nYSecs);
  fPoints << doubletStrength.reshaped(nXsecs, nYSecs); // no minus

  Eigen::ArrayX3d inducedVelocities(nXsecs * nYSecs, 3);
  inducedVelocities << -centralDifference<true>(xPoints, fPoints).reshaped(),
      -centralDifference<false>(yPoints, fPoints).reshaped(), -sourceStrength;

  Eigen::ArrayX3d globalVelocites(nXsecs * nYSecs, 3);

  globalVelocites << rowwiseDotProduct(panel.tangentXVectors, freeStream),
      rowwiseDotProduct(panel.tangentYVectors, freeStream),
      rowwiseDotProduct(panel.normalVectors, freeStream);

  xPoints = panel.centrePoints.col(0).reshaped(nXsecs, nYSecs);

  return globalVelocites + inducedVelocities;
}

int main(int argc, char *argv[]) {
  namespace fs = std::filesystem;
  fs::path testDataLoc(std::string(SOURCE_DIR) + "/tests/test_data");
  fs::path filePath(testDataLoc.string() + "/0012_10000.txt");

  double aoa = 1.0;
  double angleOfAttack = aoa * M_PI / 180;
  Eigen::Array3d freeStream = {std::cos(angleOfAttack), 0,
                               std::sin(angleOfAttack)};
  print(freeStream);
  auto pset = readConvertedComponentsFromFile(filePath);
  align_wake_to_flow(pset, aoa);
  auto panelGeometries = calc_panel_geometry(pset);
  auto evalPoints = create_eval_points(panelGeometries);
  auto compTaskPairs = makeComputeTaskPairs(panelGeometries, evalPoints);
  auto out =
      assembleRhs(compTaskPairs, panelGeometries, evalPoints, freeStream);
  Eigen::VectorXd sourceStrength = std::move(out.second);
  Eigen::VectorXd rhs = std::move(out.first);
  Eigen::MatrixXd lhs = assembleLhs(compTaskPairs, panelGeometries, evalPoints);
  Eigen::ArrayXd doubletStrength = SparseSolver().solve(lhs, rhs);

  SourceDoubletSingle panelMethod(panelGeometries[0].first,
                                  panelGeometries[0].second, evalPoints,
                                  std::make_unique<SparseSolver>(), aoa);
  panelMethod.setFlowParams(aoa);
  panelMethod.run();

  auto calc = post_process_body(
      panelGeometries[0].first,
      calculatePanelVelocities(panelGeometries[0], doubletStrength,
                               sourceStrength, freeStream),
      {aoa, 1, 1}, aoa);
  std::cout << "aoa: " << calc.polars["aoa"] << " CL: " << calc.polars["CL"]
            << "\n";

  return 0;
}
