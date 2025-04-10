#include "infMat.hpp"
#include "singularity/const_source.hpp"
#include <Eigen/Core>
#include <compTask.hpp>
#include <concepts.hpp>
#include <ranges>
#include <utility>

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

std::pair<Eigen::VectorXd, Eigen::VectorXd>
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

  rhs.setZero();
  Eigen::VectorXd sourceStrength(mDims);
  std::size_t iPoints = 0;
  std::ranges::for_each(rhsView, [&](const auto &pts) {
    rhs += pts.first;
    const auto rows = pts.second.rows();
    sourceStrength.middleRows(iPoints, rows) = pts.second;
    iPoints += rows;
  });
  return {rhs, sourceStrength};
}
