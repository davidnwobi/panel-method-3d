#include "infMat.hpp"
#include "singularity/const_source.hpp"
#include <Eigen/Core>
#include <compTask.hpp>
#include <concepts.hpp>
#include <ranges>
#include <utility>

std::pair<Eigen::VectorXf, Eigen::VectorXf>
assembleRhsImpl(std::span<const ComputeTask> surfacePanelCompTasks,
                const PanelGeometry<SurfacePanel> &surfacePanelGeo,
                const EvalPoints<float> &evalPoints,
                const Eigen::Ref<Eigen::Array3f> &freeStream) {

  std::size_t evalDims = evalPoints.mEvalPoints.rows();
  std::size_t surfDims = surfacePanelCompTasks.size();
  Eigen::MatrixXf sourceInfluenceMat = makeInfluenceMatrix<SourceP, true>(
      evalDims, surfDims, surfacePanelCompTasks);
  Eigen::VectorXf sourceStrength =
      rowwiseDotProduct(surfacePanelGeo.normalVectors, freeStream);
  return {-(sourceInfluenceMat * sourceStrength), sourceStrength};
}

std::pair<Eigen::VectorXf, Eigen::VectorXf>
assembleRhs(std::span<const ComputeTaskPair> compTaskPairs,
            std::span<const PanelGeometryPair> panelGeometries,
            const EvalPoints<float> &evalPoints,
            const Eigen::Ref<Eigen::Array3f> &freeStream) {

  auto rhsView =
      RANGE(compTaskPairs.size()) | views::transform([&](std::size_t idx) {
        return assembleRhsImpl(compTaskPairs[idx].first,
                               panelGeometries[idx].first, evalPoints,
                               freeStream);
      });

  std::size_t mDims = evalPoints.mEvalPoints.rows();
  Eigen::VectorXf rhs(mDims);

  rhs.setZero();
  Eigen::VectorXf sourceStrength(mDims);
  std::size_t iPoints = 0;
  std::ranges::for_each(rhsView, [&](const auto &pts) {
    rhs += pts.first;
    const auto rows = pts.second.rows();
    sourceStrength.middleRows(iPoints, rows) = pts.second;
    iPoints += rows;
  });
  return {rhs, sourceStrength};
}
