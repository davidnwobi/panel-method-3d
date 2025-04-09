#include <Eigen/Core>
#include <ranges>
#include <compTask.hpp>
#include <concepts.hpp>
#include "utils/utils.hpp"
#include "singularity/const_doublet.hpp"
#include "infMat.hpp"

Eigen::MatrixXd
assembleLhsImpl(std::span<const ComputeTask> surfacePanelCompTasks,
                std::span<const ComputeTask> wakePanelCompTasks,
                const PanelGeometry<WakePanel> &wakePanelGeo,
                const EvalPoints<double> &evalPoints) {

  std::size_t evalDims = evalPoints.mEvalPoints.rows();
  std::size_t surfDims = surfacePanelCompTasks.size();
  std::size_t wakeDims = wakePanelCompTasks.size();
  print("Eval Dims: ", evalDims, "Surf Dims: ", surfDims,
        "Wake Dims: ", wakeDims);
  Eigen::MatrixXd surfaceInfluenceMatrix = makeInfluenceMatrix<DoubletP, true>(
      evalDims, surfDims, surfacePanelCompTasks);

  if (wakeDims == 0) {
    return surfaceInfluenceMatrix;
  }
  Eigen::MatrixXd wakeInfluenceMatrix = makeInfluenceMatrix<DoubletP, false>(
      evalDims, wakeDims, wakePanelCompTasks);
  //  combine source and wake
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
