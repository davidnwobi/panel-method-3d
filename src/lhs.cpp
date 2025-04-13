#include "infMat.hpp"
#include "mat_reader/mat_reader.hpp"
#include "panel_geo/panel_geo.hpp"
#include "singularity/const_doublet.hpp"
#include "singularity/const_source.hpp"
#include "singularity/const_source_doublet.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <compTask.hpp>
#include <concepts.hpp>
#include <ranges>

template <typename Derived1, typename Derived2>
void assembleLhsImpl(Eigen::MatrixBase<Derived1> &lhs,
                     Eigen::MatrixBase<Derived2> &rhs_mat,
                     const PanelGeometryPair &panelGeometries,
                     const EvalPoints<float> &evalPoints, std::size_t offset) {
#if (BENCHMARKING == 0)
  print(__PRETTY_FUNCTION__);
#endif

  std::size_t evalDims = evalPoints.mEvalPoints.rows();
  const auto &surf = panelGeometries.first;
  const auto &wake = panelGeometries.second;
  std::size_t surfDims = surf.centrePoints.rows();
  std::size_t wakeDims = wake.centrePoints.rows();

  ComputeTask compTask;
  ComputeTask compTaskSelf;
  compTaskSelf.indices = {0};
  compTaskSelf.points = (Eigen::ArrayX3f(1, 3) << 0, 0, 0).finished();

  // Only small improvement
  // 59 -> 55
  /*
   * Coaleasing function incurs suprisinly small gain. one would expect an
   * increase making only one j12 call instead of two significantly reduced
   * overhead*/
  for (auto i : RANGE(surfDims)) {
    createInfluenceComputeTask(compTask, surf, evalPoints, i);
    compTaskSelf.face = compTask.face;
    Eigen::Block lhs_block(lhs.derived(), 0, i, lhs.rows(), 1);
    Eigen::Block rhs_block(rhs_mat.derived(), 0, i, rhs_mat.rows(), 1);
    SourceDoubletP::calcInfluenceImpl(rhs_block, lhs_block, compTask);
    lhs(i + offset, i) = DoubletP::calcSelfInfluence(compTaskSelf);
  }

  // for (auto i : RANGE(surfDims)) {
  //   createInfluenceComputeTask(compTask, surf, evalPoints, i);
  //   compTaskSelf.face = compTask.face;
  //   lhs(Eigen::placeholders::all, i) = DoubletP::calcInfluence(compTask);
  //   rhs_mat(Eigen::placeholders::all, i) = SourceP::calcInfluence(compTask);
  //   lhs(i + offset, i) = DoubletP::calcSelfInfluence(compTaskSelf);
  // }

  if (wakeDims == 0) {
    return;
  }

  Eigen::ArrayXf wakeInfluence;
  int lowerFaceIdx;
  int upperFaceIdx;
  for (auto iWakeP : RANGE(wakeDims)) {
    createInfluenceComputeTask(compTask, wake, evalPoints, iWakeP);
    lowerFaceIdx = wake.mSurface.mTrailingEdgeIdx(iWakeP, 0);
    upperFaceIdx = wake.mSurface.mTrailingEdgeIdx(iWakeP, 1);
    wakeInfluence = DoubletP::calcInfluence(compTask);
    lhs(Eigen::placeholders::all, lowerFaceIdx).array() -= wakeInfluence;
    lhs(Eigen::placeholders::all, upperFaceIdx).array() += wakeInfluence;
  }

#if (BENCHMARKING == 0)
  print("OUT OF: ", __PRETTY_FUNCTION__);
#endif
}

template <typename Derived>
float sparsity(const Eigen::ArrayBase<Derived> &mat) {
  return ((float)(mat.abs() < 1e-6).count()) /
         ((float)(mat.rows() * mat.cols()));
}
std::tuple<Eigen::MatrixXf, Eigen::VectorXf, Eigen::VectorXf>
assembleLhs(std::span<const PanelGeometryPair> panelGeometries,
            const EvalPoints<float> &evalPoints,
            const Eigen::Ref<Eigen::Array3f> &freeStream) {

#if (BENCHMARKING == 0)
  print(__PRETTY_FUNCTION__);
#endif
  std::size_t mDims = evalPoints.mEvalPoints.rows();
  Eigen::MatrixXf lhs(mDims, mDims);
  Eigen::VectorXf rhs(mDims);
  rhs.setZero();
  Eigen::VectorXf sourceStrength(mDims);
  std::size_t iPoints = 0;
  for (auto i : RANGE(panelGeometries.size())) {

    const auto &surf = panelGeometries[i].first;
    const auto cols = surf.centrePoints.rows();
    Eigen::Block<MatrixXf, -1, -1, true> lhsBlock(lhs.derived(), 0, iPoints,
                                                  mDims, cols);
    Eigen::MatrixXf sourceInfluenceMat(mDims, cols);
    assembleLhsImpl(lhsBlock, sourceInfluenceMat, panelGeometries[i],
                    evalPoints, iPoints);
    sourceStrength.middleRows(iPoints, cols) =
        rowwiseDotProduct(surf.normalVectors, freeStream);
    rhs += -sourceInfluenceMat * sourceStrength.middleRows(iPoints, cols);
    iPoints += cols;
  }
#if (BENCHMARKING == 0)
  print("OUT OF: ", __PRETTY_FUNCTION__);
#endif
  return std::make_tuple(lhs, rhs, sourceStrength);
}
