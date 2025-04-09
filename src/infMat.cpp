#include <Eigen/Core>
#include <compTask.hpp>
#include <infMat.hpp>
#include <singularity/const_doublet.hpp>
#include <singularity/const_source.hpp>
#include <iostream>
#include <ranges>
#include <utils/utils.hpp>
#include <vector>
namespace views = std::views;
template <typename Singularity, bool SelfInfluence> // concept constrain
Eigen::ArrayXXd makeInfluenceMatrixImpl(int m, int n, std::span<const ComputeTask> compTaskVec) {
#if (BENCHMARKING == 0)
  print(__PRETTY_FUNCTION__);
#endif
  Eigen::ArrayXXd infMat(m, n);
  infMat.setZero();

  std::vector<std::size_t> partioned_indices(compTaskVec[0].points.rows());
  Eigen::ArrayXd norms(compTaskVec[0].points.rows());
  Eigen::ArrayXi isNear(norms.size());
  Eigen::ArrayXd solution;
  ComputeTask temp;
  temp.points.conservativeResizeLike(compTaskVec[0].points);

  double limit = 5;
  for (auto i : RANGE(compTaskVec.size())) {
    const auto &face = compTaskVec[i].face;

    // Reference diameter
    double maxDiameter = std::max(
        (face.points.row(0) - face.points.row(2)).matrix().norm(), 
      (face.points.row(1) - face.points.row(3)).matrix().norm());
    
    // split into near and far point
    norms = compTaskVec[i].points.matrix().rowwise().norm();
    isNear = (norms < (limit * maxDiameter)) // Distance Condition
                    .select(Eigen::ArrayXi::Ones(norms.size()),
                            Eigen::ArrayXi::Zero(norms.size()));

    std::iota(partioned_indices.begin(), partioned_indices.end(), 0);
    auto splitLoc = std::partition(
        partioned_indices.begin(), partioned_indices.end(),
        [&isNear](std::size_t idx) { return isNear[idx]; });

    temp.face = compTaskVec[i].face;

    // update tempTask with far points;
    auto nearIndex = std::span<std::size_t>(partioned_indices.begin(), splitLoc);
    if (nearIndex.size() > 0){
      temp.points = compTaskVec[i].points(nearIndex, Eigen::placeholders::all);
      infMat(nearIndex, i) = Singularity::calcInfluence(temp);
    }

    // update tempTask with far points;k
    auto farIndex = std::span<std::size_t>(splitLoc, partioned_indices.end());
      //print(((double) farIndex.size() )/((double) (compTaskVec[0].points.rows())));
    if (farIndex.size() > 0){
      temp.points = compTaskVec[i].points(farIndex, Eigen::placeholders::all);
      infMat(farIndex, i) = Singularity::calcInfluenceFar(temp);
    }
    // is this allocationg new memory
    if constexpr (SelfInfluence) {
      ComputeTask temp;
      temp.face = compTaskVec[i].face;
      temp.indices = {0};
      temp.points = (Eigen::ArrayX3d(1, 3) << 0, 0, 0).finished();
      infMat(temp.face.faceIdx, i) = Singularity::calcSelfInfluence(temp);
    }
    std::iota(partioned_indices.begin(), partioned_indices.end(), 0);
  }
  // print(infMat.topLeftCorner(10, 10));
#if (BENCHMARKING == 0)
  print("OUT OF: ", __PRETTY_FUNCTION__);
#endif
  return infMat;
}
template <typename Singularity, bool SelfInfluence> // concept constrain
Eigen::ArrayXXd makeInfluenceMatrix(int m, int n, std::span<const ComputeTask> compTaskVec) {
#if (BENCHMARKING == 0)
  print(__PRETTY_FUNCTION__);
#endif
  Eigen::ArrayXXd infMat(m, n);
  infMat.setZero();

  std::vector<std::size_t> partioned_indices(compTaskVec[0].points.rows());

  for (auto i : RANGE(compTaskVec.size())) {

    infMat(Eigen::placeholders::all, i) = Singularity::calcInfluence(compTaskVec[i]);

    // is this allocationg new memory
    if constexpr (SelfInfluence) {
      ComputeTask temp;
      temp.face = compTaskVec[i].face;
      temp.indices = {0};
      temp.points = (Eigen::ArrayX3d(1, 3) << 0, 0, 0).finished();
      infMat(temp.face.faceIdx, i) = Singularity::calcSelfInfluence(temp);
    }
  }
  // print(infMat.topLeftCorner(10, 10));
#if (BENCHMARKING == 0)
  print("OUT OF: ", __PRETTY_FUNCTION__);
#endif
  return infMat;
}

template Eigen::ArrayXXd makeInfluenceMatrix<DoubletP, true>(int m, int n, std::span<const ComputeTask> compTaskVec);
template Eigen::ArrayXXd makeInfluenceMatrix<DoubletP, false>(int m, int n, std::span<const ComputeTask> compTaskVec);
template Eigen::ArrayXXd makeInfluenceMatrix<SourceP, true>(int m, int n, std::span<const ComputeTask> compTaskVec);
template Eigen::ArrayXXd makeInfluenceMatrix<SourceP, false>(int m, int n, std::span<const ComputeTask> compTaskVec);

