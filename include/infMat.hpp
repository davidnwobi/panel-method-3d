#pragma once
#include "compTask.hpp"
#include "singularity/const_doublet.hpp"
#include <Eigen/Core>
#include <iostream>
#include <ranges>
#include <utils/utils.hpp>
#include <vector>
namespace views = std::views;
#define RANGE(n) views::iota(0, (int)n)
typedef Array<bool, Dynamic, 1> ArrayXb;
template <typename Singularity, bool SelfInfluence> // concept constrain
Eigen::ArrayXXd makeInfluenceMatrix(int m, int n,
                                    std::span<const ComputeTask> compTaskVec) {
#if (BENCHMARKING == 0)
  print(__PRETTY_FUNCTION__);
#endif
  Eigen::ArrayXXd infMat(m, n);
  infMat.setZero();

  std::vector<std::size_t> partioned_indices(compTaskVec[0].points.rows());
  Eigen::ArrayXd norms(compTaskVec[0].points.rows());
  Eigen::ArrayXi isFarAway(norms.size());
  Eigen::ArrayXd solution;
  ComputeTask temp;
  temp.points.conservativeResizeLike(compTaskVec[0].points);

  double limit = 10;
  for (auto i : RANGE(compTaskVec.size())) {
    const auto &face = compTaskVec[i].face;

    // Reference diameter
    double maxDiameter =
        (face.points.row(0) - face.points.row(2)).matrix().norm();
    maxDiameter = std::max(
        maxDiameter, (face.points.row(0) - face.points.row(2)).matrix().norm());
    // split into near and far point
    norms = compTaskVec[i].points.matrix().rowwise().norm();
    isFarAway = (norms < limit * maxDiameter) // Distance Condition
                    .select(Eigen::ArrayXi::Ones(norms.size()),
                            Eigen::ArrayXi::Zero(norms.size()));

    std::iota(partioned_indices.begin(), partioned_indices.end(), 0);
    auto splitLoc = std::partition(
        partioned_indices.begin(), partioned_indices.end(),
        [&isFarAway](std::size_t idx) { return isFarAway[idx]; });

    temp.face = compTaskVec[i].face;

    // update tempTask with far points;
    auto nearIndex =
        std::span<std::size_t>(partioned_indices.begin(), splitLoc);
    temp.points = compTaskVec[i].points(nearIndex, Eigen::placeholders::all);
    infMat(nearIndex, i) = Singularity::calcInfluence(temp);

    // update tempTask with far points;
    auto farIndex = std::span<std::size_t>(splitLoc, partioned_indices.end());
    temp.points = compTaskVec[i].points(farIndex, Eigen::placeholders::all);

    if constexpr (std::is_same_v<DoubletP, Singularity>) {
      infMat(farIndex, i) = Singularity::calcInfluence(temp);
    } else {
      infMat(farIndex, i) = Singularity::calcInfluence(temp);
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
