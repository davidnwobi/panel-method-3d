#pragma once
#include "compTask.hpp"
#include <Eigen/Core>
#include <iostream>
#include <ranges>
#include <utils/utils.hpp>
#include <vector>
namespace views = std::views;
#define RANGE(n) views::iota(0, (int)n)

template <typename Singularity, bool SelfInfluence> // concept constrain
Eigen::ArrayXXd
makeInfluenceMatrix(int m, int n, const std::vector<ComputeTask> &compTaskVec) {
  print(__PRETTY_FUNCTION__);
  Eigen::ArrayXXd infMat(m, n);
  infMat.setZero();
  auto sortable = compTaskVec[0].indices;
  std::sort(sortable.begin(), sortable.end());
  // PRINT_RANGE(sortable);
  for (auto i : RANGE(compTaskVec.size())) {

    infMat(compTaskVec[i].indices, i) =
        Singularity::calcInfluence(compTaskVec[i]);
    ;

    if constexpr (SelfInfluence) {
      ComputeTask temp;
      temp.face = compTaskVec[i].face;
      temp.indices = {0};
      temp.points = (Eigen::ArrayX3d(1, 3) << 0, 0, 0).finished();
      // infMat(i, i) = Singularity::calcSelfInfluence(temp);
    }
  }
  // print(infMat.topLeftCorner(10, 10));
  print("OUT OF: ", __PRETTY_FUNCTION__);
  return infMat;
}
