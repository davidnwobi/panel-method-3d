#pragma once
#include "compTask.hpp"
#include <Eigen/Core>
#include <iostream>
#include <ranges>
#include <vector>
namespace views = std::views;
#define RANGE(n) views::iota(0, (int)n)

template <typename Singularity> // concept constrain
Eigen::ArrayXXd
makeInfluenceMatrix(const std::vector<ComputeTask> &compTaskVec) {
  std::size_t n = compTaskVec.size();
  Eigen::ArrayXXd infMat(compTaskVec[0].points.rows(), n);
  for (auto i : RANGE(n)) {
    infMat.col(i) = Singularity::calcInfluence(compTaskVec[i]);
  }
  return infMat;
}
