#pragma once
#include "compTask.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <iostream>
#include <ranges>
#include <span>
#include <utils/utils.hpp>
#include <vector>
namespace views = std::views;
#define RANGE(n) views::iota(0, (int)n)

template <typename Singularity, bool SelfInfluence> // concept constrain
Eigen::ArrayXXd makeInfluenceMatrix(int m, int n,
                                    std::vector<ComputeTask> &compTaskVec) {
  print(__PRETTY_FUNCTION__);
  static std::size_t SINGLE_INT = 0;
  Eigen::ArrayXXd infMat(m, n);
  infMat.setZero();

  std::vector<std::size_t> partioned_indices(compTaskVec[0].points.cols());
  std::iota(partioned_indices.begin(), partioned_indices.end(), 0);

  Eigen::ArrayXd solution;
  for (auto i : RANGE(compTaskVec.size())) {
    const auto &face = compTaskVec[i].face;

    // Reference diameter: largest diameter between vertices
    double maxDiameter =
        (face.points.row(0) - face.points.row(2)).matrix().norm();
    maxDiameter = std::max(
        maxDiameter, (face.points.row(0) - face.points.row(2)).matrix().norm());

    // get near and far points
    std::size_t IPoint = 0;
    auto distanceCondition = [&maxDiameter](double distance) {
      return distance < 5 * maxDiameter;
    };
    auto splitLoc = std::partition(
        partioned_indices.begin(), partioned_indices.end(),
        [&distanceCondition, &compTaskVec, i, &IPoint](std::size_t idx) {
          return distanceCondition(
              compTaskVec[i].points.col(IPoint++).matrix().norm());
        });
    auto splitLocMatrix = std::partition(
        compTaskVec[i].points.colwise().begin(),
        compTaskVec[i].points.colwise().end(),
        [&distanceCondition](const Eigen::Ref<Eigen::Vector3d> &point) {
          return distanceCondition(point.norm());
        });

    ComputeTask temp;
    temp.face = compTaskVec[i].face;
    std::size_t partition_size =
        std::distance(compTaskVec[i].points.colwise().begin(), splitLocMatrix);

    // update tempTask with near points;
    temp.points = Eigen::Ref<Eigen::Array3Xd>(
        compTaskVec[i].points.leftCols(partition_size));
    solution = Singularity::calcInfluence(temp);
    infMat(std::span<std::size_t>(partioned_indices.begin(), splitLoc), i) =
        solution;

    // update computeTask with far points;
    Index numCols = compTaskVec[i].points.cols() - partition_size;
    temp.points = Eigen::Ref<Eigen::Array3Xd>(
        compTaskVec[i].points.middleCols(partition_size, numCols));
    solution =
        Singularity::calcInfluence(temp); // is this allocationg new memory
    infMat(std::span<std::size_t>(splitLoc, partioned_indices.end()), i) =
        solution;

    if constexpr (SelfInfluence) {
      temp.points = (Eigen::Array3Xd(3, 1) << 0, 0, 0).finished();
      infMat(i, i) = Singularity::calcSelfInfluence(temp);
    }
  }
  // print(infMat.topLeftCorner(10, 10));
#if (BENCHMARKING == 0)
  print("OUT OF: ", __PRETTY_FUNCTION__);
#endif
  return infMat;
}
