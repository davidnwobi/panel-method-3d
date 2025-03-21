#pragma once
#include "singularity/const_source_far.hpp"
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

struct SourceP : IConstant3dSingularity<SourceP> {

  using RowArray3d = Eigen::Array<double, 1, 3, Eigen::RowMajor>;


  static Eigen::ArrayXd calcInfluenceImpl(const ComputeTask &compTask);

  static Eigen::ArrayXd calcInfluenceFarImpl(const ComputeTask &compTask);
  static double calcSelfInfluenceImpl(const ComputeTask &compTask);
};
