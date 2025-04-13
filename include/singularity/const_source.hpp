#pragma once
#include "singularity/const_source_far.hpp"
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

struct SourceP : IConstant3dSingularity<SourceP> {

  using RowArray3f = Eigen::Array<float, 1, 3, Eigen::RowMajor>;


  static Eigen::ArrayXf calcInfluenceImpl(const ComputeTask &compTask);

  static Eigen::ArrayXf calcInfluenceFarImpl(const ComputeTask &compTask);
  static float calcSelfInfluenceImpl(const ComputeTask &compTask);
};
