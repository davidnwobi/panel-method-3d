#pragma once
#include "compTask.hpp"
#include "singularity/const_doublet_far.hpp"
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>
#include <valarray>

struct DoubletP : IConstant3fSingularity<DoubletP> {
  static Eigen::ArrayXf term(const Eigen::Ref<const Eigen::ArrayX3f> &points,
                             const Eigen::Ref<const Eigen::Array3f> &node1,
                             const Eigen::Ref<const Eigen::Array3f> &node2);

  static Eigen::ArrayXf calcInfluenceImpl(const ComputeTask &compTask);

  static Eigen::ArrayXf calcInfluenceFarImpl(const ComputeTask &compTask);
  static float calcSelfInfluenceImpl(const ComputeTask &compTask);
};
