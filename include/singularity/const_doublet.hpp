#pragma once
#include "compTask.hpp"
#include "singularity/const_doublet_far.hpp"
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>
#include <valarray>

struct DoubletP : IConstant3dSingularity<DoubletP> {
  static Eigen::ArrayXd term(const Eigen::Ref<const Eigen::Array3Xd> &points,
                             const Eigen::Ref<const Eigen::Array3d> &node1,
                             const Eigen::Ref<const Eigen::Array3d> &node2);

  static Eigen::ArrayXd calcInfluenceImpl(const ComputeTask &compTask);

  static Eigen::ArrayXd calcInfluenceFarImpl(const ComputeTask &compTask);
  static double calcSelfInfluenceImpl(const ComputeTask &compTask);
};
