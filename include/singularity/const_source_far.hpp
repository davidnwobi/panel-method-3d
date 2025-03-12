
#pragma once
#include "compTask.hpp"
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

struct SourceFar : IConstant3dSingularity<SourceFar> {

  static Eigen::ArrayXd calcInfluenceImpl(const ComputeTask &compTask);

  static double calcSelfInfluenceImpl(const ComputeTask &compTask);
};
