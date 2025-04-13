
#pragma once
#include "compTask.hpp"
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

struct SourceFar : IConstant3dSingularity<SourceFar> {

  static Eigen::ArrayXf calcInfluenceImpl(const ComputeTask &compTask);

  static float calcSelfInfluenceImpl(const ComputeTask &compTask);
};
