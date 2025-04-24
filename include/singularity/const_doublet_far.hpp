
#pragma once
#include "compTask.hpp"
#include "singularity/iconst_sing.hpp"
#include <Eigen/Core>
#include <numbers>

struct DoubletFar : IConstant3fSingularity<DoubletFar> {

  static Eigen::ArrayXf calcInfluenceImpl(const ComputeTask &compTask); 
  static float calcSelfInfluenceImpl(const ComputeTask &compTask);
};
