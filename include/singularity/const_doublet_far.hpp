
#pragma once
#include "compTask.hpp"
#include "singularity/iconst_sing.hpp"
#include <Eigen/Core>
#include <numbers>

struct DoubletFar : IConstant3dSingularity<DoubletFar> {

  static Eigen::ArrayXd calcInfluenceImpl(const ComputeTask &compTask); 
  static double calcSelfInfluenceImpl(const ComputeTask &compTask);
};
