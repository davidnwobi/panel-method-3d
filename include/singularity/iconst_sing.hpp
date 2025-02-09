#pragma once
#include "compTask.hpp"
#include <Eigen/Core>

template <typename Derived> struct IConstant3dSingularity {
  static Eigen::ArrayXd calcInfluence(const ComputeTask &compTask) {
    return Derived::calcInfluenceImpl(compTask);
  }
  static double calcSelfInfluence(const ComputeTask &compTask) {
    return Derived::calcSelfInfluenceImpl(compTask);
  }
};
