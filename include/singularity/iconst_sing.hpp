#pragma once
#include "compTask.hpp"
#include <Eigen/Core>

template <typename Derived> struct IConstant3dSingularity {
  static Eigen::ArrayXf calcInfluence(const ComputeTask &compTask) {
    return Derived::calcInfluenceImpl(compTask);
  }
  static Eigen::ArrayXf calcInfluenceFar(const ComputeTask &compTask) {
    return Derived::calcInfluenceFarImpl(compTask);
  }
  static float calcSelfInfluence(const ComputeTask &compTask) {
    return Derived::calcSelfInfluenceImpl(compTask);
  }
};
