#pragma once
#include "compTask.hpp"
#include <Eigen/Core>

struct SourceDoubletP {
  static void calcInfluenceImpl(Eigen::Ref<Eigen::ArrayXf> sourceMat,
                                Eigen::Ref<Eigen::ArrayXf> doubletMat,
                                const ComputeTask &compTask);

  static float calcSelfInfluenceImpl();
};
