#pragma once
#include "compTask.hpp"
#include <Eigen/Core>

struct SourceDoubletP {
  static void calcInfluenceImpl(Eigen::Ref<Eigen::ArrayXd> sourceMat,
                                Eigen::Ref<Eigen::ArrayXd> doubletMat,
                                const ComputeTask &compTask);

  static double calcSelfInfluenceImpl();
};
