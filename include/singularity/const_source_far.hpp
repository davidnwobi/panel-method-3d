
#pragma once
#include "compTask.hpp"
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

struct SourceFar : IConstant3dSingularity<SourceFar> {

  static Eigen::ArrayXd calcInfluenceImpl(const ComputeTask &compTask) {
    const Eigen::ArrayXd norms =
        (compTask.points -
         compTask.face.centrePoint.replicate(1, compTask.points.cols()))
            .matrix()
            .colwise()
            .norm();
    return -compTask.face.area /
           (4 * std::numbers::pi_v<double> * norms).array();
  }

  static double calcSelfInfluenceImpl(const ComputeTask &compTask) {
    return calcInfluenceImpl(compTask)(0);
  }
};
