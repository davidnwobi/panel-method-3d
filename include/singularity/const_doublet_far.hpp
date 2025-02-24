
#pragma once
#include "compTask.hpp"
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

struct DoubletFar : IConstant3dSingularity<DoubletFar> {

  static Eigen::ArrayXd calcInfluenceImpl(const ComputeTask &compTask) {
    const Eigen::ArrayXd norms = (compTask.points.transpose() -
                                  compTask.face.centrePoint.replicate(
                                      1, compTask.points.transpose().cols()))
                                     .matrix()
                                     .colwise()
                                     .norm();
    print(norms.topRows(11));
    return -compTask.face.centrePoint(2) * compTask.face.area /
           (4 * std::numbers::pi_v<double> * norms.pow(3));
  }

  static double calcSelfInfluenceImpl(const ComputeTask &compTask) {
    return calcInfluenceImpl(compTask)(0);
  }
};
