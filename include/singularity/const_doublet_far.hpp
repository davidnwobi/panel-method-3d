
#pragma once
#include "compTask.hpp"
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

struct DoubletFar : IConstant3dSingularity<DoubletFar> {

  static Eigen::ArrayXd calcInfluenceImpl(const ComputeTask &compTask) {
    const Eigen::ArrayXd norms =
        (compTask.points - compTask.face.centrePoint.transpose().replicate(
                               compTask.points.rows(), 1))
            .matrix()
            .rowwise()
            .norm();
    return -compTask.face.area * (compTask.points.col(2)) /
           (4 * std::numbers::pi_v<double> * norms.pow(3));
  }

  static double calcSelfInfluenceImpl(const ComputeTask &compTask) {
    return calcInfluenceImpl(compTask)(0);
  }
};
