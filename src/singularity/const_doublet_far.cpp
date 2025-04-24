

#include "compTask.hpp"
#include "singularity/const_doublet_far.hpp"
#include <Eigen/Core>
#include <numbers>


   Eigen::ArrayXf DoubletFar::calcInfluenceImpl(const ComputeTask &compTask) {
    const Eigen::ArrayXf norms =
        (compTask.points - compTask.face.centrePoint.transpose().replicate(
                               compTask.points.rows(), 1))
            .matrix()
            .rowwise()
            .norm();
    return -compTask.face.area * (compTask.points.col(2)) /
           (4 * std::numbers::pi_v<float> * norms.pow(3));
  }

  float DoubletFar::calcSelfInfluenceImpl(const ComputeTask &compTask) {
    return calcInfluenceImpl(compTask)(0);
  }
