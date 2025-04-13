
#include "compTask.hpp"
#include "singularity/const_source_far.hpp"
#include <Eigen/Core>
#include <numbers>

   Eigen::ArrayXf SourceFar::calcInfluenceImpl(const ComputeTask &compTask) {

    const Eigen::ArrayXf norms =
        (compTask.points - compTask.face.centrePoint.transpose().replicate(
                               compTask.points.rows(), 1))
            .matrix()
            .rowwise()
            .norm();
    return -compTask.face.area /
           (4 * std::numbers::pi_v<float> * norms).array();
  }

   float SourceFar::calcSelfInfluenceImpl(const ComputeTask &compTask) {
    return calcInfluenceImpl(compTask)(0);
  }
