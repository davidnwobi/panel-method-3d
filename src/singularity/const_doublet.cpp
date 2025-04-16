

#include "singularity/const_doublet.hpp"
#include "compTask.hpp"
#include "singularity/const_doublet_far.hpp"
#include "singularity/internal_functions.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <limits>
#include <numbers>

// Eigen::ArrayXf DoubletP::calcInfluenceImpl(const ComputeTask &compTask) {
//   // std::abs(point(2)) < 1e-6,
//
//   using namespace Eigen;
//   const auto &fPoints = compTask.face.points;
//   ArrayXXf term1(compTask.points.rows(), fPoints.rows());
//   ArrayXf norms(4);
//   apply_adjacent_circular(
//       fPoints.rowwise().begin(), fPoints.rowwise().end(),
//       term1.colwise().begin(),
//       [&](const Eigen::Ref<const Eigen::RowVector3f> &node1,
//           const Eigen::Ref<const Eigen::RowVector3f> &node2) {
//         return J12(compTask.points, node1, node2);
//       });
//   apply_adjacent_circular(
//       fPoints.rowwise().begin(), fPoints.rowwise().end(), norms.begin(),
//       [&](const Eigen::Ref<const Eigen::RowVector3f> &node1,
//           const Eigen::Ref<const Eigen::RowVector3f> &node2) {
//         return (node2 - node1).matrix().norm();
//       });
//   for (int i = 0; i < 4; i++) {
//     if (norms(i) < 1e-10) {
//       term1.col(i) = Eigen::ArrayXf::Zero(term1.rows());
//     }
//   }
//   // print("facePoints: ", compTask.points.topRows(10));
//
//   ArrayXf inf = -1 / (4 *
//   std::numbers::pi_v<float>)*((term1).rowwise().sum()); return inf;
// }

Eigen::ArrayXf DoubletP::calcInfluenceImpl(const ComputeTask &compTask) {
  // std::abs(point(2)) < 1e-6,
  const auto &fPoints = compTask.face.points;
  int sides = fPoints.rows();
  using namespace Eigen;

  ArrayXf norms(sides);
  for (Eigen::Index i = 0; i < sides; i++) {
    norms[i] = (fPoints.row(i) - fPoints.row((i + 1) % sides)).matrix().norm();
  }
  size_t N = compTask.points.rows();
  ArrayXf J12_(N);

  float *j12 = const_cast<float *>(J12_.data());
  float *x = const_cast<float *>(compTask.points.data());
  float *y = const_cast<float *>(compTask.points.data() + N);
  float *z = const_cast<float *>(compTask.points.data() + N * 2);
  Array3Xf tPoints = fPoints.transpose();
  ArrayXf infMat(compTask.points.rows());
  infMat.setZero();

  for (Eigen::Index i = 0; i < sides; i++) {
    if (norms[i] > 1e-10) {
      float *node1 = const_cast<float *>(tPoints.data() + i * 3);
      float *node2 =
          const_cast<float *>(tPoints.data() + ((i + 1) % sides) * 3);
      J12_NORM(j12, x, y, z, node1, node2, N);
      infMat += J12_;
    }
  }
  infMat *= -1 / (4 * std::numbers::pi_v<float>);
  return infMat;
}

Eigen::ArrayXf DoubletP::calcInfluenceFarImpl(const ComputeTask &compTask) {
  return DoubletFar::calcInfluenceImpl(compTask);
}

float DoubletP::calcSelfInfluenceImpl(const ComputeTask &compTask) {
  UNUSED(compTask);
  return 0.5;
}
