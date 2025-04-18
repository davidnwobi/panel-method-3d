

#include "singularity/const_doublet.hpp"
#include "compTask.hpp"
#include "singularity/const_doublet_far.hpp"
#include "singularity/internal_functions.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <limits>
#include <numbers>

// Eigen::ArrayXd DoubletP::calcInfluenceImpl(const ComputeTask &compTask) {
//   // std::abs(point(2)) < 1e-6,
//
//   using namespace Eigen;
//   const auto &fPoints = compTask.face.points;
//   ArrayXXd term1(compTask.points.rows(), fPoints.rows());
//   ArrayXd norms(4);
//   apply_adjacent_circular(
//       fPoints.rowwise().begin(), fPoints.rowwise().end(),
//       term1.colwise().begin(),
//       [&](const Eigen::Ref<const Eigen::RowVector3d> &node1,
//           const Eigen::Ref<const Eigen::RowVector3d> &node2) {
//         return J12(compTask.points, node1, node2);
//       });
//   apply_adjacent_circular(
//       fPoints.rowwise().begin(), fPoints.rowwise().end(), norms.begin(),
//       [&](const Eigen::Ref<const Eigen::RowVector3d> &node1,
//           const Eigen::Ref<const Eigen::RowVector3d> &node2) {
//         return (node2 - node1).matrix().norm();
//       });
//   for (int i = 0; i < 4; i++) {
//     if (norms(i) < 1e-10) {
//       term1.col(i) = Eigen::ArrayXd::Zero(term1.rows());
//     }
//   }
//   // print("facePoints: ", compTask.points.topRows(10));
//
//   ArrayXd inf = -1 / (4 *
//   std::numbers::pi_v<double>)*((term1).rowwise().sum()); return inf;
// }

Eigen::ArrayXd DoubletP::calcInfluenceImpl(const ComputeTask &compTask) {
  // std::abs(point(2)) < 1e-6,
  const auto &fPoints = compTask.face.points;
  int sides = fPoints.rows();
  using namespace Eigen;

  ArrayXd norms(sides);
  for (Eigen::Index i = 0; i < sides; i++) {
    norms[i] = (fPoints.row(i) - fPoints.row((i + 1) % sides)).matrix().norm();
  }
  size_t N = compTask.points.rows();
  ArrayXd J12_(N);

  double *j12 = const_cast<double *>(J12_.data());
  double *x = const_cast<double *>(compTask.points.data());
  double *y = const_cast<double *>(compTask.points.data() + N);
  double *z = const_cast<double *>(compTask.points.data() + N * 2);
  Array3Xd tPoints = fPoints.transpose();
  ArrayXd infMat(compTask.points.rows());
  infMat.setZero();

  for (Eigen::Index i = 0; i < sides; i++) {
    if (norms[i] > 1e-10) {
      double *node1 = const_cast<double *>(tPoints.data() + i * 3);
      double *node2 =
          const_cast<double *>(tPoints.data() + ((i + 1) % sides) * 3);
      J12_NORM(j12, x, y, z, node1, node2, N);
      infMat += J12_;
    }
  }
  infMat *= -1 / (4 * std::numbers::pi_v<double>);
  return infMat;
}

Eigen::ArrayXd DoubletP::calcInfluenceFarImpl(const ComputeTask &compTask) {
  return DoubletFar::calcInfluenceImpl(compTask);
}

double DoubletP::calcSelfInfluenceImpl(const ComputeTask &compTask) {
  UNUSED(compTask);
  return 0.5;
}
