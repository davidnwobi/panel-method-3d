
#include "singularity/const_source_doublet.hpp"
#include "singularity/const_source_far.hpp"
#include "singularity/internal_functions.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

// using RowArray3d = Eigen::Array<double, 1, 3, Eigen::RowMajor>;
// Eigen::ArrayXd part1term1(const Eigen::Ref<const Eigen::ArrayX3d> &points,
//                           const Eigen::Ref<const Eigen::Array3d> &node1,
//                           const Eigen::Ref<const Eigen::Array3d> &node2) {
//
//   using namespace Eigen;
//
//   ArrayXd t1 = points.col(0) - node1(0);
//   double t2 = node2(1) - node1(1);
//
//   ArrayXd t3 = points.col(1) - node1(1);
//   double t4 = node2(0) - node1(0);
//   double d = (node2 - node1).matrix().stableNorm();
//   ;
//
//   return ((t1 * t2) - (t3 * t4)) / d;
// }
//
// Eigen::ArrayXd part1term2(const Eigen::Ref<const Eigen::ArrayX3d> &points,
//                           const Eigen::Ref<const Eigen::Array3d> &node1,
//                           const Eigen::Ref<const Eigen::Array3d> &node2) {
//
//   ArrayXd r1 = (points - node1.transpose().replicate(points.rows(), 1))
//                    .matrix()
//                    .rowwise()
//                    .stableNorm();
//   ArrayXd r2 = (points - node2.transpose().replicate(points.rows(), 1))
//                    .matrix()
//                    .rowwise()
//                    .stableNorm();
//   double d = (node2 - node1).matrix().stableNorm();
//
//   return ((r1 + r2 + d) / (r1 + r2 - d)).log();
// }
// Eigen::ArrayXd part2term0(const Eigen::Ref<const Eigen::ArrayX3d> &points,
//                           const Eigen::Ref<const Eigen::Array3d> &node1,
//                           const Eigen::Ref<const Eigen::Array3d> &node2) {
//
//   using namespace Eigen;
//   // 0 -> x ; 1 -> y ; 2 -> z
//   auto ek = [&points](const Eigen::Ref<const Eigen::Array3d> &faceV) {
//     return (points.col(0) - faceV(0)).square() + points.col(2).square();
//   };
//   auto hk = [&points](const Eigen::Ref<const Eigen::Array3d> &faceV) {
//     return (points.col(0) - faceV(0)) * (points.col(1) - faceV(1));
//   };
//   auto m = [](const Eigen::Ref<const Eigen::Array3d> &point1,
//               const Eigen::Ref<const Eigen::Array3d> &point2) {
//     if (std::abs(point2(0) - point1(0)) < 1e-10) {
//       return sgn<double>((point2(1) - point1(1))) *
//              std::numeric_limits<double>::infinity();
//     }
//     if (std::abs(point2(1) - point1(1)) < 1e-10) {
//       return 0.0;
//     }
//     return (point2(1) - point1(1)) / (point2(0) - point1(0));
//   };
//   auto r = [&points](const Eigen::Ref<const Eigen::Array3d> &faceV) {
//     return (points - faceV.transpose().replicate(points.rows(), 1))
//         .matrix()
//         .rowwise()
//         .stableNorm();
//   };
//
//   using cAr = const Eigen::Ref<const ArrayXd> &;
//   auto termP = [&points](double m, cAr e, cAr h, cAr r) {
//     return (m * e - h).atan2(points.col(2) * r); // y/x => y.atan2(x)
//   };
//
//   double m12 = m(node1, node2);
//
//   ArrayXd diff = termP(m12, ek(node1), hk(node1), r(node1)) -
//                  termP(m12, ek(node2), hk(node2), r(node2));
//   return diff.sin().atan2(diff.cos());
// }
// Eigen::ArrayXd part2term(const Eigen::Ref<const Eigen::ArrayX3d> &points,
//                          const Eigen::Ref<const Eigen::Array3d> &node1,
//                          const Eigen::Ref<const Eigen::Array3d> &node2) {
//   Eigen::ArrayXd R12 = part1term1(points, node1, node2);
//   ArrayXd r1 = (points - node1.transpose().replicate(points.rows(), 1))
//                    .matrix()
//                    .rowwise()
//                    .stableNorm();
//   ArrayXd r2 = (points - node2.transpose().replicate(points.rows(), 1))
//                    .matrix()
//                    .rowwise()
//                    .stableNorm();
//
//   double d = (node2 - node1).matrix().stableNorm();
//   double C12 = (node2(0) - node1(0)) / d;
//   double S12 = (node2(1) - node1(1)) / d;
//   ArrayXd s12_1 =
//       (node1(0) - points.col(0)) * C12 + (node1(1) - points.col(1)) * S12;
//   ArrayXd s12_2 =
//       (node2(0) - points.col(0)) * C12 + (node2(1) - points.col(1)) * S12;
//
//   ArrayXd y = R12 * points.col(2) * (r1 * s12_2 - r2 * s12_1);
//   ArrayXd x = r1 * r2 * R12.square() + points.col(2).square() * s12_2 *
//   s12_1; return y.atan2(x);
// }

// Eigen::ArrayXd SourceP::calcInfluenceImpl(const ComputeTask &compTask) {
//
//   const auto &fPoints = compTask.face.points;
//   using namespace Eigen;
//
//   ArrayXXd part1t1(compTask.points.rows(), fPoints.rows());
//   ArrayXXd part1t2(compTask.points.rows(), fPoints.rows());
//   ArrayXXd part2t(compTask.points.rows(), fPoints.rows());
//   ArrayXd norms(4);
//
//   apply_adjacent_circular(
//       fPoints.rowwise().begin(), fPoints.rowwise().end(),
//       part1t1.colwise().begin(),
//       [&](const Eigen::Ref<const Eigen::RowVector3d> &node1,
//           const Eigen::Ref<const Eigen::RowVector3d> &node2) {
//         return R12(compTask.points, node1, node2);
//       });
//   apply_adjacent_circular(
//       fPoints.rowwise().begin(), fPoints.rowwise().end(),
//       part1t2.colwise().begin(),
//       [&](const Eigen::Ref<const Eigen::RowVector3d> &node1,
//           const Eigen::Ref<const Eigen::RowVector3d> &node2) {
//         return Q12(compTask.points, node1, node2);
//       });
//   apply_adjacent_circular(
//       fPoints.rowwise().begin(), fPoints.rowwise().end(),
//       part2t.colwise().begin(),
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
//       part1t1.col(i) = Eigen::ArrayXd::Zero(part1t1.rows());
//       part1t2.col(i) = Eigen::ArrayXd::Zero(part1t2.rows());
//       part2t.col(i) = Eigen::ArrayXd::Zero(part2t.rows());
//     }
//   }
//
//   ArrayXd term1 = (part1t1 * part1t2).rowwise().sum();
//   ArrayXd term2 = -compTask.points.col(2).abs() * (part2t.rowwise().sum());
//   return -1 / (4 * std::numbers::pi_v<double>)*(-term1 + term2);
// }
//
void SourceDoubletP::calcInfluenceImpl(Eigen::Ref<Eigen::ArrayXd> sourceMat,
                                       Eigen::Ref<Eigen::ArrayXd> doubletMat,
                                       const ComputeTask &compTask) {
  const auto &fPoints = compTask.face.points;
  int sides = fPoints.rows();
  using namespace Eigen;

  ArrayXd norms(sides);
  for (Eigen::Index i = 0; i < sides; i++) {
    norms[i] = (fPoints.row(i) - fPoints.row((i + 1) % sides)).matrix().norm();
  }

  ArrayXd R12_(compTask.points.rows());
  ArrayXd Q12_(compTask.points.rows());
  ArrayXd J12_(compTask.points.rows());
  for (Eigen::Index i = 0; i < fPoints.rows(); i++) {
    if (norms[i] > 1e-10) {
      // No real improvement over uncoalsesd. Slower for smaller data
      R12_Q12_J12(R12_, Q12_, J12_, compTask.points, fPoints.row(i),
                  fPoints.row((i + 1) % sides));
      sourceMat += -R12_ * Q12_;
      doubletMat += J12_;
    }
  }

  sourceMat += -compTask.points.col(2).abs() * doubletMat;
  sourceMat *= -1 / (4 * std::numbers::pi_v<double>);
  doubletMat *= -1 / (4 * std::numbers::pi_v<double>);
}

// Eigen::ArrayXd SourceP::calcInfluenceImpl(const ComputeTask &compTask) {
//
//   const auto &fPoints = compTask.face.points;
//   int sides = fPoints.rows();
//   using namespace Eigen;
//
//   ArrayXd norms(sides);
//   for (Eigen::Index i = 0; i < sides; i++) {
//     norms[i] = (fPoints.row(i) - fPoints.row((i + 1) %
//     sides)).matrix().norm();
//   }
//   ArrayXd infMat(compTask.points.rows());
//   ArrayXd temp(compTask.points.rows());
//   infMat.setZero();
//
//   for (Eigen::Index i = 0; i < fPoints.rows(); i++) {
//     if (norms[i] > 1e-10) {
//       infMat +=
//           -R12(compTask.points, fPoints.row(i), fPoints.row((i + 1) % sides))
//           * Q12(compTask.points, fPoints.row(i), fPoints.row((i + 1) %
//           sides));
//     }
//   }
//
//   temp.setZero();
//   for (Eigen::Index i = 0; i < sides; i++) {
//     if (norms[i] > 1e-10) {
//       temp +=
//           J12(compTask.points, fPoints.row(i), fPoints.row((i + 1) % sides));
//     }
//   }
//   infMat += -compTask.points.col(2).abs() * temp;
//   infMat *= -1 / (4 * std::numbers::pi_v<double>);
//   return infMat;
// }
double SourceDoubletP::calcSelfInfluenceImpl() { return 0.5; }
