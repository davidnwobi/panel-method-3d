#include "singularity/internal_functions.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>

using namespace Eigen;
using RowArray3d = Eigen::Array<double, 1, 3, Eigen::RowMajor>;
Eigen::ArrayXd R12(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                   const Eigen::Ref<const Eigen::Array3d> &node1,
                   const Eigen::Ref<const Eigen::Array3d> &node2) {
  const double dx = node2(0) - node1(0);
  const double dy = node2(1) - node1(1);
  // 2D distance between node1 and node2
  const double d = std::sqrt(dx * dx + dy * dy);

  Eigen::ArrayXd out(points.rows());

  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    const double px = points(i, 0);
    const double py = points(i, 1);

    // ( (px - node1.x)*dy - (py - node1.y)*dx ) / d
    out[i] = ((px - node1(0)) * dy - (py - node1(1)) * dx) / d;
  }

  return out;
}
// Eigen::ArrayXd R12_L(const Eigen::Ref<const Eigen::ArrayX3d> &points,
//                      const Eigen::Ref<const Eigen::Array3d> &node1,
//                      const Eigen::Ref<const Eigen::Array3d> &node2) {
//
//   ArrayXd t1 = points.col(0) - node1(0);
//   double t2 = node2(1) - node1(1);
//
//   ArrayXd t3 = points.col(1) - node1(1);
//   double t4 = node2(0) - node1(0);
//
//   double d = (node2 - node1).matrix().stableNorm();
//   ;
//
//   return ((t1 * t2) - (t3 * t4)) / d;
// }

Eigen::ArrayXd Q12(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                   const Eigen::Ref<const Eigen::Array3d> &node1,
                   const Eigen::Ref<const Eigen::Array3d> &node2) {
  const double x1 = node1(0), y1 = node1(1);
  const double x2 = node2(0), y2 = node2(1);

  const double dx = node2(0) - node1(0);
  const double dy = node2(1) - node1(1);
  // 2D distance between node1 and node2
  const double d = std::sqrt(dx * dx + dy * dy);

  Eigen::ArrayXd out(points.rows());
  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    double px = points(i, 0);
    double py = points(i, 1);
    double pz = points(i, 2);

    const double dx1 = px - x1;
    const double dy1 = py - y1;
    const double r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));

    const double dx2 = px - x2;
    const double dy2 = py - y2;
    const double r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));

    out[i] = std::log((r1 + r2 + d) / (r1 + r2 - d));
  }
  return out;
}
// Eigen::ArrayXd Q12_L(const Eigen::Ref<const Eigen::ArrayX3d> &points,
//                      const Eigen::Ref<const Eigen::Array3d> &node1,
//                      const Eigen::Ref<const Eigen::Array3d> &node2) {
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
// Eigen::ArrayXd J12(const Eigen::Ref<const Eigen::ArrayX3d> &points,
//                    const Eigen::Ref<const Eigen::Array3d> &node1,
//                    const Eigen::Ref<const Eigen::Array3d> &node2) {
//   const double x1 = node1(0), y1 = node1(1);
//   const double x2 = node2(0), y2 = node2(1);
//
//   // "m" slope function
//   auto slope = [&](double xA, double yA, double xB, double yB) {
//     const double dx = xB - xA;
//     const double dy = yB - yA;
//     // handle near‐vertical or near‐horizontal
//     if (std::fabs(dx) < 1e-14) {
//       return sgn(dy) * std::numeric_limits<double>::infinity();
//     }
//     if (std::fabs(dy) < 1e-14) {
//       return 0.0;
//     }
//     return dy / dx;
//   };

// precompute slope for the two nodes
const double m12 = slope(x1, y1, x2, y2);

// We'll accumulate the difference
Eigen::ArrayXd out(points.rows());

for (Eigen::Index i = 0; i < points.rows(); ++i) {
  double px = points(i, 0);
  double py = points(i, 1);
  double pz = points(i, 2);

  // ek(faceV) = (px - faceV.x)^2 + pz^2
  // hk(faceV) = (px - faceV.x)*(py - faceV.y)
  // r(faceV)  = sqrt( (px-faceV.x)^2 + (py-faceV.y)^2 + (pz-faceV.z)^2 )

  // For node1:
  const double dx1 = px - x1;
  const double dy1 = py - y1;
  const double e1 = dx1 * dx1 + pz * pz; // ek(node1)
  const double h1 = dx1 * dy1;           // hk(node1)
  const double r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));

  // For node2:
  const double dx2 = px - x2;
  const double dy2 = py - y2;
  const double e2 = dx2 * dx2 + pz * pz; // ek(node2)
  const double h2 = dx2 * dy2;           // hk(node2)
  const double r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));

  // termP(m, e, h, r) = atan( (m*e - h) / (pz*r) )
  // watch for pz=0.0?
  auto termP = [&](double m, double e, double h, double rr) {
    // if pz=0, you might want to handle that carefully
    const double denom = pz * rr;
    return std::atan((m * e - h) / denom);
  };

  // difference of termP for node1, node2
  out[i] = termP(m12, e1, h1, r1) - termP(m12, e2, h2, r2);
}

return out;
}
// Eigen::ArrayXd J12_OLD(const Eigen::Ref<const Eigen::ArrayX3d> &points,
//                        const Eigen::Ref<const Eigen::Array3d> &node1,
//                        const Eigen::Ref<const Eigen::Array3d> &node2) {
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
//       return sgn_<double>((point2(1) - point1(1))) *
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
//     return ((m * e - h) / (points.col(2) * r)).atan(); // y/x => y.atan2(x)
//   };
//
//   double m12 = m(node1, node2);
//
//   ArrayXd diff = termP(m12, ek(node1), hk(node1), r(node1)) -
//                  termP(m12, ek(node2), hk(node2), r(node2));
//   return diff;
// }
