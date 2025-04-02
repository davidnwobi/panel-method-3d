#include "singularity/internal_functions.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>

using namespace Eigen;
using RowArray3d = Eigen::Array<double, 1, 3, Eigen::RowMajor>;
Eigen::ArrayXd R12(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                   const Eigen::Ref<const Eigen::Array3d> &node1,
                   const Eigen::Ref<const Eigen::Array3d> &node2) {

  ArrayXd t1 = points.col(0) - node1(0);
  double t2 = node2(1) - node1(1);

  ArrayXd t3 = points.col(1) - node1(1);
  double t4 = node2(0) - node1(0);

  double d = (node2 - node1).matrix().stableNorm();
  ;

  return ((t1 * t2) - (t3 * t4)) / d;
}

Eigen::ArrayXd Q12(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                   const Eigen::Ref<const Eigen::Array3d> &node1,
                   const Eigen::Ref<const Eigen::Array3d> &node2) {

  ArrayXd r1 = (points - node1.transpose().replicate(points.rows(), 1))
                   .matrix()
                   .rowwise()
                   .stableNorm();
  ArrayXd r2 = (points - node2.transpose().replicate(points.rows(), 1))
                   .matrix()
                   .rowwise()
                   .stableNorm();
  double d = (node2 - node1).matrix().stableNorm();

  return ((r1 + r2 + d) / (r1 + r2 - d)).log();
}
Eigen::ArrayXd J12_OLD(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                       const Eigen::Ref<const Eigen::Array3d> &node1,
                       const Eigen::Ref<const Eigen::Array3d> &node2) {

  using namespace Eigen;
  // 0 -> x ; 1 -> y ; 2 -> z
  auto ek = [&points](const Eigen::Ref<const Eigen::Array3d> &faceV) {
    return (points.col(0) - faceV(0)).square() + points.col(2).square();
  };
  auto hk = [&points](const Eigen::Ref<const Eigen::Array3d> &faceV) {
    return (points.col(0) - faceV(0)) * (points.col(1) - faceV(1));
  };
  auto m = [](const Eigen::Ref<const Eigen::Array3d> &point1,
              const Eigen::Ref<const Eigen::Array3d> &point2) {
    if (std::abs(point2(0) - point1(0)) < 1e-10) {
      return sgn_<double>((point2(1) - point1(1))) *
             std::numeric_limits<double>::infinity();
    }
    if (std::abs(point2(1) - point1(1)) < 1e-10) {
      return 0.0;
    }
    return (point2(1) - point1(1)) / (point2(0) - point1(0));
  };
  auto r = [&points](const Eigen::Ref<const Eigen::Array3d> &faceV) {
    return (points - faceV.transpose().replicate(points.rows(), 1))
        .matrix()
        .rowwise()
        .stableNorm();
  };

  using cAr = const Eigen::Ref<const ArrayXd> &;
  auto termP = [&points](double m, cAr e, cAr h, cAr r) {
    return ((m * e - h) / (points.col(2) * r)).atan(); // y/x => y.atan2(x)
  };

  double m12 = m(node1, node2);

  ArrayXd diff = termP(m12, ek(node1), hk(node1), r(node1)) -
                 termP(m12, ek(node2), hk(node2), r(node2));
  return diff;
}
Eigen::ArrayXd J12(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                   const Eigen::Ref<const Eigen::Array3d> &node1,
                   const Eigen::Ref<const Eigen::Array3d> &node2) {
  // return J12_OLD(points, node1, node2);
  Eigen::ArrayXd R12_ = R12(points, node1, node2);
  ArrayXd r1 = (points - node1.transpose().replicate(points.rows(), 1))
                   .matrix()
                   .rowwise()
                   .stableNorm();
  ArrayXd r2 = (points - node2.transpose().replicate(points.rows(), 1))
                   .matrix()
                   .rowwise()
                   .stableNorm();

  double d = (node2 - node1).matrix().stableNorm();
  double C12 = (node2(0) - node1(0)) / d;
  double S12 = (node2(1) - node1(1)) / d;
  ArrayXd s12_1 =
      (node1(0) - points.col(0)) * C12 + (node1(1) - points.col(1)) * S12;
  ArrayXd s12_2 =
      (node2(0) - points.col(0)) * C12 + (node2(1) - points.col(1)) * S12;

  ArrayXd y = R12_ * points.col(2).abs() * (r1 * s12_2 - r2 * s12_1);
  ArrayXd x = r1 * r2 * R12_.square() + points.col(2).square() * s12_2 * s12_1;
  return y.atan2(x);
}

Eigen::ArrayXi baryCheck(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                         const Eigen::Ref<const Eigen::Array3d> &A,
                         const Eigen::Ref<const Eigen::Array3d> &B,
                         const Eigen::Ref<const Eigen::Array3d> &C) {
  ArrayXd v0 = C - A;
  ArrayXd v1 = B - A;
  ArrayX3d v2 = points - A.transpose().replicate(points.rows(), 1);

  double dot00 = v0.matrix().dot(v0.matrix());
  double dot01 = v0.matrix().dot(v1.matrix());
  ArrayXd dot02 = rowwiseDotProduct(v2, v0);
  double dot11 = v1.matrix().dot(v1.matrix());
  ArrayXd dot12 = rowwiseDotProduct(v2, v1);
  double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
  ArrayXd u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  ArrayXd v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  return (u >= 0 && v >= 0 && (u + v) < 1)
      .select(ArrayXi::Ones(points.rows(), 1), ArrayXi::Zero(points.rows(), 1));
}
