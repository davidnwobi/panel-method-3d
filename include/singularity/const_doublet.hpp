#pragma once
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

template <bool SelfInfluence = true>
struct DoubletP : IConstant3dSingularity<DoubletP, SelfInfluence> {
  using RowArray3d = Eigen::Array<double, 1, 3, Eigen::RowMajor>;
  static Eigen::ArrayXd term(const Eigen::ArrayX3d &points,
                             const RowArray3d &node1, const RowArray3d &node2) {

    using namespace Eigen;

    // 0 -> x ; 1 -> y ; 2 -> z
    auto ek = [&points](const RowArray3d &faceV) {
      return (points.col(0) - faceV(0)).square() + points.col(2).square();
    };
    auto hk = [&points](const RowArray3d &faceV) {
      return (points.col(0) - faceV(0)) * (points.col(1) - faceV(1));
    };
    auto m = [](const RowArray3d &point1, const RowArray3d &point2) {
      return (point2(1) - point1(1)) / (point2(0) - point1(0));
    };
    auto r = [&points](const RowArray3d &faceV) {
      return (points.rowwise() - faceV).matrix().rowwise().norm();
    };

    using cAr = const ArrayXd &;
    auto termP = [&points](double m, cAr e, cAr h, cAr r) {
      return (m * e - h).atan2(points.col(2) * r); // y/x
      // return (points.col(2) * r).atan2(m * e - h); // y/x
      //  return ((m * e - h) / (points.col(2) * r)).atan(); // y/x
      //   return ((points.col(2) * r) / (m * e - h)).atan(); // y/x
    };

    double m12 = m(node1, node2);

    ArrayXd e1 = ek(node1);
    ArrayXd e2 = ek(node2);
    ArrayXd h1 = hk(node1);
    ArrayXd h2 = hk(node2);
    ArrayXd r1 = r(node1);
    ArrayXd r2 = r(node2);

    ArrayXd pt1 = termP(m12, e1, h1, r1);
    ArrayXd pt2 = termP(m12, e2, h2, r2);
    return pt1 - pt2;
  }

  static Eigen::ArrayXd calcInfluenceImpl(const ComputeTask &compTask) {
    // std::abs(point(2)) < 1e-6,
    using namespace Eigen;
    ArrayXXd term1(compTask.indices.size(), compTask.face.points.rows());

    apply_adjacent_circular(
        compTask.face.points.rowwise().begin(),
        compTask.face.points.rowwise().end(), term1.colwise().begin(),
        [&](const RowArray3d &node1, const RowArray3d &node2) {
          return term(compTask.points, node1, node2);
        });
    ArrayXd inf = 1 / (4 * std::numbers::pi_v<double>)*(term1).rowwise().sum();

    // Self influence;
    if constexpr (SelfInfluence) {
      ;
      inf(compTask.face.faceIdx) = -0.5;
    }
    return inf;
  }
};
