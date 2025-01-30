#pragma once
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

template <bool SelfInfluence = true>
struct SourceP : IConstant3dSingularity<SourceP, SelfInfluence> {

  using RowArray3d = Eigen::Array<double, 1, 3, Eigen::RowMajor>;
  static Eigen::ArrayXd part1term1(const Eigen::ArrayX3d &points,
                                   const RowArray3d &node1,
                                   const RowArray3d &node2) {

    using namespace Eigen;

    ArrayXd t1 = points.col(0) - node1(0);
    double t2 = node2(1) - node1(1);

    ArrayXd t3 = points.col(1) - node1(1);
    double t4 = node2(0) - node1(0);

    double d = (node2 - node1).matrix().norm();
    ;

    return ((t1 * t2) - (t3 * t4)) / d;
  }

  static Eigen::ArrayXd part1term2(const Eigen::ArrayX3d &points,
                                   const RowArray3d &node1,
                                   const RowArray3d &node2) {
    // auto point = mEvalPoints.get().mEvalPoints.row(centerPointIdx);
    // auto node1 =
    // mPanelGeoRef.get().combinedSurface.getPoints().row(nodeIdx1); auto node2
    // = mPanelGeoRef.get().combinedSurface.getPoints().row(nodeIdx2);

    ArrayXd r1 = (points.rowwise() - node1).matrix().rowwise().norm();
    ArrayXd r2 = (points.rowwise() - node2).matrix().rowwise().norm();
    double d = (node2 - node1).matrix().norm();

    return ((r1 + r2 + d) / (r1 + r2 - d)).log();
  }

  static Eigen::ArrayXd part2term(const Eigen::ArrayX3d &points,
                                  const RowArray3d &node1,
                                  const RowArray3d &node2) {

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
      //    return (points.col(2) * r).atan2(m * e - h); // y/x
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

    using namespace Eigen;

    ArrayXXd part1t1(compTask.indices.size(), compTask.face.points.rows());
    ArrayXXd part1t2(compTask.indices.size(), compTask.face.points.rows());
    ArrayXXd part2t(compTask.indices.size(), compTask.face.points.rows());

    apply_adjacent_circular(
        compTask.face.points.rowwise().begin(),
        compTask.face.points.rowwise().end(), part1t1.colwise().begin(),
        [&](const RowArray3d &node1, const RowArray3d &node2) {
          return part1term1(compTask.points, node1, node2);
        });

    apply_adjacent_circular(
        compTask.face.points.rowwise().begin(),
        compTask.face.points.rowwise().end(), part1t2.colwise().begin(),
        [&](const RowArray3d &node1, const RowArray3d &node2) {
          return part1term2(compTask.points, node1, node2);
        });

    apply_adjacent_circular(
        compTask.face.points.rowwise().begin(),
        compTask.face.points.rowwise().end(), part2t.colwise().begin(),
        [&](const RowArray3d &node1, const RowArray3d &node2) {
          return part2term(compTask.points, node1, node2);
        });

    ArrayXd term1 = (part1t1 * part1t2).rowwise().sum();
    ArrayXd term2 = -compTask.points.col(2).abs() * (part2t.rowwise().sum());

    return -1 / (4 * std::numbers::pi_v<double>)*(term1 + term2);
  }
};
