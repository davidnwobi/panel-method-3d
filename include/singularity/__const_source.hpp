#pragma once
#include "singularity/const_source_far.hpp"
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

struct SourceP : IConstant3dSingularity<SourceP> {

  using RowArray3d = Eigen::Array<double, 1, 3, Eigen::RowMajor>;
  static Eigen::ArrayXd
  part1term1(const Eigen::Ref<const Eigen::Array3Xd> &points,
             const Eigen::Ref<const Eigen::Array3d> &node1,
             const Eigen::Ref<const Eigen::Array3d> &node2) {

    using namespace Eigen;

    ArrayXd t1 = points.row(0) - node1(0);
    double t2 = node2(1) - node1(1);

    ArrayXd t3 = points.row(1) - node1(1);
    double t4 = node2(0) - node1(0);

    double d = (node2 - node1).matrix().norm();
    ;

    return ((t1 * t2) - (t3 * t4)) / d;
  }

  static Eigen::ArrayXd
  part1term2(const Eigen::Ref<const Eigen::Array3Xd> &points,
             const Eigen::Ref<const Eigen::Array3d> &node1,
             const Eigen::Ref<const Eigen::Array3d> &node2) {
    // auto point = mEvalPoints.get().mEvalPoints.row(centerPointIdx);
    // auto node1 =
    // mPanelGeoRef.get().combinedSurface.getPoints().row(nodeIdx1); auto node2
    // = mPanelGeoRef.get().combinedSurface.getPoints().row(nodeIdx2);

    ArrayXd r1 =
        (points - node1.replicate(1, points.cols())).matrix().colwise().norm();
    ArrayXd r2 =
        (points - node2.replicate(1, points.cols())).matrix().colwise().norm();
    double d = (node2 - node1).matrix().norm();

    return ((r1 + r2 + d) / (r1 + r2 - d)).log();
  }

  static Eigen::ArrayXd
  part2term(const Eigen::Ref<const Eigen::Array3Xd> &points,
            const Eigen::Ref<const Eigen::Array3d> &node1,
            const Eigen::Ref<const Eigen::Array3d> &node2) {

    using namespace Eigen;
    // 0 -> x ; 1 -> y ; 2 -> z
    auto ek = [&points](const Eigen::Ref<const Eigen::Array3d> &faceV) {
      return (points.row(0) - faceV(0)).square() + points.row(2).square();
    };
    auto hk = [&points](const Eigen::Ref<const Eigen::Array3d> &faceV) {
      return (points.row(0) - faceV(0)) * (points.row(1) - faceV(1));
    };
    auto m = [](const Eigen::Ref<const Eigen::Array3d> &point1,
                const Eigen::Ref<const Eigen::Array3d> &point2) {
      return (point2(1) - point1(1)) / (point2(0) - point1(0));
    };
    auto r = [&points](const Eigen::Ref<const Eigen::Array3d> &faceV) {
      return (points - faceV.replicate(1, points.cols()))
          .matrix()
          .colwise()
          .norm();
    };

    using cAr = const Eigen::Ref<const ArrayXd> &;
    auto termP = [&points](double m, cAr e, cAr h, cAr r) {
      return (m * e - h).atan2(points.row(2).transpose() * r); // y/x
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

    const Eigen::Array3Xd fPoints = compTask.face.points.transpose();
    using namespace Eigen;

    ArrayXXd part1t1(fPoints.cols(), compTask.points.transpose().cols());
    ArrayXXd part1t2(fPoints.cols(), compTask.points.transpose().cols());
    ArrayXXd part2t(fPoints.cols(), compTask.points.transpose().cols());

    apply_adjacent_circular(fPoints.colwise().begin(), fPoints.colwise().end(),
                            part1t1.rowwise().begin(),
                            [&](const Eigen::Ref<const Eigen::Array3d> &node1,
                                const Eigen::Ref<const Eigen::Array3d> &node2) {
                              return part1term1(compTask.points.transpose(),
                                                node1, node2);
                            });

    apply_adjacent_circular(fPoints.colwise().begin(), fPoints.colwise().end(),
                            part1t2.rowwise().begin(),
                            [&](const Eigen::Ref<const Eigen::Array3d> &node1,
                                const Eigen::Ref<const Eigen::Array3d> &node2) {
                              return part1term2(compTask.points.transpose(),
                                                node1, node2);
                            });

    apply_adjacent_circular(fPoints.colwise().begin(), fPoints.colwise().end(),
                            part2t.rowwise().begin(),
                            [&](const Eigen::Ref<const Eigen::Array3d> &node1,
                                const Eigen::Ref<const Eigen::Array3d> &node2) {
                              return part2term(compTask.points.transpose(),
                                               node1, node2);
                            });

    ArrayXd term1 = (part1t1 * part1t2).colwise().sum();
    ArrayXd term2 =
        -compTask.points.col(2).abs() * (part2t.colwise().sum().transpose());

    return -1 / (4 * std::numbers::pi_v<double>)*(term1 + term2);
  }

  static Eigen::ArrayXd calcInfluenceFarImpl(const ComputeTask &compTask) {
    return SourceFar::calcInfluenceImpl(compTask);
  }
  static double calcSelfInfluenceImpl(const ComputeTask &compTask) {
    return calcInfluenceImpl(compTask)(0);
  }
};
