#pragma once
#include "compTask.hpp"
#include "singularity/const_doublet_far.hpp"
#include "singularity/iconst_sing.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>
#include <valarray>

struct DoubletP : IConstant3dSingularity<DoubletP> {
  static Eigen::ArrayXf term(const Eigen::Ref<const Eigen::Array3Xd> &points,
                             const Eigen::Ref<const Eigen::Array3f> &node1,
                             const Eigen::Ref<const Eigen::Array3f> &node2) {

    using namespace Eigen;
    // 0 -> x ; 1 -> y ; 2 -> z
    auto ek = [&points](const Eigen::Ref<const Eigen::Array3f> &faceV) {
      return (points.row(0) - faceV(0)).square() + points.row(2).square();
    };
    auto hk = [&points](const Eigen::Ref<const Eigen::Array3f> &faceV) {
      return (points.row(0) - faceV(0)) * (points.row(1) - faceV(1));
    };
    auto m = [](const Eigen::Ref<const Eigen::Array3f> &point1,
                const Eigen::Ref<const Eigen::Array3f> &point2) {
      return (point2(1) - point1(1)) / (point2(0) - point1(0));
    };
    auto r = [&points](const Eigen::Ref<const Eigen::Array3f> &faceV) {
      return (points - faceV.replicate(1, points.cols()))
          .matrix()
          .colwise()
          .norm();
    };

    using cAr = const Eigen::Ref<const ArrayXf> &;
    auto termP = [&points](float m, cAr e, cAr h, cAr r) {
      return (m * e - h).atan2(points.row(2).transpose() * r); // y/x
    };

    float m12 = m(node1, node2);

    ArrayXf e1 = ek(node1);
    ArrayXf e2 = ek(node2);
    ArrayXf h1 = hk(node1);
    ArrayXf h2 = hk(node2);
    ArrayXf r1 = r(node1);
    ArrayXf r2 = r(node2);

    ArrayXf pt1 = termP(m12, e1, h1, r1);
    ArrayXf pt2 = termP(m12, e2, h2, r2);
    return pt1 - pt2;
  }

  static Eigen::ArrayXf calcInfluenceImpl(const ComputeTask &compTask) {
    // std::abs(point(2)) < 1e-6,

    using namespace Eigen;
    const Eigen::Array3Xd fPoints = compTask.face.points.transpose();
    ArrayXXf term1(fPoints.cols(), compTask.points.transpose().cols());
    apply_adjacent_circular(fPoints.colwise().begin(), fPoints.colwise().end(),
                            term1.rowwise().begin(),
                            [&](const Eigen::Ref<const Eigen::Array3f> &node1,
                                const Eigen::Ref<const Eigen::Array3f> &node2) {
                              return term(compTask.points.transpose(), node1,
                                          node2);
                            });
    ArrayXf inf = 1 / (4 * std::numbers::pi_v<float>)*(term1).colwise().sum();
    return inf;
  }

  static Eigen::ArrayXf calcInfluenceFarImpl(const ComputeTask &compTask) {
    return DoubletFar::calcInfluenceImpl(compTask);
  }
  static float calcSelfInfluenceImpl(const ComputeTask &compTask) {
    UNUSED(compTask);
    return 0.5;
  }
};
