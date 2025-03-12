


#include "compTask.hpp"
#include "singularity/const_doublet_far.hpp"
#include "singularity/const_doublet.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>

   Eigen::ArrayXd DoubletP::term(const Eigen::Ref<const Eigen::Array3Xd> &points,
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

   Eigen::ArrayXd DoubletP::calcInfluenceImpl(const ComputeTask &compTask) {
    // std::abs(point(2)) < 1e-6,

    using namespace Eigen;
    const Eigen::Array3Xd fPoints = compTask.face.points.transpose();
    ArrayXXd term1(fPoints.cols(), compTask.points.transpose().cols());
    ArrayXd norms(4);
    apply_adjacent_circular(fPoints.colwise().begin(), fPoints.colwise().end(),
                            term1.rowwise().begin(),
                            [&](const Eigen::Ref<const Eigen::Array3d> &node1,
                                const Eigen::Ref<const Eigen::Array3d> &node2) {
                              return term(compTask.points.transpose(), node1,
                                          node2);
                            });
    apply_adjacent_circular(fPoints.colwise().begin(), fPoints.colwise().end(),
                            norms.begin(),
                            [&](const Eigen::Ref<const Eigen::Array3d> &node1,
                                const Eigen::Ref<const Eigen::Array3d> &node2) {
                            
                           return  (node2-node1).matrix().norm();
                              });
    for (int i = 0; i < 4; i++){
      if (norms(i) < 1e-6){
        term1(i, Eigen::placeholders::all) = Eigen::RowVectorXd::Zero(term1.cols());
      }
    }

  // print("\n\n\n\n");
    ArrayXd inf = 1 / (4 * std::numbers::pi_v<double>)*(term1).colwise().sum();
    return inf;
  }

  Eigen::ArrayXd DoubletP::calcInfluenceFarImpl(const ComputeTask &compTask) {
    return DoubletFar::calcInfluenceImpl(compTask);
  }

  double DoubletP::calcSelfInfluenceImpl(const ComputeTask &compTask) {
    UNUSED(compTask);
    return 0.5;
  }
