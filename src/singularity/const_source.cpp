
#include "singularity/const_source_far.hpp"
#include "singularity/const_source.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>


  using RowArray3d = Eigen::Array<double, 1, 3, Eigen::RowMajor>;
   Eigen::ArrayXd
  SourceP::part1term1(const Eigen::Ref<const Eigen::Array3Xd> &points,
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

   Eigen::ArrayXd
  SourceP::part1term2(const Eigen::Ref<const Eigen::Array3Xd> &points,
             const Eigen::Ref<const Eigen::Array3d> &node1,
             const Eigen::Ref<const Eigen::Array3d> &node2) {


    ArrayXd r1 =
        (points - node1.replicate(1, points.cols())).matrix().colwise().norm();
    ArrayXd r2 =
        (points - node2.replicate(1, points.cols())).matrix().colwise().norm();
    double d = (node2 - node1).matrix().norm();

    return ((r1 + r2 + d) / (r1 + r2 - d)).log();
  }

   Eigen::ArrayXd
  SourceP::part2term(const Eigen::Ref<const Eigen::Array3Xd> &points,
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
      if (std::abs(point2(0) - point1(0)) < 1e-6){
        return sgn<double>((point2(1) - point1(1))) * std::numeric_limits<double>::infinity();
      } 
      if (std::abs(point2(1) - point1(1)) < 1e-6){
         return 0.0;
      } 
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
    pt1 = (pt1 > 0).select(pt1, pt1+2*std::numbers::pi_v<double>);
    ArrayXd pt2 = termP(m12, e2, h2, r2);
    pt2 = (pt2 > 0).select(pt2, pt2+2*std::numbers::pi_v<double>);

    return pt1-pt2;
  }
   Eigen::ArrayXd SourceP::calcInfluenceImpl(const ComputeTask &compTask) {

    const Eigen::Array3Xd fPoints = compTask.face.points.transpose();
    using namespace Eigen;

    ArrayXXd part1t1(fPoints.cols(), compTask.points.transpose().cols());
    ArrayXXd part1t2(fPoints.cols(), compTask.points.transpose().cols());
    ArrayXXd part2t(fPoints.cols(), compTask.points.transpose().cols());
    ArrayXd norms(4);

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

    apply_adjacent_circular(fPoints.colwise().begin(), fPoints.colwise().end(),
                            norms.begin(),
                            [&](const Eigen::Ref<const Eigen::Array3d> &node1,
                                const Eigen::Ref<const Eigen::Array3d> &node2) {
                            
                           return  (node2-node1).matrix().norm();
                              });
    for (int i = 0; i < 4; i++){
      if (norms(i) < 1e-10){
        part1t1(i, Eigen::placeholders::all) = Eigen::RowVectorXd::Zero(part1t1.cols());
        part1t2(i, Eigen::placeholders::all) = Eigen::RowVectorXd::Zero(part1t2.cols());
        part2t(i, Eigen::placeholders::all) = Eigen::RowVectorXd::Zero(part2t.cols());
      }
    }


    ArrayXd term1 = (part1t1 * part1t2).colwise().sum();
    ArrayXd term2 =
        -compTask.points.col(2).abs() * (part2t.colwise().sum().transpose());

    return -1 / (4 * std::numbers::pi_v<double>)*(term1 + term2);
  }

   Eigen::ArrayXd SourceP::calcInfluenceFarImpl(const ComputeTask &compTask) {
    return SourceFar::calcInfluenceImpl(compTask);
  }
   double SourceP::calcSelfInfluenceImpl(const ComputeTask &compTask) {
    return calcInfluenceImpl(compTask)(0);
  }
