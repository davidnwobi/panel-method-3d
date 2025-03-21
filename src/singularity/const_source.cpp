
#include "singularity/const_source_far.hpp"
#include "singularity/const_source.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numbers>


  using RowArray3d = Eigen::Array<double, 1, 3, Eigen::RowMajor>;
   Eigen::ArrayXd
  part1term1(const Eigen::Ref<const Eigen::ArrayX3d> &points,
             const Eigen::Ref<const Eigen::Array3d> &node1,
             const Eigen::Ref<const Eigen::Array3d> &node2) {

    using namespace Eigen;

    ArrayXd t1 = points.col(0) - node1(0);
    double t2 = node2(1) - node1(1);

    ArrayXd t3 = points.col(1) - node1(1);
    double t4 = node2(0) - node1(0);

    double d = (node2 - node1).matrix().norm();
    ;

    return ((t1 * t2) - (t3 * t4)) / d;
  }

   Eigen::ArrayXd
  part1term2(const Eigen::Ref<const Eigen::ArrayX3d> &points,
             const Eigen::Ref<const Eigen::Array3d> &node1,
             const Eigen::Ref<const Eigen::Array3d> &node2) {


    ArrayXd r1 =
        (points - node1.transpose().replicate(points.rows(), 1)).matrix().rowwise().norm();
    ArrayXd r2 =
        (points - node2.transpose().replicate(points.rows(), 1)).matrix().rowwise().norm();
    double d = (node2 - node1).matrix().norm();

    return ((r1 + r2 + d) / (r1 + r2 - d)).log();
  }
   Eigen::ArrayXd part2term(const Eigen::Ref<const Eigen::ArrayX3d> &points,
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
      if (std::abs(point2(0) - point1(0)) < 1e-10){
        return sgn<double>((point2(1) - point1(1))) * std::numeric_limits<double>::infinity();
      } 
      if (std::abs(point2(1) - point1(1)) < 1e-10){
         return 0.0;
      } 
      return (point2(1) - point1(1)) / (point2(0) - point1(0));
    };
    auto r = [&points](const Eigen::Ref<const Eigen::Array3d> &faceV) {
      return (points - faceV.transpose().replicate(points.rows(),1))
          .matrix()
          .rowwise()
          .norm();
    };

    using cAr = const Eigen::Ref<const ArrayXd> &;
    auto termP = [&points](double m, cAr e, cAr h, cAr r) {
      return (m * e - h).atan2(points.col(2) * r); // y/x
    };

    double m12 = m(node1, node2);

    ArrayXd diff =  termP(m12, ek(node1), hk(node1), r(node1)) - termP(m12, ek(node2), hk(node2), r(node2));
      return diff.sin().atan2(diff.cos());
  }


     Eigen::ArrayXd SourceP::calcInfluenceImpl(const ComputeTask &compTask) {

    const auto & fPoints = compTask.face.points;
    using namespace Eigen;

    ArrayXXd part1t1(compTask.points.rows(), fPoints.rows());
    ArrayXXd part1t2(compTask.points.rows(), fPoints.rows());
    ArrayXXd part2t(compTask.points.rows(), fPoints.rows());
    ArrayXd norms(4);

    apply_adjacent_circular(fPoints.rowwise().begin(), fPoints.rowwise().end(),
                            part1t1.colwise().begin(),
                            [&](const Eigen::Ref<const Eigen::RowVector3d> &node1,
                                const Eigen::Ref<const Eigen::RowVector3d> &node2) {
                              return part1term1(compTask.points, node1,
                                          node2);
                            });
    apply_adjacent_circular(fPoints.rowwise().begin(), fPoints.rowwise().end(),
                            part1t2.colwise().begin(),
                            [&](const Eigen::Ref<const Eigen::RowVector3d> &node1,
                                const Eigen::Ref<const Eigen::RowVector3d> &node2) {
                              return part1term2(compTask.points, node1,
                                          node2);
                            });
    apply_adjacent_circular(fPoints.rowwise().begin(), fPoints.rowwise().end(),
                            part2t.colwise().begin(),
                            [&](const Eigen::Ref<const Eigen::RowVector3d> &node1,
                                const Eigen::Ref<const Eigen::RowVector3d> &node2) {
                              return part2term(compTask.points, node1,
                                          node2);
                            });
    apply_adjacent_circular(fPoints.rowwise().begin(), fPoints.rowwise().end(),
                            norms.begin(),
                            [&](const Eigen::Ref<const Eigen::RowVector3d> &node1,
                                const Eigen::Ref<const Eigen::RowVector3d> &node2) {
                            
                           return  (node2-node1).matrix().norm();

                             });
    for (int i = 0; i < 4; i++){
      if (norms(i) < 1e-10){
        part1t1.col(i) = Eigen::ArrayXd::Zero(part1t1.rows());
        part1t2.col(i) = Eigen::ArrayXd::Zero(part1t2.rows());
        part2t.col(i) = Eigen::ArrayXd::Zero(part2t.rows());
      }
    }


    ArrayXd term1 = (part1t1 * part1t2).rowwise().sum();
    ArrayXd term2 =
        -compTask.points.col(2).abs() * (-part2t.rowwise().sum());

    return -1 / (4 * std::numbers::pi_v<double>)*(term1 + term2);
  }

   Eigen::ArrayXd SourceP::calcInfluenceFarImpl(const ComputeTask &compTask) {
    return SourceFar::calcInfluenceImpl(compTask);
  }
   double SourceP::calcSelfInfluenceImpl(const ComputeTask &compTask) {
    return calcInfluenceImpl(compTask)(0);
  }
