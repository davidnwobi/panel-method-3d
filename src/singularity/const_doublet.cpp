


#include "compTask.hpp"
#include "singularity/const_doublet_far.hpp"
#include "singularity/const_doublet.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <limits>
#include <numbers>
static bool can_print = false;
   Eigen::ArrayXd DoubletP::term(const Eigen::Ref<const Eigen::ArrayX3d> &points,
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

   Eigen::ArrayXd DoubletP::calcInfluenceImpl(const ComputeTask &compTask) {
    // std::abs(point(2)) < 1e-6,

    using namespace Eigen;
    const auto & fPoints = compTask.face.points;
    ArrayXXd term1(compTask.points.rows(), fPoints.rows());
    ArrayXd norms(4);
    apply_adjacent_circular(fPoints.rowwise().begin(), fPoints.rowwise().end(),
                            term1.colwise().begin(),
                            [&](const Eigen::Ref<const Eigen::RowVector3d> &node1,
                                const Eigen::Ref<const Eigen::RowVector3d> &node2) {
                              return term(compTask.points, node1,
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
        term1.col(i) = Eigen::ArrayXd::Zero(term1.rows());
      }
    }

    ArrayXd inf = 1 / (4 * std::numbers::pi_v<double>)*(term1).rowwise().sum();
    return inf;
  }

  Eigen::ArrayXd DoubletP::calcInfluenceFarImpl(const ComputeTask &compTask) {
    return DoubletFar::calcInfluenceImpl(compTask);
  }

  double DoubletP::calcSelfInfluenceImpl(const ComputeTask &compTask) {
    UNUSED(compTask);
    return -0.5;
  }
