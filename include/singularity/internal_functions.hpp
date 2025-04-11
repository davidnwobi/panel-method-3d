#include <Eigen/Core>

template <typename T> int sgn_(T val) {
  if (val > T(0))
    return 1; // positive
  if (val < T(0))
    return -1; // negative
  return 0;    // zero
}
Eigen::ArrayXd R12(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                   const Eigen::Ref<const Eigen::Array3d> &node1,
                   const Eigen::Ref<const Eigen::Array3d> &node2);

Eigen::ArrayXd Q12(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                   const Eigen::Ref<const Eigen::Array3d> &node1,
                   const Eigen::Ref<const Eigen::Array3d> &node2);

Eigen::ArrayXd J12(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                   const Eigen::Ref<const Eigen::Array3d> &node1,
                   const Eigen::Ref<const Eigen::Array3d> &node2);

void R12_Q12_J12(Eigen::Ref<Eigen::ArrayXd> R12_,
                 Eigen::Ref<Eigen::ArrayXd> Q12_,
                 Eigen::Ref<Eigen::ArrayXd> J12_,
                 const Eigen::Ref<const Eigen::ArrayX3d> &points,
                 const Eigen::Ref<const Eigen::Array3d> &node1,
                 const Eigen::Ref<const Eigen::Array3d> &node2);
