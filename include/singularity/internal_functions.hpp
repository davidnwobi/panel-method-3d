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

Eigen::ArrayXd J12_OLD(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                       const Eigen::Ref<const Eigen::Array3d> &node1,
                       const Eigen::Ref<const Eigen::Array3d> &node2);

Eigen::ArrayXd J12(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                   const Eigen::Ref<const Eigen::Array3d> &node1,
                   const Eigen::Ref<const Eigen::Array3d> &node2);
Eigen::ArrayXi baryCheck(const Eigen::Ref<const Eigen::ArrayX3d> &points,
                         const Eigen::Ref<const Eigen::Array3d> &A,
                         const Eigen::Ref<const Eigen::Array3d> &B,
                         const Eigen::Ref<const Eigen::Array3d> &C);
