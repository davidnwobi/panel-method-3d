#include <Eigen/Core>

template <typename T> int sgn_(T val) {
  if (val > T(0))
    return 1; // positive
  if (val < T(0))
    return -1; // negative
  return 0;    // zero
}
Eigen::ArrayXf R12(const Eigen::Ref<const Eigen::ArrayX3f> &points,
                   const Eigen::Ref<const Eigen::Array3f> &node1,
                   const Eigen::Ref<const Eigen::Array3f> &node2);

Eigen::ArrayXf Q12(const Eigen::Ref<const Eigen::ArrayX3f> &points,
                   const Eigen::Ref<const Eigen::Array3f> &node1,
                   const Eigen::Ref<const Eigen::Array3f> &node2);

Eigen::ArrayXf J12(const Eigen::Ref<const Eigen::ArrayX3f> &points,
                   const Eigen::Ref<const Eigen::Array3f> &node1,
                   const Eigen::Ref<const Eigen::Array3f> &node2);

void R12_Q12_J12(Eigen::Ref<Eigen::ArrayXf> R12_,
                 Eigen::Ref<Eigen::ArrayXf> Q12_,
                 Eigen::Ref<Eigen::ArrayXf> J12_,
                 const Eigen::Ref<const Eigen::ArrayX3f> &points,
                 const Eigen::Ref<const Eigen::Array3f> &node1,
                 const Eigen::Ref<const Eigen::Array3f> &node2);
