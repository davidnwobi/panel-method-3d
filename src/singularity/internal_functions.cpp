// #include "singularity/internal_functions.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <cmath>

#define CLAMP_TO 1e9

using namespace Eigen;
using RowArray3f = Eigen::Array<float, 1, 3, Eigen::RowMajor>;
Eigen::ArrayXf R12(const Eigen::Ref<const Eigen::ArrayX3f> &points,
                   const Eigen::Ref<const Eigen::Array3f> &node1,
                   const Eigen::Ref<const Eigen::Array3f> &node2) {
  const float dx = node2(0) - node1(0);
  const float dy = node2(1) - node1(1);
  // 2D distance between node1 and node2
  const float d = std::sqrt(dx * dx + dy * dy);

  Eigen::ArrayXf out(points.rows());

  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    const float px = points(i, 0);
    const float py = points(i, 1);

    // ( (px - node1.x)*dy - (py - node1.y)*dx ) / d
    out[i] = ((px - node1(0)) * dy - (py - node1(1)) * dx) / d;
  }

  return out;
}

Eigen::ArrayXf Q12(const Eigen::Ref<const Eigen::ArrayX3f> &points,
                   const Eigen::Ref<const Eigen::Array3f> &node1,
                   const Eigen::Ref<const Eigen::Array3f> &node2) {
  const float x1 = node1(0), y1 = node1(1);
  const float x2 = node2(0), y2 = node2(1);

  const float dx = node2(0) - node1(0);
  const float dy = node2(1) - node1(1);
  // 2D distance between node1 and node2
  const float d = std::sqrt(dx * dx + dy * dy);

  Eigen::ArrayXf out(points.rows());
  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    float px = points(i, 0);
    float py = points(i, 1);
    float pz = points(i, 2);

    const float dx1 = px - x1;
    const float dy1 = py - y1;
    const float r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));

    const float dx2 = px - x2;
    const float dy2 = py - y2;
    const float r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));

    out[i] = std::log((r1 + r2 + d) / (r1 + r2 - d));
  }
  return out;
}

Eigen::ArrayXf J12(const Eigen::Ref<const Eigen::ArrayX3f> &points,
                   const Eigen::Ref<const Eigen::Array3f> &node1,
                   const Eigen::Ref<const Eigen::Array3f> &node2) {
  const float x1 = node1(0), y1 = node1(1);
  const float x2 = node2(0), y2 = node2(1);

  // "m" slope function
  auto slope = [&](float xA, float yA, float xB, float yB) {
    const float dx = xB - xA;
    const float dy = yB - yA;
    // handle near‐vertical or near‐horizontal
    if (std::fabs(dx) < 1e-14) {
      return sgn(dy) * std::numeric_limits<float>::infinity();
    }
    if (std::fabs(dy) < 1e-14) {
      return 0.0f;
    }
    return dy / dx;
  };

  // precompute slope for the two nodes
  const float m12 = slope(x1, y1, x2, y2);

  // We'll accumulate the difference
  Eigen::ArrayXf out(points.rows());

  for (Eigen::Index i = 0; i < points.rows(); ++i) {
    const float px = points(i, 0);
    const float py = points(i, 1);
    const float pz = points(i, 2);

    // ek(faceV) = (px - faceV.x)^2 + pz^2
    // hk(faceV) = (px - faceV.x)*(py - faceV.y)
    // r(faceV)  = sqrt( (px-faceV.x)^2 + (py-faceV.y)^2 + (pz-faceV.z)^2 )

    // For node1:
    const float dx1 = px - x1;
    const float dy1 = py - y1;
    const float e1 = dx1 * dx1 + pz * pz; // ek(node1)
    const float h1 = dx1 * dy1;           // hk(node1)
    const float r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));

    // For node2:
    const float dx2 = px - x2;
    const float dy2 = py - y2;
    const float e2 = dx2 * dx2 + pz * pz; // ek(node2)
    const float h2 = dx2 * dy2;           // hk(node2)
    const float r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));

    // termP(m, e, h, r) = atan( (m*e - h) / (pz*r) )
    // watch for pz=0.0?
    auto termP = [&](float m, float e, float h, float rr) {
      // if pz=0, you might want to handle that carefully
      const float denom = pz * rr;
      return std::atan((m * e - h) / denom);
    };

    // difference of termP for node1, node2
    out[i] = termP(m12, e1, h1, r1) - termP(m12, e2, h2, r2);
  }

  return out;
}

float slope(float xA, float yA, float xB, float yB) {
  const float dx = xB - xA;
  const float dy = yB - yA;
  // handle near‐vertical or near‐horizontal
  if (std::fabs(dx) < 1e-14) {
    return std::copysign(CLAMP_TO, dy);
  }
  if (std::fabs(dy) < 1e-14) {
    return 0.0f;
  }
  return dy / dx;
};

template <typename VectorArr, typename PointsArr, typename PointVec>
void R12_Q12_J12(Eigen::ArrayBase<VectorArr> R12_, Eigen::Ref<VectorArr> Q12_,
                 Eigen::Ref<VectorArr> J12_,
                 const Eigen::Ref<PointsArr> &points,
                 const Eigen::Ref<PointVec> &node1,
                 const Eigen::Ref<PointVec> &node2) {

  // void R12_Q12_J12(Eigen::Ref<Eigen::ArrayXf> R12_,
  //                  Eigen::Ref<Eigen::ArrayXf> Q12_,
  //                  Eigen::Ref<Eigen::ArrayXf> J12_,
  //                  const Eigen::Ref<const Eigen::ArrayX3f> &points,
  //                  const Eigen::Ref<const Eigen::Array3f> &node1,
  //                  const Eigen::Ref<const Eigen::Array3f> &node2) {

  const float x1 = node1(0), y1 = node1(1);
  const float x2 = node2(0), y2 = node2(1);
  const float m12 = slope(x1, y1, x2, y2);

  const float dx = node2(0) - node1(0);
  const float dy = node2(1) - node1(1);
  const float d = std::sqrt(dx * dx + dy * dy);

  const auto &px = points.col(0);
  const auto &py = points.col(1);
  const auto &pz = points.col(2);

  R12_ = ((px - node1(0)) * dy - (py - node1(1)) * dx) / d;
  const Eigen::ArrayXf dx1 = px - x1;
  const Eigen::ArrayXf dy1 = py - y1;
  const Eigen::ArrayXf e1 = dx1 * dx1 + pz * pz; // ek(node1)
  const Eigen::ArrayXf h1 = dx1 * dy1;           // hk(node1)
  const Eigen::ArrayXf r1 = (dx1 * dx1 + dy1 * dy1 + (pz * pz)).sqrt();

  const Eigen::ArrayXf dx2 = px - x2;
  const Eigen::ArrayXf dy2 = py - y2;
  const Eigen::ArrayXf e2 = dx2 * dx2 + pz * pz; // ek(node2)
  const Eigen::ArrayXf h2 = dx2 * dy2;           // hk(node2)
  const Eigen::ArrayXf r2 = (dx2 * dx2 + dy2 * dy2 + (pz * pz)).sqrt();

  Q12_ = ((r1 + r2 + d) / (r1 + r2 - d)).log();
  const Eigen::ArrayXf a1 = (m12 * e1 - h1) / (pz * r1);
  const Eigen::ArrayXf a2 = (m12 * e2 - h2) / (pz * r2);

  J12_ = a1.atan() - a2.atan();

  // print("\n\n", J12_);
  // for (Eigen::Index i = 0; i < points.rows(); ++i) {
  //   const float px = points(i, 0);
  //   const float py = points(i, 1);
  //   const float pz = points(i, 2);
  //
  //   R12_[i] = ((px - node1(0)) * dy - (py - node1(1)) * dx) / d;
  //
  //   // For node1:
  //   const float dx1 = px - x1;
  //   const float dy1 = py - y1;
  //   const float e1 = dx1 * dx1 + pz * pz; // ek(node1)
  //   const float h1 = dx1 * dy1;           // hk(node1)
  //   const float r1 = std::sqrt(dx1 * dx1 + dy1 * dy1 + (pz * pz));
  //
  //   // For node2:
  //   const float dx2 = px - x2;
  //   const float dy2 = py - y2;
  //   const float e2 = dx2 * dx2 + pz * pz; // ek(node2)
  //   const float h2 = dx2 * dy2;           // hk(node2)
  //   const float r2 = std::sqrt(dx2 * dx2 + dy2 * dy2 + (pz * pz));
  //
  //   Q12_[i] = std::log((r1 + r2 + d) / (r1 + r2 - d));
  //
  //   // termP(m, e, h, r) = atan( (m*e - h) / (pz*r) )
  //   // watch for pz=0.0?
  //   // auto termP = [&](float m, float e, float h, float rr) {
  //   //   // if pz=0, you might want to handle that carefully
  //   //   const float denom = pz * rr;
  //   //   return std::atan((m * e - h) / denom);
  //   // };
  //   // J12_[i] = termP(m12, e1, h1, r1) - termP(m12, e2, h2, r2);
  //}
}
// Eigen::ArrayXf J12_OLD(const Eigen::Ref<const Eigen::ArrayX3f> &points,
//                        const Eigen::Ref<const Eigen::Array3f> &node1,
//                        const Eigen::Ref<const Eigen::Array3f> &node2) {
//
//   using namespace Eigen;
//   // 0 -> x ; 1 -> y ; 2 -> z
//   auto ek = [&points](const Eigen::Ref<const Eigen::Array3f> &faceV) {
//     return (points.col(0) - faceV(0)).square() + points.col(2).square();
//   };
//   auto hk = [&points](const Eigen::Ref<const Eigen::Array3f> &faceV) {
//     return (points.col(0) - faceV(0)) * (points.col(1) - faceV(1));
//   };
//   auto m = [](const Eigen::Ref<const Eigen::Array3f> &point1,
//               const Eigen::Ref<const Eigen::Array3f> &point2) {
//     if (std::abs(point2(0) - point1(0)) < 1e-10) {
//       return sgn_<float>((point2(1) - point1(1))) *
//              std::numeric_limits<float>::infinity();
//     }
//     if (std::abs(point2(1) - point1(1)) < 1e-10) {
//       return 0.0;
//     }
//     return (point2(1) - point1(1)) / (point2(0) - point1(0));
//   };
//   auto r = [&points](const Eigen::Ref<const Eigen::Array3f> &faceV) {
//     return (points - faceV.transpose().replicate(points.rows(), 1))
//         .matrix()
//         .rowwise()
//         .stableNorm();
//   };
//
//   using cAr = const Eigen::Ref<const ArrayXf> &;
//   auto termP = [&points](float m, cAr e, cAr h, cAr r) {
//     return ((m * e - h) / (points.col(2) * r)).atan(); // y/x => y.atan2(x)
//   };
//
//   float m12 = m(node1, node2);
//
//   ArrayXf diff = termP(m12, ek(node1), hk(node1), r(node1)) -
//                  termP(m12, ek(node2), hk(node2), r(node2));
//   return diff;
// }
//
// Eigen::ArrayXf Q12_L(const Eigen::Ref<const Eigen::ArrayX3f> &points,
//                      const Eigen::Ref<const Eigen::Array3f> &node1,
//                      const Eigen::Ref<const Eigen::Array3f> &node2) {
//
//   ArrayXf r1 = (points - node1.transpose().replicate(points.rows(), 1))
//                    .matrix()
//                    .rowwise()
//                    .stableNorm();
//   ArrayXf r2 = (points - node2.transpose().replicate(points.rows(), 1))
//                    .matrix()
//                    .rowwise()
//                    .stableNorm();
//   float d = (node2 - node1).matrix().stableNorm();
//
//   return ((r1 + r2 + d) / (r1 + r2 - d)).log();
// }
// Eigen::ArrayXf R12_L(const Eigen::Ref<const Eigen::ArrayX3f> &points,
//                      const Eigen::Ref<const Eigen::Array3f> &node1,
//                      const Eigen::Ref<const Eigen::Array3f> &node2) {
//
//   ArrayXf t1 = points.col(0) - node1(0);
//   float t2 = node2(1) - node1(1);
//
//   ArrayXf t3 = points.col(1) - node1(1);
//   float t4 = node2(0) - node1(0);
//
//   float d = (node2 - node1).matrix().stableNorm();
//   ;
//
//   return ((t1 * t2) - (t3 * t4)) / d;
// }
