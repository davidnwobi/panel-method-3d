#include <Eigen/Core>
#define CLAMP_TO 1e9

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

// void R12_Q12_J12(Eigen::Ref<Eigen::ArrayXf> R12_,
//                  Eigen::Ref<Eigen::ArrayXf> Q12_,
//                  Eigen::Ref<Eigen::ArrayXf> J12_,
//                  const Eigen::Ref<const Eigen::ArrayX3f> &points,
//                  const Eigen::Ref<const Eigen::Array3f> &node1,
//                  const Eigen::Ref<const Eigen::Array3f> &node2);
//
template <typename T> T slope(T xA, T yA, T xB, T yB) {
  const T dx = xB - xA;
  const T dy = yB - yA;
  // handle near‐vertical or near‐horizontal
  if (std::fabs(dx) < 1e-14) {
    return std::copysign(CLAMP_TO, dy);
  }
  if (std::fabs(dy) < 1e-14) {
    return 0.0f;
  }
  return dy / dx;
};
template <typename VectorArr, typename PointsArr, typename RowPointVec>
void R12_Q12_J12(Eigen::ArrayBase<VectorArr> &R12_,
                 Eigen::ArrayBase<VectorArr> &Q12_,
                 Eigen::ArrayBase<VectorArr> &J12_,
                 const Eigen::ArrayBase<PointsArr> &points,
                 const Eigen::ArrayBase<RowPointVec> &node1,
                 const Eigen::ArrayBase<RowPointVec> &node2) {

  // void R12_Q12_J12(Eigen::Ref<Eigen::ArrayXf> R12_,
  //                  Eigen::Ref<Eigen::ArrayXf> Q12_,
  //                  Eigen::Ref<Eigen::ArrayXf> J12_,
  //                  const Eigen::Ref<const Eigen::ArrayX3f> &points,
  //                  const Eigen::Ref<const Eigen::Array3f> &node1,
  //                  const Eigen::Ref<const Eigen::Array3f> &node2) {

  // http://ithare.com/infographics-operation-costs-in-cpu-clock-cycles/
  // https://www.agner.org/optimize/instruction_tables.pdf
  const float x1 = node1(0), y1 = node1(1);
  const float x2 = node2(0), y2 = node2(1);
  const float m12 = slope<float>(x1, y1, x2, y2);

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
