#include "panel_geo/panel_geo.hpp"
#include "surface/surface_panel.hpp"
#include "surface/wake_panel.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <ranges>
#include <type_traits>

namespace PanelGeometryUtils {

Eigen::MatrixXf rowwiseCross(const Eigen::MatrixXf &A,
                             const Eigen::MatrixXf &B) {
  assert(A.cols() == 3 && B.cols() == 3 && A.rows() == B.rows());
  Eigen::MatrixXf C(A.rows(), 3);

  // C.col(0) = A.col(1)*B.col(2) - A.col(2)*B.col(1)
  C.col(0) = A.col(1).cwiseProduct(B.col(2)) - A.col(2).cwiseProduct(B.col(1));

  // C.col(1) = A.col(2)*B.col(0) - A.col(0)*B.col(2)
  C.col(1) = A.col(2).cwiseProduct(B.col(0)) - A.col(0).cwiseProduct(B.col(2));

  // C.col(2) = A.col(0)*B.col(1) - A.col(1)*B.col(0)
  C.col(2) = A.col(0).cwiseProduct(B.col(1)) - A.col(1).cwiseProduct(B.col(0));

  return C;
}

Eigen::Array3Xd colwiseCross(const Eigen::Ref<const Eigen::Array3Xd> &A,
                             const Eigen::Ref<const Eigen::Array3Xd> &B) {
  Eigen::Array3Xd out(3, A.cols());
  auto colwiseCrossImpl =
      [](const Eigen::Ref<const Eigen::Vector3d> &a,
         const Eigen::Ref<const Eigen::Vector3d> &b) -> Eigen::Vector3d {
    return a.cross(b);
  };
  std::transform(A.colwise().begin(), A.colwise().end(), B.colwise().begin(),
                 out.colwise().begin(), colwiseCrossImpl);
  return out;
}
} // namespace PanelGeometryUtils

template <SurfaceType T>
PanelGeometry<T>::PanelGeometry(const T &surface) : mSurface(surface) {
  if constexpr (std::is_same_v<T, WakePanel>) {
    if (surface.mPoints.rows() < 1) {
      return;
    }
  }
  panelGeoInit();
}

template <SurfaceType T>
PanelGeometry<T>::PanelGeometry(T &&surface) noexcept : mSurface(surface) {
  if constexpr (std::is_same_v<T, WakePanel>) {
    if (surface.mPoints.rows() < 1) {
      return;
    }
  }
  panelGeoInit();
}

template <SurfaceType T> void PanelGeometry<T>::panelGeoInit() {
  calculateCentrePointsandVectors();

  int nPanels = centrePoints.rows();
  conversionMatrices.reserve(nPanels);
  localFaceVertices.reserve(nPanels);
  areas.resize(nPanels);

  for (int iPanel = 0; iPanel < nPanels; iPanel++) {
    conversionMatrices.emplace_back(createLocalConversionMatrix(iPanel));
  }
  for (int iPanel = 0; iPanel < nPanels; iPanel++) {
    const auto &faceRow = mSurface.mFaceNodeIdx.row(iPanel);
    localFaceVertices.emplace_back(convertToLocal(
        iPanel, mSurface.mPoints(faceRow, Eigen::placeholders::all)));
  }

  for (int iPanel = 0; iPanel < nPanels; iPanel++) {
    areas(iPanel) = calcPolyArea(localFaceVertices[iPanel]);
  }
}

template <typename Derived> void normalize(DenseBase<Derived> &mat) {
  for (int i = 0; i < mat.rows(); i++) {
    mat.row(i).matrix().stableNormalize();
  }
}
template <SurfaceType T>
void PanelGeometry<T>::calculateCentrePointsandVectors() {

  print(__PRETTY_FUNCTION__);
  int numRows = mSurface.mFaceNodeIdx.rows();

  normalVectors.setZero(numRows, VecType::ColsAtCompileTime);

  auto calcLineCenterPoints = [&](int startIdx, int endIdx) -> Eigen::ArrayX3f {
    const auto &surface = mSurface;
    return ((surface.mPoints(surface.mFaceNodeIdx.col(endIdx),
                             Eigen::placeholders::all) +
             surface.mPoints(surface.mFaceNodeIdx.col(startIdx),
                             Eigen::placeholders::all)) /
            2);
  };

  Eigen::ArrayX3f c01 = calcLineCenterPoints(0, 1);
  Eigen::ArrayX3f c12 = calcLineCenterPoints(1, 2);
  Eigen::ArrayX3f c23 = calcLineCenterPoints(2, 3);
  Eigen::ArrayX3f c30 = calcLineCenterPoints(3, 0);

  // std::cout << c01 << "\n" << c12 << "\n" << c23 << "\n" << c30 << "\n\n";
  centrePoints = ((c01 + c23) / 2); // Pick any opposite sides
  const auto &surface = mSurface;
  // tangetial vector in the x direction wrt face

  tangentYVectors = -(c30 - c12);
  normalize(tangentYVectors);

  // tangetial vector in the y direction wrt face
  tangentXVectors = (c23 - c01);
  normalize(tangentXVectors);
  // tangentXVectors.matrix().stableNormalize();

  // normal vector in the z direction wrt face
  normalVectors =
      PanelGeometryUtils::rowwiseCross(tangentXVectors, tangentYVectors);
  normalize(normalVectors);

  tangentYVectors =
      PanelGeometryUtils::rowwiseCross(normalVectors, tangentXVectors);

  normalize(tangentYVectors);

  //
  // tangentYVectors = -(surface.mPoints(surface.mFaceNodeIdx.col(3),
  // Eigen::placeholders::all)-
  //            surface.mPoints(surface.mFaceNodeIdx.col(0),
  //            Eigen::placeholders::all
  //                                        ));
  // if ((tangentYVectors.matrix().rowwise().norm()).sum() < 1e-6){
  // tangentYVectors = (surface.mPoints(surface.mFaceNodeIdx.col(1),
  // Eigen::placeholders::all)-
  //            surface.mPoints(surface.mFaceNodeIdx.col(2),
  //            Eigen::placeholders::all
  //                                        ));
  // }
  // tangentYVectors.matrix().rowwise().normalize();
  //
  // // tangetial vector in the y direction wrt face
  // tangentXVectors = -(c30 - c12).transpose();
  // tangentXVectors.matrix().rowwise().normalize();
  //
  // // normal vector in the z direction wrt face
  // normalVectors =
  // PanelGeometryUtils::colwiseCross(tangentXVectors.transpose(),
  //                                                  tangentYVectors.transpose())
  //                     .transpose();
  // normalVectors.matrix().rowwise().normalize();
  //
  // tangentYVectors =
  // PanelGeometryUtils::colwiseCross(normalVectors.transpose(),
  //                                                   tangentXVectors.transpose()).transpose();
  // tangentYVectors.matrix().rowwise().normalize();
  // centrePoints = centrePoints - normalVectors * 0.0001;
}

template <SurfaceType T>
Eigen::Isometry3f
PanelGeometry<T>::createLocalConversionMatrix(std::size_t faceIdx) {
  Eigen::Matrix3f rotationMatrix;
  rotationMatrix.col(0) = tangentXVectors.transpose().col(faceIdx);
  rotationMatrix.col(1) = tangentYVectors.transpose().col(faceIdx);
  rotationMatrix.col(2) = normalVectors.transpose().col(faceIdx);

  Eigen::Isometry3f transformLocalToGlobal = Eigen::Isometry3f::Identity();
  transformLocalToGlobal.linear() = rotationMatrix;
  transformLocalToGlobal.translation() = centrePoints.row(faceIdx);

  // https://gamemath.com/book/orient.html
  return transformLocalToGlobal.inverse();
}

template <SurfaceType T>
Eigen::ArrayX3f PanelGeometry<T>::convertToLocal(int faceIdx,
                                                 const ArrayX3f &points) const {

  return (conversionMatrices[faceIdx] * (points.transpose().matrix()))
      .transpose();
  // const Eigen::Index m = points.rows();
  // const Eigen::Index n = points.cols();
  // Eigen::ArrayX3f out(m, n);
  //
  // for (Eigen::Index i = 0; i < m; i++) {
  //   out.row(i).transpose() =
  //       (conversionMatrices[faceIdx] * points.row(i).transpose());
  // }
  // return out;
}

template <SurfaceType T>
float PanelGeometry<T>::calcPolyArea(const Eigen::ArrayX3f &vertices) const {
  Eigen::ArrayXf partAreaSum(vertices.rows());

  // Shoelace Formula
  apply_adjacent_circular(vertices.rowwise().begin(), vertices.rowwise().end(),
                          partAreaSum.begin(),
                          [](const RowVector3f &v1, const RowVector3f &v2) {
                            return v1(0) * v2(1) - v1(1) * v2(0);
                          });
  return std::abs(0.5 * partAreaSum.sum());
}

template struct PanelGeometry<SurfacePanel>;
template struct PanelGeometry<WakePanel>;
