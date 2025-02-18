#include "panel_geo/panel_geo.hpp"
#include "surface/surface_panel.hpp"
#include "surface/wake_panel.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <algorithm>
#include <ranges>

namespace PanelGeometryUtils {

Eigen::MatrixXd rowwiseCross(const Eigen::MatrixXd &A,
                             const Eigen::MatrixXd &B) {
  assert(A.cols() == 3 && B.cols() == 3 && A.rows() == B.rows());
  Eigen::MatrixXd C(A.rows(), 3);

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
  panelGeoInit();
}

template <SurfaceType T>
PanelGeometry<T>::PanelGeometry(T &&surface) noexcept : mSurface(surface) {
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
        iPanel, mSurface.mPoints.transpose()(Eigen::placeholders::all, faceRow)
                    .transpose()));
  }

  for (int iPanel = 0; iPanel < nPanels; iPanel++) {
    areas(iPanel) = calcPolyArea(localFaceVertices[iPanel]);
  }
}

template <SurfaceType T>
void PanelGeometry<T>::calculateCentrePointsandVectors() {

  int numRows = mSurface.mFaceNodeIdx.rows();

  normalVectors.setZero(numRows, VecType::ColsAtCompileTime);

  auto calcLineCenterPoints = [&](int startIdx, int endIdx) -> Eigen::Array3Xd {
    auto surface = mSurface;
    return ((surface.mPoints.transpose()(Eigen::placeholders::all,
                                         surface.mFaceNodeIdx.col(endIdx)) +
             surface.mPoints.transpose()(Eigen::placeholders::all,
                                         surface.mFaceNodeIdx.col(startIdx))) /
            2)
        .eval();
  };

  Eigen::Array3Xd c01 = calcLineCenterPoints(0, 1);
  Eigen::Array3Xd c12 = calcLineCenterPoints(1, 2);
  Eigen::Array3Xd c23 = calcLineCenterPoints(2, 3);
  Eigen::Array3Xd c30 = calcLineCenterPoints(3, 1);

  // std::cout << c01 << "\n" << c12 << "\n" << c23 << "\n" << c30 << "\n\n";
  centrePoints = ((c01 + c23) / 2).transpose(); // Pick any opposite sides

  // tangetial vector in the x direction wrt face
  tangentYVectors = -(c23 - c01).transpose();
  tangentYVectors.matrix().rowwise().normalize();

  // tangetial vector in the y direction wrt face
  tangentXVectors = -(c30 - c12).transpose();
  tangentXVectors.matrix().rowwise().normalize();

  // normal vector in the z direction wrt face
  normalVectors = PanelGeometryUtils::colwiseCross(tangentXVectors.transpose(),
                                                   tangentYVectors.transpose())
                      .transpose();
  normalVectors.matrix().rowwise().normalize();

  // centrePoints = centrePoints - normalVectors * 0.0001;
}

template <SurfaceType T>
Eigen::Isometry3d
PanelGeometry<T>::createLocalConversionMatrix(std::size_t faceIdx) {
  Eigen::Matrix3d rotationMatrix;
  rotationMatrix.col(0) = tangentXVectors.transpose().col(faceIdx);
  rotationMatrix.col(1) = tangentYVectors.transpose().col(faceIdx);
  rotationMatrix.col(2) = normalVectors.transpose().col(faceIdx);

  Eigen::Isometry3d transformLocalToGlobal = Eigen::Isometry3d::Identity();
  transformLocalToGlobal.linear() = rotationMatrix;
  transformLocalToGlobal.translation() = centrePoints.row(faceIdx);

  // https://gamemath.com/book/orient.html
  return transformLocalToGlobal.inverse();
}

template <SurfaceType T>
Eigen::ArrayX3d PanelGeometry<T>::convertToLocal(int faceIdx,
                                                 const ArrayX3d &points) const {

  return (conversionMatrices[faceIdx] * (points.transpose().matrix()))
      .transpose();
}

template <SurfaceType T>
double PanelGeometry<T>::calcPolyArea(const Eigen::ArrayX3d &vertices) const {
  Eigen::ArrayXd partAreaSum(vertices.rows());

  // Shoelace Formula
  apply_adjacent_circular(vertices.rowwise().begin(), vertices.rowwise().end(),
                          partAreaSum.begin(),
                          [](const RowVector3d &v1, const RowVector3d &v2) {
                            return v1(0) * v2(1) - v1(1) * v2(0);
                          });

  return std::abs(0.5 * partAreaSum.sum());
}

template struct PanelGeometry<SurfacePanel>;
template struct PanelGeometry<WakePanel>;
