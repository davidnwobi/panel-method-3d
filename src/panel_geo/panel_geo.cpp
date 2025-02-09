#include "panel_geo/panel_geo.hpp"
#include "surface/surface_panel.hpp"
#include "surface/wake_panel.hpp"
#include "utils/utils.hpp"

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

    const auto &faceRow = mSurface.mFaceNodeIdx.row(iPanel);
    localFaceVertices.emplace_back(convertToLocal(
        iPanel, mSurface.mPoints(faceRow, Eigen::placeholders::all)));

    areas(iPanel) = calcPolyArea(localFaceVertices[iPanel]);
  }
}

template <SurfaceType T>
void PanelGeometry<T>::calculateCentrePointsandVectors() {

  int numRows = mSurface.mFaceNodeIdx.rows();

  normalVectors.setZero(numRows, VecType::ColsAtCompileTime);

  auto calcLineCenterPoints = [&](int startIdx, int endIdx) {
    auto surface = mSurface;
    return ((surface.mPoints(surface.mFaceNodeIdx.col(endIdx),
                             Eigen::placeholders::all) +
             surface.mPoints(surface.mFaceNodeIdx.col(startIdx),
                             Eigen::placeholders::all)) /
            2)
        .eval();
  };

  Eigen::ArrayX3d c01 = calcLineCenterPoints(0, 1);
  Eigen::ArrayX3d c12 = calcLineCenterPoints(1, 2);
  Eigen::ArrayX3d c23 = calcLineCenterPoints(2, 3);
  Eigen::ArrayX3d c30 = calcLineCenterPoints(3, 1);

  // std::cout << c01 << "\n" << c12 << "\n" << c23 << "\n" << c30 << "\n\n";
  centrePoints = (c01 + c23) / 2; // Pick any opposite sides

  // tangetial vector in the x direction wrt face
  tangentXVectors = c23 - c01;
  tangentXVectors.matrix().rowwise().normalize();

  // tangetial vector in the y direction wrt face
  tangentYVectors = (c30 - c12);
  tangentYVectors.matrix().rowwise().normalize();

  // normal vector in the z direction wrt face
  normalVectors =
      PanelGeometryUtils::rowwiseCross(tangentXVectors, tangentYVectors);
  normalVectors.matrix().rowwise().normalize();

  // centrePoints = centrePoints - normalVectors * 0.0001;
}

template <SurfaceType T>
Eigen::Isometry3d
PanelGeometry<T>::createLocalConversionMatrix(std::size_t faceIdx) {
  Eigen::Matrix3d rotationMatrix;
  rotationMatrix.col(0) = tangentXVectors.row(faceIdx).transpose();
  rotationMatrix.col(1) = tangentYVectors.row(faceIdx).transpose();
  rotationMatrix.col(2) = normalVectors.row(faceIdx).transpose();

  Eigen::Isometry3d transformLocalToGlobal = Eigen::Isometry3d::Identity();
  transformLocalToGlobal.linear() = rotationMatrix;
  transformLocalToGlobal.translation() = centrePoints.row(faceIdx);

  // https://gamemath.com/book/orient.html
  return transformLocalToGlobal.inverse();
}

template <SurfaceType T>
Eigen::ArrayX3d PanelGeometry<T>::convertToLocal(int faceIdx,
                                                 const ArrayX3d &points) const {
  // NOTE: This whole loop is inefficient. Eigen can broadcast
  // TODO: Fix this
  int nPoints = points.rows();
  Eigen::ArrayX3d convertedPoints(nPoints, 3);
  for (int iPoint = 0; iPoint < nPoints; iPoint++) {
    convertedPoints.row(iPoint) = (conversionMatrices[faceIdx] *
                                   (points.row(iPoint).transpose()).matrix());
  }
  return convertedPoints;
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
