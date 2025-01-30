#pragma once
#include "concepts.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

template <SurfaceType T> struct PanelGeometry;

namespace PanelGeometryUtils {

Eigen::MatrixXd rowwiseCross(const Eigen::MatrixXd &A,
                             const Eigen::MatrixXd &B);

} // namespace PanelGeometryUtils

template <SurfaceType T> struct PanelGeometry {

public:
  using Scalar = typename SurfacePanel::NodeMatrix::Scalar;
  using PointType = Eigen::Array<Scalar, 1, 3>;
  using VecType = PointType;
  // Constrain with concepts to be an Eigen indexable type: arrayXd, array,
  // vector

  SurfacePanel::NodeMatrix centrePoints;
  SurfacePanel::NodeMatrix tangentXVectors;
  SurfacePanel::NodeMatrix tangentYVectors;
  SurfacePanel::NodeMatrix normalVectors;

  Eigen::ArrayXd areas;
  std::vector<Eigen::Isometry3d> conversionMatrices;
  std::vector<Eigen::ArrayX3d> localFaceVertices;

  T mSurface;

  PanelGeometry() = default;

  PanelGeometry(const T &surface);
  PanelGeometry(T &&surface) noexcept;
  void panelGeoInit();
  void calculateCentrePointsandVectors();
  Eigen::Isometry3d createLocalConversionMatrix(std::size_t faceIdx);
  Eigen::ArrayX3d convertToLocal(int faceIdx, const ArrayX3d &points) const;
  double calcPolyArea(const Eigen::ArrayX3d &vertices) const;
};
