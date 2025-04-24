#pragma once
#include "concepts.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "surface/surface_panel.hpp"
#include "surface/wake_panel.hpp"

template <SurfaceType T> struct PanelGeometry;

namespace PanelGeometryUtils {

Eigen::MatrixXf rowwiseCross(const Eigen::MatrixXf &A,
                             const Eigen::MatrixXf &B);

} // namespace PanelGeometryUtils

template <SurfaceType T> struct PanelGeometry {

public:
  using Scalar = typename SurfacePanel::NodeMatrix::Scalar;
  using PointType = Eigen::Array<Scalar, 1, 3>;
  using VecType = PointType;
  // Constrain with concepts to be an Eigen indexable type: arrayXf, array,
  // vector

  SurfacePanel::NodeMatrix centrePoints;
  SurfacePanel::NodeMatrix tangentXVectors;
  SurfacePanel::NodeMatrix tangentYVectors;
  SurfacePanel::NodeMatrix normalVectors;

  Eigen::ArrayXf areas;
  std::vector<Eigen::Isometry3f> conversionMatrices;
  std::vector<Eigen::ArrayX3f> localFaceVertices;

  T mSurface;

  PanelGeometry() = default;

  PanelGeometry(const T &surface);
  PanelGeometry(T &&surface) noexcept;
  void panelGeoInit();
  void calculateCentrePointsandVectors();
  Eigen::Isometry3f createLocalConversionMatrix(std::size_t faceIdx);
  Eigen::ArrayX3f convertToLocal(int faceIdx, const ArrayX3f &points) const;
  float calcPolyArea(const Eigen::ArrayX3f &vertices) const;
};

using PanelGeometryPair =
    std::pair<PanelGeometry<SurfacePanel>, PanelGeometry<WakePanel>>;

