#pragma once
#include "concepts.hpp"
#include "evalPoints.hpp"
#include "panel_geo/panel_geo.hpp"
#include <Eigen/Core>
#include <vector>

struct ComputeTask {
  //
  struct Face {
    Eigen::ArrayX3d points;
    std::size_t faceIdx;
  };
  Face face;
  std::vector<std::size_t> indices;
  Eigen::ArrayX3d points;
};

// Ctor
template <SurfaceType Surface>
ComputeTask createInfluenceComputeTask(const PanelGeometry<Surface> &panelGeo,
                                       EvalPoints<double> evalPoints,
                                       std::size_t faceIdx,
                                       const std::vector<std::size_t> &idx) {
  ComputeTask compTask;

  compTask.face.faceIdx = faceIdx;
  compTask.face.points = panelGeo.localFaceVertices[faceIdx];

  // Convert Points
  compTask.indices = idx;
  compTask.points = panelGeo.convertToLocal(faceIdx, evalPoints.mEvalPoints);
  return compTask;
}
