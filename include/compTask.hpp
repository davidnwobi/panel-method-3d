#pragma once
#include "concepts.hpp"
#include "evalPoints.hpp"
#include "panel_geo/panel_geo.hpp"
#include <Eigen/Core>
#include <span>
#include <vector>

struct ComputeTask {
  //
  struct Face {
    Eigen::ArrayX3f points;
    std::size_t faceIdx;
    Eigen::Array3f centrePoint;
    float area;
  };
  Face face;
  std::vector<std::size_t> indices;
  Eigen::ArrayX3f points;
};

using ComputeTaskPair =
    std::pair<std::vector<ComputeTask>, std::vector<ComputeTask>>;
// Ctor
template <SurfaceType Surface>
ComputeTask createInfluenceComputeTask(const PanelGeometry<Surface> &panelGeo,
                                       const EvalPoints<float> &evalPoints,
                                       std::size_t faceIdx) {
  ComputeTask compTask;

  compTask.face.faceIdx = faceIdx;
  compTask.face.centrePoint = panelGeo.centrePoints.row(faceIdx);
  compTask.face.area = panelGeo.areas(faceIdx);
  compTask.face.points = panelGeo.localFaceVertices[faceIdx];
  compTask.points = panelGeo.convertToLocal(faceIdx, evalPoints.mEvalPoints);

  return compTask;
}

template <SurfaceType Surface>
void createInfluenceComputeTask(ComputeTask &compTask,
                                const PanelGeometry<Surface> &panelGeo,
                                const EvalPoints<float> &evalPoints,
                                std::size_t faceIdx) {

  compTask.face.faceIdx = faceIdx;
  compTask.face.centrePoint = panelGeo.centrePoints.row(faceIdx);
  compTask.face.area = panelGeo.areas(faceIdx);
  compTask.face.points = panelGeo.localFaceVertices[faceIdx];
  compTask.points = panelGeo.convertToLocal(faceIdx, evalPoints.mEvalPoints);
}

ComputeTaskPair makeComputeTasksPairImpl(const PanelGeometryPair &panelGeometry,
                                         const EvalPoints<float> &evalPoints);

std::vector<ComputeTaskPair>
makeComputeTaskPairs(std::span<PanelGeometryPair> panelGeometries,
                     const EvalPoints<float> &evalPoints);
