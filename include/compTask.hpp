#pragma once
#include "concepts.hpp"
#include "evalPoints.hpp"
#include "panel_geo/panel_geo.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <vector>

struct ComputeTask {
  //
  struct Face {
    Eigen::ArrayX3d points;
    std::size_t faceIdx;
    Eigen::Array3d centrePoint;
    double area;
  };
  Face face;
  std::vector<std::size_t> indices;
  Eigen::ArrayX3d points;
};

// Ctor
static int i = 0;
template <SurfaceType Surface>
ComputeTask createInfluenceComputeTask(const PanelGeometry<Surface> &panelGeo,
                                       EvalPoints<double> evalPoints,
                                       std::size_t faceIdx,
                                       const std::vector<std::size_t> &idx) {
  ComputeTask compTask;

  compTask.face.faceIdx = faceIdx;
  compTask.face.centrePoint = panelGeo.centrePoints.row(faceIdx);
  compTask.face.area = panelGeo.areas(faceIdx);
  // print("ID: ", faceIdx, " Area: ", compTask.face.area,
  //      " CenterPoint: ", compTask.face.centrePoint);

  compTask.face.points = panelGeo.localFaceVertices[faceIdx];

  // Convert Points
  compTask.points = panelGeo.convertToLocal(faceIdx, evalPoints.mEvalPoints);
  // print("centers ",
  //(evalPoints.mEvalPoints(idx, Eigen::placeholders::all)).row(i).eval());
  // print("Local XNormals", (panelGeo.tangentXVectors).row(i).eval());
  // print("Local YNormals", (panelGeo.tangentYVectors).row(i).eval());
  // print("Local Normals", (panelGeo.normalVectors).row(i++).eval());

  return compTask;
}
