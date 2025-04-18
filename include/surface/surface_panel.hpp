#pragma once
#include <Eigen/Core>

using namespace Eigen;
struct SurfacePanel {

  using NodeMatrix = Array<double, Dynamic, 3>;
  using FaceNodeIdxMatrix = Array<int, Dynamic, 4>;

  NodeMatrix mPoints;
  FaceNodeIdxMatrix mFaceNodeIdx;
  std::size_t nXsecs;
  std::size_t nYsecs;

  SurfacePanel() = default;
  SurfacePanel(NodeMatrix &&points, FaceNodeIdxMatrix &&faceNodeIdx,
               std::size_t nXsecs, std::size_t nYsecs) noexcept;
  std::string to_string(bool verbose = false);
};
