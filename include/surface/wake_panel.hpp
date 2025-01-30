#pragma once
#include "surface/surface_panel.hpp"
#include <Eigen/Core>

using namespace Eigen;
struct WakePanel : public SurfacePanel {

  using TrailingEdgeIdxMatrix = Array<int, Dynamic, 2>;

  TrailingEdgeIdxMatrix mTrailingEdgeIdx;

  WakePanel() = default;

  WakePanel(const SurfacePanel &surface);
  WakePanel(SurfacePanel &&surface) noexcept;
  WakePanel(NodeMatrix &&points, FaceNodeIdxMatrix &&faceNodeIdx,
            TrailingEdgeIdxMatrix &&trailingEdgeIdx, std::size_t nXsecs,
            std::size_t nYsecs) noexcept;
};
