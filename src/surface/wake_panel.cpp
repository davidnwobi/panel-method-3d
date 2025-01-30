#include "surface/wake_panel.hpp"
#include "surface/surface_panel.hpp"
#include <Eigen/Core>

WakePanel::WakePanel(const SurfacePanel &surface) : SurfacePanel(surface) {}
WakePanel::WakePanel(SurfacePanel &&surface) noexcept
    : SurfacePanel(std::move(surface)) {}
WakePanel::WakePanel(NodeMatrix &&points, FaceNodeIdxMatrix &&faceNodeIdx,
                     TrailingEdgeIdxMatrix &&trailingEdgeIdx,
                     std::size_t nXsecs, std::size_t nYsecs) noexcept
    : SurfacePanel(std::move(points), std::move(faceNodeIdx), nXsecs, nYsecs),
      mTrailingEdgeIdx(std::move(trailingEdgeIdx)) {}
