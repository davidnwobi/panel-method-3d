#include "surface/surface_panel.hpp"
#include <string>

SurfacePanel::SurfacePanel(NodeMatrix &&points, FaceNodeIdxMatrix &&faceNodeIdx,
                           std::size_t nXsecs, std::size_t nYsecs) noexcept
    : mPoints(std::move(points)), mFaceNodeIdx(std::move(faceNodeIdx)),
      nXsecs(nXsecs), nYsecs(nYsecs) {}

std::string SurfacePanel::to_string(bool verbose) {
  std::ostringstream oss;

  oss << "Surface with: \n";
  oss << mPoints.rows() << " Nodes\n";
  oss << mFaceNodeIdx.rows() << " Faces\n";
  if (verbose) {
    oss << "# Nodes x y z\n";
    oss << mPoints;
    oss << "\n\n# Faces p0 p1 p2 p3\n";
    oss << mFaceNodeIdx;
  }
  return oss.str();
}
