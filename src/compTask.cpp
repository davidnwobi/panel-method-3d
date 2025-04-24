#include "compTask.hpp"
#include "concepts.hpp"
#include "evalPoints.hpp"
#include "panel_geo/panel_geo.hpp"
#include "utils/utils.hpp"
#include <Eigen/Core>
#include <numeric>
#include <ranges>
#include <vector>


ComputeTaskPair makeComputeTasksPairImpl(const PanelGeometryPair &panelGeometry,
                                         const EvalPoints<float> &evalPoints) {
  int surfPanels = panelGeometry.first.centrePoints.rows();
  int wakePanels = panelGeometry.second.centrePoints.rows();
  auto surfaceComputeTaskView =
      RANGE(surfPanels) |
      std::views::transform([&panelGeometry, &evalPoints](int faceIdx) {
        return createInfluenceComputeTask(panelGeometry.first, evalPoints,
                                          faceIdx);
      });
  auto wakeComputeTaskView =
      RANGE(wakePanels) |
      std::views::transform([&panelGeometry, &evalPoints](int faceIdx) {
        return createInfluenceComputeTask(panelGeometry.second, evalPoints,
                                          faceIdx);
      });

  ComputeTaskPair compTaskPair;
  compTaskPair.first.reserve(surfPanels);
  compTaskPair.second.reserve(wakePanels);
  std::ranges::copy(surfaceComputeTaskView,
                    std::back_inserter(compTaskPair.first));
  std::ranges::copy(wakeComputeTaskView,
                    std::back_inserter(compTaskPair.second));
  return compTaskPair;
};

auto makeChunkData(const auto &panelGeometry) {

  auto chunkSize = panelGeometry | std::views::transform([](const auto &pg) {
                     return pg.centrePoints.rows();
                   });
  std::vector<size_t> chunkStart(chunkSize.size(), 0);
  std::exclusive_scan(chunkSize.begin(), chunkSize.end(), chunkStart.begin(),
                      0);
  return std::pair{chunkStart, chunkSize};
}
std::vector<ComputeTaskPair>
makeComputeTaskPairs(std::span<PanelGeometryPair> panelGeometries,
                     const EvalPoints<float> &evalPoints) {

  int nBodies = panelGeometries.size();

  auto pairs =
      RANGE(nBodies) |
      std::views::transform([&panelGeometries, &evalPoints](int iBody) {
        return makeComputeTasksPairImpl(panelGeometries[iBody], evalPoints);
      });

  std::vector<ComputeTaskPair> compTaskPairs;
  compTaskPairs.reserve(nBodies);
  std::ranges::copy(pairs, std::back_inserter(compTaskPairs));

  // Generate correct faceIdx for both surfaces for 0 to the total number of
  // panels
  auto [surfChunkStart, surfChunkSize] =
      makeChunkData(panelGeometries | std::views::transform([](const auto &pgPair) {
                      return pgPair.first;
                    }));
  auto surfFaceIdxView =
      RANGE(nBodies) | std::views::transform([&](auto idx) {
        return std::views::iota(surfChunkStart[idx],
                           surfChunkStart[idx] + surfChunkSize[idx]);
      });

  for (auto i : RANGE(nBodies)) {
    for (auto j : RANGE(compTaskPairs[i].first.size())) {
      compTaskPairs[i].first[j].face.faceIdx = surfFaceIdxView[i][j];
    }
  }
  return compTaskPairs;
}
