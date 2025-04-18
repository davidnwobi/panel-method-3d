#include "evalPoints.hpp"
#include <numeric>

EvalPoints<double>
create_eval_points(std::span<const PanelGeometryPair> panelGeometries) {
  auto totalEvalPoints =
      std::accumulate(panelGeometries.begin(), panelGeometries.end(), 0,
                      [](std::size_t count, const PanelGeometryPair &panelGeo) {
                        return count + panelGeo.first.centrePoints.rows();
                      });

  EvalPoints<double> evalPoints(totalEvalPoints);
  std::size_t iPoints = 0;
  auto pointsView = panelGeometries | std::views::transform([](auto const &g) {
                      return g.first.centrePoints;
                    });
  std::ranges::for_each(pointsView, [&](auto const &pts) {
    const auto rows = pts.rows();
    evalPoints.mEvalPoints.middleRows(iPoints, rows) = pts;
    iPoints += rows;
  });
  return evalPoints;
}
