#include "post_processing.hpp"
#include "central_difference.hpp"
#include "concepts.hpp"
#include "helpers.hpp"
#include "panel_geo/panel_geo.hpp"
#include "surface/surface_panel.hpp"
#include "surface/wake_panel.hpp"
#include "utils/utils.hpp"

void postProcessPanelResults(
    const PanelGeometry<SurfacePanel> surfacePanelGeo,
    const Eigen::Ref<const Eigen::ArrayXXf> &surfaceVelocties,
    AeroResults &out) {

  Eigen::ArrayXf dCp = 1 - (surfaceVelocties.rowwise().squaredNorm());

  float q =
      0.5 * out.lastParams.rho * out.lastParams.Vinf * out.lastParams.Vinf;
  Eigen::ArrayXf dP = q * dCp;
  Eigen::ArrayX3f dF =
      -(surfacePanelGeo.normalVectors.colwise() * (dP * surfacePanelGeo.areas));

  out.panelResults["dCp"] = dCp;
  out.panelResults["dVx"] = surfaceVelocties.col(0).eval();
  out.panelResults["dVy"] = surfaceVelocties.col(1).eval();
  out.panelResults["dVz"] = surfaceVelocties.col(2).eval();
  out.panelResults["dP"] = dP.eval();
  out.panelResults["dFx"] = dF.col(0).eval();
  out.panelResults["dFy"] = dF.col(1).eval();
  out.panelResults["dFz"] = dF.col(2).eval();

  Eigen::Index nXsecs = surfacePanelGeo.mSurface.nXsecs;
  Eigen::Index nYsecs = surfacePanelGeo.mSurface.nYsecs;
  const auto &y_n = surfacePanelGeo.mSurface.mPoints.col(1);
  const auto &mu = out.panelResults["mu"];

  using namespace Eigen::placeholders;

  const Eigen::ArrayXf cl_doublet_alpha =
      -2 * (mu.reshaped(nXsecs, nYsecs)(nXsecs - 1, all) -
            mu.reshaped(nXsecs, nYsecs)(0, all));

  out.polars["CL"] =
      (cl_doublet_alpha *
       (y_n.reshaped(nXsecs + 1, nYsecs + 1).row(nXsecs / 2).tail(nYsecs) -
        y_n.reshaped(nXsecs + 1, nYsecs + 1).row(nXsecs / 2).head(nYsecs))
           .abs()
           .transpose())
          .sum() /
      out.refGeom.refArea;
}
void postProcessPolars(AeroResults &out) {
  float q =
      0.5 * out.lastParams.rho * out.lastParams.Vinf * out.lastParams.Vinf;
  Eigen::Array3f F(3);
  F << out.panelResults["dFx"].sum(), out.panelResults["dFy"].sum(),
      out.panelResults["dFz"].sum();
  Eigen::ArrayXf CF = F / (q * out.refGeom.refArea);
  // float CL = (-CF(0) * std::sin(out.lastParams.aoa * M_PI / 180) +
  //              CF(2) * std::cos(out.lastParams.aoa * M_PI / 180));
  float CD = (F(0) * std::cos(out.lastParams.aoa * M_PI / 180) +
               F(2) * std::sin(out.lastParams.aoa * M_PI / 180));

  out.polars["aoa"] = out.lastParams.aoa;
  out.polars["Fx"] = F(0);
  out.polars["Fy"] = F(1);
  out.polars["Fz"] = F(2);
  out.polars["CFx"] = CF(0);
  out.polars["CFy"] = CF(1);
  out.polars["CFz"] = CF(2);
  // out.polars["CL"] = CL;
  out.polars["CD"] = CD;
}
auto hChunk1D(const Eigen::Ref<const Eigen::ArrayXf> &combined,
              std::span<const size_t> chunkStart,
              std::span<const size_t> chunkSize) {
  auto chunkedView =
      RANGE(chunkStart.size()) |
      std::views::transform([&](size_t iChunk) -> Eigen::ArrayXf {
        return combined.middleRows(chunkStart[iChunk], chunkSize[iChunk]);
      });
  return chunkedView;
}

Eigen::ArrayXXf calculatePanelVelocities(
    const PanelGeometry<SurfacePanel> &panel,
    const Eigen::Ref<const Eigen::ArrayXf> &doubletStrength,
    const Eigen::Ref<const Eigen::ArrayXf> &sourceStrength,
    const Eigen::Ref<const Eigen::ArrayXf> &freeStream) {
  std::size_t nYSecs = panel.mSurface.nYsecs;
  std::size_t nXsecs = panel.mSurface.nXsecs;

  // Is this being reshaped properly? yes

  // Convert points to panel coordinate surface chord-wise from trailing edge
  // bottom to trailing edge top
  // is solution solved  in the same order>	depends on if face has the same
  // order, yes compute task <- centerpoints <- face

  // 1): Convert the centrepoints to panel reference frame
  const auto &pcp = panel.centrePoints;
  Eigen::ArrayXf Sx(nXsecs * nYSecs);
  Sx.setZero();
  for (int iY = 0; iY < nYSecs; iY++) {
    Sx.middleRows(iY * nXsecs + 1, nXsecs - 1) =
        (pcp.middleRows(iY * nXsecs + 1, nXsecs - 1) -
         pcp.middleRows(iY * nXsecs, nXsecs - 1))
            .matrix()
            .rowwise()
            .norm();
    std::inclusive_scan(Sx.begin() + (iY * nXsecs),
                        Sx.begin() + (nXsecs * (iY + 1)),
                        Sx.begin() + (iY * nXsecs), std::plus<float>{});
  }

  Eigen::ArrayXXf yPoints(nXsecs, nYSecs);
  yPoints.setZero();
  for (int iX = 0; iX < nXsecs; iX++) {
    for (int iY = 1; iY < nYSecs; iY++) {
      yPoints(iX, iY) =
          (pcp.row(iX + iY * nXsecs) - pcp.row(iX + (iY - 1) * nXsecs))
              .matrix()
              .norm() +
          yPoints(iX, iY - 1);
    }
  }
  Eigen::ArrayXXf zPoints = pcp.col(2).reshaped(nXsecs, nYSecs);

  // 2:) u = -d(mu)/d(x_l); v = -d(mu)/d(y_l); w = sigma
  Eigen::ArrayXXf fPoints(nXsecs, nYSecs);
  fPoints << doubletStrength.reshaped(nXsecs, nYSecs); // no minus

  Eigen::ArrayX3f inducedVelocities(nXsecs * nYSecs, 3);
  inducedVelocities << -centralDifference<true>(Sx.reshaped(nXsecs, nYSecs),
                                                fPoints)
                            .reshaped(),
      -centralDifference<false>(yPoints, fPoints).reshaped(), -sourceStrength;

  Eigen::ArrayX3f globalVelocites(nXsecs * nYSecs, 3);

  globalVelocites << rowwiseDotProduct(panel.tangentXVectors, freeStream),
      rowwiseDotProduct(panel.tangentYVectors, freeStream),
      rowwiseDotProduct(panel.normalVectors, freeStream);

  return globalVelocites + inducedVelocities;
}

AeroResults
postProcessBodyImpl(const PanelGeometry<SurfacePanel> surfacePanelGeo,
                    const Eigen::Ref<const Eigen::ArrayXXf> &surfaceVelocities,
                    const Eigen::Ref<const Eigen::ArrayXXf> &doubletStrength,
                    const FlowParams &flowParams, float refArea) {
  AeroResults results;
  results.lastParams = flowParams;
  results.refGeom = {refArea};
  results.panelResults["mu"] = doubletStrength;
  postProcessPanelResults(surfacePanelGeo, surfaceVelocities, results);
  postProcessPolars(results);
  return results;
}
std::vector<AeroResults>
postProcessBody(std::span<const PanelGeometryPair> panelGeometries,
                const Eigen::Ref<const Eigen::ArrayXf> &doubletStrengths,
                const Eigen::Ref<const Eigen::ArrayXf> &sourceStrengths,
                const FlowParams &flowParams, const ReferenceGeom &refGeom) {

  auto chunkSize =
      panelGeometries | views::transform([](const PanelGeometryPair &pair) {
        return pair.first.centrePoints.rows();
      });
  std::vector<size_t> chunkStart(chunkSize.size(), 0);
  std::partial_sum(chunkSize.begin(), chunkSize.end() - 1,
                   chunkStart.begin() + 1);

  auto chunkedDoublet =
      RANGE(chunkSize.size()) |
      std::views::transform([&](size_t iChunk) -> Eigen::ArrayXf {
        return doubletStrengths.middleRows(chunkStart[iChunk],
                                           chunkSize[iChunk]);
      });
  auto chunkedSource =
      RANGE(chunkSize.size()) |
      std::views::transform([&](size_t iChunk) -> Eigen::ArrayXf {
        return sourceStrengths.middleRows(chunkStart[iChunk],
                                          chunkSize[iChunk]);
      });
  auto freeStream = getFreeStream(flowParams.aoa, flowParams.Vinf);
  auto computedVelocitiesView =
      RANGE(panelGeometries.size()) |
      std::views::transform([&](std::size_t idx) {
        return calculatePanelVelocities(panelGeometries[idx].first,
                                        chunkedDoublet[idx], chunkedSource[idx],
                                        freeStream);
      });
  auto resultsView =
      RANGE(panelGeometries.size()) | views::transform([&](std::size_t idx) {
        return postProcessBodyImpl(
            panelGeometries[idx].first, computedVelocitiesView[idx],
            chunkedDoublet[idx], flowParams, refGeom.refArea);
      });
  std::vector<AeroResults> results(chunkSize.size());
  std::ranges::copy(resultsView, results.begin());
  return results;
}

void writeBodyData(const std::string outfile, const PanelGeometryPair &ppair,
                   AeroResults &results) {

  std::vector<std::string> headers = {"x",   "y",   "z",   "A",  "dCp",
                                      "dVx", "dVy", "dVz", "dP", "dFx",
                                      "dFy", "dFz", "mu"};
  savetxt(outfile,
          (Eigen::ArrayXXf(ppair.first.centrePoints.rows(), headers.size())
               << ppair.first.centrePoints.col(0),
           ppair.first.centrePoints.col(1), ppair.first.centrePoints.col(2),
           ppair.first.areas, results.panelResults["dCp"],
           results.panelResults["dVx"], results.panelResults["dVy"],
           results.panelResults["dVz"], results.panelResults["dP"],
           results.panelResults["dFx"], results.panelResults["dFy"],
           results.panelResults["dFz"], results.panelResults["mu"])
              .finished(),
          " ", headers);
}

template void accumulateTotalPolars<std::vector<std::vector<AeroResults>>>(
    std::string outdir, std::vector<std::vector<AeroResults>> &&r);
