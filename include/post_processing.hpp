#pragma once
#include "concepts.hpp"
#include "panel_geo/panel_geo.hpp"
#include "aerocalcs/aerocalcsingle.hpp"
#include <Eigen/Core>
#include "mat_reader/mat_reader.hpp"
#include "helpers.hpp"
#include "utils/utils.hpp"
#include <iostream>


void postProcessPanelResults(
    const PanelGeometry<SurfacePanel> surfacePanelGeo,
    const Eigen::Ref<const Eigen::ArrayXXd> &surfaceVelocties,
    AeroResults &out);
  
void postProcessPolars(AeroResults &out);
  
auto hChunk1D(const Eigen::Ref<const Eigen::ArrayXd> &combined,
              std::span<const size_t> chunkStart,
              std::span<const size_t> chunkSize);
  

Eigen::ArrayXXd calculatePanelVelocities(
    const PanelGeometry<SurfacePanel> &panel,
    const Eigen::Ref<const Eigen::ArrayXd> &doubletStrength,
    const Eigen::Ref<const Eigen::ArrayXd> &sourceStrength,
    const Eigen::Ref<const Eigen::ArrayXd> &freeStream);
  

AeroResults
postProcessBodyImpl(const PanelGeometry<SurfacePanel> surfacePanelGeo,
                    const Eigen::Ref<const Eigen::ArrayXXd> &surfaceVelocities,
                    const Eigen::Ref<const Eigen::ArrayXXd> &doubletStrength,
                    const FlowParams &flowParams, double refArea);

std::vector<AeroResults>
postProcessBody(std::span<const PanelGeometryPair> panelGeometries,
                const Eigen::Ref<const Eigen::ArrayXd> &doubletStrengths,
                const Eigen::Ref<const Eigen::ArrayXd> &sourceStrengths,
                const FlowParams &flowParams, const ReferenceGeom &refGeom);

void writeBodyData(const std::string outfile, const PanelGeometryPair &ppair,
                   AeroResults &results) ;

  
auto postProcessTotalPolars (auto &&outO) {
  auto out = std::move(outO);
  double q =
      0.5 * out.lastParams.rho * out.lastParams.Vinf * out.lastParams.Vinf;
  Eigen::Array3d F(3);
  F << out.polars["Fx"], out.polars["Fy"], out.polars["Fz"];
  Eigen::ArrayXd CF = F / (q * out.refGeom.refArea);
  double CL = (-CF(0) * std::sin(out.lastParams.aoa * M_PI / 180) +
               CF(2) * std::cos(out.lastParams.aoa * M_PI / 180));
  double CD = (CF(0) * std::cos(out.lastParams.aoa * M_PI / 180) +
               CF(2) * std::sin(out.lastParams.aoa * M_PI / 180));

  out.polars["aoa"] = out.lastParams.aoa;
  out.polars["CFx"] = CF(0);
  out.polars["CFy"] = CF(1);
  out.polars["CFz"] = CF(2);
  out.polars["CL"] = CL;
  out.polars["CD"] = CD;
  return out;
}



template <AeroResults_range_range Ra> 
void accumulateTotalPolars(std::string outdir,
                            Ra &&R) {

	namespace views = std::views;
  // Accumulate Forces
  PRINT_TYPE(R[0]);
  auto totalResults =
      R | views::transform([](AeroResults_range auto &&results) {
        AeroResults accResults = results[0];
        std::ranges::for_each(results.begin() + 1, results.end(),
                              [&accResults](auto &&res) {
                                accResults.polars["Fx"] += res.polars["Fx"];
                                accResults.polars["Fy"] += res.polars["Fy"];
                                accResults.polars["Fz"] += res.polars["Fz"];
                              });
        return accResults;
      });
  auto totalPolars = views::transform(
      totalResults, [](auto &&res) { return postProcessTotalPolars(res); });

  std::vector<std::string> headers = {"aoa", "Fx",  "Fy", "Fz", "CFx",
                                      "CFy", "CFz", "CL", "CD"};
  Eigen::ArrayXXd polars(totalPolars.size(), headers.size());
  for (auto i : RANGE(totalPolars.size())) {
    for (auto j : RANGE(headers.size())) {
      polars(i, j) = totalPolars[i].polars[headers[j]];
    }
  }
  std::cout << polars;
  savetxt(outdir + "/polars.dat", polars, " ", headers);
}

