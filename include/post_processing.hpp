#pragma once
#include "panel_geo/panel_geo.hpp"
#include "aerocalcs/aerocalcsingle.hpp"
#include <Eigen/Core>


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

  


