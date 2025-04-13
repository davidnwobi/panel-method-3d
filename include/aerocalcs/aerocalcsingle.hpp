#pragma once
#include "surface/surface_reader.hpp"
#include <Eigen/Core>
#include <string>
#include <unordered_map>

struct FlowParams {
  float aoa;
  float rho;
  float Vinf;
};
struct ReferenceGeom {
  float refArea;
};
struct AeroResults {
  PanelSet pSet;

  using AeroPanelResults = std::unordered_map<std::string, Eigen::ArrayXf>;
  using AeroSpanResults = std::unordered_map<std::string, Eigen::ArrayXf>;
  using AeroPolars = std::unordered_map<std::string, float>;

  ReferenceGeom refGeom;
  AeroPanelResults panelResults;
  AeroSpanResults spanResults;
  AeroPolars polars;
  FlowParams lastParams;

  AeroResults() = default;
  AeroResults(const AeroResults &result) = default;
};
