#pragma once
#include "surface/surface_reader.hpp"
#include <Eigen/Core>
#include <string>
#include <unordered_map>

struct FlowParams {
  double aoa;
  double rho;
  double Vinf;
};
struct ReferenceGeom {
  double refArea;
};
struct AeroResults {
  PanelSet pSet;

  using AeroPanelResults = std::unordered_map<std::string, Eigen::ArrayXd>;
  using AeroSpanResults = std::unordered_map<std::string, Eigen::ArrayXd>;
  using AeroPolars = std::unordered_map<std::string, double>;

  ReferenceGeom refGeom;
  AeroPanelResults panelResults;
  AeroSpanResults spanResults;
  AeroPolars polars;
  FlowParams lastParams;

  AeroResults() = default;
  AeroResults(const AeroResults &result) = default;
};
