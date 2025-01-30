
#include "panel_method/source_doublet_single.hpp"
#include "solver/isolver.hpp"
#include "surface/surface_reader.hpp"
#include <Eigen/Core>
#include <panel_geo/panel_geo.hpp>
#include <string>
#include <unordered_map>

class AeroCalcSingle {
  PanelSet pSet;

public:
  struct FlowParams {
    double aoa;
    double rho;
    double Vinf;
  };
  struct ReferenceGeom {
    double refArea;
  };
  using AeroPanelResults = std::unordered_map<std::string, Eigen::ArrayXd>;
  using AeroSpanResults = std::unordered_map<std::string, Eigen::ArrayXd>;
  using AeroPolars = std::unordered_map<std::string, double>;

  ReferenceGeom refGeom;
  AeroPanelResults panelResults;
  AeroSpanResults spanResults;
  AeroPolars polars;

  AeroCalcSingle(PanelSet &&pSet, ReferenceGeom &&refGeom,
                 std::unique_ptr<ISolver> &&solver);
  void run(FlowParams &&params);

private:
  FlowParams lastParams;
  PanelGeometry<SurfacePanel> surfacePanelGeo;
  PanelGeometry<WakePanel> wakePanelGeo;
  EvalPoints<double> evalPoints;
  SourceDoubletSingle pm;

  void post_process();
  void postProcessPanelResults();
  void postProcessPolars();
  void postProcessSpanResults();
};
