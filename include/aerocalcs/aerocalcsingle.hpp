
#include "panel_method/ipanel.hpp"
#include "panel_method/source_doublet_single.hpp"
#include "solver/isolver.hpp"
#include "surface/surface_reader.hpp"
#include <Eigen/Core>
#include <panel_geo/panel_geo.hpp>
#include <string>
#include <type_traits>
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

  template <typename ConcretePanelMethod>
  AeroCalcSingle(PanelSet &&pSet, ReferenceGeom &&refGeom,
                 std::type_identity<ConcretePanelMethod> &&,
                 std::unique_ptr<ISolver> &&solver)
      : pSet(pSet), refGeom(refGeom), surfacePanelGeo(pSet.body),
        wakePanelGeo(pSet.wake), evalPoints(surfacePanelGeo.centrePoints),
        pm(std::make_unique<ConcretePanelMethod>(surfacePanelGeo, wakePanelGeo,
                                                 evalPoints, std::move(solver),
                                                 0.0)) {}
  void run(FlowParams &&params);

private:
  FlowParams lastParams;
  PanelGeometry<SurfacePanel> surfacePanelGeo;
  PanelGeometry<WakePanel> wakePanelGeo;
  EvalPoints<double> evalPoints;
  std::unique_ptr<IPanelMethod> pm;

  void post_process();
  void postProcessPanelResults();
  void postProcessPolars();
  void postProcessSpanResults();
};
