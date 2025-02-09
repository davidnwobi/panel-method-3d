#pragma once
#include "evalPoints.hpp"
#include "panel_geo/panel_geo.hpp"
#include "solver/isolver.hpp"
#include "surface/surface_panel.hpp"
#include "surface/wake_panel.hpp"
#include <Eigen/Core>
#include <memory>

class IPanelMethod {

protected:
  std::reference_wrapper<const PanelGeometry<SurfacePanel>> surfacePanelRef;
  std::reference_wrapper<const PanelGeometry<WakePanel>> wakePanelRef;
  std::reference_wrapper<const EvalPoints<double>> evalPointsRef;
  Eigen::RowVector3d freeStream;
  Eigen::VectorXd solution;
  Eigen::ArrayX3d velocities;
  std::unique_ptr<ISolver> solver;

  virtual Eigen::MatrixXd assembleLhs() = 0;
  virtual Eigen::VectorXd assembleRhs() = 0;
  virtual Eigen::MatrixXd calculatePanelVelocities() = 0;

public:
  IPanelMethod(const PanelGeometry<SurfacePanel> &surfacePanelGeo,
               const PanelGeometry<WakePanel> &wakePanelGeo,
               const EvalPoints<double> &evalPoints,
               std::unique_ptr<ISolver> &&solver)
      : surfacePanelRef(surfacePanelGeo), wakePanelRef(wakePanelGeo),
        evalPointsRef(evalPoints), solver(std::move(solver)) {}

  Eigen::ArrayX3d getComputedVelocites() const { return velocities; }
  Eigen::VectorXd getSolution() const { return solution; }
  Eigen::RowVector3d getfreeStream() const { return freeStream; }
  void setFlowParams(double AoAd) {
    double angleOfAttack = AoAd * M_PI / 180;
    freeStream = {std::cos(angleOfAttack), 0, std::sin(angleOfAttack)};
  };
  virtual void run() = 0;
  virtual ~IPanelMethod() = default;
};
