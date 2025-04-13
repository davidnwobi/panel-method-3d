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
  std::reference_wrapper<const EvalPoints<float>> evalPointsRef;
  Eigen::RowVector3f freeStream;
  Eigen::ArrayX3f velocities;
  std::unique_ptr<ISolver> solver;

  virtual Eigen::MatrixXf assembleLhs() = 0;
  virtual Eigen::VectorXf assembleRhs() = 0;
  virtual Eigen::MatrixXf calculatePanelVelocities() = 0;

public:
  Eigen::VectorXf solution;
  IPanelMethod(const PanelGeometry<SurfacePanel> &surfacePanelGeo,
               const PanelGeometry<WakePanel> &wakePanelGeo,
               const EvalPoints<float> &evalPoints,
               std::unique_ptr<ISolver> &&solver)
      : surfacePanelRef(surfacePanelGeo), wakePanelRef(wakePanelGeo),
        evalPointsRef(evalPoints), solver(std::move(solver)) {}

  Eigen::ArrayX3f getComputedVelocites() const { return velocities; }
  Eigen::VectorXf &getSolution() { return solution; }
  Eigen::RowVector3f getfreeStream() const { return freeStream; }
  void setFlowParams(float AoAd) {
    float angleOfAttack = AoAd * M_PI / 180;
    freeStream = {std::cos(angleOfAttack), 0, std::sin(angleOfAttack)};
  };
  virtual void run() = 0;
  virtual ~IPanelMethod() = default;
};
