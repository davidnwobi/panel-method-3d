#pragma once
#include "compTask.hpp"
#include "panel_method/ipanel.hpp"
#include "solver/isolver.hpp"
#include <Eigen/Core>
#include <memory>

class SourceDoubletSingle : public IPanelMethod {
  using IPM = IPanelMethod;

private:
  Eigen::VectorXd sourceStrength;
  std::vector<ComputeTask> surfacePanelCompTasks;
  std::vector<ComputeTask> wakePanelCompTasks;

protected:
  Eigen::MatrixXd assembleLhs() override;
  Eigen::VectorXd assembleRhs() override;
  Eigen::MatrixXd calculatePanelVelocities() override;

public:
  SourceDoubletSingle(const PanelGeometry<SurfacePanel> &surfacePanelGeo,
                      const PanelGeometry<WakePanel> &wakePanelGeo,
                      const EvalPoints<double> &evalPoints,
                      std::unique_ptr<ISolver> &&solver, double AoAd);
  void run() override;
  Eigen::ArrayXd getSource() const;
};
