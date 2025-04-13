#pragma once
#include "compTask.hpp"
#include "panel_method/ipanel.hpp"
#include "solver/isolver.hpp"
#include <Eigen/Core>
#include <memory>

class SourceDoubletSingle : public IPanelMethod {
  using IPM = IPanelMethod;

private:
  std::vector<ComputeTask> surfacePanelCompTasks;
  std::vector<ComputeTask> wakePanelCompTasks;

protected:
  Eigen::MatrixXf assembleLhs() override;
  Eigen::VectorXf assembleRhs() override;
  Eigen::MatrixXf calculatePanelVelocities() override;

public:
  Eigen::VectorXf sourceStrength;
  SourceDoubletSingle(const PanelGeometry<SurfacePanel> &surfacePanelGeo,
                      const PanelGeometry<WakePanel> &wakePanelGeo,
                      const EvalPoints<float> &evalPoints,
                      std::unique_ptr<ISolver> &&solver, float AoAd);
  void run() override;
  Eigen::ArrayXf getSource() const;
};
