#pragma once
#include "compTask.hpp"
#include "panel_method/ipanel.hpp"
#include "solver/isolver.hpp"
#include <Eigen/Core>
#include <memory>

class SourceDoubletSingleOct : public IPanelMethod {
  using IPM = IPanelMethod;

private:
  Eigen::VectorXf sourceStrength;
  std::vector<ComputeTask> surfacePanelCompTasks;
  std::vector<ComputeTask> wakePanelCompTasks;

protected:
  Eigen::MatrixXf assembleLhs() override;
  Eigen::VectorXf assembleRhs() override;
  Eigen::MatrixXf calculatePanelVelocities() override;

public:
  SourceDoubletSingleOct(const PanelGeometry<SurfacePanel> &surfacePanelGeo,
                         const PanelGeometry<WakePanel> &wakePanelGeo,
                         const EvalPoints<float> &evalPoints,
                         std::unique_ptr<ISolver> &&solver, float AoAd);
  void run() override;
  Eigen::ArrayXf getSource() const;
};
