#pragma once
#include <Eigen/Core>
#include <span>
#include <compTask.hpp>
#include <concepts.hpp>

Eigen::MatrixXd
assembleLhsImpl(std::span<const ComputeTask> surfacePanelCompTasks,
                std::span<const ComputeTask> wakePanelCompTasks,
                const PanelGeometry<WakePanel> &wakePanelGeo,
                const EvalPoints<double> &evalPoints);


Eigen::MatrixXd assembleLhs(std::span<const ComputeTaskPair> compTaskPairs,
                            std::span<const PanelGeometryPair> panelGeometries,
                            const EvalPoints<double> &evalPoints);
  
