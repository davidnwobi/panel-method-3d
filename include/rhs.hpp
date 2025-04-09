#pragma once
#include <Eigen/Core>
#include <span>
#include <compTask.hpp>
#include <concepts.hpp>
#include <utility>

std::pair<Eigen::VectorXd, Eigen::VectorXd>
assembleRhsImpl(std::span<const ComputeTask> surfacePanelCompTasks,
                const PanelGeometry<SurfacePanel> &surfacePanelGeo,
                const EvalPoints<double> &evalPoints,
                const Eigen::Ref<Eigen::Array3d> &freeStream);
  

std::pair<Eigen::VectorXd, Eigen::VectorXd>
assembleRhs(std::span<const ComputeTaskPair> compTaskPairs,
            std::span<const PanelGeometryPair> panelGeometries,
            const EvalPoints<double> &evalPoints,
            const Eigen::Ref<Eigen::Array3d> &freeStream);
