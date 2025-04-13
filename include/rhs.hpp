#pragma once
#include <Eigen/Core>
#include <span>
#include <compTask.hpp>
#include <concepts.hpp>
#include <utility>

std::pair<Eigen::VectorXf, Eigen::VectorXf>
assembleRhsImpl(std::span<const ComputeTask> surfacePanelCompTasks,
                const PanelGeometry<SurfacePanel> &surfacePanelGeo,
                const EvalPoints<float> &evalPoints,
                const Eigen::Ref<Eigen::Array3f> &freeStream);
  

std::pair<Eigen::VectorXf, Eigen::VectorXf>
assembleRhs(std::span<const ComputeTaskPair> compTaskPairs,
            std::span<const PanelGeometryPair> panelGeometries,
            const EvalPoints<float> &evalPoints,
            const Eigen::Ref<Eigen::Array3f> &freeStream);
