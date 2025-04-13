#pragma once
#include <Eigen/Core>
#include <compTask.hpp>
#include <concepts.hpp>
#include <span>

std::tuple<Eigen::MatrixXf, Eigen::VectorXf, Eigen::VectorXf>
assembleLhs(std::span<const PanelGeometryPair> panelGeometries,
            const EvalPoints<float> &evalPoints,
            const Eigen::Ref<Eigen::Array3f> &freeStream);
