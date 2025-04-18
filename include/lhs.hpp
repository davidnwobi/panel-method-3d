#pragma once
#include <Eigen/Core>
#include <compTask.hpp>
#include <concepts.hpp>
#include <span>

std::tuple<Eigen::MatrixXd, Eigen::VectorXd, Eigen::VectorXd>
assembleLhs(std::span<const PanelGeometryPair> panelGeometries,
            const EvalPoints<double> &evalPoints,
            const Eigen::Ref<Eigen::Array3d> &freeStream);
