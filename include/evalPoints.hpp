#pragma once
#include <Eigen/Core>
#include "panel_geo/panel_geo.hpp"

template <class T = float> struct EvalPoints {
  Eigen::Array<T, -1, 3> mEvalPoints;

  EvalPoints() = default;
  EvalPoints(std::size_t rows) { mEvalPoints.setZero(rows, 3); }
  EvalPoints(const Eigen::Array<T, -1, 3> &arr) { mEvalPoints = arr; }
};



EvalPoints<float>
create_eval_points(std::span<const PanelGeometryPair> panelGeometries);

