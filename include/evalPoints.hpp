#pragma once
#include <Eigen/Core>

template <class T = double> struct EvalPoints {
  Eigen::Array<T, -1, 3> mEvalPoints;

  EvalPoints() = default;
  EvalPoints(std::size_t rows) { mEvalPoints.setZero(rows, 3); }
  EvalPoints(const Eigen::Array<T, -1, 3> &arr) { mEvalPoints = arr; }
};
