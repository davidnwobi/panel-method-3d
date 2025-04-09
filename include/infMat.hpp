#pragma once
#include "compTask.hpp"
#include <Eigen/Core>
#include <ranges>
#include <utils/utils.hpp>

namespace views = std::views;
#define RANGE(n) views::iota(0, (int)n)
typedef Array<bool, Dynamic, 1> ArrayXb;
template <typename Singularity, bool SelfInfluence> // concept constrain
Eigen::ArrayXXd makeInfluenceMatrix(int m, int n, std::span<const ComputeTask> compTaskVec);
