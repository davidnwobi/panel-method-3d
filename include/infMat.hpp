#pragma once
#include "compTask.hpp"
#include <Eigen/Core>
#include <ranges>
#include <utils/utils.hpp>
#include "concepts.hpp"

namespace views = std::views;
typedef Array<bool, Dynamic, 1> ArrayXb;
template <typename Singularity, bool SelfInfluence> // concept constrain
Eigen::ArrayXXf makeInfluenceMatrix(int m, int n, std::span<const ComputeTask> compTaskVec);
