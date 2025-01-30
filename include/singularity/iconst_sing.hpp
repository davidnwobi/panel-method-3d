#pragma once
#include <Eigen/Core>
#include <numbers>
#include <iterator>
#include <type_traits>
#include "utils/utils.hpp"

template<template<bool> typename Derived, bool SelfInfluence = true>
struct IConstant3dSingularity{
	static Eigen:: ArrayXd calcInfluence(const ComputeTask& compTask) {
		return Derived<SelfInfluence>::calcInfluenceImpl(compTask);
	}
};

