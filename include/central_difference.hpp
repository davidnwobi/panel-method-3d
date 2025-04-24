#pragma once
#include <concepts>
#include <Eigen/Core>
#include "utils/utils.hpp"

template <std::floating_point T>
inline T DifferenceScalar(T x1, T x2, T f1, T f2){
	return (f2 - f1)/(x2 - x1);
}

template <std::floating_point T>
inline T CentralDifferenceScalar(T xm1, T x0, T xp1, T fm1, T f0, T fp1){
	T h1 = x0 - xm1;
	T h2 = xp1 - x0;
	return ((-f0+fp1)*h1*h1+(f0-fm1)* h2*h2)/(h1*h2*(h1+h2));
}
template <std::floating_point T>
inline T ForwardDifferenceScalar(T x0, T xp1, T xp2, T f0, T fp1, T fp2){
	T h1 = xp1 - x0;
	T h2 = xp2 - xp1;
	
	return  (-f0 + fp1)/h1 + (fp1 - fp2)/h2 + (-f0 + fp2)/(h1 + h2);
}

template <std::floating_point T>
inline T BackwardsDifferenceScalar(T xp2, T xp1, T x0, T fp2, T fp1, T f0){
	T h1 = x0 - xp1;
	T h2 = xp1 - xp2;
	return  (fp2* h1*h1 - fp1 * (h1 + h2) * (h1 + h2) + f0*h2*(2*h1 + h2))/(h1*h2*(h1 + h2));
}

// template <bool ColWise=true, class Derived1, class Derived2>
// Eigen::ArrayXXf centralDifference(const Eigen::DenseBase<Derived1>& x, const Eigen::DenseBase<Derived2>&  f){
// 	if constexpr(!ColWise){
// 		Eigen::ArrayXXf out = centralDifference<true>(x.transpose(), f.transpose());
// 		out.transposeInPlace();
// 		return out; 
// 	}	
// 	
// 	
// 	Eigen::ArrayXXf out(x.rows(), x.cols());
// 	if (x.rows() ==1){
// 		out << f;
// 		return x;
// 	}
// 	for (int i = 0; i < x.cols(); i++){
// 		out(0, i) = DifferenceScalar(x(0,i), x(1,i), f(0,i), f(1,i));
// 		for(int j = 1; j < x.rows()-1; j++){
// 			out(j, i) = DifferenceScalar(x(j-1,i), x(j+1,i), f(j-1,i), f(j+1,i));
// 		}
// 		auto n = x.rows();
// 		out(n-1, i) = DifferenceScalar(x(n-2,i), x(n-1,i), f(n-2,i), f(n-1,i));
// 	}
// 	return out;
// }

template <bool ColWise=true, class Derived1, class Derived2>
Eigen::ArrayXXf centralDifference(const Eigen::DenseBase<Derived1>& x, const Eigen::DenseBase<Derived2>&  f){
	if constexpr(!ColWise){
		Eigen::ArrayXXf out = centralDifference<true>(x.transpose(), f.transpose());
		out.transposeInPlace();
		return out; 
	}	
	
	
	Eigen::ArrayXXf out(x.rows(), x.cols());
	if (x.rows() ==1){
		out << f;
		return x;
	}
	for (int i = 0; i < x.cols(); i++){
		out(0, i) = ForwardDifferenceScalar(x(0,i), x(1,i), x(2,i), f(0,i), f(1,i), f(2,i));
		for(int j = 1; j < x.rows()-1; j++){
			out(j, i) = CentralDifferenceScalar(x(j-1,i), x(j,i), x(j+1,i), f(j-1,i), f(j,i), f(j+1,i));
		}
		auto n = x.rows();
		out(n-1, i) = BackwardsDifferenceScalar(x(n-3,i), x(n-2,i), x(n-1,i), f(n-3,i), f(n-2,i), f(n-1,i));
	}
	return out;
}

