#pragma once
#include <concepts>
#include <Eigen/Core>

template <std::floating_point T>
inline T DifferenceScalar(T x1, T x2, T f1, T f2){
	return (f2 - f1)/(x2 - x1);
}



template <bool ColWise=true, class EigenMatType>
EigenMatType centralDifference(const EigenMatType& x, const EigenMatType& f){
	if constexpr(!ColWise){
		auto out = centralDifference<true, EigenMatType>(x.transpose(), f.transpose());
		out.transposeInPlace();
		return out; 
	}	
	
	
	Eigen::ArrayXXd out(x.rows(), x.cols());
	if (x.rows() ==1){
		out << f;
		return x;
	}
	for (int i = 0; i < x.cols(); i++){
		out(0, i) = DifferenceScalar(x(0,i), x(1,i), f(0,i), f(1,i));
		for(int j = 1; j < x.rows()-1; j++){
			out(j, i) = DifferenceScalar(x(j-1,i), x(j+1,i), f(j-1,i), f(j+1,i));
		}
		auto n = x.rows();
		out(n-1, i) = DifferenceScalar(x(n-2,i), x(n-1,i), f(n-2,i), f(n-1,i));
	}
	return out;
}

