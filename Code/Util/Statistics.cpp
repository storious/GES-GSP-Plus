//
//  Statistics.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "Statistics.h"
#include <algorithm>

template <typename T>
void Statistics::getMeanAndVariance(const std::vector<T>& _vec,
	double& _mean, double& _var) {
	_mean = 0, _var = 0;

	const int count = (int)_vec.size();
	for (int i = 0; i < count; ++i) {
		_mean += _vec[i];
	} _mean /= count;

	for (int i = 0; i < count; ++i) {
		_var += (_vec[i] * _vec[i]);
	}
	_var = (_var / count) - (_mean * _mean);

}

template <typename T>
void Statistics::getMeanAndSTD(const std::vector<T>& _vec,
	double& _mean, double& _std) {
	getMeanAndVariance(_vec, _mean, _std);
	_std = sqrt(_std);

}

template <typename T>
void Statistics::getMin(const std::vector<T>& _vec, double& _min) {
	_min = *std::min_element(_vec.begin(), _vec.end());
}

template <typename T>
void Statistics::getMax(const std::vector<T>& _vec, double& _max) {
	_max = *std::max_element(_vec.begin(), _vec.end());
}

template <typename T>
void Statistics::getMinAndMax(const std::vector<T>& _vec, double& _min, double& _max) {
	getMin<T>(_vec, _min);
	getMax<T>(_vec, _max);
}

template <typename T>
void Statistics::getMedianWithCopyData(const std::vector<T>& _vec, double& _median) {
	std::vector<T> v = _vec;
	getMedianWithoutCopyData(v, _median);
}

template <typename T>
void Statistics::getMedianWithoutCopyData(std::vector<T>& _vec, double& _median) {
	size_t n = _vec.size() / 2;
	std::nth_element(_vec.begin(), _vec.begin() + n, _vec.end());
	_median = _vec[n];
	if ((_vec.size() & 1) == 0) {
		std::nth_element(_vec.begin(), _vec.begin() + n - 1, _vec.end());
		_median = (_median + _vec[n - 1]) * 0.5;
	}
}

template <typename T>
Statistics::Statistics(const std::vector<T>& _vec) {
	getMeanAndVariance<T>(_vec, mean, var);
	std = sqrt(var);
	getMinAndMax<T>(_vec, min, max);
}

template void Statistics::getMeanAndVariance<   int>(const std::vector<   int>& _vec, double& _mean, double& _var);
template void Statistics::getMeanAndVariance< float>(const std::vector< float>& _vec, double& _mean, double& _var);
template void Statistics::getMeanAndVariance<double>(const std::vector<double>& _vec, double& _mean, double& _var);

template void Statistics::getMeanAndSTD<   int>(const std::vector<   int>& _vec, double& _mean, double& _std);
template void Statistics::getMeanAndSTD< float>(const std::vector< float>& _vec, double& _mean, double& _std);
template void Statistics::getMeanAndSTD<double>(const std::vector<double>& _vec, double& _mean, double& _std);

template void Statistics::getMin<   int>(const std::vector<   int>& _vec, double& _min);
template void Statistics::getMin< float>(const std::vector< float>& _vec, double& _min);
template void Statistics::getMin<double>(const std::vector<double>& _vec, double& _min);

template void Statistics::getMax<   int>(const std::vector<   int>& _vec, double& _max);
template void Statistics::getMax< float>(const std::vector< float>& _vec, double& _max);
template void Statistics::getMax<double>(const std::vector<double>& _vec, double& _max);

template void Statistics::getMinAndMax<   int>(const std::vector<   int>& _vec, double& _min, double& _max);
template void Statistics::getMinAndMax< float>(const std::vector< float>& _vec, double& _min, double& _max);
template void Statistics::getMinAndMax<double>(const std::vector<double>& _vec, double& _min, double& _max);

template void Statistics::getMedianWithCopyData<   int>(const std::vector<   int>& _vec, double& _median);
template void Statistics::getMedianWithCopyData< float>(const std::vector< float>& _vec, double& _median);
template void Statistics::getMedianWithCopyData<double>(const std::vector<double>& _vec, double& _median);

template void Statistics::getMedianWithoutCopyData<   int>(std::vector<   int>& _vec, double& _median);
template void Statistics::getMedianWithoutCopyData< float>(std::vector< float>& _vec, double& _median);
template void Statistics::getMedianWithoutCopyData<double>(std::vector<double>& _vec, double& _median);

template Statistics::Statistics(const std::vector<   int>& _vec);
template Statistics::Statistics(const std::vector< float>& _vec);
template Statistics::Statistics(const std::vector<double>& _vec);
