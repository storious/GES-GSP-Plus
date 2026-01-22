//
//  APAP_Stitching.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "APAP_Stitching.h"

void APAP_Stitching::apap_project(const vector<Point2> &_p_src,    //计算出每个网格点对应的单应性矩阵，并且将对应网格点变换到目标位置
								  const vector<Point2> &_p_dst,
								  const vector<Point2> &_src, 
								  vector<Point2> &_dst,   
								  vector<Mat> &_homographies)
{

	if (_p_src.empty() || _p_dst.empty() || _p_src.size() != _p_dst.size())
	{
		// 如果匹配点集为空，或者数量不匹配，直接返回。
		// 将 _dst 设置为 _src 的副本，_homographies 清空。
		_dst = _src;
		_homographies.clear();
		return;
	}
	vector<Point2> nf1, nf2, cf1, cf2;  //nf1，和nf2是归一化加上缩放后的点集，cf1，cf2是各自方向分别缩放后的点集

	Mat N1, N2, C1, C2;  //N1，N2是归一化加上缩放矩阵，C1，C2是各自方向分别缩放的矩阵

	N1 = getNormalize2DPts(_p_src, nf1);  //归一化加上缩放
	N2 = getNormalize2DPts(_p_dst, nf2);

	C1 = getConditionerFromPts(nf1);  //返回的是个矩阵
	C2 = getConditionerFromPts(nf2);
	cf1.reserve(nf1.size());
	cf2.reserve(nf2.size());

	for (int i = 0; i < nf1.size(); ++i)  //这里就是把点乘上矩阵，得到最终的点
	{
		cf1.emplace_back(nf1[i].x * C1.at<double>(0, 0) + C1.at<double>(0, 2),
						 nf1[i].y * C1.at<double>(1, 1) + C1.at<double>(1, 2));

		cf2.emplace_back(nf2[i].x * C2.at<double>(0, 0) + C2.at<double>(0, 2),
						 nf2[i].y * C2.at<double>(1, 1) + C2.at<double>(1, 2));
	}

	double sigma_inv_2 = 1. / (APAP_SIGMA * APAP_SIGMA), gamma = APAP_GAMMA;  //用来算“局部权重”的两个参数，应该是一个高斯权重的sigma，gamma是个下限，防止权重为0

	MatrixXd A = MatrixXd::Zero(cf1.size() * DIMENSION_2D,             //[Eigen]  匹配点数量 * 2 行，单应性未知数个数
								HOMOGRAPHY_VARIABLES_COUNT);           //初始化一个矩阵  (匹配点*2,9)

#ifndef DP_NO_LOG
	if (_dst.empty() == false)
	{
		_dst.clear();
		printError("F(apap_project) dst is not empty");
	}
	if (_homographies.empty() == false)
	{
		_homographies.clear();
		printError("F(apap_project) homographies is not empty");
	}
#endif
	_dst.reserve(_src.size());
	_homographies.reserve(_src.size());

	for (int i = 0; i < _src.size(); ++i)  //每个网格点和所有特征点计算单应性
	{

		for (int j = 0; j < _p_src.size(); ++j) //每个匹配点
		{

			Point2 d = _src[i] - _p_src[j];
			double www = MAX(gamma, exp(-sqrt(d.x * d.x + d.y * d.y) * sigma_inv_2));  //高斯权重  通过特征点距离网格距离来算的，看来确实是这样
			// x(h11​x+h12​y+h13​)−x′(h31​x+h32​y+h33​)=0
			// y(h21​x+h22​y+h23​)−y′(h31​x+h32​y+h33​)=0​
			A(2 * j, 0) = www * cf1[j].x;    // x' = (h11*x + h12*y + h13) / (h31*x + h32*y + h33) 
			A(2 * j, 1) = www * cf1[j].y;
			A(2 * j, 2) = www * 1;
			A(2 * j, 6) = www * -cf2[j].x * cf1[j].x;
			A(2 * j, 7) = www * -cf2[j].x * cf1[j].y;
			A(2 * j, 8) = www * -cf2[j].x;

			A(2 * j + 1, 3) = www * cf1[j].x;
			A(2 * j + 1, 4) = www * cf1[j].y;
			A(2 * j + 1, 5) = www * 1;
			A(2 * j + 1, 6) = www * -cf2[j].y * cf1[j].x;
			A(2 * j + 1, 7) = www * -cf2[j].y * cf1[j].y;
			A(2 * j + 1, 8) = www * -cf2[j].y;
		}

		JacobiSVD<MatrixXd, HouseholderQRPreconditioner> jacobi_svd(A, ComputeThinV); //用 SVD 从加权矩阵 A 里，找出对应最小奇异值的那个向量，当作局部 Homography
		MatrixXd V = jacobi_svd.matrixV();  //把 SVD 分解得到的矩阵 V 取出来。
		Mat H(3, 3, CV_64FC1);
		for (int j = 0; j < V.rows(); ++j)  //把 9个元素，重新组成 3x3 矩阵
		{
			H.at<double>(j / 3, j % 3) = V(j, V.rows() - 1);
		}
		H = C2.inv() * H * C1;  //反归一化，把“在归一化坐标系里算出来的 H”，变回原始像素坐标系
		H = N2.inv() * H * N1;  //因为之前的运算是在归一化的坐标系下进行所以算出来的矩阵H也是归一化坐标系下的，需要反归一化回去

		_dst.emplace_back(applyTransform3x3(_src[i].x, _src[i].y, H));   //用H把点变换过去
		_homographies.emplace_back(H); //保存这个点对应的单应性矩阵
	}
}
