//
//  Transform.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitching__Transform__
#define __UglyMan_Stitching__Transform__

#include "../Configure.h"
using namespace cv;

Mat getConditionerFromPts(const std::vector<Point2>& pts);

Mat getNormalize2DPts(const std::vector<Point2>& pts,
	std::vector<Point2>& newpts);

template <typename T>
T normalizeAngle(T x);

template <typename T>
Point_<T> applyTransform3x3(T x, T y, const Mat& matT);

template <typename T>
Point_<T> applyTransform2x3(T x, T y, const Mat& matT);

template <typename T>
Size_<T> normalizeVertices(std::vector<std::vector<Point_<T> > >& vertices);

template <typename T>
Rect_<T> getVerticesRects(const std::vector<Point_<T> >& vertices);

template <typename T>
std::vector<Rect_<T> > getVerticesRects(const std::vector<std::vector<Point_<T> > >& vertices);

template <typename T>
T getSubpix(const Mat& img, const Point2f& pt);

template <typename T, size_t n>
Vec<T, n> getSubpix(const Mat& img, const Point2f& pt);

template <typename T>
Vec<T, 3> getEulerZXYRadians(const Mat_<T>& rot_matrix);

template <typename T>
bool isEdgeIntersection(const Point_<T>& src_1, const Point_<T>& dst_1,
	const Point_<T>& src_2, const Point_<T>& dst_2,
	double* scale_1 = NULL, double* scale_2 = NULL);
template <typename T>
bool isRotationInTheRange(const T rotation, const T min_rotation, const T max_rotation);

//字符串分割
void SpiltString(std::string str, std::vector<std::string>& res, std::string delim);

//类型转换 std::string—int、double、float
template <class Type>
Type stringToNum(std::string str);

Vector3d trans2Vector(Point2f point);

#endif /* defined(__UglyMan_Stitching__Transform__) */
