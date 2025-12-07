//
//  ImageData.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitching__ImageData__
#define __UglyMan_Stitching__ImageData__

#include <memory>
#include "../Util/Statistics.h"
#include "FeatureController.h"
#include "../Mesh/MeshGrid.h"
#include <opencv2\imgproc\types_c.h>
#include "../Util/EdgeDetection.h"
#include "../Util/Thin.h"
#include <opencv2/ximgproc.hpp>
using namespace cv::ximgproc;
using namespace cv;

class LineData {
public:
	LineData(const Point2& _a,
		const Point2& _b,
		const double _width,
		const double _length);
	Point2 data[2];
	double width, length;
private:
};

typedef const bool (LINES_FILTER_FUNC)(const double _data, \
	const Statistics& _statistics);

LINES_FILTER_FUNC LINES_FILTER_NONE;
LINES_FILTER_FUNC LINES_FILTER_WIDTH;
LINES_FILTER_FUNC LINES_FILTER_LENGTH;


class ImageData {
public:
	std::string file_name, file_extension;
	std::pair<double, double> LatALon; //image latitude and longitude
	const std::string* file_dir, * debug_dir;
	ImageData(const std::string& _file_dir,
		const std::string& _file_full_name,
		LINES_FILTER_FUNC* _width_filter,
		LINES_FILTER_FUNC* _length_filter,
		const std::string* _debug_dir = NULL);

	const Mat& getGreyImage() const;
	const std::vector<LineData>& getLines() const;
	const std::vector<Point2>& getFeaturePoints() const;
	const std::vector<FeatureDescriptor>& getFeatureDescriptors() const;
	const std::vector<std::vector<Point>> getContentSamplesPoint(std::vector<double>& weights) const;

	void clear();

	Mat img, rgba_img, alpha_mask;
	std::unique_ptr<Mesh2D> mesh_2d;

private:
	LINES_FILTER_FUNC* width_filter, * length_filter;

	mutable Mat grey_img;
	mutable std::vector<LineData> img_lines;
	mutable std::vector<Point2> feature_points;
	mutable std::vector<FeatureDescriptor> feature_descriptors;
	std::vector<Vec4f> findLine(Mat& gray) const;
};

bool sortForPoint(Point a, Point b);

bool equalForPoint(Point a, Point b);
void connectSmallLine(std::vector<std::vector<Point>> contours, std::vector<Vec4i> hierarchy, std::vector<std::vector<Point>>& contoursConnected);
std::vector<std::vector<Point>> connectCollineationLine(std::vector<std::vector<Point>>& contours, std::vector <double >& lengths_out, std::vector<std::vector<Point>>& static_sample, int image_width, int image_height);
std::pair<Point, Point> findStartEndPoint(std::vector<Point > contour, Vec4i fitline);
std::pair<Point, Point> findStartEndPoint(std::vector<Point > contour);

std::pair<Point, Point> findLineMinAndMax(std::vector<Point > contour);
bool isClose(std::pair<Point, Point> pair1, std::pair<Point, Point> pair2);
bool isExtend(std::pair<Point, Point> mmpair1, std::pair<Point, Point> mmpair2, std::pair<Point, Point> sepair1, std::pair<Point, Point> sepair2);
bool isParallax(std::pair<Point, Point> mmpair1, std::pair<Point, Point> mmpair2);
double PointDist(Point p1, Point p2);
double getLineWeight(std::vector<Point> line);

void transLines2Contours(std::vector<std::vector<Point>>& contours, std::vector<Vec4f> lines);
std::pair<double, double> transRectangular2Polar(Vec4f, int image_width, int image_height);
#endif /* defined(__UglyMan_Stitching__ImageData__) */
