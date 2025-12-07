
//
//  Test code. For reference only,there might be a lot errors.
//
#pragma once
#include "../Configure.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../Util/Thin.h"
#include "../Util/EdgeDetection.h"
#include "../Util/Transform.h"
#include <math.h>
#include <opencv2/ximgproc.hpp>
#include <io.h>

using namespace cv;
using namespace cv::ximgproc;

const double MESH_GRID_SIZE = 40;

bool sortForPoint1(Point a, Point b);
bool equalForPoint1(Point a, Point b);
void connectSmallLine1(std::vector<std::vector<Point>> contours, std::vector<Vec4i> hierarchy, std::vector<std::vector<Point>>& contoursConnected);
std::pair<Point, Point> findStartEndPoint1(std::vector<Point > contour);
std::pair<Point, Point> findStartEndPoint1(std::vector<Point > contour, Vec4i fitline);
std::pair<Point, Point> findLineMinAndMax1(std::vector<Point > contour);
std::vector<std::vector<Point>> connectCollineationLine1(std::vector<std::vector<Point>>& input, std::vector <double >& lengths_out, std::vector<std::vector<Point>>& static_sample_out, int image_width, int image_height);
bool isClose1(std::pair<Point, Point> pair1, std::pair<Point, Point> pair2);
double PointDist1(Point p1, Point p2);
void breakCurveInflectionPoint(std::vector<std::vector<Point>>& curves);
std::vector<double> getCurvature(std::vector<cv::Point> const& vecContourPoints, int step);
double getLineWeight1(std::vector<Point> line);
std::vector<Vec4f> delectParallaxAndNear(std::vector<Vec4f> lines);
std::vector<Vec4f> findLine1(Mat& gray);
bool isParallax1(std::pair<Point, Point> mmpair1, std::pair<Point, Point> mmpair2);
bool isExtend1(std::pair<Point, Point> mmpair1, std::pair<Point, Point> mmpair2, std::pair<Point, Point> sepair1, std::pair<Point, Point> sepair2);
bool isClose1(std::pair<Point, Point> stdpair1, std::pair<Point, Point> pair2);

struct ContoursConnectionObj {
	std::pair<Point, Point> SEPoint;
	double k;

};

namespace TestSP {
	int testTriangle(double x1, double y1, double x2, double y2, double x3, double y3);
	void testContours();
	void testVector();
	void testNormalWayExtract();
	void testLineProcess();

}
