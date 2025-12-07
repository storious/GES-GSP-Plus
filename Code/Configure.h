
// #define DP_NO_LOG
#define ASSESSMENT

#ifndef __UglyMan_Stitching__Configure__
#define __UglyMan_Stitching__Configure__

#include "Debugger/ErrorController.h"

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <set>
#include <algorithm>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/ximgproc.hpp"

#include <Eigen/SVD>
#include <Eigen/IterativeLinearSolvers>
#include <Eigen/Sparse>
#include <unsupported/Eigen/SparseExtra>
#include <opencv2/dnn.hpp>
#include <opencv2/dnn/layer.details.hpp>
using namespace Eigen;

using namespace cv::dnn;

/******************************/
/******* you may adjust *******/
/******************************/
enum TYPE
{
	GSP = 0,
	GES_GSP,
};

constexpr int RUN_TYPE = TYPE::GES_GSP; // 0:GSP 1:GES-GSP

const std::string TXT_NAME = "-STITCH-GRAPH.txt";

/*** data setting ***/
constexpr int GRID_SIZE = 40;
constexpr int DOWN_SAMPLE_IMAGE_SIZE = 800 * 600;
// Contour length/image shortest edge ratio
constexpr double CONTENT_LENGTH_THRESHOLD = 0.15;

// HED threshold
constexpr double HED_THRESHOLD = 0.5;
constexpr int threshold_value = 120;

// HED path
const std::string MODEL_CONFIG = R"(.\Code\model\deploy.prototxt)";
const std::string MODEL_BIN = R"(.\Code\model\hed_pretrained_bsds.caffemodel)";

/*** APAP ***/
constexpr double APAP_GAMMA = 0.0015;
constexpr double APAP_SIGMA = 8.5;

/*** matching method ***/
const std::string FEATURE_RATIO_TEST_THRESHOLD_STRING = "15e-1"; // 15*10^-1=1.5
const double FEATURE_RATIO_TEST_THRESHOLD = atof(FEATURE_RATIO_TEST_THRESHOLD_STRING.c_str());

/*** homography based ***/
constexpr double GLOBAL_HOMOGRAPHY_MAX_INLIERS_DIST = 5.;
constexpr double LOCAL_HOMOGRAPHY_MAX_INLIERS_DIST = 3.;
constexpr int LOCAL_HOMOGRAPHY_MIN_FEATURES_COUNT = 40;

/*** Deprecated vlfeat sift ***/
// INFO: use opencv implement SIFT and remove the vlfeat
constexpr int SIFT_FEATURE_COUNT = 0; // the maximum of the number of feature
constexpr int SIFT_LEVEL_COUNT = 3;
const int SIFT_MINIMUM_OCTAVE_INDEX = 0; // don't need this parament
constexpr double SIFT_PEAK_THRESH = 0.;
constexpr double SIFT_EDGE_THRESH = 20.;

/*** init feature ***/
constexpr double INLIER_TOLERANT_STD_DISTANCE = 4.25; /* mean + 4.25 * std */

/*** sRANSAC ***/
constexpr double GLOBAL_TRUE_PROBABILITY = 0.225;
constexpr double LOCAL_TRUE_PROBABILITY = 0.2;
constexpr double OPENCV_DEFAULT_CONFIDENCE = 0.995;

/*** sparse linear system ***/
constexpr double STRONG_CONSTRAINT = 1e4;

/*** bundle adjustment ***/
constexpr int CRITERIA_MAX_COUNT = 1000;
constexpr double CRITERIA_EPSILON = DBL_EPSILON;

/*** 2D Method ***/
constexpr double TOLERANT_ANGLE = 1.5;

/*** 3D Method ***/
constexpr double LAMBDA_GAMMA = 10;

/******************************/
/******************************/
/******************************/

/*** rotation method setting ***/
enum GLOBAL_ROTATION_METHODS
{
	GLOBAL_ROTATION_2D_METHOD = 0,
	GLOBAL_ROTATION_3D_METHOD,
	GLOBAL_ROTATION_METHODS_SIZE
};
const std::string GLOBAL_ROTATION_METHODS_NAME[GLOBAL_ROTATION_METHODS_SIZE] = {
	"[2D]", "[3D]"};

/* blending method setting */
enum BLENDING_METHODS
{
	BLEND_AVERAGE = 0,
	BLEND_LINEAR,
	BLEND_METHODS_SIZE
};
const std::string BLENDING_METHODS_NAME[BLEND_METHODS_SIZE] = {
	"[BLEND_AVERAGE]", "[BLEND_LINEAR]"};

/* type */
using FLOAT_TYPE = float;
using Size2 = cv::Size_<FLOAT_TYPE> ;
using Point2 = cv::Point_<FLOAT_TYPE> ;
using Rect2 = cv::Rect_<FLOAT_TYPE> ;

const int DIMENSION_2D = 2;
const int HOMOGRAPHY_VARIABLES_COUNT = 9;

/* AutoStitch */
enum AUTO_STITCH_WAVE_CORRECTS
{
	WAVE_X = 0,
	WAVE_H,
	WAVE_V
};
const AUTO_STITCH_WAVE_CORRECTS WAVE_CORRECT = WAVE_H;
const std::string AUTO_STITCH_WAVE_CORRECTS_NAME[] = {"", "[WAVE_H]", "[WAVE_V]"};

#endif /* defined(__UglyMan_Stitching__Configure__) */
