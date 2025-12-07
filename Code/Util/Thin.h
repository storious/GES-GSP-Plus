#pragma once
#include "../Configure.h"
#include "opencv2/opencv.hpp"
#include "Feature/ImageData.h"
using namespace cv;

void thin(cv::Mat srcImage, cv::Mat& dst, double kernalSizeTimes);
void thinTest(cv::Mat srcImage, cv::Mat& dst, double kernalSizeTimes);
