#pragma once
#include "EdgeDetection.h"
/// <summary>
/// HED detector
/// </summary>
/// <param name="src"></param>
/// <param name="dst"></param>
/// <param name="threshold"></param>
void edgeDetection(cv::Mat& src, cv::Mat& dst, double threshold)
{
	cv::Mat img = src.clone();
	cv::Size reso(img.rows, img.cols);
	cv::Mat blob = cv::dnn::blobFromImage(img, threshold, reso, false, false);

	//Set your HED files path.
	std::string modelCfg = MODEL_CONFIG; // R"(path\to\GES-GSP-Stitching-Plus\Code\model\deploy.prototxt)";
	std::string modelBin = MODEL_BIN;// R"(path\to\GES-GSP-Stitching-Plus\Code\model\hed_pretrained_bsds.caffemodel)";

	Net net = cv::dnn::readNet(modelCfg, modelBin);
	if (net.empty()) {
		std::cout << "net empty" << std::endl;
	}
	net.setInput(blob);
	cv::Mat out = net.forward();
	resize(out.reshape(1, reso.height), out, img.size());

	out.convertTo(dst, CV_8UC1, 255);
}
