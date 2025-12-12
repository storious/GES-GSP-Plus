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
	Mat img = src.clone();
	Size reso(img.cols,img.rows);
	Mat blob = cv::dnn::blobFromImage(img, threshold, reso, false, false);

	//Set your HED files path.
	string modelCfg = R"(.\code\model\deploy.prototxt)";
	string modelBin = R"(.\code\model\hed_pretrained_bsds.caffemodel)";
	Net net = cv::dnn::readNet(modelCfg, modelBin);
	if (net.empty()) {
		std::cout << "net empty" << std::endl;
	}
	net.setInput(blob);
	Mat out = net.forward(); //一般输出是1*1*H*W的4维矩阵
	resize(out.reshape(1, reso.height), out, img.size()); //返回到2维，大小和输入图像相同

	out.convertTo(dst, CV_8UC1, 255);


}
