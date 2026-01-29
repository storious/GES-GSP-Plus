//
//  Blending.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "Blending.h"


Mat getMatOfLinearBlendWeight(const Mat& image) { //返回权重矩阵的，就是中间权重高周围底
	/*假设5*5的图像，则权重矩阵为：
	1  2  3  2  1
	2  4  6  4  2  
	3  6  9  6  3
	2  4  6  4  2
	1  2  3  2  1
	*/
	Mat result(image.size(), CV_32FC1, Scalar::all(0));
	for (int y = 0; y < result.rows; ++y) {
		int w_y = min(y + 1, result.rows - y);
		for (int x = 0; x < result.cols; ++x) {
			result.at<float>(y, x) = min(x + 1, result.cols - x) * w_y;
		}
	}
	return result;
}


vector<Mat> getMatsLinearBlendWeight(const vector<Mat>& images) { //返回权重矩阵的，就是中间权重高周围底
	vector<Mat> result;
	result.reserve(images.size());
	for (int i = 0; i < images.size(); ++i) {
		result.emplace_back(getMatOfLinearBlendWeight(images[i]));
	}
	return result;
}
//就是说变形后的图已经在画布上了，但是问题是不在同一个画布，现在要做的就是将这些不在同一个画布的图放到一起
Mat Blending(const vector<Mat>& images,
	const vector<Point2>& origins, //每张图的源点，位于画布上的位置
	const Size2 target_size,  //最小矩形大小
	const vector<Mat>& weight_mask, //权重矩阵
	const bool ignore_weight_mask) {

	cv::TickMeter tm;
tm.start();



	


	Mat result = Mat::zeros(round(max(target_size.height, 0.0f)), round(max(target_size.width, 0.0f)), CV_8UC4);  //round是四舍五入函数取整数 float转int

	vector<Rect2> rects;
	rects.reserve(origins.size());
	for (int i = 0; i < origins.size(); ++i) {
		rects.emplace_back(origins[i], images[i].size());  //x,y,width,height
	}
	for (int y = 0; y < result.rows; ++y) {  //几行几列,遍历画布上的每个像素
		for (int x = 0; x < result.cols; ++x) {

			Point2i p(x, y);

			Vec3f pixel_sum(0, 0, 0);

			float weight_sum = 0.f;
			//遍历每张图获取，处理该位置的像素值，对有贡献的全加起来，最后做一个归一化
			for (int i = 0; i < rects.size(); ++i) {  //遍历每张图
				Point2i pv(round(x - origins[i].x), round(y - origins[i].y)); //获取该图对应位置的像素值，相对于该图的源点坐标
				if (pv.x >= 0 && pv.x < images[i].cols &&
					pv.y >= 0 && pv.y < images[i].rows) {

					Vec4b v = images[i].at<Vec4b>(pv); 

					Vec3f value = Vec3f(v[0], v[1], v[2]);
					if (ignore_weight_mask) {
						if (v[3] > 127) {
							pixel_sum += value;
							weight_sum += 1.f;
						}
					}
					else {

						float weight = weight_mask[i].at<float>(pv);
						pixel_sum += weight * value;
						weight_sum += weight;
					}
				}
			}
			//归一化
			if (weight_sum) {
				pixel_sum /= weight_sum;

				result.at<Vec4b>(p) = Vec4b(round(pixel_sum[0]), round(pixel_sum[1]), round(pixel_sum[2]), 255);
			}
		}
	}
	
tm.stop();
std::cout << "Blending Time: " << tm.getTimeMilli() << " ms" << std::endl;
	return result;
}
