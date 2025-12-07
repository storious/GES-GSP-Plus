#ifndef __Stitching__FeatureController__
#define __Stitching__FeatureController__

#include "../Configure.h"

using Point2 = cv::Point_<float>;

const int SIFT_DESCRIPTOR_DIM = 128;

class FeatureDescriptor {
public:
	void addDescriptor(const cv::Mat& _descriptor);
	static double getDistance(const FeatureDescriptor& _descriptor1,
		const FeatureDescriptor& _descriptor2,
		const double _threshold);
	std::vector<cv::Mat> data;
};

class FeatureController {
public:
	static void detect(const cv::Mat& _grey_img,
		std::vector<Point2>& _feature_points,
		std::vector<FeatureDescriptor>& _feature_descriptors);

private:
};

#endif /* defined(__Stitching__FeatureController__) */
