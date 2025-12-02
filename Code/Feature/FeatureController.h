#ifndef __Stitching__FeatureController__
#define __Stitching__FeatureController__

#include "../Configure.h"

const int SIFT_DESCRIPTOR_DIM = 128;

class FeatureDescriptor {
public:
	void addDescriptor(const Mat& _descriptor);
	static double getDistance(const FeatureDescriptor& _descriptor1,
		const FeatureDescriptor& _descriptor2,
		const double _threshold);
	vector<Mat> data;
};

class FeatureController {
public:
	static void detect(const Mat& _grey_img,
		vector<Point2>& _feature_points,
		vector<FeatureDescriptor>& _feature_descriptors);

private:
};

#endif /* defined(__Stitching__FeatureController__) */
