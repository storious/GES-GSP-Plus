#include "FeatureController.h"
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
using namespace cv;

void FeatureDescriptor::addDescriptor(const Mat &_descriptor)
{
    data.emplace_back(_descriptor);
}

double FeatureDescriptor::getDistance(const FeatureDescriptor &_descriptor1,const FeatureDescriptor &_descriptor2, double _threshold) //计算两个描述子的距离，返回最小距离
{
    if (_descriptor1.data.empty() || _descriptor2.data.empty()) {
        return FLT_MAX;
    }

    double result = FLT_MAX;
    for (size_t i = 0; i < _descriptor1.data.size(); ++i)  //[fixMe]
    {
        if (_descriptor1.data[i].empty()) continue;
        
        for (size_t j = 0; j < _descriptor2.data.size(); ++j)
        {
            if (_descriptor2.data[j].empty()) continue;
            
            double distance = norm(_descriptor1.data[i], _descriptor2.data[j], NORM_L2); //NORM_L2 用的是L2范数就是欧氏距离
            result = min(result, distance);
            if (result >= _threshold)
                break;
        }
    }
    return result;
}


void FeatureController::detect(const Mat &_grey_img,
                                      vector<Point2f> &_feature_points,
                                      vector<FeatureDescriptor> &_feature_descriptors)  //获取特征点和描述子
{
    _feature_points.clear();
    _feature_descriptors.clear();

    // input the constant parament
    Ptr<SIFT> sift = SIFT::create(
        SIFT_FEATURE_COUNT, // nfeatures  要检测的最大特征点数量
        SIFT_LEVEL_COUNT,        // nOctaveLayers 每个金字塔组中的层数
        SIFT_PEAK_THRESH,        // contrastThreshold 对比度阈值（过滤弱特征点）
        SIFT_EDGE_THRESH        // edgeThreshold 过滤边缘响应强的点
    );
    vector<KeyPoint> keypoints;
    Mat descriptors;

    sift->detectAndCompute(_grey_img, noArray(), keypoints, descriptors);

    for (size_t i = 0; i < keypoints.size(); ++i) {
    _feature_points.emplace_back(keypoints[i].pt.x, keypoints[i].pt.y);
    
    FeatureDescriptor descriptor;
    
    if (!descriptors.empty() && i < descriptors.rows) {
        descriptor.addDescriptor(descriptors.row(i));
        _feature_descriptors.emplace_back(descriptor);
    }
}
}
