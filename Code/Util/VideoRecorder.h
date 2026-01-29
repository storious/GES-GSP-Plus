#include <iostream>
#include "./Configure.h"

class VideoRecorder {
private:
    cv::VideoWriter writer;
    bool isInitialized = false;
    std::string filename;
    double fps;

public:
    // 构造函数：设定文件名和帧率
    VideoRecorder(std::string name = "live_stitching.avi", double _fps = 25.0);

    // 核心函数：传入每一帧图像
    void recordFrame(const cv::Mat& frame);

    // 释放资源
    void finish() ;
};