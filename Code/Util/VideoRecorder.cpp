#include <filesystem>
#include "./Configure.h"
namespace fs = std::filesystem;

class VideoRecorder {
private:
    cv::VideoWriter writer;
    bool isInitialized = false;
    std::string filename;
    double fps;

public:

    VideoRecorder(std::string name = "result.avi", double _fps = 25.0) 
        : filename(name), fps(_fps) {

        }

    void recordFrame(const cv::Mat& frame) {
        if(!isInitialized){// 只在第一帧初始化
            int fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
            writer.open(filename,fourcc,fps,frame.size(),true);
            isInitialized=true;        
            std::cout << "Video Recording Started: "<< filename << std::endl;
        }
        if (frame.empty()) return;
        if (!writer.isOpened()) {
            std::cerr << "Error: Could not open VideoWriter! Path: "
                << filename << std::endl;
            return;
        }
        fs::path p(filename);
        if (p.has_parent_path()) {
            fs::create_directories(p.parent_path());
        }

        writer<<(frame);
    }

    void finish() {
        if (writer.isOpened()) {
            writer.release();
        }
    }
    
    ~ VideoRecorder(void){
        cout<<"VideoRecorder is being deleted" <<endl;
    };


};