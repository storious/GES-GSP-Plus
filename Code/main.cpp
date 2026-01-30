#include <iostream>
#include "./Stitching/NISwGSP_Stitching.h"
#include "./Util/VideoRecorder.cpp"
#include <chrono>
#include <optional>
#include <filesystem>
namespace fs= std::filesystem;


struct CameraFrame {
    cv::Mat img1;
    cv::Mat img2;
};
std::queue<CameraFrame> frameBuffer; // 存放待处理帧的队列
std::mutex mtx;                      // 保证队列线程安全
bool isRunning = true;               // 控制程序运行
std::atomic<bool> keepDisplaying{true};


int have_read_mesh=0; //初始化为0
size_t countImageFiles(const std::string& dir)
{
    const std::vector<std::string> image_formats = {
        ".bmp",".dib",".jpeg",".jpg",".jpe",".png",
        ".pbm",".pgm",".ppm",".sr",".ras",".tiff",".tif"
    };

    size_t count = 0;

    for (const auto& entry : std::filesystem::directory_iterator(dir)) {
        if (!entry.is_regular_file()) continue;

        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

        if (std::find(image_formats.begin(), image_formats.end(), ext)
            != image_formats.end())
        {
            ++count;
        }
    }
    return count;
}
void getImages(const std::string& dir, std::vector<std::string>& images)
{
    for (auto& p : fs::directory_iterator(dir))
    {
        if (p.is_regular_file())
            images.push_back(p.path().string());
    }
    std::sort(images.begin(), images.end()); // ⚠️ 很关键
}
void videoStitch(const string &argv){
		
		std::optional<MultiImages> multi_images;   // ① 只定义，不构造
		std::optional<NISwGSP_Stitching> niswgsp;  
		std::optional<VideoRecorder> video_recorder;  
		

		//获取图像名字用于构造文件夹
		vector<string> cam0_imgs,cam1_imgs;
		getImages("./input-data/" + argv + "/video1/", cam0_imgs);
		getImages("./input-data/" + argv + "/video2/", cam1_imgs);

		// 获取路径信息
		std::string token1, token2;

		auto pos = argv.find('/');
		if (pos != std::string::npos) {
			token1 = argv.substr(0, pos);        // part1
			token2 = argv.substr(pos + 1);       // case1
		}

		String file_dir="./input-data/" + argv;   //./input-data/part1/case1
		string outputVideoPath="./input-data/0_results/"+token1+"_"+token2+"-result/"+"live_stitching.avi";
		int image_count = countImageFiles(file_dir+ "/video1/");  
		
		video_recorder.emplace(outputVideoPath,25);

		Mat blend_linear; //Matrix（矩阵）opencv的
		vector<vector<Point2> > original_vertices;
		/* 
			通过提取video1文件夹下的图片数量，来决定读取多少张图片进行拼接，
			获取token2文件夹下的两个文件夹中的图片，
			根据cnt来读取图片将两张图片组成一个文件夹file_dir，
			然后传给MultiImages去读取
		*/
		for(int cnt =0 ; cnt<image_count;cnt++) {
			std::error_code ec;
			String file_dir="./input-data/" + argv ;   //./input-data/part1/case1
			fs::create_directories(file_dir+ "/" +to_string(cnt), ec);//./input-data/part1/case1/0  ./input-data/part1/case1/1

			// 输出结果检测
			if (ec) {
				// 场景 A: 发生了真正的错误（如权限不足、路径非法）
				cout << "创建失败！" << endl;
				cout << "传入的路径: [" << file_dir+ "/" +to_string(cnt) << "]" << endl;
				cout << "错误原因: " << ec.message() << endl;
			} else {
				// 场景 B & C: 路径现在是可用的
				cout << "---------------------路径检查通过。---------------------" << endl;
				cout << "传入的路径: [" << file_dir+ "/" +to_string(cnt) << "]" << endl;
			}

			fs::copy(cam0_imgs[cnt], file_dir + "/" +to_string(cnt) + "/0.jpg", fs::copy_options::overwrite_existing);
			fs::copy(cam1_imgs[cnt], file_dir + "/" +to_string(cnt) + "/1.jpg", fs::copy_options::overwrite_existing);
			String sitich_graph_file= token1 +"_"+ token2+TXT_NAME; //part1_case1-STITCH-GRAPH.txt
			fs::copy_file(file_dir +"/"+sitich_graph_file, file_dir + "/" +to_string(cnt)+"/"+sitich_graph_file, fs::copy_options::overwrite_existing); 
			
			multi_images.emplace(
				file_dir+ "/" +to_string(cnt), LINES_FILTER_WIDTH, LINES_FILTER_LENGTH);  //./input-data/part1/case1/0
			/* 2D */
			niswgsp.emplace(*multi_images); //就是给NISwGSP和MeshOptimization里面给了MultiImage的对象可以调用
			niswgsp->setWeightToAlignmentTerm(1);
			niswgsp->setWeightToLocalSimilarityTerm(0.75);
			niswgsp->setWeightToGlobalSimilarityTerm(6, 20, GLOBAL_ROTATION_2D_METHOD);  //beta  gamma 
			niswgsp->setWeightToContentPreservingTerm(1.5);
			switch (RUN_TYPE)
			{
			case TYPE::GES_GSP:
				blend_linear = niswgsp->solve_content(BLEND_LINEAR, original_vertices);
				break;
			case TYPE::GSP:
				blend_linear = niswgsp->solve(BLEND_LINEAR, original_vertices);
				break;
			}


			video_recorder->recordFrame(blend_linear);

			niswgsp->writeImage(blend_linear, cnt, BLENDING_METHODS_NAME[BLEND_LINEAR]);
			if(RUN_WAY==0){
			niswgsp->assessment(original_vertices);
			}
			have_read_mesh=0;
		}
		video_recorder->finish();
}
void captureThread() {
    cv::VideoCapture cap1(1); // 摄像头0
    cv::VideoCapture cap2(2); // 摄像头1
    while (isRunning) {
        CameraFrame cf;  
		//同时读取
		if (cap2.read(cf.img1) && cap1.read(cf.img2)) {
			std::lock_guard<std::mutex> lock(mtx);
			// 如果处理太慢，清空旧帧，只保留最新的一帧
			if (frameBuffer.size() > 1) {
				frameBuffer.pop();
			}
			frameBuffer.push(cf);
			
			// 检查按键（空格键）
			
		}
		//std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
	cv::destroyWindow("Camera 1");
	cv::destroyWindow("Camera 2");

    cap1.release();
    cap2.release();

}


void displayCamera1(){
	 cv::VideoCapture cap1(1);  // 摄像头0
    if (!cap1.isOpened()) {
        std::cout << "无法打开摄像头1!" << std::endl;
        return;
    }
    
    cv::namedWindow("Camera 1", cv::WINDOW_AUTOSIZE);
    
    cv::Mat frame1;
    while (keepDisplaying) {
        cap1 >> frame1;
        if (frame1.empty()) break;
        
        cv::imshow("Camera 1", frame1);
        
        // 按ESC键退出
        if (cv::waitKey(1) == 27) {
            keepDisplaying = false;
            break;
        }
        
        // 稍微休眠，避免占用太多CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    
    cv::destroyWindow("Camera 1");
    cap1.release();
}

// 显示摄像头2的线程函数
void displayCamera2() {
    cv::VideoCapture cap2(2);  // 摄像头1
    if (!cap2.isOpened()) {
        std::cout << "无法打开摄像头2!" << std::endl;
        return;
    }
    
    cv::namedWindow("Camera 2", cv::WINDOW_AUTOSIZE);
    
    cv::Mat frame2;
    while (keepDisplaying) {
        cap2 >> frame2;
        if (frame2.empty()) break;
        
        cv::imshow("Camera 2", frame2);
        
        // 按ESC键退出
        if (cv::waitKey(1) == 27) {
            keepDisplaying = false;
            break;
        }
        
        // 稍微休眠
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    
    cv::destroyWindow("Camera 2");
    cap2.release();
}


int main(int argc, const char *argv[])
{




	std::thread t1(captureThread);

	Eigen::initParallel(); /* remember to turn off "Hardware Multi-Threading */
	CV_DNN_REGISTER_LAYER_CLASS(Crop, CropLayer);
	std::cout << "nThreads = " << Eigen::nbThreads() << endl;
	std::cout << "[#Images : " << argc - 1 << "]" << endl;

	// 创建窗口（可选，设置 WINDOW_NORMAL 可以手动调整大小）


	cv::namedWindow("Camera 1", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Camera 2", cv::WINDOW_AUTOSIZE);
	for (int i = 1; i < argc; ++i) {
		cout << "i = " << i << ", [Images : " << argv[i] << "]" << endl;    //part1/case1
		while(isRunning){

			int key = cv::waitKey(1);
			if (key == 27) isRunning = false; // 按 ESC 退出

			CameraFrame currentTask;
			bool hasData = false;

			

			// 2. 检查队列里是否有新图片
			{
				std::lock_guard<std::mutex> lock(mtx);
				if (!frameBuffer.empty()) {
					currentTask = frameBuffer.front();
					frameBuffer.pop();
					hasData = true;
					if (key == 32) {  // 空格键 ASCII码32
						// 显示当前捕获的图像
					}
				}
			}

			// 处理数据
			if (hasData) {
				cv::imshow("Camera 1", currentTask.img1);  // 假设img1是摄像头1
				cv::imshow("Camera 2", currentTask.img2);  // 假设img2是摄像头2
				if(key==32){
					cv::imshow("Camera 1 preview",currentTask.img1);
					cv::imshow("Camera 2 preview",currentTask.img2);
					std::cout<<"捕获快照"<<endl;
				}
			}
		//videoStitch(argv[i]);
		}

		
	}
	t1.join();

	return 0;
}
