#include <iostream>
#include "./Stitching/NISwGSP_Stitching.h"
#include "./Util/VideoRecorder.cpp"
#include <chrono>
#include <optional>
#include <filesystem>
namespace fs= std::filesystem;

std::deque<FrameData> cam0_queue;
std::deque<FrameData> cam1_queue;

std::mutex mtx0, mtx1;
std::atomic<bool> running(true);

int have_read_mesh=0; //初始化为0
int GRID_SIZE_w = 40;
int GRID_SIZE_h = 40;
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

struct FrameData {
    cv::Mat img;
    int64_t timestamp;
};

void captureCamera(
    int cam_id,
    std::deque<FrameData>& queue,
    std::mutex& mtx
) {
    cv::VideoCapture cap(cam_id);
    if (!cap.isOpened()) {
        std::cerr << "Camera " << cam_id << " open failed\n";
        return;
    }

    cv::Mat frame;
    while (running) {
        cap >> frame;
        if (frame.empty()) continue;

        FrameData data;
        data.img = frame.clone();
        data.timestamp = cv::getTickCount();

        {
            std::lock_guard<std::mutex> lock(mtx);
            queue.push_back(data);
            if (queue.size() > 3) queue.pop_front(); // 防止堆积
        }
    }
}


int main(int argc, const char *argv[])
{

	Eigen::initParallel(); /* remember to turn off "Hardware Multi-Threading */
	CV_DNN_REGISTER_LAYER_CLASS(Crop, CropLayer);
	std::cout << "nThreads = " << Eigen::nbThreads() << endl;
	std::cout << "[#Images : " << argc - 1 << "]" << endl;


	auto start = std::chrono::high_resolution_clock::now();
	//time_t start = clock();
	// TimeCalculator timer;
	//cout<<"i=0"<<argv[0]<<endl; //显示的ges_stitching.exe绝对地址
	for (int i = 1; i < argc; ++i) {
		cout << "i = " << i << ", [Images : " << argv[i] << "]" << endl;    //part1/case1
		std::optional<MultiImages> multi_images;   // ① 只定义，不构造
		std::optional<NISwGSP_Stitching> niswgsp;  
		std::optional<VideoRecorder> video_recorder;  
		

		//获取图像名字用于构造文件夹
		vector<string> cam0_imgs,cam1_imgs;
		getImages("./input-data/" + string(argv[i]) + "/video1/", cam0_imgs);
		getImages("./input-data/" + string(argv[i]) + "/video2/", cam1_imgs);

		// 获取路径信息
		char *path = strdup(argv[i]);  //part1/case1  ->变成part1
		char *token1 = strtok(path, "/");  //part1
		char *token2 = strtok(NULL, "/");  //case1
		String file_dir="./input-data/" + string(argv[i]);   //./input-data/part1/case1
		string outputVideoPath="./input-data/0_results/"+string(token1)+"_"+string(token2)+"-result/"+"live_stitching.avi";
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
		for(int cnt=0;cnt<image_count/10;cnt++)
		{
			

			std::error_code ec;
			String file_dir="./input-data/" + string(argv[i]) ;   //./input-data/part1/case1
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
			String sitich_graph_file= string(token1) +"_"+ string(token2)+TXT_NAME; //part1_case1-STITCH-GRAPH.txt
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
	return 0;
}
