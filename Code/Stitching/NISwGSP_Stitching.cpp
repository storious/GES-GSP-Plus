

#include "NISwGSP_Stitching.h"

NISwGSP_Stitching::NISwGSP_Stitching(const MultiImages &_multi_images) : MeshOptimization(_multi_images)
{
}

void NISwGSP_Stitching::setWeightToAlignmentTerm(const double _weight)
{
	MeshOptimization::setWeightToAlignmentTerm(_weight);
}

void NISwGSP_Stitching::setWeightToLocalSimilarityTerm(const double _weight)
{
	MeshOptimization::setWeightToLocalSimilarityTerm(_weight);
}

void NISwGSP_Stitching::setWeightToGlobalSimilarityTerm(const double _weight_beta,
														const double _weight_gamma,
														const enum GLOBAL_ROTATION_METHODS _global_rotation_method)
{
	MeshOptimization::setWeightToGlobalSimilarityTerm(_weight_beta, _weight_gamma, _global_rotation_method);
}

void NISwGSP_Stitching::setWeightToContentPreservingTerm(const double _weight)
{
	MeshOptimization::setWeightToContentPreservingTerm(_weight);
}

bool createYamlFile(const std::string& folderPath,
                   const std::string& fileName,
                   const cv::Mat& data = cv::Mat()) {
	namespace fs=std::filesystem;
    try {
        // 1. 确保文件夹存在
        if (!fs::exists(folderPath)) {
            fs::create_directories(folderPath);
        }
        
        // 2. 构建完整路径
        std::string fullPath = (fs::path(folderPath) / fileName).string();
        
        // 3. 创建OpenCV YAML文件
        cv::FileStorage fs(fullPath, cv::FileStorage::WRITE);
        if (!fs.isOpened()) {
            std::cerr << "无法创建YAML文件: " << fullPath << std::endl;
            return false;
        }
        
        // 4. 写入数据
        fs << "created_time" << cv::getTickCount();
        
        if (!data.empty()) {
            fs << "data_matrix" << data;
        }
        
        // 示例数据
        std::vector<int> sampleData = {1, 2, 3, 4, 5};
        fs << "sample_array" << sampleData;
        
        fs.release();
        std::cout << "YAML文件创建成功: " << fullPath << std::endl;
        return true;
        
    } catch (const fs::filesystem_error& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return false;
    }
}

Mat NISwGSP_Stitching::solve(const BLENDING_METHODS &_blend_method, vector<vector<Point2>> &original_vertices)
{
	const MultiImages &multi_images = getMultiImages();	
	vector<Triplet<double>> triplets;
	vector<pair<int, double>> b_vector;

	if(RUN_WAY==0){
	reserveData(triplets, b_vector, DIMENSION_2D);

	triplets.emplace_back(0, 0, STRONG_CONSTRAINT);
	triplets.emplace_back(1, 1, STRONG_CONSTRAINT);
	b_vector.emplace_back(0, STRONG_CONSTRAINT);
	b_vector.emplace_back(1, STRONG_CONSTRAINT);

	prepareAlignmentTerm(triplets);
	prepareSimilarityTerm(triplets, b_vector);

	original_vertices = getImageVerticesBySolving(triplets, b_vector);  //得到的是变化后的每个图像的网格顶点坐标
	}
	else if(RUN_WAY==1){
		namespace file_s=std::filesystem;
		string filename="./"+multi_images.parameter.file_name+"/mesh/mesh_vertices.yml";
		if(!original_vertices.empty() && !original_vertices[0].empty()){
			std::cout << "数据检测通过，点数为: " << original_vertices[0].size() << std::endl;
			//read
			have_read_mesh=1;
		}else{
			reserveData(triplets, b_vector, DIMENSION_2D);   //这里涉及到后面一个一个函数		if (feature_matches[m1][m2].size() == 0) 越界 因为没有定义 找一下
			triplets.emplace_back(0, 0, STRONG_CONSTRAINT);
			triplets.emplace_back(1, 1, STRONG_CONSTRAINT);
			b_vector.emplace_back(0, STRONG_CONSTRAINT);
			b_vector.emplace_back(1, STRONG_CONSTRAINT);

			prepareAlignmentTerm(triplets);
			prepareSimilarityTerm(triplets, b_vector);
			original_vertices = getImageVerticesBySolving(triplets, b_vector);
		// //[add]
		// namespace file_s=std::filesystem;
		// string filename="./"+multi_images.parameter.file_name+"/mesh/mesh_vertices.yml";
		// if(file_s::exists(filename)){
		// 	//read
		// 	have_read_mesh=1;

		// 	int num_images = 0;
		// 	int num_vertices = 0;
		// 	original_vertices.resize(multi_images.images_data.size());
		// 	FileStorage fs(filename, FileStorage::READ);
		// 	fs["num_images"] >> num_images;
		// 	fs["num_vertices_per_image"] >> num_vertices;
		// 	for (int i = 0; i < num_images; ++i)
		// 	{
		// 		original_vertices[i].resize(multi_images.images_data[i].mesh_2d->getVertices().size());
		// 		fs["image_" + to_string(i)] >> original_vertices[i];

		// 		// ===== 强校验（非常推荐）=====
		// 		CV_Assert(original_vertices[i].size() == num_vertices);
		// 	}
		// 	fs.release();
		// }else{
		// 	reserveData(triplets, b_vector, DIMENSION_2D);   //这里涉及到后面一个一个函数		if (feature_matches[m1][m2].size() == 0) 越界 因为没有定义 找一下
		// 	triplets.emplace_back(0, 0, STRONG_CONSTRAINT);
		// 	triplets.emplace_back(1, 1, STRONG_CONSTRAINT);
		// 	b_vector.emplace_back(0, STRONG_CONSTRAINT);
		// 	b_vector.emplace_back(1, STRONG_CONSTRAINT);

		// 	prepareAlignmentTerm(triplets);
		// 	prepareSimilarityTerm(triplets, b_vector);
		// 	original_vertices = getImageVerticesBySolving(triplets, b_vector);
		// 	createYamlFile("./"+multi_images.parameter.file_name+"/mesh","mesh_vertices.yml");
		// 	//write
		// 	//============== [add]
		// 	// ===== meta 信息（防止以后对不上）=====[add]
		// 	FileStorage fs(filename, FileStorage::WRITE);
		// 	fs << "num_images" << (int)original_vertices.size();
		// 	fs << "num_vertices_per_image" << (int)original_vertices[0].size();

		// 	// ===== 每张图的 mesh =====
		// 	for (int i = 0; i < original_vertices.size(); ++i)
		// 	{
		// 		fs << ("image_" + to_string(i)) << original_vertices[i];
		// 	}
		// 	fs.release();
		// 	//================
			
		}
	}
	Size2 target_size = normalizeVertices(original_vertices);

	Mat result = multi_images.textureMapping(original_vertices, target_size, _blend_method);

#ifndef DP_NO_LOG
	multi_images.writeResultWithMesh(result, original_vertices, "-[NISwGSP]" + GLOBAL_ROTATION_METHODS_NAME[getGlobalRotationMethod()] + BLENDING_METHODS_NAME[_blend_method] + "[Mesh]", false);
	multi_images.writeResultWithMesh(result, original_vertices, "-[NISwGSP]" + GLOBAL_ROTATION_METHODS_NAME[getGlobalRotationMethod()] + BLENDING_METHODS_NAME[_blend_method] + "[Border]", true);
#endif
	return result;
}


Mat NISwGSP_Stitching::solve_content(const BLENDING_METHODS &_blend_method, vector<vector<Point2>> &original_vertices)
{
	const MultiImages &multi_images = getMultiImages();

	vector<Triplet<double>> triplets;  //Eigen 的三元组
	vector<pair<int, double>> b_vector;

	reserveData_content(triplets, b_vector, DIMENSION_2D); //2D就是2  这里计算Global true probablity  这一步其实就是在为能量项的计算分配空间 ，但是里面还做了全局单应性获取，局部单应性获取，

  	triplets.emplace_back(0, 0, STRONG_CONSTRAINT);
	triplets.emplace_back(1, 1, STRONG_CONSTRAINT);
	b_vector.emplace_back(0, STRONG_CONSTRAINT);
	b_vector.emplace_back(1, STRONG_CONSTRAINT);

	prepareAlignmentTerm(triplets);
	prepareSimilarityTerm(triplets, b_vector);

	prepareContentPreservingTerm(triplets, b_vector);

	original_vertices = getImageVerticesBySolving(triplets, b_vector);

	Size2 target_size = normalizeVertices(original_vertices);

	Mat result = multi_images.textureMapping(original_vertices, target_size, _blend_method);
	//显示图像
	// Mat show;
	// if (result.channels() == 4)
	// 	cvtColor(result, show, COLOR_BGRA2BGR);
	// else
	// 	show = result;
	// imshow("pano", show);
	// waitKey();


#ifndef DP_NO_LOG
	multi_images.writeResultWithMesh(result, original_vertices, "-[DPS]" + GLOBAL_ROTATION_METHODS_NAME[getGlobalRotationMethod()] + BLENDING_METHODS_NAME[_blend_method] + "[Mesh]", false);
	multi_images.writeResultWithMesh(result, original_vertices, "-[DPS]" + GLOBAL_ROTATION_METHODS_NAME[getGlobalRotationMethod()] + BLENDING_METHODS_NAME[_blend_method] + "[Border]", true);
#endif

	return result;
}

void NISwGSP_Stitching::writeImage(const Mat &_image, int _image_index, const string _blend_method_name) const
{
	const MultiImages &multi_images = getMultiImages();
	const Parameter &parameter = multi_images.parameter;
	//string file_name = parameter.file_name;

	if (_image.empty())
	{
		return;
	}

	if (RUN_TYPE == TYPE::GES_GSP)
	{
		imwrite(parameter.result_dir + "/" + to_string(_image_index) + "-" +
					"Ours_" +
					".png",
				_image);
	}
	else
	{
		imwrite(parameter.result_dir + "/" + to_string(_image_index) + "-" +
					"GSP_" +
					".png",
				_image);
	}
}

/// <summary>
/// Assessment
/// </summary>
void NISwGSP_Stitching::assessment(const vector<vector<Point2>> original_vertices)
{
	double RMSE = getRMSE(original_vertices);
	// MDR
	pair<double, double> W_Residual = getWarpingResidual(original_vertices);
}

pair<double, double> NISwGSP_Stitching::getWarpingResidual(vector<vector<Point2>> _vertices)
{
	return getMultiImages().getWarpingResidual(_vertices);
}

double NISwGSP_Stitching::getRMSE(vector<vector<Point2>> _vertices)
{
	return getMultiImages().getRMSE(_vertices);
}
