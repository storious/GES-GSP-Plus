//
//  Parameter.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "Parameter.h"

namespace fs = std::filesystem; // alias
// 创建文件夹
// 读取图片拼接文件信息，设置局部全局单应性内点最小距离等
// 获取指定文件夹下所有图片文件名
// 查看所有图像是否连通
vector<string> getImageFileFullNamesInDir(const string &dir_name) // 获取当前文件夹所有图片名，不包括子文件
{
	vector<string> result;
	const vector<string> image_formats = {
		".bmp", ".dib",
		".jpeg", ".jpg", ".jpe", ".JPG",
		".jp2",
		".png", ".PNG",
		".pbm", ".pgm", ".ppm",
		".sr", ".ras",
		".tiff", ".tif"};

	try
	{
		// C++17 style
		for (const auto &entry : fs::directory_iterator(dir_name)) //遍历文件中每个内容
		{
			// ensure not a subdirectory
			if (entry.is_regular_file())
			{
				string file_ext = entry.path().extension().string();
				// ignore lower/upper case
				std::transform(file_ext.begin(), file_ext.end(), file_ext.begin(), ::tolower); //扩展名变成小写

				for (const auto &ext : image_formats)
				{
					if (file_ext == ext)
					{
						// only add filename
						result.emplace_back(entry.path().filename().string());
						break;
					}
				}
			}
		}
	}
	catch (const fs::filesystem_error &e)
	{
		printError(string("F(getImageFileFullNamesInDir) ") + e.what());
	}
	return result;
}

bool isFileExist(const string &name)
{
	// C++17 style
	return fs::exists(name);
}

Parameter::Parameter(const string &_file_name)
{
	file_name = _file_name;
	file_dir = "./input-data/" + _file_name + "/";
	result_dir = "./input-data/0_results/" + _file_name + "-result/";

	// use <filesystem> cross platform
	fs::create_directories("./input-data/0_results/");
	fs::create_directories(result_dir);

#ifndef DP_NO_LOG
	debug_dir = "./input-data/1_debugs/" + _file_name + "-result/";
	fs::create_directories("./input-data/1_debugs/");
	fs::create_directories(debug_dir);
#endif

	stitching_parse_file_name = file_dir + _file_name + TXT_NAME;  //

	image_file_full_names = getImageFileFullNamesInDir(file_dir);

	/*** configure ***/
	grid_size = GRID_SIZE;
	down_sample_image_size = DOWN_SAMPLE_IMAGE_SIZE;
	if (isFileExist(stitching_parse_file_name))
	{


		
		const InputParser input_parser(stitching_parse_file_name);

		global_homography_max_inliers_dist = input_parser.get<double>("*global_homography_max_inliers_dist", &GLOBAL_HOMOGRAPHY_MAX_INLIERS_DIST);
		local_homogrpahy_max_inliers_dist = input_parser.get<double>("*local_homogrpahy_max_inliers_dist", &LOCAL_HOMOGRAPHY_MAX_INLIERS_DIST);
		local_homography_min_features_count = input_parser.get<int>("*local_homography_min_features_count", &LOCAL_HOMOGRAPHY_MIN_FEATURES_COUNT);
		images_count = input_parser.get<int>("images_count");
		center_image_index = input_parser.get<int>("center_image_index");
		center_image_rotation_angle = input_parser.get<double>("center_image_rotation_angle");


		assert(image_file_full_names.size() == images_count);
		assert(center_image_index >= 0 && center_image_index < images_count);
		/*************/

		images_match_graph_manually.resize(images_count);
		for (int i = 0; i < images_count; ++i)
		{
			images_match_graph_manually[i].resize(images_count, false);
			vector<int> labels = input_parser.getVec<int>("matching_graph_image_edges-" + to_string(i), false); //false指的是如果键不存在，返回的kongxiangl
			for (int j = 0; j < labels.size(); ++j)
			{
				images_match_graph_manually[i][labels[j]] = true;
			}
		}

		//从设置的中间心图像出发，遍历整张图，确保所有图像都是连通的
		queue<int> que;
		vector<bool> label(images_count, false);
		que.push(center_image_index);
		while (que.empty() == false)
		{
			int n = que.front();
			que.pop();  //删除队首元素
			label[n] = true;
			for (int i = 0; i < images_count; ++i)
			{
				if (!label[i] && (images_match_graph_manually[n][i] || images_match_graph_manually[i][n]))
				{
					que.push(i);
				}
			}
		}
		assert(std::all_of(label.begin(), label.end(), [](bool i)
						   { return i; }));

		/*************/

#ifndef DP_NO_LOG
		cout << "center_image_index = " << center_image_index << endl;
		cout << "center_image_rotation_angle = " << center_image_rotation_angle << endl;
		cout << "images_count = " << images_count << endl;
#endif
	}
	
}

const vector<vector<bool>> &Parameter::getImagesMatchGraph() const
{
	if (images_match_graph_manually.empty())
	{
		printError("F(getImagesMatchGraph) image match graph verification [2] didn't be implemented yet");
		return images_match_graph_automatically; /* TODO */
	}
	return images_match_graph_manually;
}

const vector<pair<int, int>> &Parameter::getImagesMatchGraphPairList() const //获取图像匹配元组，例如（i,j）
{
	if (images_match_graph_pair_list.empty())
	{
		const vector<vector<bool>> &images_match_graph = getImagesMatchGraph(); //获得图像匹配关系
		for (int i = 0; i < images_match_graph.size(); ++i)
		{
			for (int j = 0; j < images_match_graph[i].size(); ++j)
			{
				if (images_match_graph[i][j])
				{
					images_match_graph_pair_list.emplace_back(i, j);
				}
			}
		}
	}
	return images_match_graph_pair_list;
}
