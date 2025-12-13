#include <iostream>
#include "./Stitching/NISwGSP_Stitching.h"
#include <chrono>

int GRID_SIZE_w = 40;
int GRID_SIZE_h = 40;

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
		cout << "i = " << i << ", [Images : " << argv[i] << "]" << endl;
		MultiImages multi_images(argv[i], LINES_FILTER_WIDTH, LINES_FILTER_LENGTH); //顺便把图片名字也存储再ImageData中了

		/* 2D */
		NISwGSP_Stitching niswgsp(multi_images); //就是给NISwGSP和MeshOptimization里面给了MultiImage的对象可以调用
		niswgsp.setWeightToAlignmentTerm(1);
		niswgsp.setWeightToLocalSimilarityTerm(0.75);
		niswgsp.setWeightToGlobalSimilarityTerm(6, 20, GLOBAL_ROTATION_2D_METHOD);
		niswgsp.setWeightToContentPreservingTerm(1.5);
		Mat blend_linear; //Matrix（矩阵）opencv的
		vector<vector<Point2> > original_vertices;

		switch (RUN_TYPE)
		{
		case TYPE::GES_GSP:
			blend_linear = niswgsp.solve_content(BLEND_LINEAR, original_vertices);
			break;
		case TYPE::GSP:
			blend_linear = niswgsp.solve(BLEND_LINEAR, original_vertices);
			break;
		}
		auto end = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = end - start;
		std::cout << "Time:" << elapsed.count() << endl;
		niswgsp.writeImage(blend_linear, BLENDING_METHODS_NAME[BLEND_LINEAR]);

		niswgsp.assessment(original_vertices);
	}

	return 0;
}
