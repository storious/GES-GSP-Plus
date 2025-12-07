#include <iostream>
#include "Stitching/NISwGSP_Stitching.h"
#include <chrono>

int GRID_SIZE_w = 40;
int GRID_SIZE_h = 40;

int main(int argc, const char *argv[])
{
	Eigen::initParallel(); 

	CV_DNN_REGISTER_LAYER_CLASS(Crop, CropLayer);
	std::cout << "nThreads = " << Eigen::nbThreads() << std::endl;
	std::cout << "[#Images : " << argc - 1 << "]" << std::endl;

	auto start = std::chrono::high_resolution_clock::now();
	for (int i = 1; i < argc; ++i)
	{
		std::cout << "i = " << i << ", [Images : " << argv[i] << "]" << std::endl;
		MultiImages multi_images(argv[i], LINES_FILTER_WIDTH, LINES_FILTER_LENGTH);

		/* 2D */
		NISwGSP_Stitching niswgsp(multi_images);
		niswgsp.setWeightToAlignmentTerm(1);
		niswgsp.setWeightToLocalSimilarityTerm(0.75);
		niswgsp.setWeightToGlobalSimilarityTerm(6, 20, GLOBAL_ROTATION_2D_METHOD);
		niswgsp.setWeightToContentPreservingTerm(1.5);
		Mat blend_linear;
		std::vector<std::vector<Point2>> original_vertices;

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
		std::cout << "Time:" << elapsed.count() << std::endl;
		niswgsp.writeImage(blend_linear, BLENDING_METHODS_NAME[BLEND_LINEAR]);

		niswgsp.assessment(original_vertices);
	}

	return 0;
}
