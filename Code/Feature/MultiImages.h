//
//  MultiImages.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitching__MultiImages__
#define __UglyMan_Stitching__MultiImages__

#include <queue>
#include "../Configure.h"
#include "Stitching/Parameter.h"
#include "ImageData.h"
#include "Util/Statistics.h"
#include "Debugger/ImageDebugger.h"
#include "Stitching/APAP_Stitching.h"
#include "Util/Blending.h"
#include "Debugger/ColorMap.h"

#include <opencv2/calib3d.hpp> /* CV_RANSAC */
#include <opencv2/stitching/detail/autocalib.hpp> /* ImageFeatures, MatchesInfo */
#include <opencv2/stitching/detail/camera.hpp> /* CameraParams */
#include <opencv2/stitching/detail/motion_estimators.hpp> /* BundleAdjusterBase */
using namespace cv;

const int pair_COUNT = 2;

class FeatureDistance {
public:
	double distance;
	int feature_index[pair_COUNT];
	FeatureDistance() {
		feature_index[0] = feature_index[1] = -1;
		distance = FLT_MAX;
	}
	FeatureDistance(const double _distance,
		const int _p_1,
		const int _feature_index_1,
		const int _feature_index_2) {
		distance = _distance;
		feature_index[_p_1] = _feature_index_1;
		feature_index[!_p_1] = _feature_index_2;
	}
	bool operator < (const FeatureDistance& fd) const {
		return distance > fd.distance;
	}
private:
};

class SimilarityElements {
public:
	double scale;
	double theta;
	SimilarityElements(const double _scale,
		const double _theta) {
		scale = _scale;
		theta = _theta;
	}
private:
};

class MultiImages {
public:
	MultiImages(const std::string& _file_name,
		LINES_FILTER_FUNC* _width_filter = &LINES_FILTER_NONE,
		LINES_FILTER_FUNC* _length_filter = &LINES_FILTER_NONE);

	const std::vector<detail::ImageFeatures>& getImagesFeaturesByMatchingPoints() const;
	const std::vector<detail::MatchesInfo>& getPairwiseMatchesByMatchingPoints() const;
	const std::vector<detail::CameraParams>& getCameraParams() const;

	const std::vector<std::vector<bool> >& getImagesFeaturesMaskByMatchingPoints() const;

	const std::vector<std::vector<std::vector<std::pair<int, int> > > >& getFeaturepairs() const;
	const std::vector<std::vector<std::vector<Point2> > >& getFeatureMatches() const;

	const std::vector<std::vector<std::vector<bool> > >& getAPAPOverlapMask() const;
	const std::vector<std::vector<std::vector<Mat> > >& getAPAPHomographies() const;
	const std::vector<std::vector<std::vector<Point2> > >& getAPAPMatchingPoints() const;

	const std::vector<std::vector<InterpolateVertex> >& getInterpolateVerticesOfMatchingPoints() const;

	const std::vector<int>& getImagesVerticesStartIndex() const;
	const std::vector<SimilarityElements>& getImagesSimilarityElements(const enum GLOBAL_ROTATION_METHODS& _global_rotation_method) const;
	const std::vector<std::vector<std::pair<double, double> > >& getImagesRelativeRotationRange() const;

	const std::vector<std::vector<double> >& getImagesGridSpaceMatchingPointsWeight(const double _global_weight_gamma) const;

	const  std::vector<std::vector<std::vector<double> >>& getSamplesWeight() const;

	const std::vector<Point2>& getImagesLinesProject(const int _from, const int _to) const;

	const std::vector<Mat>& getImages() const;

	//Wasted code.
	const double getRansacDiffWeight(const std::pair<int, int>& _index_pair) const;
	const float getOverlap(std::pair<int, int> _mask_pair) const;


	FLOAT_TYPE getImagesMinimumLineDistortionRotation(const int _from, const int _to) const;
	const std::vector<std::vector<std::vector<Point> > >& getContentSamplePoints() const;
	const std::vector<std::vector<std::vector<InterpolateVertex> > >& getSamplesInterpolation() const;
	const std::vector < std::vector<std::vector<std::pair<double, double> >> >& getTermUV() const;

	Mat textureMapping(const std::vector<std::vector<Point2> >& _vertices,
		const Size2& _target_size,
		const BLENDING_METHODS& _blend_method) const;

	Mat textureMapping(const std::vector<std::vector<Point2> >& _vertices,
		const Size2& _target_size,
		const BLENDING_METHODS& _blend_method,
		std::vector<Mat>& _warp_images) const;

	void writeResultWithMesh(const Mat& _result,
		const std::vector<std::vector<Point2> >& _vertices,
		const std::string& _postfix,
		const bool _only_border) const;

	void drawRansac(const int img_index, const int img_index_second,
		const std::vector<std::pair<int, int> >& _initial_indices, const std::vector<char>& _mask) const;

	std::vector<ImageData> images_data;
	Parameter parameter;
	mutable std::vector<std::vector<double > >            content_line_weights;

	double getRMSE(std::vector<std::vector<Point2> > _vertices) const;
	std::pair<double, double> getWarpingResidual(std::vector<std::vector<Point2> > _vertices) const;
private:
	/*** Debugger ***/
	void writeImageOfFeaturepairs(const std::string& _name,
		const std::pair<int, int>& _index_pair,
		const std::vector<std::pair<int, int> >& _pairs) const;
	/****************/

	void doFeatureMatching() const;
	void initialFeaturePairsSpace() const;
	void initialRansacDiffpairs() const;
	//Wasted code 
	void updataRansacDiff(const std::pair<int, int>& _index_pair, const std::vector<Point2> srcPoints, const std::vector<Point2> dstPoints, const std::vector<char> final_mask, const Mat H) const;
	//Wasted code 
	double generateRansacAvgDiff(const std::pair<int, int>& _index_pair) const;
	//Wasted code 
	void generateRansacDiffWeight(std::vector<std::pair<int, int> > pair) const;

	std::vector<std::pair<int, int> > getInitialFeaturepairs(const std::pair<int, int>& _match_pair) const;

	std::vector<std::pair<int, int> > getFeaturepairsBySequentialRANSAC(const std::pair<int, int>& _match_pair,
		const std::vector<Point2>& _X,
		const std::vector<Point2>& _Y,
		const std::vector<std::pair<int, int> >& _initial_indices) const;

	const  std::vector<std::vector<std::pair<double, double>>> calcTriangleUV(const std::vector<std::vector<Point>> samples) const;

	const std::vector<std::vector<std::vector<Point2> > >& getTwoImgFeatureMatches(std::pair<int, int> _mask_pair_) const;

	std::vector<std::pair<int, int>> getTwoImgFeaturepairs(std::pair<int, int> _mask_pair_) const;


	mutable std::vector<std::vector<std::vector<double>>> ransacDiff;
	mutable std::vector<std::vector<double>> ransacAvgDiff;
	mutable std::vector<std::vector<double>> ransacDiffWeight;
	const std::string txtName = "./RansacDst//";

	mutable std::vector<detail::ImageFeatures> images_features;
	mutable std::vector<detail::MatchesInfo>   pairwise_matches;
	mutable std::vector<detail::CameraParams>  camera_params;
	mutable std::vector<std::vector<bool> > images_features_mask;

	mutable std::vector<std::vector<std::vector<std::pair<int, int> > > > feature_pairs;
	mutable std::vector<std::vector<std::vector<Point2> > > feature_matches; /* [m1][m2][j], img1 j_th matches */

	mutable std::vector<std::vector<std::vector<bool> > >   apap_overlap_mask;
	mutable std::vector<std::vector<std::vector<Mat> > >    apap_homographies;
	mutable std::vector<std::vector<std::vector<Point2> > > apap_matching_points;

	mutable std::vector<std::vector<InterpolateVertex> > mesh_interpolate_vertex_of_feature_pts;
	mutable std::vector<std::vector<InterpolateVertex> > mesh_interpolate_vertex_of_matching_pts;

	mutable std::vector<int> images_vertices_start_index;
	mutable std::vector<SimilarityElements> images_similarity_elements_2D;
	mutable std::vector<SimilarityElements> images_similarity_elements_3D;
	mutable std::vector<std::vector<std::pair<double, double> > > images_relative_rotation_range;

	mutable std::vector<std::vector<double> > images_polygon_space_matching_pts_weight;
	mutable std::vector<std::vector<std::vector<double> > >  samplesWeight;
	/* Line */
	mutable std::vector<std::vector<FLOAT_TYPE> > images_minimum_line_distortion_rotation;
	mutable std::vector<std::vector<std::vector<Point2> > > images_lines_projects; /* [m1][m2] img1 lines project on img2 */

	mutable std::vector<Mat> images;

	mutable std::vector<std::vector<std::vector<Point> > >            content_sample_points;

	mutable std::vector<std::vector<std::vector<InterpolateVertex> > > content_mesh_interpolation;
	mutable std::vector < std::vector<std::vector<std::pair<double, double>> >> content_term_uv;


};
double Point2dDis(Point2d p1, Point2d p2);
int VerifyVertices(Point2d p1, int m1Tom2_p1_index, const std::vector<Indices>& m1_polygons_indice, const std::vector<Point2>& m1_vertices);
Mat _getAffineTransform(std::vector<std::vector<Point2>>& _vertices, int m1, const std::vector<Indices>& m1_polygons_indice, int m1Tom2_p1_index, int m1Tom2_p1_temp, const std::vector<Point2>& m1_vertices);
std::pair<double, double> getLineResidual(std::vector<Point2f> _vertices, Vec4f _line_param);
#endif /* defined(__UglyMan_Stitching__MultiImages__) */
