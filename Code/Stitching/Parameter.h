//
//  Parameter.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
// 	storious modified
//

#ifndef __UglyMan_Stitching__Parameter__
#define __UglyMan_Stitching__Parameter__

#include <queue>
#include <filesystem>
#include "../Configure.h"
#include "Util/InputParser.h"


class Parameter {
public:
	Parameter(const std::string& _file_name);

	std::string file_name, file_dir;
	std::string stitching_parse_file_name;

	std::string result_dir, debug_dir;
	std::vector<std::string> image_file_full_names;
	/* configure */
	int grid_size;
	int down_sample_image_size;

	/* stitching file */
	double global_homography_max_inliers_dist;
	double  local_homogrpahy_max_inliers_dist;
	int  local_homography_min_features_count;

	int images_count;

	int center_image_index;
	double center_image_rotation_angle;

	const std::vector<std::vector<bool> >& getImagesMatchGraph() const;
	const std::vector<std::pair<int, int> >& getImagesMatchGraphPairList() const;
private:
	mutable std::vector<std::vector<bool> >   images_match_graph_manually;
	mutable std::vector<std::vector<bool> >   images_match_graph_automatically; /* TODO */
	mutable std::vector<std::pair<int, int> > images_match_graph_pair_list;
};

#endif /* defined(__UglyMan_Stitching__Parameter__) */
