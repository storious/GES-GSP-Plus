//
//  Blending.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitiching__Blending__
#define __UglyMan_Stitiching__Blending__

#include "../Configure.h"
using namespace cv;

Mat getMatOfLinearBlendWeight(const Mat& image);

std::vector<Mat> getMatsLinearBlendWeight(const std::vector<Mat>& images);

Mat Blending(const std::vector<Mat>& images,
	const std::vector<Point2>& origins,
	const Size2 target_size,
	const std::vector<Mat>& weight_mask,
	const bool ignore_weight_mask = true);

#endif /* defined(__UglyMan_Stitiching__Blending__) */
