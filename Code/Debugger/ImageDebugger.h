//
//  ImageDebugger.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitching__ImageDebugger__
#define __UglyMan_Stitching__ImageDebugger__

#include "../Configure.h"
using namespace cv;

Mat getImageOfFeaturePairs(const Mat& img1,
	const Mat& img2,
	const std::vector<Point2>& f1,
	const std::vector<Point2>& f2);

#endif /* defined(__UglyMan_Stitching__ImageDebugger__) */
