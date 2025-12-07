//
//  InputParser.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitching__InputParser__
#define __UglyMan_Stitching__InputParser__

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>

#include "Debugger/ErrorController.h"

class InputParser {
public:
	template <typename T>
	T get(const std::string& key, const T* ptr = NULL) const;

	template <typename T>
	std::vector<T> getVec(const std::string& key,
		const bool sure_exist = true) const;

	InputParser(const std::string& file_name);
private:
	std::map<std::string, std::string> data;

};

#endif /* defined(__UglyMan_Stitching__InputParser__) */
