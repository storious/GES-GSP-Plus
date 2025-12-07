//
//  InputParser.cpp
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#include "InputParser.h"

class StrTok
{
public:
	StrTok(const std::string &str_input,
		   const std::string &str_delim = " ,")
	{
		std::string::size_type nHead, nTail;

		nHead = str_input.find_first_not_of(str_delim, 0);

		nTail = str_input.find_first_of(str_delim, nHead);

		while (nHead != std::string::npos || nTail != std::string::npos)
		{
			tokened.emplace_back(str_input.substr(nHead, nTail - nHead));
			nHead = str_input.find_first_not_of(str_delim, nTail);
			nTail = str_input.find_first_of(str_delim, nHead);
		}
	}
	std::vector<std::string> tokened;
};

InputParser::InputParser(const std::string &file_name)
{
	std::ifstream file(file_name);
	std::string line;

	while (getline(file, line))
	{
		StrTok st(line, " {|}\n\r\t");
		data[st.tokened[0]] = st.tokened[1];
	}
	file.close();
}

template <typename T>
T transfer(const std::string &str)
{
	// INFO: C++17
	if constexpr (std::is_same_v<T, int>)
		return std::stoi(str);
	else if constexpr (std::is_same_v<T, float>)
	{
		return std::stof(str);
	}
	else if constexpr (std::is_same_v<T, double>)
	{
		return std::stod(str);
	}
	else
	{
		// 如果类型不匹配，编译时就会报错，这比运行时打印错误更好
		static_assert(!sizeof(T), "transfer() is not implemented for this type");
		// printError("F(transfer) unsupported type");
		// return T{}; // 返回一个值初始化的 T
	}
}

template <>
std::string transfer<std::string>(const std::string &str)
{
	return str;
}

template <typename T>
T InputParser::get(const std::string &key, const T *ptr) const
{
	T result;
	if (data.find(key) == data.end())
	{
		if (ptr == NULL)
		{
			printError("F(get) key error: " + key);
		}
		else
		{
			result = *ptr;
		}
	}
	else
	{
		const std::string value = data.at(key);
		result = transfer<T>(value);
	}
	return result;
}

template <typename T>
std::vector<T> InputParser::getVec(const std::string &key,
								   const bool sure_exist) const
{
	std::vector<T> result;
	if (data.find(key) == data.end())
	{
		if (sure_exist)
		{
			printError("F(getVec) key error: " + key);
		}
	}
	else
	{
		const std::string value = data.at(key);
		StrTok st(value, " (,)\n\r\t");
		result.reserve(st.tokened.size());
		for (int i = 0; i < st.tokened.size(); ++i)
		{
			result.emplace_back(transfer<T>(st.tokened[i]));
		}
	}
	return result;
}

template int InputParser::get<int>(const std::string &key, const int *ptr) const;
template float InputParser::get<float>(const std::string &key, const float *ptr) const;
template double InputParser::get<double>(const std::string &key, const double *ptr) const;
template std::string InputParser::get<std::string>(const std::string &key, const std::string *ptr) const;

template std::vector<int> InputParser::getVec<int>(const std::string &key, const bool sure_exist) const;
template std::vector<float> InputParser::getVec<float>(const std::string &key, const bool sure_exist) const;
template std::vector<double> InputParser::getVec<double>(const std::string &key, const bool sure_exist) const;
template std::vector<std::string> InputParser::getVec<std::string>(const std::string &key, const bool sure_exist) const;
