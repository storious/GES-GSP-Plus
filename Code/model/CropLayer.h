#pragma once
#include "../Configure.h"
class CropLayer : public Layer
{

public:
	CropLayer(const LayerParams &params) : cv::dnn::Layer(params) {}

	static cv::Ptr<cv::dnn::Layer> create(cv::dnn::LayerParams &params)
	{
		return cv::makePtr<CropLayer>(params); // use cv::makePtr
	}

	bool getMemoryShapes(const std::vector<MatShape> &inputs,
						 const int requiredOutputs,
						 std::vector<MatShape> &outputs,
						 std::vector<MatShape> &internals) const override
	{

		CV_UNUSED(requiredOutputs);
		CV_UNUSED(internals);
		std::vector<int> outShape = {inputs[0][0], inputs[0][1], inputs[1][2], inputs[1][3]};
		outputs.assign(1, outShape);
		return false;
	}

	void forward(cv::InputArrayOfArrays inputs_arr, cv::OutputArrayOfArrays outputs_arr, cv::OutputArrayOfArrays internals) override
	{
		std::vector<cv::Mat> inputs, outputs;
		inputs_arr.getMatVector(inputs);
		outputs_arr.getMatVector(outputs);

		cv::Mat &inp = inputs[0];
		cv::Mat &out = outputs[0];

		const int ystart = (inp.size[2] - out.size[2]) / 2;
		const int xstart = (inp.size[3] - out.size[3]) / 2;
		const int yend = ystart + out.size[2];
		const int xend = xstart + out.size[3];

		const int batchSize = inp.size[0];
		const int numChannels = inp.size[1];

		for (int i = 0; i < batchSize; ++i)
		{ 
			for (int j = 0; j < numChannels; ++j)
			{
				cv::Mat plane(inp.size[2], inp.size[3], CV_32F, inp.ptr<float>(i, j));
				cv::Mat crop = plane(cv::Range(ystart, yend), cv::Range(xstart, xend));
				cv::Mat targ(out.size[2], out.size[3], CV_32F, out.ptr<float>(i, j));
				crop.copyTo(targ);
			}
		}
	}
};
