#ifndef TERRAIN_TERRAIN_H_
#define TERRAIN_TERRAIN_H_

#include "FlowField.h"

class Terrain
{
public:
	void UpdateElementMask(cv::Mat mask, float weight);
	cv::Mat elementMask;//0 ground, 1 river, 2 grass, 4 population, 8 guidance
	cv::Mat weightMask;
	cv::Mat heightMask; //float array

	cv::Mat VisualizeField();
	int TokenFromScalar(const cv::Vec3b& c);

	FlowField field;
};

#endif