#ifndef TERRAIN_TERRAIN_H_
#define TERRAIN_TERRAIN_H_

#include "FlowField.h"
#include "RoadGenerator.h"
#include <fstream>

class Terrain
{
public:
	void UpdateElementMask(cv::Mat mask, cv::Mat directionGuide, double weight);
	void UpdateCurrentWeight(cv::Mat currentMask, double weight);
	cv::Mat elementMask;//0 ground, 1 river, 2 grass, 4 population, 8 guidance
	cv::Mat weightMask;
	cv::Mat directionMask;
	cv::Mat heightMask; //float array

	cv::Mat VisualizeField();
	cv::Mat VisualizeElement();
	int TokenFromScalar(const cv::Vec3b& c);

	void SaveToFile(FILE* fp);
	void LoadFromFile(FILE* fp);

	FlowField field;
	Road road;
	int version{1};
};

#endif