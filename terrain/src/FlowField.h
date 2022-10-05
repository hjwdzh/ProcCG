#ifndef TERRAIN_FLOWFIELD_H_
#define TERRAIN_FLOWFIELD_H_

#include "TensorField.h"

// interface for computing flow field (orientation/position)
class FlowField
{
public:
	FlowField();
	~FlowField();
	// compute the orientation field. nRosy = 2/4
	void CreateOrientationField(int nRosy = 4);
	// compute the position field with unit length as unit
	void CreatePositionField(double unit);
	// set the valid mask
	void SetValidMask(cv::Mat mask);
	// set guidance mask
	void SetGuidanceMask(cv::Mat guidance, cv::Mat weight);
	// visualize the orientation field
	cv::Mat VisualizeTensorField() {
		return tensorFields_[0].VisualizeTensorField();
	}
	// get the orientation field as an image
	cv::Mat OrientationField() {
		return tensorFields_[0].OrientationField();
	}

	cv::Mat validMask_;							// valid Mask
	std::vector<TensorField> tensorFields_;		// tensor field as a pyramid
private:
	void ApplyMask(cv::Mat mask, cv::Mat maskWeight, int token,
		std::vector<Vector2>& directions, std::vector<double>& directionWeights, cv::Mat& validMask_,
		int vertical);
};

#endif