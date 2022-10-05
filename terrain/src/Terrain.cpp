#include "Terrain.h"

void Terrain::UpdateElementMask(cv::Mat imgMask, float weight)
{
	if (elementMask.cols == 0) {
		elementMask = cv::Mat(imgMask.rows, imgMask.cols, CV_8U);
	}
	if (weightMask.cols == 0) {
		weightMask = cv::Mat(imgMask.rows, imgMask.cols, CV_32F);
	}
	for (int i = 0; i < imgMask.rows; ++i) {
		for (int j = 0; j < imgMask.cols; ++j) {
			auto c = imgMask.at<cv::Vec3b>(i, j);
			auto c0 = elementMask.at<unsigned char>(i, j);
			int token = TokenFromScalar(c);
			if (token == 0)
				continue;
			if (token < 3) {
				c0 = token;
			}
			else if (token == 16) {
				c0 = 0;
			}
			else {
				c0 = c0 | token;
				weightMask.at<float>(i, j) = weight;
			}
			elementMask.at<unsigned char>(i, j) = c0;
		}
	}
	field = FlowField();
	field.SetValidMask(elementMask);
	field.SetGuidanceMask(elementMask, weightMask);
	field.CreateOrientationField();
}

int Terrain::TokenFromScalar(const cv::Vec3b& c)
{
	using Vector3 = Eigen::Vector3d;
	Vector3 directions[] = {{0x38,0xB3,0xD0},
		{0x7E,0xC8,0x50},{1,0,0},{1,1,0},{1,1,1}};
	for (auto& d : directions) d.normalize();
	Vector3 d(c.val[0], c.val[1], c.val[2]);
	d.normalize();
	int bestId = -1;
	double bestDot = -1;
	for (int i = 0; i < 5; ++i) {
		double dot = directions[i].dot(d);
		if (dot > bestDot) {
			bestDot = dot;
			bestId = i;
		}
	}
	return (1 << bestId);
}

cv::Mat Terrain::VisualizeField()
{
	cv::Mat vis = field.VisualizeTensorField();
	for (int i = 0; i < elementMask.rows; ++i) {
		for (int j = 0; j < elementMask.cols; ++j) {
			float r = vis.at<cv::Vec3b>(i, j).val[0] / 255.0;
			int token = elementMask.at<unsigned char>(i, j);
			if (token == 0) continue;
			if (token == 1)
				vis.at<cv::Vec3b>(i, j) = cv::Vec3b(0x38 * r, 0xB3 * r, 0xD0 * r);
			if (token == 2)
				vis.at<cv::Vec3b>(i, j) = cv::Vec3b(0x7E * r, 0xC8 * r, 0x50 * r);
			if (token & 0x04)
				vis.at<cv::Vec3b>(i, j) = cv::Vec3b(0xFF * r, 0, 0);
			if (token & 0x08)
				vis.at<cv::Vec3b>(i, j) = cv::Vec3b(0xFF * r, 0xFF * r, 0);
		}
	}
	return vis;
}
