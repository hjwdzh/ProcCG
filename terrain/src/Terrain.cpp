#include "Terrain.h"

void Terrain::UpdateElementMask(cv::Mat imgMask, cv::Mat directionGuide, double weight)
{
	if (elementMask.cols == 0) {
		elementMask = cv::Mat::zeros(imgMask.rows, imgMask.cols, CV_8U);
	}
	if (weightMask.cols == 0) {
		weightMask = cv::Mat::zeros(imgMask.rows, imgMask.cols, CV_32F);
	}
	if (directionMask.cols == 0) {
		directionMask = cv::Mat::zeros(imgMask.rows, imgMask.cols, CV_8UC3);
	}
	if (directionGuide.cols > 0) {
		for (int i = 0; i < directionGuide.rows; ++i) {
			for (int j = 0; j < directionGuide.cols; ++j) {
				auto c = directionGuide.at<cv::Vec3b>(i, j);
				if (c.val[2] == 0)
					directionMask.at<cv::Vec3b>(i, j) = cv::Vec3b(127, 127, 127);
				else if (c.val[2] == 255)
					directionMask.at<cv::Vec3b>(i, j) = c;
			}
		}
	}
	for (int i = 0; i < imgMask.rows; ++i) {
		for (int j = 0; j < imgMask.cols; ++j) {
			cv::Vec3b c = imgMask.at<cv::Vec3b>(i, j);
			unsigned char c0 = elementMask.at<unsigned char>(i, j);
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
			if (c0 != elementMask.at<unsigned char>(i, j)) {
				elementMask.at<unsigned char>(i, j) = c0;
			}
		}
	}
	field = FlowField();
	field.SetValidMask(elementMask, directionMask);
	field.SetGuidanceMask(elementMask, weightMask, directionMask);
	field.CreateOrientationField();
}

void Terrain::UpdateCurrentWeight(cv::Mat currentMask, double weight)
{
	for (int i = 0; i < currentMask.rows; ++i) {
		for (int j = 0; j < currentMask.cols; ++j) {
			if (currentMask.at<unsigned char>(i, j)) {
				field.tensorFields_[0].weight1Rosy[i * currentMask.cols + j] = weight;
			}
		}
	}
	cv::Mat bigMask(currentMask.rows, currentMask.cols, CV_8U);
	for (int i = 0; i < currentMask.rows; ++i) {
		for (int j = 0; j < currentMask.cols; ++j) {
			double w = field.tensorFields_[0].weight1Rosy[i * currentMask.cols + j];
			if (w > 20) {
				bigMask.at<unsigned char>(i, j) = 255;
			}
		}
	}
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

void Terrain::SaveToFile(FILE* fp)
{
	auto SaveImage = [](cv::Mat m, FILE* fp, int type, int channel) {
		fwrite(&m.rows, sizeof(int), 1, fp);
		fwrite(&m.cols, sizeof(int), 1, fp);
		fwrite(&type, sizeof(int), 1, fp);
		fwrite(&channel, sizeof(int), 1, fp);
		if (m.rows > 0) {
			int size = (type == 0) ? 1 : 4;
			size *= channel;
			fwrite(m.data, sizeof(unsigned char), size * m.rows * m.cols, fp);
		}
	};
	fwrite(&version, sizeof(int), 1, fp);
	if (version >= 1) {
		printf("A...\n");
		SaveImage(elementMask, fp, 0, 1);
		printf("B...\n");
		SaveImage(weightMask, fp, 1, 1);
		printf("C...\n");
		SaveImage(directionMask, fp, 0, 3);
		printf("D...\n");
		SaveImage(heightMask, fp, 1, 1);
	}
}

void Terrain::LoadFromFile(FILE* fp)
{
	auto LoadImage = [](cv::Mat& m, FILE* fp) {
		int rows, cols, type, channel;
		fread(&rows, sizeof(int), 1, fp);
		fread(&cols, sizeof(int), 1, fp);
		fread(&type, sizeof(int), 1, fp);
		fread(&channel, sizeof(int), 1, fp);
		if (rows == 0) {
			m = cv::Mat();
			return;
		}
		if (type == 0) {
			if (channel == 1)
				m = cv::Mat(rows, cols, CV_8U);
			else
				m = cv::Mat(rows, cols, CV_8UC3);
		}
		else {
			if (channel == 1)
				m = cv::Mat(rows, cols, CV_32F);
			else
				m = cv::Mat(rows, cols, CV_32FC3);			
		}
		int size = (type == 0) ? 1 : 4;
		size *= channel;
		fread(m.data, sizeof(unsigned char), size * m.rows * m.cols, fp);
	};
	fread(&version, sizeof(int), 1, fp);
	if (version >= 1) {
		LoadImage(elementMask, fp);
		LoadImage(weightMask, fp);
		LoadImage(directionMask, fp);
		LoadImage(heightMask, fp);
	}
	field = FlowField();
	field.SetValidMask(elementMask, directionMask);
	field.SetGuidanceMask(elementMask, weightMask, directionMask);
	field.CreateOrientationField();
}
