#include "FlowField.h"

#include <unordered_set>
#include <Eigen/Dense>

FlowField::FlowField()
{}

FlowField::~FlowField()
{}

void FlowField::ApplyMask(cv::Mat mask, cv::Mat maskWeight, int token,
	std::vector<Vector2>& directions, std::vector<double>& directionWeights, cv::Mat& validMask_,
	int vertical)
{
	// find the boundary points
	std::vector<int> boundaryPointId(mask.rows * mask.cols, -1);
	std::vector<Vector2> points;
	std::vector<std::unordered_set<int> > links;

	for (int i = 0; i < mask.rows; ++i) {
		for (int j = 0; j < mask.cols; ++j) {
			if ((mask.at<unsigned char>(i, j) & token) == 0)
				continue;
			bool isBoundary = false;
			for (int dy = -1; dy < 2; ++dy) {
				for (int dx = -1; dx < 2; ++dx) {
					if (i + dy < 0 || i + dy >= mask.rows || j + dx < 0 || j + dx >= mask.cols) {
						continue;
					}
					if ((mask.at<unsigned char>(i + dy, j + dx) & token) == 0) {
						isBoundary = true;
						break;
					}
				}
				if (isBoundary)
					break;
			}
			if (isBoundary) {
				validMask_.at<unsigned char>(i, j) = 1;
				boundaryPointId[i * mask.cols + j] = points.size();
				points.push_back(Vector2(j, i));
			}
			else
				validMask_.at<unsigned char>(i, j) = 2;
		}
	}

	// for boundary points, build their neighbor graph
	links.resize(points.size());
	for (int i = 0; i < mask.rows; ++i) {
		for (int j = 0; j < mask.cols; ++j) {
			if (validMask_.at<unsigned char>(i, j) != 1)
				continue;
			directionWeights[i * mask.cols + j] = maskWeight.at<float>(i, j);
			int pid = boundaryPointId[i * mask.cols + j];
			for (int dy = -1; dy < 2; ++dy) {
				for (int dx = -1; dx < 2; ++dx) {
					if (i + dy < 0 || i + dy >= mask.rows || j + dx < 0 || j + dx >= mask.cols) {
						continue;
					}
					if (validMask_.at<unsigned char>(i + dy, j + dx) == 1) {
						int pid1 = boundaryPointId[(i + dy) * mask.cols + (j + dx)];
						if (pid != pid1) {
							links[pid].insert(pid1);
							links[pid1].insert(pid);
						}
					}
				}
			}
		}
	}

	// line fitting to determine the boundary direction
	for (int i = 0; i < mask.rows; ++i) {
		for (int j = 0; j < mask.cols; ++j) {
			if (validMask_.at<unsigned char>(i, j) != 1)
				continue;
			int nj = boundaryPointId[i * mask.cols + j];
			std::queue<std::pair<int, int> > q;
			q.push(std::make_pair(nj, 0));
			std::vector<Vector2> pts;
			std::unordered_set<int> s;
			s.insert(nj);
			while (!q.empty()) {
				auto info = q.front();
				int x = info.first;
				int t = info.second;
				q.pop();
				pts.push_back(points[x]);
				if (t == 10)
					continue;
				for (auto& y : links[x]) {
					if (!s.count(y)) {
						s.insert(y);
						q.push(std::make_pair(y, t + 1));
					}
				}
			}
			Vector2 center(0, 0);
			for (auto& p : pts)
				center += p;
			center /= (double)pts.size();
			Eigen::MatrixXd m = Eigen::MatrixXd::Zero(2, 2);
			for (auto& p : pts) {
				m += (p - center) * (p - center).transpose();
			}
			Eigen::JacobiSVD<Eigen::MatrixXd> svd(m, Eigen::ComputeFullU | Eigen:: ComputeFullV);
			directions[i * mask.cols + j] = svd.matrixU().col(0);
			if (vertical) {
				directions[i * mask.cols + j] = Vector2(-directions[i * mask.cols + j][1],
					directions[i * mask.cols + j][0]);
			}
		}
	}
}

void FlowField::SetValidMask(cv::Mat mask, cv::Mat directionMask)
{
	// initialize the tensorfield
	tensorFields_.resize(1);
	auto& boundaryDir_ = tensorFields_[0].boundaryDir;
	validMask_ = cv::Mat::zeros(mask.rows, mask.cols, CV_8U);
	boundaryDir_.resize(mask.rows * mask.cols);
	memset(boundaryDir_.data(), 0, sizeof(Vector2) * boundaryDir_.size());
	tensorFields_[0].width = mask.cols;
	tensorFields_[0].height = mask.rows;
	tensorFields_[0].boundaryWeight.resize(mask.rows * mask.cols, 0);
	tensorFields_[0].softNRosy.resize(mask.rows * mask.cols);
	memset(tensorFields_[0].softNRosy.data(), 0, sizeof(Vector2) * boundaryDir_.size());
	tensorFields_[0].weightNRosy.resize(mask.rows * mask.cols, 0);
	tensorFields_[0].soft1Rosy.resize(mask.rows * mask.cols);
	memset(tensorFields_[0].soft1Rosy.data(), 0, sizeof(Vector2) * boundaryDir_.size());
	tensorFields_[0].weight1Rosy.resize(mask.rows * mask.cols, 0);

	ApplyMask(mask, 1e5 * cv::Mat::ones(mask.rows, mask.cols, CV_32F), 0x03,
		boundaryDir_, tensorFields_[0].boundaryWeight, validMask_, 0);

	if (directionMask.cols > 0) {
		for (int i = 0; i < directionMask.rows; ++i) {
			for (int j = 0; j < directionMask.cols; ++j) {
				auto c = directionMask.at<cv::Vec3b>(i, j);
				if (c.val[2] == 255 && validMask_.at<unsigned char>(i, j) != 0) {
					Vector2 p((c.val[0] - 127), (c.val[1] - 127));
					boundaryDir_[i * directionMask.cols + j] = p.normalized();
					tensorFields_[0].boundaryWeight[i * directionMask.cols + j] = 1e5;
				}
			}
		}
	}
}

void FlowField::SetGuidanceMask(cv::Mat guidance, cv::Mat weight, cv::Mat directionMask)
{
	cv::Mat validMask = cv::Mat::zeros(guidance.rows, guidance.cols, CV_8U);
	ApplyMask(guidance, weight, 0x04, tensorFields_[0].soft1Rosy, tensorFields_[0].weight1Rosy, validMask, 1);
	validMask = cv::Mat::zeros(guidance.rows, guidance.cols, CV_8U);
	ApplyMask(guidance, weight, 0x08, tensorFields_[0].soft1Rosy, tensorFields_[0].weight1Rosy, validMask, 0);
	if (directionMask.cols > 0) {
		for (int i = 0; i < directionMask.rows; ++i) {
			for (int j = 0; j < directionMask.cols; ++j) {
				auto c = directionMask.at<cv::Vec3b>(i, j);
				if (c.val[2] == 255 && (guidance.at<unsigned char>(i, j) & 0x0C)) {
					Vector2 p((c.val[0] - 127), (c.val[1] - 127));
					tensorFields_[0].soft1Rosy[i * directionMask.cols + j] = p.normalized();
					tensorFields_[0].weight1Rosy[i * directionMask.cols + j] = 1e5;
				}
			}
		}
	}
}

void FlowField::CreateOrientationField(int nRosy)
{
	// set the initial tensor parameters
	int numLevel = 0;
	tensorFields_.resize(100);
	tensorFields_[0].pixelStride = 1;
	tensorFields_[0].nRosy = nRosy;
	// keep downsampling to form the pyramid
	for (int i = 0; i < 100; ++i) {
		if (tensorFields_[i].width == 1 && tensorFields_[i].height == 1) {
			numLevel = i + 1;
			break;
		}
		tensorFields_[i].Downsample(tensorFields_[i + 1]);
	}
	tensorFields_.resize(numLevel);
	// coarse to fine optimization
	// upsample from coarser level and optimize the field
	for (int i = numLevel - 1; i >= 0; --i) {
		if (i == numLevel - 1) {
			tensorFields_[i].orientationField = tensorFields_[i].boundaryDir;
			if (tensorFields_[i].orientationField[0].norm() == 0) {
				tensorFields_[i].orientationField[0] = Vector2(1, 0);
			}
		} else {
			tensorFields_[i + 1].UpsampleOrientation(tensorFields_[i]);
		}
		tensorFields_[i].OptimizeOrientationField();
	}
}

void FlowField::CreatePositionField(double unit)
{
	// coarse to fine optimization
	// upsample from coarser level and optimize the field
	int numLevel = tensorFields_.size();
	for (int i = numLevel - 1; i >= 0; --i) {
		if (i == numLevel - 1) {
			tensorFields_[i].positionField.resize(1);
			tensorFields_[i].positionField[0] = Vector2(0, 0);
		} else {
			tensorFields_[i + 1].UpsamplePosition(tensorFields_[i], unit);
			tensorFields_[i].OptimizePositionField(unit);
		}
	}
}

