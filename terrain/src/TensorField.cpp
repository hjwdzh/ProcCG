#include "TensorField.h"

int TensorField::Trace(Vector2 point, std::vector<Vector2>& polyline, double len, int isMajor, int directions) const
{
	std::list<Vector2> pts;
	pts.push_back(point);
	std::unordered_map<int, int> visited;
	int counter = 0;
	int counter0 = 0;
	for (int i = 0; i < directions; ++i) {
		point = pts.back();
		Vector2 dir = orientationField[(int)point[1] * width + (int)point[0]];
		if (isMajor == 0)
			dir = Vector2(-dir[1], dir[0]);
		if (i == 1)
			dir = -dir;
		double l = len * 0.5;
		while (l > 0 && point[0] >= 0 && point[1] >= 0 && point[0] < width && point[1] < height) {
			int nx = point[0];
			int ny = point[1];
			int id = ny * width + nx;
			if (visited.count(id) && visited[id] < counter - 3 && l < len * 0.5 - 1.0)
				break;
			counter += 1;
			if (visited.count(id) == 0) {
				visited[id] = counter;
			}
			Vector2 newDir = orientationField[(int)point[1] * width + (int)point[0]];
			if (isMajor == 0)
				newDir = Vector2(-newDir[1], newDir[0]);
			newDir = SumNRosy(newDir, dir, nRosy) - dir;
			double t0 = newDir[0] < 0 ? ((int)point[0] - point[0]) / newDir[0] : (1 + (int)point[0] - point[0]) / newDir[0];
			double t1 = newDir[1] < 0 ? ((int)point[1] - point[1]) / newDir[1] : (1 + (int)point[1] - point[1]) / newDir[1];
			double t = 1;
			if (newDir[0] != 0 && t0 < t1 || newDir[1] == 0)
				t = t0;
			if (newDir[1] != 0 && t1 < t0 || newDir[0] == 0)
				t = t1;
			t += 1e-6;
			point += newDir * t;
			dir = newDir;
			if (point[0] >= 0 && point[1] >= 0 && point[0] < width && point[1] < height) {
				if (i == 0)
					pts.push_front(point);
				else
					pts.push_back(point);
			}
			l -= 1.0;
		}
		if (i == 0)
			counter0 = pts.size() - 1;
	}

	polyline.insert(polyline.end(), pts.begin(), pts.end());
	return counter0;
}

cv::Mat TensorField::VisualizeTensorField()
{
	cv::Mat vis(height, width, CV_8U);
	for (int i = 0; i < vis.rows; ++i) {
		for (int j = 0; j < vis.cols; ++j) {
			vis.at<unsigned char>(i, j) = 128;
		}
	}
	for (int i = 0; i < width / 2; ++i) {
		std::vector<Vector2> polyline;
		Vector2 seed(rand() % width, rand() % height);
		auto color = cv::Scalar(rand() % 128 + 64);
		Trace(seed, polyline, 10000, rand() % 2, 2);
		for (int j = 0; j < polyline.size() - 1; ++j) {
			cv::Point2f p0(polyline[j][0], polyline[j][1]);
			cv::Point2f p1(polyline[j + 1][0], polyline[j + 1][1]);
			cv::line(vis, p0, p1, color, 2);
		}
	}
	return vis;
}

cv::Mat TensorField::OrientationField()
{
	cv::Mat vis(height, width, CV_32FC2);	
	for (int i = 0; i < vis.rows; ++i) {
		for (int j = 0; j < vis.cols; ++j) {
			vis.at<cv::Vec2f>(i, j) = cv::Vec2f(orientationField[i * vis.cols + j][0],
				orientationField[i * vis.cols + j][1]);
		}
	}
	return vis;
}


void TensorField::OptimizeOrientationField()
{
	int diff[4][2] = {{0, 1}, {1, 0}, {-1, 0}, {0, -1}};
	for (int steps = 0; steps < 3; ++steps) {
		for (int di = 0; di < 2; ++di) {
			for (int dj = 0; dj < 2; ++dj) {

#pragma omp parallel for
				for (int i = 0; i < height / 2; ++i) {
					for (int j = 0; j < width / 2; ++j) {
						int ni = i * 2 + di;
						int nj = j * 2 + dj;
						if (ni == height || nj == width)
							continue;
						// compute direct constaint
						Vector2 dir1Rosy = boundaryDir[ni * width + nj] * boundaryWeight[ni * width + nj] +
							soft1Rosy[ni * width + nj] * weight1Rosy[ni * width + nj];
						// NRosy vectors
						Vector2 dirNRosy = softNRosy[ni * width + nj] * weightNRosy[ni * width + nj];
						Vector2 dirConstraint = SumNRosy(dirNRosy, dir1Rosy, nRosy);
						// compute regularization constraint
						dirNRosy = Vector2(0, 0);
						for (int k = 0; k < 4; ++k) {
							int mi = ni + diff[k][0];
							int mj = nj + diff[k][1];
							if (mi < 0 || mi >= height || mj < 0 || mj >= width)
								continue;
							if (dirNRosy.norm() < orientationField[mi * width + mj].norm())
								dirNRosy = SumNRosy(dirNRosy, orientationField[mi * width + mj], nRosy);
							else
								dirNRosy = SumNRosy(orientationField[mi * width + mj], dirNRosy, nRosy);
						}
						if (dirNRosy.norm() < dirConstraint.norm())
							orientationField[ni * width + nj] = SumNRosy(dirNRosy, dirConstraint, nRosy).normalized();
						else
							orientationField[ni * width + nj] = SumNRosy(dirConstraint, dirNRosy, nRosy).normalized();
					}
				}
			}
		}
	}
	// follow the boundary
	std::queue<int> q;
	std::vector<int> visited(boundaryDir.size());
	for (int i = 0; i < boundaryDir.size(); ++i) {
		if (boundaryDir[i].norm() > 0) {
			q.push(i);
			orientationField[i] = SumNRosy(orientationField[i], boundaryDir[i], nRosy) - boundaryDir[i];
			visited[i] = 1;
		}
	}
	while (!q.empty()) {
		int t = q.front();
		q.pop();
		int x = t % width;
		int y = t / width;
		for (int k = 0; k < 4; ++k) {
			int nx = x + diff[k][0];
			int ny = y + diff[k][1];
			int nt = ny * width + nx;
			if (nx < 0 || nx >= width || ny < 0 || ny >= height || visited[nt])
				continue;
			orientationField[nt] = SumNRosy(orientationField[nt], orientationField[t], nRosy) - orientationField[t];
			visited[nt] = 1;
			q.push(nt);
		}
	}
}

void TensorField::UpsampleOrientation(TensorField& other)
{
	other.orientationField.resize(other.width * other.height);
#pragma omp parallel for
	for (int i = 0; i < other.height; ++i) {
		for (int j = 0; j < other.width; ++j) {
			int ni = i / 2;
			int nj = j / 2;
			other.orientationField[i * other.width + j] = orientationField[ni * width + nj];
		}
	}
}

void TensorField::UpsamplePosition(TensorField& other, double unit)
{
	other.positionField.resize(other.width * other.height);
	for (int i = 0; i < other.height; ++i) {
		for (int j = 0; j < other.width; ++j) {
			int ni = i / 2;
			int nj = j / 2;
			other.positionField[i * other.width + j] = Vector2(0, 0);
			Vector2 pos = Position(ni * width + nj, unit);
			Vector2 diff = other.Coordinate(i * other.width + j, pos, unit);
			other.positionField[i * other.width + j] = diff;
		}
	}
}


void TensorField::OptimizePositionField(double unit)
{
	int diff[4][2] = {{0, 1}, {1, 0}, {-1, 0}, {0, -1}};
	for (int steps = 0; steps < 20; ++steps) {
		for (int di = 0; di < 2; ++di) {
			for (int dj = 0; dj < 2; ++dj) {
				for (int i = 0; i < height / 2; ++i) {
					for (int j = 0; j < width / 2; ++j) {
						int ni = i * 2 + di;
						int nj = j * 2 + dj;
						if (ni == height || nj == width)
							continue;
						Vector2 diffSum(0, 0);
						int count = 0;
						for (int l = 0; l < 4; ++l) {
							int mi = ni + diff[l][0];
							int mj = nj + diff[l][1];
							if (mj < 0 || mj >= width || mi < 0 || mi >= height)
								continue;

							Vector2 proposal = Position(mi * width + mj, unit);
							Vector2 proposalCoord = Coordinate(ni * width + nj, proposal, unit);

							count += 1;
							diffSum += proposalCoord;
							positionField[ni * width + nj] = Round(diffSum / count);
							diffSum = positionField[ni * width + nj] * count;
						}
					}
				}
			}
		}
	}
}


void TensorField::Downsample(TensorField& other)
{
	other.nRosy = nRosy;
	other.width = (width + 1) / 2;
	other.height = (height + 1) / 2;
	other.pixelStride = pixelStride * 2;
	other.boundaryDir.resize(other.width * other.height);
	memset(other.boundaryDir.data(), 0, sizeof(Vector2) * other.boundaryDir.size());
	other.boundaryWeight.resize(other.boundaryDir.size(), 0);
	other.soft1Rosy.resize(other.boundaryDir.size());
	memset(other.soft1Rosy.data(), 0, sizeof(Vector2) * other.boundaryDir.size());
	other.weight1Rosy.resize(other.boundaryDir.size(), 0);
	other.softNRosy.resize(other.boundaryDir.size());
	memset(other.softNRosy.data(), 0, sizeof(Vector2) * other.boundaryDir.size());
	other.weightNRosy.resize(other.boundaryDir.size(), 0);
	for (int i = 0; i < other.height; ++i) {
		for (int j = 0; j < other.width; ++j) {
			Vector2 bdir(0, 0);
			double bw = 0;
			Vector2 dirNRosy(0, 0);
			double d4w = 0;
			Vector2 dir1Rosy(0, 0);
			double d1w = 0;
			for (int di = 0; di < 2; ++di) {
				for (int dj = 0; dj < 2; ++dj) {
					int ni = i * 2 + di;
					int nj = j * 2 + dj;
					if (ni == height || nj == width)
						continue;
					double w = boundaryWeight[ni * width + nj];
					bdir += w * boundaryDir[ni * width + nj];
					bw += w;
					w = weight1Rosy[ni * width + nj];
					dir1Rosy += w * soft1Rosy[ni * width + nj];
					d1w += w;
					w = weightNRosy[ni * width + nj];
					dirNRosy = SumNRosy(dirNRosy, softNRosy[ni * width + nj], nRosy);
					d4w += w;
				}
			}
			if (bw > 0) {
				other.boundaryDir[i * other.width + j] = bdir / bw;
				other.boundaryWeight[i * other.width + j] = bw;
			}
			if (d4w > 0) {
				other.softNRosy[i * other.width + j] = dirNRosy / d4w;
				other.weightNRosy[i * other.width + j] = d4w;
			}
			if (d1w > 0) {
				other.soft1Rosy[i * other.width + j] = dir1Rosy / d1w;
				other.weight1Rosy[i * other.width + j] = d1w;
			}
		}
	}
}