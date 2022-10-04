#ifndef TERRAIN_TENSORFIELD_H_
#define TERRAIN_TENSORFIELD_H_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

using Vector2 = Eigen::Vector2d;

inline Vector2 SumNRosy(const Vector2& p, const Vector2& d, int n)
{
	if (n == 2) {
		if (p.dot(d) > 0)
			return p + d;
		else
			return -p + d;
	}
	Vector2 p1(p[1], -p[0]);
	Vector2 p2(-p[0], -p[1]);
	Vector2 p3(-p[1], p[0]);
	double t0 = p.dot(d);
	double t1 = p1.dot(d);
	double t2 = p2.dot(d);
	double t3 = p3.dot(d);
	if (t0 > t1 && t0 > t2 && t0 > t3)
		return p + d;
	if (t1 > t2 && t1 > t3)
		return p1 + d;
	if (t2 > t3)
		return p2 + d;
	return p3 + d;
}

struct TensorField
{
	int width, height;
	int pixelStride;
	int nRosy;

	std::vector<Vector2> boundaryDir, softNRosy, soft1Rosy;
	std::vector<double> boundaryWeight, weightNRosy, weight1Rosy;
	std::vector<Vector2> orientationField, positionField;
	int Trace(Vector2 point, std::vector<Vector2>& polyline, double len, int isMajor, int directions) const;
	void Downsample(TensorField& other);
	void UpsampleOrientation(TensorField& other);
	void OptimizeOrientationField();
	void UpsamplePosition(TensorField& other, double unit);
	void OptimizePositionField(double unit);
	cv::Mat VisualizeTensorField();
	cv::Mat OrientationField();

	Vector2 Position(int i, double unit) const {
		int x = i % width;
		int y = i / width;
		Vector2 origin(x * pixelStride, y * pixelStride);
		Vector2 qx = orientationField[i];
		Vector2 qy(-qx[1], qx[0]);
		Vector2 t = positionField[i];
		return origin + unit * (qx * t[0] + qy * t[1]);
	}
	Vector2 Round(Vector2 diff, const Vector2 origin = Vector2(0, 0)) const
	{
		for (int k = 0; k < 2; ++k) {
			diff[k] -= (int)diff[k];
			while (diff[k] - origin[k] < -0.5) diff[k] += 1;
			while (diff[k] - origin[k] > 0.5) diff[k] -= 1;
		}
		return diff;
	}
	Vector2 Coordinate(int i, const Vector2& p, double unit) const {
		int x = i % width;
		int y = i / width;
		Vector2 origin(x * pixelStride, y * pixelStride);
		Vector2 qx = orientationField[i];
		Vector2 qy(-qx[1], qx[0]);
		Vector2 diff = (p - origin) / unit;
		diff = Vector2(diff.dot(qx), diff.dot(qy));
		return Round(diff, positionField[i]);
	}
};

#endif