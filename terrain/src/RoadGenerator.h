#ifndef TERRAIN_ROAD_GENERATOR_H_
#define TERRAIN_ROAD_GENERATOR_H_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

#include "FlowField.h"

using Vector2 = Eigen::Vector2d;
using Vector2i = Eigen::Vector2i;
using Vector3i = Eigen::Vector3i;
using PolyLine = std::vector<Vector2>;

struct Road
{
	std::vector<Vector2> coordinates;					// end point coordinates
	std::vector<std::unordered_map<int, int> > arcs;	// arc end points as link graph end_i: <end_j, arcID_ij>
	std::vector<std::vector<int> > polylines;			// arrays of endpoint indices as a line strip
	std::vector<int> layers;							// layers of the road (size=polylines), minor = 0
	std::vector<double> widths;							// road widths (size=polylines)
	int width, height;

	cv::Mat VisualizeRoadMap() {
		cv::Mat vis = cv::Mat::zeros(height, width, CV_8UC3);
		cv::Scalar colors[3] = {{255, 0, 0}, {0, 255, 0}, {0, 0, 255}};
		for (int i = 0; i < polylines.size(); ++i) {
			for (int j = 0; j < polylines[i].size() - 1; ++j) {
				auto p0 = coordinates[polylines[i][j]];
				auto p1 = coordinates[polylines[i][j + 1]];
				cv::Point2f q0(p0[0], p0[1]);
				cv::Point2f q1(p1[0], p1[1]);
				cv::line(vis, q0, q1, colors[layers[i]], widths[i]);
			}
		}
		return vis;
	}
};

class RoadGenerator
{
public:
	struct RoadGenParam
	{
		double minSpace;
		int layer;
	};
	void GenerateRoad(const RoadGenParam& param, const FlowField& field);
	void UpsampleRoadGraph(int fromLayer, int toLayer, int space = 2);
	void SetRoadWidth(const std::vector<double>& w) {
		road.widths.resize(road.layers.size());
		for (int i = 0; i < road.widths.size(); ++i) {
			road.widths[i] = w[road.layers[i]];
		}
	}
	Road road;

private:
	void RasterizePolyline(const PolyLine& line, cv::Mat majorMask, cv::Mat minorMask, double thickness, const FlowField& field);
	void RoadGraphFromPolylines(const RoadGenParam& param, const std::vector<PolyLine>& polylines);

	cv::Mat DistanceField(const FlowField& field);
};
#endif