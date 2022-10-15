#ifndef TERRAIN_ROAD_GENERATOR_H_
#define TERRAIN_ROAD_GENERATOR_H_

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <unordered_set>
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
	int width{0}, height{0};

	void SaveToFile(FILE* fp);
	void LoadFromFile(FILE* fp);

	cv::Mat VisualizeRoadMap(std::unordered_set<int>* selectedCorners = 0,
		std::unordered_set<int>* selectedLines = 0) {
		cv::Mat vis = cv::Mat::zeros(height, width, CV_8UC3);
		cv::Scalar colors[3] = {{255, 255, 255}, {255, 200, 0}, {255, 165, 0}};
		for (int i = 0; i < polylines.size(); ++i) {
			cv::Scalar c = colors[layers[i]];
			if (selectedLines && selectedLines->count(i))
				c = cv::Scalar(0, 255, 0);
			for (int j = 0; j < polylines[i].size() - 1; ++j) {
				auto p0 = coordinates[polylines[i][j]];
				auto p1 = coordinates[polylines[i][j + 1]];
				cv::Point2f q0(p0[0], p0[1]);
				cv::Point2f q1(p1[0], p1[1]);
				cv::line(vis, q0, q1, c, widths[i]);
			}
		}
		if (selectedCorners) {
			for (auto& id : *selectedCorners) {
				cv::Point2f p(coordinates[id][0], coordinates[id][1]);
				cv::circle(vis, p, 2, cv::Scalar(0, 255, 255), 2);
			}
		}
		return vis;
	}
};

struct RoadSelector
{
	std::unordered_set<int> selectedCorners;
	std::unordered_set<int> selectedLines;
	std::unordered_set<int> selectedCornersConnect;
	std::unordered_set<int> selectedLinesConnect;
	void ClearSelection() {
		ClearConnectSelection();
		selectedCorners.clear();
		selectedLines.clear();
	}
	void ClearConnectSelection() {
		selectedCornersConnect.clear();
		selectedLinesConnect.clear();
	}
	void ConnectSelection()
	{
		if (!road) return;
		ClearConnectSelection();
		std::unordered_set<int> candidatePoints;
		for (auto& c : selectedCorners) {
			candidatePoints.insert(c);
		}
		for (auto& l : selectedLines) {
			candidatePoints.insert(road->polylines[l][0]);
			candidatePoints.insert(road->polylines[l].back());
		}
		std::unordered_set<int> seeds, seedLines;
		bool needInsert = true;
		while (!candidatePoints.empty()) {
			if (needInsert) {
				int t = *candidatePoints.begin();
				seeds.insert(t);
				candidatePoints.erase(t);
				if (candidatePoints.empty())
					break;
			}
			needInsert = true;
			std::unordered_map<int, double> pointDistances;
			std::unordered_map<int, std::pair<int, int> > prevPt;
			std::priority_queue<std::pair<double, int> > q;
			for (auto& s : seeds) {
				pointDistances[s] = 0;
				prevPt[s] = std::make_pair(-1, -1);
				q.push(std::make_pair(0, s));
			}
			while (!q.empty()) {
				auto info = q.top();
				q.pop();
				if (-info.first > pointDistances[info.second])
					continue;
				if (!seeds.count(info.second) && candidatePoints.count(info.second)) {
					int v = info.second;
					while (!seeds.count(v)) {
						if (candidatePoints.count(v))
							candidatePoints.erase(v);
						seeds.insert(v);
						auto prev = prevPt[v];
						seedLines.insert(prev.second);
						v = prev.first;
					}
					needInsert = false;
					break;
				}
				for (auto& j : road->arcs[info.second]) {
					double distance = 0;
					if (!selectedLines.count(j.second))
						distance = (road->coordinates[j.first] - road->coordinates[info.second]).norm();
					if (!pointDistances.count(j.first) || pointDistances[j.first] > distance - info.first) {
						pointDistances[j.first] = distance - info.first;
						prevPt[j.first] = std::make_pair(info.second, j.second);
						q.push(std::make_pair(-distance + info.first, j.first));
					}
				}
			}
		}
		for (auto& s : seeds) {
			selectedCornersConnect.insert(s);
		}
		for (auto& s : seedLines) {
			selectedLinesConnect.insert(s);
		}
		for (auto& s : selectedLines) {
			selectedLinesConnect.insert(s);
		}
	}
	void SelectCorner(Vector2 pt) {
		if (!road) return;
		int bestPt = -1;
		double bestLen = 1e30;
		for (int i = 0; i < road->coordinates.size(); ++i) {
			if (road->arcs[i].empty()) continue;
			double len = (road->coordinates[i] - pt).norm();
			if (len < bestLen) {
				bestLen = len;
				bestPt = i;
			}
		}
		if (bestLen < 10) {
			selectedCorners.insert(bestPt);
		}
	}
	void SelectArc(Vector2 pt) {
		if (!road) return;
		int bestPt = -1;
		double bestLen = 1e30;
		for (int i = 0; i < road->polylines.size(); ++i) {
			for (int j = 0; j < road->polylines[i].size(); ++j) {
				double len = (road->coordinates[road->polylines[i][j]] - pt).norm();
				if (len < bestLen) {
					bestLen = len;
					bestPt = i;
				}
			}
		}
		if (bestLen < 10) {
			selectedLines.insert(bestPt);
		}
	}
	Road* road{0};
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
	void GeneratePositionGraph(const RoadGenParam& param, FlowField& field,
		std::vector<Vector2>& positions, std::vector<Vector2i>& links);
	
	Road road;

private:
	void RasterizePolyline(const PolyLine& line, cv::Mat majorMask, cv::Mat minorMask, double thickness, const FlowField& field);
	void RoadGraphFromPolylines(const RoadGenParam& param, const std::vector<PolyLine>& polylines);
	cv::Mat DistanceField(const FlowField& field);
};
#endif