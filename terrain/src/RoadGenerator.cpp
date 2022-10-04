#include "RoadGenerator.h"

#include "DisajointTree.h"
#include <unordered_set>

void RoadGenerator::RasterizePolyline(const PolyLine& line, cv::Mat majorMask, cv::Mat minorMask,
	double thickness, const FlowField& field)
{
	// BFS to expand the polyline to specify its neighborhood in major eigen field and minor eigen field
	std::unordered_map<int, Vector2> visited;
	// put initial the seeds into the queue for endpoints of the strip
	std::queue<Vector3i> q;
	for (int i = 0; i < line.size() - 1; ++i) {
		int x = line[i][0];
		int y = line[i][1];
		int id = y * majorMask.cols + x;
		if (visited.count(id))
			continue;
		Vector2 refD = (line[i + 1] - line[i]).normalized();
		Vector2 tarD = field.tensorFields_[0].orientationField[id];
		if (std::abs(tarD.dot(refD)) < 0.5) {
			visited[id] = Vector2(-tarD[1], tarD[0]);
			minorMask.at<unsigned char>(y, x) = 0;
		} else {
			visited[id] = tarD;
			majorMask.at<unsigned char>(y, x) = 0;
		}
		q.push(Vector3i(x, y, 0));
	}
	int dx[] = {-1, 0, 1, 0};
	int dy[] = {0, 1, 0, -1};
	while (!q.empty()) {
		auto info = q.front();
		q.pop();
		int id0 = info[1] * majorMask.cols + info[0];
		Vector2 refD = visited[id0];
		for (int i = 0; i < 4; ++i) {
			int nx = info[0] + dx[i];
			int ny = info[1] + dy[i];
			if (nx < 0 || ny < 0 || nx >= majorMask.cols || ny >= majorMask.rows) {
				continue;
			}
			int id = ny * majorMask.cols + nx;
			if (visited.count(id))
				continue;
			// determine whether to draw at major or minor eigen field by checking direction
			// for [0, 0.5 * thickness], set it as 0 (no road)
			// for [0.5 * thickness, thickness] set it as 1 (should not seed it) 
			Vector2 tarD = field.tensorFields_[0].orientationField[id];
			if (std::abs(refD.dot(tarD)) < 0.5) {
				visited[id] = Vector2(-tarD[1], tarD[0]);
				if (info[2] < thickness * 0.3)
					minorMask.at<unsigned char>(ny, nx) = 0;
				else if (minorMask.at<unsigned char>(ny, nx) > 0)
					minorMask.at<unsigned char>(ny, nx) = 1;
			} else {
				visited[id] = tarD;
				if (info[2] < thickness * 0.3)
					majorMask.at<unsigned char>(ny, nx) = 0;
				else if (majorMask.at<unsigned char>(ny, nx) > 0)
					majorMask.at<unsigned char>(ny, nx) = 1;
			}
			if (info[2] + 1 < thickness)
				q.push(Vector3i(nx, ny, info[2] + 1));
		}
	}
}

cv::Mat RoadGenerator::DistanceField(const FlowField& field)
{
	cv::Mat distMask = cv::Mat::zeros(field.validMask_.rows, field.validMask_.cols, CV_32F);
	std::queue<int> q;
	for (int i = 0; i < distMask.rows; ++i) {
		for (int j = 0; j < distMask.cols; ++j) {
			if (field.validMask_.at<unsigned char>(i, j) < 2) {
				distMask.at<float>(i, j) = 0;
				q.push(i * distMask.cols + j);
			}
			else
				distMask.at<float>(i, j) = 1e30;
		}
	}
	while (!q.empty()) {
		int dx[] = {-1, 0, 1, 0};
		int dy[] = {-1, 0, 1, 0};
		int t = q.front();
		q.pop();
		int x = t % distMask.cols;
		int y = t / distMask.cols;
		for (int j = 0; j < 4; ++j) {
			int nx = x + dx[j];
			int ny = y + dy[j];
			if (nx < 0 || nx >= distMask.cols || ny < 0 || ny >= distMask.rows)
				continue;
			if (distMask.at<float>(ny, nx) < 1e20)
				continue;
			distMask.at<float>(ny, nx) = distMask.at<float>(y, x) + 1;
			q.push(ny * distMask.cols + nx);
		}
	}
	return distMask;
}

void RoadGenerator::GenerateRoad(const RoadGenParam& param, const FlowField& field)
{
	// initialize the minor and major eigen field mask
	// 0: no road
	// 1: potential road, do not allow to seed
	// 2: allow to seed
	cv::Mat majorMask = field.validMask_.clone();
	cv::Mat minorMask = field.validMask_.clone();
	for (int i = 0; i < majorMask.rows; ++i) {
		for (int j = 0; j < minorMask.rows; ++j) {
			if (majorMask.at<unsigned char>(i, j) < 2)
				majorMask.at<unsigned char>(i, j) = 0;
			else
				majorMask.at<unsigned char>(i, j) = 2;
			if (minorMask.at<unsigned char>(i, j) < 2)
				minorMask.at<unsigned char>(i, j) = 0;
			else
				minorMask.at<unsigned char>(i, j) = 2;
		}
	}

	// prioritize the seeds given distance to the boundary
	std::vector<std::pair<double, int> > validIndices;
	cv::Mat distMask = DistanceField(field);
	for (int i = 0; i < distMask.rows; ++i) {
		for (int j = 0; j < distMask.cols; ++j) {
			if (distMask.at<float>(i, j) > param.minSpace * 0.5) {
				validIndices.emplace_back(distMask.at<float>(i, j), i * distMask.cols + j);
			}
		}
	}
	std::sort(validIndices.begin(), validIndices.end());
	std::vector<Vector3i> validCoords;
	for (auto& ind : validIndices) {
		int i = ind.second / distMask.cols;
		int j = ind.second % distMask.cols;
		if (majorMask.at<unsigned char>(i, j) > 0) {
			validCoords.push_back(Vector3i(j, i, 1));
		}
		if (minorMask.at<unsigned char>(i, j) > 0) {
			validCoords.push_back(Vector3i(j, i, 0));
		}
	}

	// sample polylines
	std::vector<PolyLine> polylines;
	for (int seedId = 0; seedId < validCoords.size(); ++seedId) {
		int isMajor = validCoords[seedId][2];
		bool expand = true;
		Vector2 coord(validCoords[seedId][0], validCoords[seedId][1]);
		// check if seed is valid
		if ((isMajor && majorMask.at<unsigned char>(validCoords[seedId][1], validCoords[seedId][0]) < 2) ||
			(!isMajor && minorMask.at<unsigned char>(validCoords[seedId][1], validCoords[seedId][0]) < 2))
			expand = false;
		if (expand) {
			// trace a curve from flow field
			PolyLine line;
			int s = field.tensorFields_[0].Trace(coord, line, 1e6, isMajor, 2);
			if (line.size() < 2)
				continue;

			// shorten it until all endpoints are at valid area
			auto isValid = [&](int i) {
				int x = line[i][0];
				int y = line[i][1];
				Vector2 dir = (i == 0) ? (line[i + 1] - line[i]).normalized() : (line[i] - line[i - 1]).normalized();
				int isMajor = std::abs(field.tensorFields_[0].orientationField[y * majorMask.cols + x].dot(dir)) > 0.5;
				if (field.validMask_.at<unsigned char>(y, x) != 2)
					return -1;
				return (isMajor && majorMask.at<unsigned char>(y, x) > 0 ||
					!isMajor && minorMask.at<unsigned char>(y, x) > 0) ? 1 : 0;
			};

			int l = s, r = s;
			while (l > 0 && isValid(l) > 0) l -= 1;
			while (r < line.size() && isValid(r) > 0) r += 1;
			int top = 0;
			for (int i = l; i < r; ++i) {
				line[top++] = line[i];
			}
			if (top < 1)
				continue;
			line.resize(top);

			// update the major/minor eigen field mask
			polylines.push_back(line);
			RasterizePolyline(line, majorMask, minorMask, param.minSpace, field);
		}
	}

	// graph from polylines
	road.width = majorMask.cols;
	road.height = majorMask.rows;
	RoadGraphFromPolylines(param, polylines);
}

void RoadGenerator::RoadGraphFromPolylines(const RoadGenParam& param, const std::vector<PolyLine>& polylines)
{
	// step1: extract polyline endpoints as node list and record it in an image
	std::vector<std::vector<std::pair<int, int> > > vlists(road.height * road.width);
	std::vector<Vector2> nodeCoords;
	std::map<std::pair<int, int>, int> line2node;
	for (int i = 0; i < polylines.size(); ++i) {
		for (int j = 0; j < polylines[i].size(); ++j) {
			int y = polylines[i][j][1];
			int x = polylines[i][j][0];
			vlists[y * road.width + x].push_back(std::make_pair(i, j));
			line2node[std::make_pair(i, j)] = nodeCoords.size();
			nodeCoords.push_back(polylines[i][j]);
		}
	}

	// for pixel with multiple end points from different polylines, merge end points
	DisajointTree tree(nodeCoords.size());
	for (int i = 0; i < road.height; ++i) {
		for (int j = 0; j < road.width; ++j) {
			if (vlists[i * road.width + j].size() < 2)
				continue;
			std::vector<int> vs;
			for (auto& info : vlists[i * road.width + j]) {
				int v = line2node[info];
				vs.push_back(v);
			}
			for (auto& i0 : vs) {
				for (auto& i1 : vs) {
					tree.Merge(i0, i1);
				}
			}
		}
	}
	// build a link graph for merged nodes
	std::vector<std::unordered_set<int> > vlinks(nodeCoords.size());
	for (int i = 0; i < polylines.size(); ++i) {
		for (int j = 0; j < polylines[i].size() - 1; ++j) {
			int v0 = tree.Parent(line2node[std::make_pair(i, j)]);
			int v1 = tree.Parent(line2node[std::make_pair(i, j + 1)]);
			vlinks[v0].insert(v1);
			vlinks[v1].insert(v0);
		}
	}
	// extract link graph as arcs
	std::vector<std::vector<int> > arcs;
	std::vector<double> arcsLen;
	std::vector<int> visited(vlinks.size());
	for (int i = 0; i < vlinks.size(); ++i) {
		if (visited[i] || vlinks[i].size() != 2)
			continue;
		std::list<int> v;
		for (int j = 0; j < 2; ++ j) {
			int prev = *vlinks[i].begin();
			if (j == 0) {
				v.push_back(i);
			} else {
				auto it = vlinks[i].begin();
				it++;
				prev = *it;
			}
			int current = i;
			while (vlinks[current].size() == 2) {
				auto it = vlinks[current].begin();
				int v0 = *it++;
				int v1 = *it;
				int next = v0 + v1 - prev;
				if (j == 0)
					v.push_back(next);
				else
					v.push_front(next);
				prev = current;
				current = next;
			}
		}
		for (auto& vl : v)
			visited[vl] = 1;
		if (v.size() < 2) continue;
		arcs.push_back({});
		arcs.back().insert(arcs.back().end(), v.begin(), v.end());
		double l = 0;
		for (int j = 0; j + 1 < arcs.back().size(); ++j) {
			l += (nodeCoords[arcs.back()[j + 1]] - nodeCoords[arcs.back()[j]]).norm();
		}
		arcsLen.push_back(l);
	}
	// merge short arcs
	tree = DisajointTree(nodeCoords.size());
	std::vector<int> ranks(vlinks.size(), 1);
	std::vector<Vector2> mergedCoords = nodeCoords;
	int top = 0;
	for (int i = 0; i < arcsLen.size(); ++i) {
		if (arcsLen[i] < param.minSpace * 0.1 && vlinks[arcs[i][0]].size() > 2 && vlinks[arcs[i].back()].size() > 2) {
			int v0 = tree.Parent(arcs[i][0]);
			int v1 = tree.Parent(arcs[i].back());
			if (v0 == v1)
				continue;
			tree.MergeFromTo(v1, v0);
			mergedCoords[v0] = (mergedCoords[v0] * ranks[v0] + mergedCoords[v1] * ranks[v1]) / (ranks[v0] + ranks[v1]);
			ranks[v0] += ranks[v1];
		}
	}
	for (int i = 0; i < vlinks.size(); ++i) {
		for (auto& j : vlinks[i]) {
			if (vlinks[i].size() > 2 && vlinks[j].size() > 2) {
				int v0 = tree.Parent(i);
				int v1 = tree.Parent(j);
				if (v0 == v1)
					continue;
				tree.MergeFromTo(v1, v0);
				mergedCoords[v0] = (mergedCoords[v0] * ranks[v0] + mergedCoords[v1] * ranks[v1]) / (ranks[v0] + ranks[v1]);
				ranks[v0] += ranks[v1];
			}
		}
	}

	// merge single-degree arcs
	auto isSingleNode = [&](int v) {
		if (vlinks[v].size() > 1)
			return false;
		int x = nodeCoords[v][0];
		int y = nodeCoords[v][1];
		if (x <= 1 || y <= 1 || x >= road.width - 2 || y >= road.height - 2)
			return false;
		//following if allows road to reach invalid boundaries
		//if (distMask.at<float>(y, x) < 3)
		//	return false;
		return true;
	};
	std::set<std::pair<int, int> > arcsSet;
	top = 0;
	for (int i = 0; i < arcs.size(); ++i) {
		int v0 = tree.Parent(arcs[i][0]);
		int v1 = tree.Parent(arcs[i].back());
		if (v0 > v1) std::swap(v0, v1);
		if (!arcsSet.count(std::make_pair(v0, v1)) && !isSingleNode(v0) && !isSingleNode(v1)) {
			arcs[top++] = arcs[i];
		}
		arcsSet.insert(std::make_pair(v0, v1));
	}
	arcs.resize(top);

	// slightly move the arc geometry with merged end points
	std::vector<Vector2> finalCoords = nodeCoords;
	for (int i = 0; i < arcs.size(); ++i) {
		Vector2 offset0 = mergedCoords[tree.Parent(arcs[i][0])] - nodeCoords[arcs[i][0]];
		Vector2 offset1 = mergedCoords[tree.Parent(arcs[i].back())] - nodeCoords[arcs[i].back()];
		for (int j = 0; j < arcs[i].size(); ++j) {
			double w = (j + 0.0) / (arcs[i].size() - 1);
			finalCoords[tree.Parent(arcs[i][j])] = nodeCoords[arcs[i][j]] + offset0 * (1 - w) + offset1 * w;
		}
	}
	// reindex the arc geometry and save it as the road
	top = 0;
	std::unordered_map<int, int> compactIndices;
	for (auto& arc : arcs) {
		for (auto& a : arc) {
			int v = tree.Parent(a);
			if (!compactIndices.count(v)) {
				compactIndices[v] = top;
				nodeCoords[top++] = finalCoords[v];
			}
			a = compactIndices[v];
		}
	}
	nodeCoords.resize(top);
	Road& r = road;
	r.coordinates = nodeCoords;
	r.polylines = arcs;
	r.layers.resize(arcs.size(), param.layer);
	r.arcs.resize(nodeCoords.size());
	for (int i = 0; i < arcs.size(); ++i) {
		int v0 = arcs[i][0];
		int v1 = arcs[i].back();
		r.arcs[v0][v1] = i;
		r.arcs[v1][v0] = i;
	}
}

void RoadGenerator::UpsampleRoadGraph(int fromLayer, int toLayer, int space)
{
	std::vector<int> visited(road.coordinates.size(), 0);
	std::vector<int> visitedPolylines(road.polylines.size(), 0);
	std::vector<int> seeds;
	for (int i = 0; i < road.coordinates.size(); ++i) {
		if (!road.arcs[i].empty())
			seeds.push_back(i);
	}
	std::random_shuffle(seeds.begin(), seeds.end());
	for (int i = 0; i < seeds.size(); ++i) {
		if (visited[seeds[i]] != 0)
			continue;
		int version = i + 1;
		std::queue<std::pair<int, int> > q;
		q.push(std::make_pair(seeds[i], 0));
		std::vector<int> seedList;
		std::vector<int> polylineList;

		seedList.push_back(seeds[i]);
		for (auto& nj : road.arcs[seeds[i]]) {
			if (road.layers[nj.second] != fromLayer)
				continue;
			int prev = seeds[i];
			int current = nj.first;
			if (visitedPolylines[nj.second] != 0)
				continue;
			polylineList.push_back(nj.second);
			while (true) {
				q.push(std::make_pair(current, 0));
				seedList.push_back(current);
				Vector2 dir0 = road.coordinates[current] - road.coordinates[prev];
				double bestDot = -1;
				int next = -1;
				int nextArc = -1;
				for (auto& nk : road.arcs[current]) {
					if (road.layers[nk.second] != fromLayer)
						continue;
					Vector2 dir1 = (road.coordinates[nk.first] - road.coordinates[current]);
					if (dir1.dot(dir0) > bestDot && visitedPolylines[nk.second] == 0) {
						bestDot = dir1.dot(dir0);
						next = nk.first;
						nextArc = nk.second;
					}
				}
				if (bestDot > 0.7) {
					prev = current;
					current = next;
					polylineList.push_back(nextArc);
				} else {
					break;
				}
			}
		}
		int count = 0;
		for (auto& s : seedList) {
			if (visited[s] > 0)
				count += 1;
		}
		if (count > seedList.size() * 0.6) {
			continue;
		}
		for (auto& s : seedList)
			visited[s] = -version;
		for (auto& p : polylineList)
			visitedPolylines[p] = 1;
		while (!q.empty()) {
			auto info = q.front();
			q.pop();
			for (auto& nj : road.arcs[info.first]) {
				if (road.layers[nj.second] != fromLayer)
					continue;
				if (std::abs(visited[nj.first]) != version) {
					if (visited[nj.first] < 0)
						visited[nj.first] = -version;
					else
						visited[nj.first] = version;
					if (info.second + 1 < space) {
						q.push(std::make_pair(nj.first, info.second + 1));
					}
				}
			}
		}
	}
	for (int i = 0; i < visitedPolylines.size(); ++i) {
		if (visitedPolylines[i] == 1) {
			road.layers[i] = toLayer;
		}
	}
}