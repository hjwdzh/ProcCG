#ifndef TERRAIN_TRIANGULATE_H_
#define TERRAIN_TRIANGULATE_H_

#include <Eigen/Core>
using Vector2 = Eigen::Vector2d;
using Vector3i = Eigen::Vector3i;
void Triangulate2D(std::vector<Vector2>& points, std::vector<Vector3i>& faces);

#endif