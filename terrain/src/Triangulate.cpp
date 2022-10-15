#include "Triangulate.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Projection_traits_xy_3.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Triangulation_vertex_base_with_info_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_vertex_base_with_info_2<unsigned, K> Vb;
typedef CGAL::Triangulation_data_structure_2<Vb> Tds;
typedef CGAL::Delaunay_triangulation_2<K,Tds> Delaunay;
typedef Delaunay::Point   DPoint;

void Triangulate2D(std::vector<Vector2>& points, std::vector<Vector3i>& faces) {
	Delaunay dt;
	for (int i = 0; i < points.size(); ++i) {
		auto v = points[i];
		auto vh = dt.insert(DPoint(v[0], v[1]));
		vh->info() = i;
	}

	for (auto f : dt.finite_face_handles()) {
		Vector3i face;
		for (int i = 0; i < 3; ++i) {
			face[i] = f->vertex(i)->info();
		}
		faces.push_back(face);
	}
}
