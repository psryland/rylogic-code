//*********************************************
// Collision
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Lightweight topology-backed polytope views used by mixed polytope SAT adapters.
#pragma once
#include "pr/collision/forward.h"
#include "pr/collision/shape_polytope.h"
#include "pr/collision/shape_box.h"
#include "pr/collision/shape_triangle.h"

namespace pr::collision::polytope
{
	template <int VertCount, int FaceCount, int EdgeCount>
	struct ShapeView
	{
		ShapePolytope m_shape;
		std::array<v4, VertCount> m_verts;
		std::array<ShapePolyFace, FaceCount> m_faces;
		std::array<ShapePolyEdge, EdgeCount> m_edges;

		ShapeView(m4x4 const& shape_to_root, MaterialId material_id, Shape::EFlags flags)
			: m_shape(shape_to_root, material_id, flags)
			, m_verts()
			, m_faces()
			, m_edges()
		{}

		void SetFace(int face_index, int i0, int i1, int i2, Plane plane, ShapePolytope::EFaceFlags flags)
		{
			auto& face = m_shape.face(face_index);
			face.m_plane = plane;
			face.m_index[0] = i0;
			face.m_index[1] = i1;
			face.m_index[2] = i2;
			face.m_flags = flags;
		}
		void SetFace(int face_index, int i0, int i1, int i2, ShapePolytope::EFaceFlags flags)
		{
			SetFace(face_index, i0, i1, i2, Plane::FromTriangle(m_shape.vertex(i0), m_shape.vertex(i1), m_shape.vertex(i2)), flags);
		}
		void SetEdge(int edge_index, int v0, int v1, int face0, int face1, ShapePolytope::EEdgeFlags flags = ShapePolytope::EEdgeFlags::None)
		{
			auto& edge = m_shape.edge(edge_index);
			auto dir = m_shape.vertex(v1) - m_shape.vertex(v0);
			auto len_sq = LengthSq(dir);

			edge.m_direction = len_sq > Sqr(math::tiny<float>) ? (dir / Sqrt(len_sq)).w0() : v4::Zero();
			edge.m_v0 = v0;
			edge.m_v1 = v1;
			edge.m_face0 = face0;
			edge.m_face1 = face1;
			edge.m_flags = SetBits(flags, ShapePolytope::EEdgeFlags::IgnoreEdgeAxes, len_sq <= Sqr(math::tiny<float>));
			edge.m_pad[0] = 0;
			edge.m_pad[1] = 0;
			edge.m_pad[2] = 0;
		}

		void Complete()
		{
			m_shape.Complete(VertCount, FaceCount, EdgeCount);
			assert("ShapePolytope view layout mismatch" && m_shape.vert_beg() == m_verts.data());
			assert("ShapePolytope view layout mismatch" && m_shape.face_beg() == m_faces.data());
			assert("ShapePolytope view layout mismatch" && m_shape.edge_beg() == m_edges.data());
		}
	};

	// Box-as-polytope
	using BoxView = ShapeView<8, 12, 12>;
	inline BoxView MakeView(ShapeBox const& box)
	{
		auto view = BoxView(box.m_base.m_s2r, box.m_base.m_material_id, box.m_base.m_flags);
		auto r = box.m_radius;

		view.m_verts = {
			v4{-r.x, -r.y, -r.z, 1}, v4{+r.x, -r.y, -r.z, 1},
			v4{-r.x, +r.y, -r.z, 1}, v4{+r.x, +r.y, -r.z, 1},
			v4{-r.x, -r.y, +r.z, 1}, v4{+r.x, -r.y, +r.z, 1},
			v4{-r.x, +r.y, +r.z, 1}, v4{+r.x, +r.y, +r.z, 1},
		};
		view.Complete();

		constexpr auto TestAxis = ShapePolytope::EFaceFlags::None;
		constexpr auto IgnoreAxis = ShapePolytope::EFaceFlags::IgnoreFaceAxis;

		view.SetFace(0, 1, 3, 7, TestAxis);   // +X
		view.SetFace(1, 1, 7, 5, IgnoreAxis); // +X duplicate triangulation
		view.SetFace(2, 0, 4, 6, IgnoreAxis); // -X duplicate axis
		view.SetFace(3, 0, 6, 2, IgnoreAxis); // -X duplicate triangulation
		view.SetFace(4, 2, 6, 7, TestAxis);   // +Y
		view.SetFace(5, 2, 7, 3, IgnoreAxis); // +Y duplicate triangulation
		view.SetFace(6, 0, 1, 5, IgnoreAxis); // -Y duplicate axis
		view.SetFace(7, 0, 5, 4, IgnoreAxis); // -Y duplicate triangulation
		view.SetFace(8, 4, 5, 7, TestAxis);   // +Z
		view.SetFace(9, 4, 7, 6, IgnoreAxis); // +Z duplicate triangulation
		view.SetFace(10, 0, 2, 3, IgnoreAxis); // -Z duplicate axis
		view.SetFace(11, 0, 3, 1, IgnoreAxis); // -Z duplicate triangulation

		view.SetEdge(0, 0, 1,  6, 11);
		view.SetEdge(1, 1, 3,  0, 11);
		view.SetEdge(2, 2, 3,  5, 10);
		view.SetEdge(3, 0, 2,  3, 10);
		view.SetEdge(4, 4, 5,  7,  8);
		view.SetEdge(5, 5, 7,  1,  8);
		view.SetEdge(6, 6, 7,  4,  9);
		view.SetEdge(7, 4, 6,  2,  9);
		view.SetEdge(8, 0, 4,  2,  7);
		view.SetEdge(9, 1, 5,  1,  6);
		view.SetEdge(10, 2, 6,  3,  4);
		view.SetEdge(11, 3, 7,  0,  5);
		return view;
	}

	// Triangle-as-polytope
	using TriangleView = ShapeView<3, 5, 6>;
	inline TriangleView MakeView(ShapeTriangle const& tri)
	{
		auto view = TriangleView(tri.m_base.m_s2r, tri.m_base.m_material_id, tri.m_base.m_flags);
		view.m_verts = { tri.m_v.x.w1(), tri.m_v.y.w1(), tri.m_v.z.w1() };
		view.Complete();

		auto normal = tri.m_v.w.w0();
		auto has_normal = LengthSq(normal) > Sqr(math::tiny<float>);
		auto front_flags = has_normal ? ShapePolytope::EFaceFlags::None : ShapePolytope::EFaceFlags::IgnoreFaceAxis;
		auto ignored_flags = ShapePolytope::EFaceFlags::IgnoreFaceAxis;

		view.SetFace(0, 0, 1, 2, Plane(view.m_verts[0], +normal), front_flags);
		view.SetFace(1, 0, 2, 1, Plane(view.m_verts[0], -normal), ignored_flags);

		for (auto i = 0; i != 3; ++i)
		{
			auto i0 = i;
			auto i1 = (i + 1) % 3;
			auto edge = (view.m_verts[i1] - view.m_verts[i0]).w0();
			auto side_normal = Normalise(Cross(edge, normal), v4::Zero());
			view.SetFace(2 + i, i0, i1, static_cast<uint32_t>((i + 2) % 3), Plane(view.m_verts[i0], side_normal), ignored_flags);
		}

		auto edge_flags = has_normal ? ShapePolytope::EEdgeFlags::None : ShapePolytope::EEdgeFlags::IgnoreEdgeAxes;
		view.SetEdge(0, 0, 1, 0, 2, edge_flags);
		view.SetEdge(1, 0, 1, 1, 2, edge_flags);
		view.SetEdge(2, 1, 2, 0, 3, edge_flags);
		view.SetEdge(3, 1, 2, 1, 3, edge_flags);
		view.SetEdge(4, 2, 0, 0, 4, edge_flags);
		view.SetEdge(5, 2, 0, 1, 4, edge_flags);
		return view;
	}
}
