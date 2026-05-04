//*********************************************
// Collision
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include <algorithm>
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/container/byte_data.h"
#include "pr/geometry/convex_hull.h"

namespace pr::collision
{
	struct ShapePolytope
	{
		// A polytope is a convex triangle mesh with explicit face and edge topology.
		// All polytopes are assumed to be convex.

		// Mesh structure types
		using Idx = uint8_t;
		enum class EFaceFlags : uint32_t
		{
			None = 0,
			IgnoreFaceAxis = 1 << 0,
		};
		enum class EEdgeFlags : uint32_t
		{
			None = 0,
			IgnoreEdgeAxes = 1 << 0,
		};
		struct Face
		{
			Plane m_plane;
			uint32_t m_index[3];
			EFaceFlags m_flags;
		};
		struct Edge
		{
			v4 m_direction;
			uint32_t m_v0;
			uint32_t m_v1;
			uint32_t m_face0;
			uint32_t m_face1;
			EEdgeFlags m_flags;
			uint32_t m_pad[3];
		};

		Shape m_base;
		int m_vert_count;
		int m_face_count;
		int m_edge_count;
		int pad[1];
		
		// Memory layout. The following data is expected to follow this struct in memory, but is not actually part of the struct.
		// v4   m_vert[m_vert_count]
		// Face m_face[m_face_count]
		// Edge m_edge[m_edge_count]
		// byte padding[] to make the total size a multiple of 16 bytes

		explicit ShapePolytope(m4x4 const& shape_to_root = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			: m_base(EShape::Polytope, sizeof(ShapePolytope), shape_to_root, BBox::Reset(), material_id, flags)
			, m_vert_count()
			, m_face_count()
			, m_edge_count()
			, pad()
		{
			// Careful: We can't be sure of what follows this object in memory.
			// The polytope data that belongs to this array may not be there yet.
			// Defer calculating the bounding box until the caller calls 'Complete()'.
		}
		void Complete(int vert_count, int face_count, int edge_count)
		{
			m_vert_count = vert_count;
			m_face_count = face_count;
			m_edge_count = edge_count;

			m_base.m_size = s_cast<int>(PadTo(sizeof(ShapePolytope) +
				sizeof(v4) * m_vert_count +
				sizeof(Face) * m_face_count +
				sizeof(Edge) * m_edge_count,
				16));
		
			// Find the bounding box
			m_base.m_bbox = CalcBBox(*this);
		}

		// Vertex accessors
		v4 const* vert_beg() const              { return type_ptr<v4>(this + 1); }
		v4*       vert_beg()                    { return type_ptr<v4>(this + 1); }
		v4 const* vert_end() const              { return vert_beg() + m_vert_count; }
		v4*       vert_end()                    { return vert_beg() + m_vert_count; }
		v4 const& vertex(std::size_t idx) const { return vert_beg()[idx]; }
		v4&       vertex(std::size_t idx)       { return vert_beg()[idx]; }
		auto verts() const { return std::ranges::subrange(vert_beg(), vert_end()); }

		// Face accessors
		Face const* face_beg() const    { return type_ptr<Face>(vert_end()); }
		Face*       face_beg()          { return type_ptr<Face>(vert_end()); }
		Face const* face_end() const    { return face_beg() + m_face_count; }
		Face*       face_end()          { return face_beg() + m_face_count; }
		Face const& face(int idx) const { return face_beg()[idx]; }
		Face&       face(int idx)       { return face_beg()[idx]; }
		auto faces() const { return std::ranges::subrange(face_beg(), face_end()); }

		// Edge accessors
		Edge const* edge_beg() const    { return type_ptr<Edge>(face_end()); }
		Edge*       edge_beg()          { return type_ptr<Edge>(face_end()); }
		Edge const* edge_end() const    { return edge_beg() + m_edge_count; }
		Edge*       edge_end()          { return edge_beg() + m_edge_count; }
		Edge const& edge(int idx) const { return edge_beg()[idx]; }
		Edge&       edge(int idx)       { return edge_beg()[idx]; }
		auto edges() const { return std::ranges::subrange(edge_beg(), edge_end()); }

		// Conversion
		operator Shape const&() const
		{
			return m_base;
		}
		operator Shape&()
		{
			return m_base;
		}
		operator Shape const*() const
		{
			return &m_base;
		}
		operator Shape*()
		{
			return &m_base;
		}
	};
	static_assert(ShapeType<ShapePolytope>);
	static_assert((sizeof(ShapePolytope) & 0xf) == 0);
	static_assert((sizeof(ShapePolytope::Face) & 0xf) == 0);
	static_assert((sizeof(ShapePolytope::Edge) & 0xf) == 0);

	using PolyIdx       = ShapePolytope::Idx;
	using ShapePolyFace = ShapePolytope::Face;
	using ShapePolyEdge = ShapePolytope::Edge;

	inline bool IgnoreFaceAxis(ShapePolyFace const& face)
	{
		return AllSet(face.m_flags, ShapePolytope::EFaceFlags::IgnoreFaceAxis);
	}
	inline bool IgnoreEdgeAxes(ShapePolyEdge const& edge)
	{
		return AllSet(edge.m_flags, ShapePolytope::EEdgeFlags::IgnoreEdgeAxes);
	}

	// Return the bounding box for a polytope
	inline BBox pr_vectorcall CalcBBox(ShapePolytope const& shape)
	{
		auto bb = BBox::Reset();
		for (v4 v : shape.verts())
			Grow(bb, v);

		return bb;
	}

	// Return the volume of the polytope
	inline float CalcVolume(ShapePolytope const& shape)
	{
		auto volume = 0.0f;
		for (auto const& f : shape.faces())
		{
			auto a = shape.vertex(f.m_index[0]);
			auto b = shape.vertex(f.m_index[1]);
			auto c = shape.vertex(f.m_index[2]);
			volume += Triple(a, b, c); // Triple product is volume x 6
		}
		return volume / 6.0f;
	}

	// Return the centre of mass position of the polytope
	inline v4 CalcCentreOfMass(ShapePolytope const& shape)
	{
		assert("Centre of mass is undefined for an empty polytope" && shape.m_vert_count != 0 && shape.m_face_count != 0);

		auto com = v4::Zero();
		auto volume = 0.0f;
		for (auto const& f : shape.faces())
		{
			auto a = shape.vertex(f.m_index[0]);
			auto b = shape.vertex(f.m_index[1]);
			auto c = shape.vertex(f.m_index[2]);
			auto vol_x6 = Triple(a, b, c); // Triple product is volume x 6
			com	+= vol_x6 * (a + b + c);    // Divide by 4 at end
			volume += vol_x6;
		}
		volume *= 4.0f;

		// If the polytope is degenerate, use the weighted average vertex positions
		if (FEql(volume, 0.f))
		{
			com = v4::Zero();
			for (auto const& v : shape.verts()) com += v;
			volume = 1.0f * shape.m_vert_count;
		}

		return com.w0() / volume;
	}

	// Return a support vertex for a polytope
	inline v4 pr_vectorcall SupportVertex(ShapePolytope const& shape, v4 direction, int hint_vert_id, int& sup_vert_id)
	{
		assert("Invalid polytope" && shape.m_vert_count != 0);
		assert("Invalid hint vertex index" && hint_vert_id >= 0 && hint_vert_id < shape.m_vert_count);
		assert("Direction is too short" && Length(direction) > math::tiny<float>);

		sup_vert_id = hint_vert_id;
		auto sup_dist = Dot3(shape.vertex(sup_vert_id), direction);
		for (auto i = 0; i != shape.m_vert_count; ++i)
		{
			auto dist = Dot3(shape.vertex(i), direction);
			if (dist > sup_dist + math::tiny<float>)
			{
				sup_vert_id = i;
				sup_dist = dist;
			}
		}

		return shape.vertex(sup_vert_id);
	}

	// Returns the longest/shortest axis of a polytope in 'direction' (in polytope space)
	// Searching starts at 'hint_vert_id'. The spanning vertices are 'vert_id0' and 'vert_id1'
	// 'major' is true for the longest axis, false for the shortest axis
	inline void GetAxis(ShapePolytope const& shape, v4& direction, int hint_vert_id, int& vert_id0, int& vert_id1, bool major)
	{
		assert("Invalid polytope" && shape.m_vert_count >= 2);
		assert(hint_vert_id >= 0 && hint_vert_id < shape.m_vert_count);

		vert_id0 = 0;
		vert_id1 = 1;
		direction = shape.vertex(vert_id0) - shape.vertex(vert_id1);
		auto best_len_sq = LengthSq(direction);
		for (auto i = 0; i != shape.m_vert_count; ++i)
		{
			for (auto j = i + 1; j != shape.m_vert_count; ++j)
			{
				auto span = shape.vertex(i) - shape.vertex(j);
				auto len_sq = LengthSq(span);
				if ((major && len_sq > best_len_sq) || (!major && len_sq < best_len_sq && len_sq > Sqr(math::tiny<float>)))
				{
					best_len_sq = len_sq;
					direction = span;
					vert_id0 = i;
					vert_id1 = j;
				}
			}
		}
	}

	// Generate the verts of a polytope. 'verts' should point to a buffer of v4's with a length equal to the value returned from 'VertCount'
	inline void GenerateVerts(ShapePolytope const& shape, std::span<v4> verts)
	{
		assert("buffer too small" && static_cast<int>(verts.size()) >= shape.m_vert_count);
		memcpy(verts.data(), shape.vert_beg(), sizeof(v4) * shape.m_vert_count);
	}

	// Generate the edges of a polytope from the stored edge topology. 'edges' should point to a buffer of 2*the number of edges returned from 'EdgeCount'
	inline void GenerateEdges(ShapePolytope const& shape, std::span<v4> edges)
	{
		assert("buffer too small" && static_cast<int>(edges.size()) >= 2 * shape.m_edge_count);
		auto* e = edges.data();
		for (auto const& edge : shape.edges())
		{
			*e++ = shape.vertex(edge.m_v0);
			*e++ = shape.vertex(edge.m_v1);
		}
	}

	// Generate faces for a polytope from the stored face list.
	inline void GenerateFaces(ShapePolytope const& shape, std::span<int> faces)
	{
		assert("buffer too small" && static_cast<int>(faces.size()) >= 3 * shape.m_face_count);
		auto* f = faces.data();
		for (auto const& face : shape.faces())
		{
			*f++ = s_cast<int>(face.m_index[0]);
			*f++ = s_cast<int>(face.m_index[1]);
			*f++ = s_cast<int>(face.m_index[2]);
		}
	}

	// Remove the face data from a polytope
	inline void StripFaces(ShapePolytope& shape)
	{
		if (shape.m_face_count == 0 && shape.m_edge_count == 0)
			return;

		shape.m_face_count = 0;
		shape.m_edge_count = 0;
		shape.m_base.m_size = s_cast<int>(PadTo(sizeof(ShapePolytope) + sizeof(v4) * shape.m_vert_count, 16));
	}

	// Validate a polytope
	inline bool Validate(ShapePolytope const& shape, bool check_com, char const** err_msg = nullptr)
	{
		if (shape.m_vert_count < 0 || shape.m_face_count < 0 || shape.m_edge_count < 0)
		{
			if (err_msg) *err_msg = "Negative polytope count";
			return false;
		}

		for (auto const& face : shape.faces())
		{
			if (face.m_index[0] >= s_cast<uint32_t>(shape.m_vert_count) ||
				face.m_index[1] >= s_cast<uint32_t>(shape.m_vert_count) ||
				face.m_index[2] >= s_cast<uint32_t>(shape.m_vert_count))
			{
				if (err_msg) *err_msg = "Face vertex index is out of range";
				return false;
			}
			if (face.m_index[0] == face.m_index[1] || face.m_index[1] == face.m_index[2] || face.m_index[2] == face.m_index[0])
			{
				if (err_msg) *err_msg = "Face has duplicate vertex indices";
				return false;
			}

			auto plane_len_sq = LengthSq(face.m_plane.direction());
			if (Abs(plane_len_sq - 1.0f) > 1e-3f)
			{
				if (err_msg) *err_msg = "Face plane normal is not unit length";
				return false;
			}

			for (auto i = 0; i != 3; ++i)
			{
				if (Abs(Distance(face.m_plane, shape.vertex(face.m_index[i]))) > 1e-3f)
				{
					if (err_msg) *err_msg = "Face plane does not contain its vertices";
					return false;
				}
			}
			for (auto const& vert : shape.verts())
			{
				if (Distance(face.m_plane, vert) > 1e-3f)
				{
					if (err_msg) *err_msg = "Face plane normal is not outward facing";
					return false;
				}
			}
		}

		// Check the polytope describes a closed polyhedron
		if (shape.m_face_count != 0 && shape.m_face_count - shape.m_edge_count + shape.m_vert_count != 2)
		{
			if (err_msg) *err_msg = "The polytope is not a closed polyhedron!";
			return false;
		}

		for (auto const& edge : shape.edges())
		{
			if (edge.m_v0 >= s_cast<uint32_t>(shape.m_vert_count) || edge.m_v1 >= s_cast<uint32_t>(shape.m_vert_count))
			{
				if (err_msg) *err_msg = "Edge vertex index is out of range";
				return false;
			}
			if (edge.m_face0 >= s_cast<uint32_t>(shape.m_face_count) || edge.m_face1 >= s_cast<uint32_t>(shape.m_face_count))
			{
				if (err_msg) *err_msg = "Edge face index is out of range";
				return false;
			}
			if (edge.m_v0 == edge.m_v1)
			{
				if (err_msg) *err_msg = "Edge has duplicate vertex indices";
				return false;
			}

			auto edge_direction = Normalise(shape.vertex(edge.m_v1) - shape.vertex(edge.m_v0));
			if (Dot3(edge_direction, edge.m_direction) < 1.0f - 1e-3f)
			{
				if (err_msg) *err_msg = "Edge direction does not match edge vertices";
				return false;
			}

			auto occurrence_count = 0;
			auto face0_found = false;
			auto face1_found = false;
			for (auto face_index = 0; face_index != shape.m_face_count; ++face_index)
			{
				auto const& face = shape.face(face_index);
				auto has_v0 = face.m_index[0] == edge.m_v0 || face.m_index[1] == edge.m_v0 || face.m_index[2] == edge.m_v0;
				auto has_v1 = face.m_index[0] == edge.m_v1 || face.m_index[1] == edge.m_v1 || face.m_index[2] == edge.m_v1;
				if (has_v0 && has_v1)
				{
					++occurrence_count;
					face0_found |= static_cast<uint32_t>(face_index) == edge.m_face0;
					face1_found |= static_cast<uint32_t>(face_index) == edge.m_face1;
				}
			}
			if (occurrence_count != 2 || !face0_found || !face1_found)
			{
				if (err_msg) *err_msg = "Edge is not shared by exactly two recorded faces";
				return false;
			}
		}

		// Check the polytope is in centre of mass frame
		if (check_com)
		{
			if (LengthSq(CalcCentreOfMass(shape)) > Sqr(1e-3f))
			{
				if (err_msg) *err_msg = "Polytope is not in centre-of-mass frame";
				return false;
			}
		}

		return true;
	}

	// Build a ShapePolytope from a set of points using convex hull.
	// Returns the polytope packed into a byte_data<16> buffer suitable for use as a collision shape.
	// The caller owns the buffer and can access the shape via: buf.as<ShapePolytope>()
	// Note: ShapePolytope uses uint8_t vertex indices, so max 255 vertices.
	inline byte_data<16> BuildPolytopeFromPoints(std::span<v4 const> points, m4x4 const& shape_to_root = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
	{
		using Face = ShapePolytope::Face;
		using Edge = ShapePolytope::Edge;

		int points_count = int(points.size());
		assert(points_count >= 4 && "Need at least 4 non-coplanar points for a polytope");
		assert(points_count <= 255 && "ShapePolytope uses uint8_t indices, max 255 vertices");

		// Compute the convex hull of the point set.
		// ConvexHull partitions the index array so hull vertices come first,
		// and returns face index-triples referencing positions in the index array.
		std::vector<int> indices(points_count);
		for (int i = 0; i != points_count; ++i)
			indices[i] = i;

		// Allocate face buffer. A convex hull of N vertices has at most 2*(N-2) faces.
		auto max_faces = 2 * (points_count - 2);
		std::vector<int> face_buf(max_faces * 3);

		size_t hull_vert_count = 0;
		size_t hull_face_count = 0;
		auto ok = hull::ConvexHull(
			points,
			std::span<int>{indices},
			std::span<int>{face_buf},
			hull_vert_count, hull_face_count);

		if (!ok || hull_vert_count < 4 || hull_face_count < 4)
			throw std::runtime_error("ConvexHull failed: point set is degenerate (coplanar, collinear, or too few points)");

		auto vc = static_cast<int>(hull_vert_count);
		auto fc = static_cast<int>(hull_face_count);

		// Ensure consistent outward-facing winding by checking the signed volume.
		// The divergence theorem-based volume/inertia formulas require outward normals
		// (positive triple products when the origin is inside). If the convex hull
		// produced inward-facing normals, the total volume will be negative, so fix by
		// reversing the winding of every face.
		{
			auto vol = 0.0f;
			for (int f = 0; f != fc; ++f)
			{
				auto a = points[indices[face_buf[f * 3 + 0]]];
				auto b = points[indices[face_buf[f * 3 + 1]]];
				auto c = points[indices[face_buf[f * 3 + 2]]];
				vol += Triple(a, b, c);
			}
			if (vol < 0)
			{
				for (int f = 0; f != fc; ++f)
					std::swap(face_buf[f * 3 + 1], face_buf[f * 3 + 2]);
			}
		}

		// Build explicit undirected edge topology from the hull faces.
		struct EdgeDesc
		{
			uint32_t m_v0 = 0;
			uint32_t m_v1 = 0;
			uint32_t m_face0 = 0;
			uint32_t m_face1 = 0xFFFF'FFFFu;
		};
		std::vector<EdgeDesc> edge_descs;
		auto add_edge = [&](uint32_t v0, uint32_t v1, uint32_t face)
		{
			for (auto& edge : edge_descs)
			{
				if ((edge.m_v0 == v0 && edge.m_v1 == v1) || (edge.m_v0 == v1 && edge.m_v1 == v0))
				{
					if (edge.m_face1 != 0xFFFF'FFFFu)
						throw std::runtime_error("ConvexHull produced a non-manifold polytope edge");

					edge.m_face1 = face;
					return;
				}
			}

			edge_descs.push_back(EdgeDesc{ v0, v1, face });
		};
		for (int f = 0; f != fc; ++f)
		{
			auto a = static_cast<uint32_t>(face_buf[f * 3 + 0]);
			auto b = static_cast<uint32_t>(face_buf[f * 3 + 1]);
			auto c = static_cast<uint32_t>(face_buf[f * 3 + 2]);
			add_edge(a, b, static_cast<uint32_t>(f));
			add_edge(b, c, static_cast<uint32_t>(f));
			add_edge(c, a, static_cast<uint32_t>(f));
		}
		for (auto const& edge : edge_descs)
		{
			if (edge.m_face1 == 0xFFFF'FFFFu)
				throw std::runtime_error("ConvexHull produced an open polytope edge");
		}
		auto ec = static_cast<int>(edge_descs.size());

		// Allocate the byte buffer with the exact layout expected by ShapePolytope:
		//   ShapePolytope header
		//   v4    verts[vc]
		//   Face  faces[fc]
		//   Edge  edges[ec]
		//   byte  pad[padding]   (to ensure 16-byte alignment if needed)
		auto buf_size = PadTo(
			sizeof(ShapePolytope) +
			sizeof(v4)   * vc +
			sizeof(Face) * fc +
			sizeof(Edge) * ec,
			16);

		byte_data<16> buf;
		buf.resize(buf_size, std::byte{0});

		// Placement-new the ShapePolytope header at the start of the buffer
		auto& poly = *new (buf.data()) ShapePolytope(shape_to_root, material_id, flags);

		// Set counts so that the accessor methods work
		poly.m_vert_count = vc;
		poly.m_face_count = fc;
		poly.m_edge_count = ec;

		// Copy hull vertices. The ConvexHull partitioned the index array so that
		// indices[0..vc-1] are the hull vertices. Map through the index array
		// to get the original point positions.
		auto* verts = poly.vert_beg();
		for (int i = 0; i != vc; ++i)
			verts[i] = points[indices[i]];

		// Copy hull faces (already in the correct 0-based index space).
		auto* faces = poly.face_beg();
		for (int f = 0; f != fc; ++f)
		{
			faces[f].m_index[0] = static_cast<uint32_t>(face_buf[f * 3 + 0]);
			faces[f].m_index[1] = static_cast<uint32_t>(face_buf[f * 3 + 1]);
			faces[f].m_index[2] = static_cast<uint32_t>(face_buf[f * 3 + 2]);
			faces[f].m_flags = ShapePolytope::EFaceFlags::None;
		}

		// Copy edge topology. Directions are filled after the centre-of-mass shift.
		auto* edges = poly.edge_beg();
		for (int e = 0; e != ec; ++e)
		{
			edges[e].m_v0 = edge_descs[e].m_v0;
			edges[e].m_v1 = edge_descs[e].m_v1;
			edges[e].m_face0 = edge_descs[e].m_face0;
			edges[e].m_face1 = edge_descs[e].m_face1;
			edges[e].m_flags = ShapePolytope::EEdgeFlags::None;
			edges[e].m_pad[0] = edges[e].m_pad[1] = edges[e].m_pad[2] = 0;
		}

		// Keep the local shape origin at the volume centre. The original placement is preserved by moving the whole shape via m_s2r.
		{
			auto centre = CalcCentreOfMass(poly);
			if (!FEql(centre, v4::Zero()))
			{
				for (auto *vert = poly.vert_beg(), *vert_end = poly.vert_end(); vert != vert_end; ++vert)
					*vert -= centre;

				poly.m_base.m_s2r = poly.m_base.m_s2r * m4x4::Translation(centre);
			}
		}

		// Plane3 stores plane.w as -Dot(point_on_plane, normal). With outward normals,
		// points inside the convex polytope should have Distance(plane, point) <= 0.
		for (int f = 0; f != fc; ++f)
		{
			auto& face = faces[f];
			face.m_plane = Plane::FromTriangle(verts[face.m_index[0]], verts[face.m_index[1]], verts[face.m_index[2]]);
			if (Distance(face.m_plane, v4::Origin()) > 0.0f)
			{
				std::swap(face.m_index[1], face.m_index[2]);
				face.m_plane = Plane::FromTriangle(verts[face.m_index[0]], verts[face.m_index[1]], verts[face.m_index[2]]);
			}
		}

		// Mark duplicate face axes. SAT only needs one representative for each axis line, and the support-feature pass will still find the correct face.
		constexpr auto duplicate_axis_tol = 1e-4f;
		for (int f = 0; f != fc; ++f)
		{
			auto& face = faces[f];
			auto normal = face.m_plane.direction();
			for (int i = 0; i != f; ++i)
			{
				if (Abs(Dot3(normal, faces[i].m_plane.direction())) > 1.0f - duplicate_axis_tol)
				{
					face.m_flags = SetBits(face.m_flags, ShapePolytope::EFaceFlags::IgnoreFaceAxis, true);
					break;
				}
			}
		}

		for (int e = 0; e != ec; ++e)
		{
			auto& edge = edges[e];
			auto dir = verts[edge.m_v1] - verts[edge.m_v0];
			auto len = Length(dir);
			if (len <= math::tiny<float>)
				throw std::runtime_error("ConvexHull produced a zero-length polytope edge");

			edge.m_direction = (dir / len).w0();

			auto normal0 = faces[edge.m_face0].m_plane.direction();
			auto normal1 = faces[edge.m_face1].m_plane.direction();
			if (Abs(Dot3(normal0, normal1)) > 1.0f - duplicate_axis_tol)
				edge.m_flags = SetBits(edge.m_flags, ShapePolytope::EEdgeFlags::IgnoreEdgeAxes, true);
		}

		// Finalize. Complete() recalculates m_base.m_size and the bounding box.
		poly.Complete(vc, fc, ec);
		return buf;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::collision::tests
{
	PRUnitTestClass(BuildPolytopeTests)
	{
		// Build a tetrahedron polytope and validate its structure
		PRUnitTestMethod(TetrahedronPolytope)
		{
			v4 pts[] = {
				v4{0, 0, 0, 1},
				v4{2, 0, 0, 1},
				v4{1, 2, 0, 1},
				v4{1, 1, 2, 1},
			};

			auto buf = BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<ShapePolytope>();

			PR_EXPECT(poly.m_vert_count == 4);
			PR_EXPECT(poly.m_face_count == 4);
			PR_EXPECT(poly.m_base.m_type == EShape::Polytope);

			// Validate the polytope structure (topology consistency, closed polyhedron)
			char const* err = nullptr;
			PR_EXPECT(Validate(poly, false, &err));
		}

		// Build a cube polytope from 8 corner points
		PRUnitTestMethod(CubePolytope)
		{
			v4 pts[] = {
				v4{-1, -1, -1, 1}, v4{ 1, -1, -1, 1},
				v4{-1,  1, -1, 1}, v4{ 1,  1, -1, 1},
				v4{-1, -1,  1, 1}, v4{ 1, -1,  1, 1},
				v4{-1,  1,  1, 1}, v4{ 1,  1,  1, 1},
			};

			auto buf = BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<ShapePolytope>();

			PR_EXPECT(poly.m_vert_count == 8);
			PR_EXPECT(poly.m_face_count == 12); // 6 quads = 12 triangles
			PR_EXPECT(poly.m_edge_count == 18); // 12 polyhedron edges plus 6 coplanar face diagonals

			char const* err = nullptr;
			PR_EXPECT(Validate(poly, false, &err));

			auto ignored_faces = 0;
			for (auto const& face : poly.faces())
				ignored_faces += IgnoreFaceAxis(face) ? 1 : 0;

			auto ignored_edges = 0;
			for (auto const& edge : poly.edges())
				ignored_edges += IgnoreEdgeAxes(edge) ? 1 : 0;

			PR_EXPECT(ignored_faces == 9);
			PR_EXPECT(ignored_edges == 6);
		}

		// Build a polytope with interior points. Only hull verts should appear.
		PRUnitTestMethod(InteriorPointsFiltered)
		{
			v4 pts[] = {
				v4{-2, -2, -2, 1}, v4{ 2, -2, -2, 1},
				v4{-2,  2, -2, 1}, v4{ 2,  2, -2, 1},
				v4{-2, -2,  2, 1}, v4{ 2, -2,  2, 1},
				v4{-2,  2,  2, 1}, v4{ 2,  2,  2, 1},
				v4{0, 0, 0, 1},     // interior
				v4{1, 0.5f, 0.5f, 1}, // interior
			};

			auto buf = BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<ShapePolytope>();

			PR_EXPECT(poly.m_vert_count == 8);

			char const* err = nullptr;
			PR_EXPECT(Validate(poly, false, &err));
		}

		// Bounding box should tightly contain all hull vertices
		PRUnitTestMethod(BoundingBox)
		{
			v4 pts[] = {
				v4{0, 0, 0, 1},
				v4{4, 0, 0, 1},
				v4{0, 3, 0, 1},
				v4{0, 0, 2, 1},
			};

			auto buf = BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<ShapePolytope>();

			// BBox should contain all vertices
			for (auto v : poly.verts())
				PR_EXPECT(IsWithin(poly.m_base.m_bbox, v, math::tiny<float>));
		}

		// Support vertex should return the most extreme vertex in a given direction
		PRUnitTestMethod(SupportVertex)
		{
			v4 pts[] = {
				v4{0, 0, 0, 1},
				v4{3, 0, 0, 1},
				v4{0, 3, 0, 1},
				v4{0, 0, 3, 1},
			};

			auto buf = BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<ShapePolytope>();

			PR_EXPECT(FEql(poly.m_base.m_s2r.pos, v4{0.75f, 0.75f, 0.75f, 1.0f}));

			// Support in +X direction should be the vertex at (3,0,0), relative to the local centre.
			int hint = 0, sup_id = 0;
			auto sv = collision::SupportVertex(poly, v4{1, 0, 0, 0}, hint, sup_id);
			PR_EXPECT(FEql(sv, v4{2.25f, -0.75f, -0.75f, 1.0f}));

			// Support in +Y direction should be (0,3,0), relative to the local centre.
			hint = 0;
			sv = collision::SupportVertex(poly, v4{0, 1, 0, 0}, hint, sup_id);
			PR_EXPECT(FEql(sv, v4{-0.75f, 2.25f, -0.75f, 1.0f}));

			// Support in +Z direction should be (0,0,3), relative to the local centre.
			hint = 0;
			sv = collision::SupportVertex(poly, v4{0, 0, 1, 0}, hint, sup_id);
			PR_EXPECT(FEql(sv, v4{-0.75f, -0.75f, 2.25f, 1.0f}));
		}

	};
}
#endif
