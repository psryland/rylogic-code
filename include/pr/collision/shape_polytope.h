//*********************************************
// Collision
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include <algorithm>
#include <vector>
#include <array>
#include <map>
#include <cmath>
#include "pr/collision/forward.h"
#include "pr/collision/shape.h"
#include "pr/container/byte_data.h"
#include "pr/geometry/convex_hull.h"
#include "pr/geometry/tetramesh.h"

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
			int m_index[3];
			EFaceFlags m_flags;
		};
		struct Edge
		{
			v4 m_direction;
			int m_v0;
			int m_v1;
			int m_face0;
			int m_face1;
			EEdgeFlags m_flags;
			int m_pad[3];
		};
		struct Tet
		{
			int m_corner[4]; // Indices into the volume-vertex array (m_volume_vert).
		};

		Shape m_base;
		int m_vert_count;
		int m_face_count;
		int m_edge_count;
		int m_volume_vert_count; // Number of tetrahedralised interior vertices (0 unless the polytope was tessellated).
		int m_tet_count;         // Number of interior tetrahedra (0 unless the polytope was tessellated).
		int pad[3];
		
		// Memory layout. The following data is expected to follow this struct in memory, but is not actually part of the struct.
		// v4   m_vert[m_vert_count]
		// Face m_face[m_face_count]
		// Edge m_edge[m_edge_count]
		// v4   m_volume_vert[m_volume_vert_count]   (only present when the polytope was tessellated)
		// Tet  m_tet[m_tet_count]                   (only present when the polytope was tessellated)
		// byte padding[] to make the total size a multiple of 16 bytes

		explicit ShapePolytope(m4x4 const& shape_to_root = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			: m_base(EShape::Polytope, sizeof(ShapePolytope), shape_to_root, BBox::Reset(), material_id, flags)
			, m_vert_count()
			, m_face_count()
			, m_edge_count()
			, m_volume_vert_count()
			, m_tet_count()
			, pad()
		{
			// Careful: We can't be sure of what follows this object in memory.
			// The polytope data that belongs to this array may not be there yet.
			// Defer calculating the bounding box until the caller calls 'Complete()'.
		}
		void Complete(int vert_count, int face_count, int edge_count)
		{
			Complete(vert_count, face_count, edge_count, 0, 0);
		}
		void Complete(int vert_count, int face_count, int edge_count, int volume_vert_count, int tet_count)
		{
			m_vert_count = vert_count;
			m_face_count = face_count;
			m_edge_count = edge_count;
			m_volume_vert_count = volume_vert_count;
			m_tet_count = tet_count;

			m_base.m_size = s_cast<int>(PadTo(sizeof(ShapePolytope) +
				sizeof(v4) * m_vert_count +
				sizeof(Face) * m_face_count +
				sizeof(Edge) * m_edge_count +
				sizeof(v4) * m_volume_vert_count +
				sizeof(Tet) * m_tet_count,
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

		// Volume-vertex accessors (tetrahedralised interior; empty unless the polytope was tessellated)
		v4 const* volume_vert_beg() const { return type_ptr<v4>(edge_end()); }
		v4*       volume_vert_beg()       { return type_ptr<v4>(edge_end()); }
		v4 const* volume_vert_end() const { return volume_vert_beg() + m_volume_vert_count; }
		v4*       volume_vert_end()       { return volume_vert_beg() + m_volume_vert_count; }
		v4 const& volume_vertex(std::size_t idx) const { return volume_vert_beg()[idx]; }
		v4&       volume_vertex(std::size_t idx)       { return volume_vert_beg()[idx]; }
		auto volume_verts() const { return std::ranges::subrange(volume_vert_beg(), volume_vert_end()); }

		// Tetrahedron accessors (interior tetrahedralisation; empty unless the polytope was tessellated)
		Tet const* tet_beg() const    { return type_ptr<Tet>(volume_vert_end()); }
		Tet*       tet_beg()          { return type_ptr<Tet>(volume_vert_end()); }
		Tet const* tet_end() const    { return tet_beg() + m_tet_count; }
		Tet*       tet_end()          { return tet_beg() + m_tet_count; }
		Tet const& tet(int idx) const { return tet_beg()[idx]; }
		Tet&       tet(int idx)       { return tet_beg()[idx]; }
		auto tets() const { return std::ranges::subrange(tet_beg(), tet_end()); }

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
	static_assert((sizeof(ShapePolytope::Tet) & 0xf) == 0);

	using PolyIdx       = ShapePolytope::Idx;
	using ShapePolyFace = ShapePolytope::Face;
	using ShapePolyEdge = ShapePolytope::Edge;
	using ShapePolyVert = v4;

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
		assert(hint_vert_id >= 0 && hint_vert_id < shape.m_vert_count); (void)hint_vert_id;

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

		// Dropping faces/edges also orphans any interior tessellation (it is stored after the
		// edges), so clear the volume tets too and recompute the size to just the header + verts.
		shape.m_face_count = 0;
		shape.m_edge_count = 0;
		shape.m_volume_vert_count = 0;
		shape.m_tet_count = 0;
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
			if (face.m_index[0] >= shape.m_vert_count ||
				face.m_index[1] >= shape.m_vert_count ||
				face.m_index[2] >= shape.m_vert_count)
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
			if (edge.m_v0 >= shape.m_vert_count || edge.m_v1 >= shape.m_vert_count)
			{
				if (err_msg) *err_msg = "Edge vertex index is out of range";
				return false;
			}
			if (edge.m_face0 >= shape.m_face_count || edge.m_face1 >= shape.m_face_count)
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
					face0_found |= face_index == edge.m_face0;
					face1_found |= face_index == edge.m_face1;
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

	// Tetrahedralise a convex polytope by joining one interior vertex to every triangular surface face.
	// The face triangles already tile the boundary, so the resulting tetrahedra exactly tile the volume
	// in O(face count) time. Their shape does not affect volume-weighted uniform sampling correctness.
	inline void TetrahedralisePolytope(ShapePolytope const& poly, std::vector<v4>& out_verts, std::vector<ShapePolytope::Tet>& out_tets)
	{
		using Tet = ShapePolytope::Tet;

		out_verts.resize(0);
		out_tets.resize(0);

		if (poly.m_vert_count == 0 || poly.m_face_count == 0)
			return;

		// Using the calculated volume centre keeps this helper valid for any convex ShapePolytope,
		// including manually authored shapes whose local origin is not inside the hull.
		out_verts.reserve(static_cast<std::size_t>(poly.m_vert_count) + 1);
		out_verts.push_back(CalcCentreOfMass(poly).w1());
		out_verts.insert(out_verts.end(), poly.vert_beg(), poly.vert_end());

		// Share the interior vertex and the copied surface vertices between all tetrahedra. Correct
		// orientation preserves the positive-volume convention expected by the sampling CDF.
		out_tets.reserve(poly.m_face_count);
		for (auto const& face : poly.faces())
		{
			auto tet = Tet{};
			tet.m_corner[0] = 0;
			tet.m_corner[1] = static_cast<int>(face.m_index[0]) + 1;
			tet.m_corner[2] = static_cast<int>(face.m_index[1]) + 1;
			tet.m_corner[3] = static_cast<int>(face.m_index[2]) + 1;

			auto const vol6 = tetramesh::Volume6(
				out_verts[tet.m_corner[0]],
				out_verts[tet.m_corner[1]],
				out_verts[tet.m_corner[2]],
				out_verts[tet.m_corner[3]]);
			if (vol6 < 0.0f)
				std::swap(tet.m_corner[2], tet.m_corner[3]);

			out_tets.push_back(tet);
		}
	}

	// Tetrahedralise the interior of a convex polytope into well-shaped tetrahedra.
	// Fills 'out_verts' and 'out_tets' (indices into 'out_verts') with a tetrahedralisation whose
	// union exactly fills the polytope. This is used for volumetric sampling (e.g. buoyancy).
	//
	// Method: the polytope's AABB is voxelised into a regular grid of near-isosceles tetrahedra
	// (pr::tetramesh::Generate, 5 tets per cube). Each grid tetra is classified against the
	// polytope's outward face planes - those fully inside are kept whole (preserving the regular
	// interior tets), those fully outside are discarded, and straddling tets are clipped against
	// each face plane in turn, keeping the inside (Distance <= 0) half-space. Sequential clipping
	// of a convex set against convex half-spaces is order-independent, so the survivors exactly
	// tile (grid_tetra INTERSECT polytope); summed over the grid this is the whole polytope.
	//
	// 'resolution' is the number of grid cells along the polytope's longest AABB axis (clamped >= 1).
	// Higher resolution gives more, smaller interior tets (lower sampling variance) but more storage;
	// it does not change the (exact) total volume.
	inline void TessellatePolytope(ShapePolytope const& poly, int resolution, std::vector<v4>& out_verts, std::vector<ShapePolytope::Tet>& out_tets)
	{
		using Tet = ShapePolytope::Tet;
		using TetPts = std::array<v4, 4>;

		out_verts.resize(0);
		out_tets.resize(0);

		if (poly.m_vert_count == 0 || poly.m_face_count == 0)
			return;

		resolution = std::max(resolution, 1);

		// Compute the AABB of the hull vertices (component-wise min/max).
		auto lo = poly.vertex(0);
		auto hi = poly.vertex(0);
		for (auto v : poly.verts())
		{
			lo = Min(lo, v);
			hi = Max(hi, v);
		}
		auto extent = (hi - lo).w0();

		// Degenerate (zero-volume) polytopes have nothing to tessellate.
		auto longest = Max(extent.x, Max(extent.y, extent.z));
		if (longest <= math::tiny<float>)
			return;

		// Choose a near-cubic cell count per axis from the longest extent, then derive exact cell
		// sizes so the grid spans precisely [lo, hi].
		auto cell = longest / resolution;
		auto nx = std::max(1, s_cast<int>(std::lround(extent.x / cell)));
		auto ny = std::max(1, s_cast<int>(std::lround(extent.y / cell)));
		auto nz = std::max(1, s_cast<int>(std::lround(extent.z / cell)));
		auto sx = extent.x / nx;
		auto sy = extent.y / ny;
		auto sz = extent.z / nz;

		// Keep a non-zero cell size on near-flat axes so Generate stays valid.
		if (sx <= math::tiny<float>) sx = cell;
		if (sy <= math::tiny<float>) sy = cell;
		if (sz <= math::tiny<float>) sz = cell;

		// Generate the grid (Generate centres the lattice on a half-cell offset), then translate it
		// so the grid spans exactly [lo, hi].
		auto grid = tetramesh::Generate(nx, ny, nz, sx, sy, sz);
		auto offset = v4(lo.x + sx * 0.5f, lo.y + sy * 0.5f, lo.z + sz * 0.5f, 0.0f);
		for (auto& v : grid.m_verts)
			v += offset;

		// Tolerances scaled to the shape size: 'eps' classifies a point as inside when within a
		// hair of (or behind) a face plane; 'eps_vol6' drops only true numerically-degenerate tets
		// (positive-volume slivers are kept - dropping them would bias the volume / centre of mass).
		auto eps = longest * 1e-5f;
		auto eps_vol6 = 6.0f * (sx * sy * sz) * 1e-7f;

		// Collect the outward face planes once (a triangulated quad face contributes its plane more
		// than once, but clipping by a duplicate plane is idempotent, so duplicates are harmless).
		std::vector<Plane> planes;
		planes.reserve(poly.m_face_count);
		for (auto const& f : poly.faces())
			planes.push_back(f.m_plane);

		// Emit a triangular prism with end triangles (a0,a1,a2) and (b0,b1,b2), where a_i maps to
		// b_i, as three tetrahedra. This tiles the prism exactly regardless of side-quad planarity.
		auto emit_prism = [](std::vector<TetPts>& dst, v4 a0, v4 a1, v4 a2, v4 b0, v4 b1, v4 b2)
		{
			dst.push_back(TetPts{ a0, a1, a2, b0 });
			dst.push_back(TetPts{ a1, a2, b0, b1 });
			dst.push_back(TetPts{ a2, b0, b1, b2 });
		};

		// Clip every tet in 'src' against one outward plane, keeping the inside (Distance <= 0)
		// half-space, and write the result into 'dst'.
		auto clip_by_plane = [&](std::vector<TetPts> const& src, Plane plane, std::vector<TetPts>& dst)
		{
			dst.resize(0);
			for (auto const& t : src)
			{
				float d[4];
				int in[4], out[4];
				int nin = 0, nout = 0;
				for (int i = 0; i != 4; ++i)
				{
					d[i] = Distance(plane, t[i]);
					if (d[i] <= eps) in[nin++] = i;
					else out[nout++] = i;
				}

				if (nin == 4) { dst.push_back(t); continue; }
				if (nin == 0) { continue; }

				// Intersection point on the edge between inside vertex 'i' and outside vertex 'j'.
				auto isect = [&](int i, int j)
				{
					auto frac = d[i] / (d[i] - d[j]);
					return t[i] + frac * (t[j] - t[i]);
				};

				switch (nin)
				{
					case 1:
					{
						// One inside vertex: the inside piece is a single smaller tetra.
						auto a = in[0];
						dst.push_back(TetPts{ t[a], isect(a, out[0]), isect(a, out[1]), isect(a, out[2]) });
						break;
					}
					case 3:
					{
						// Three inside vertices: the inside piece is a triangular prism.
						auto o = out[0];
						emit_prism(dst,
							t[in[0]], t[in[1]], t[in[2]],
							isect(in[0], o), isect(in[1], o), isect(in[2], o));
						break;
					}
					case 2:
					{
						// Two inside vertices: the inside piece is also a triangular prism, with end
						// triangles (a, X_a0, X_a1) and (b, X_b0, X_b1) over the two outside vertices.
						auto a = in[0];
						auto b = in[1];
						emit_prism(dst,
							t[a], isect(a, out[0]), isect(a, out[1]),
							t[b], isect(b, out[0]), isect(b, out[1]));
						break;
					}
				}
			}
		};

		// Deduplicate emitted vertices on a quantised lattice so shared grid verts collapse.
		auto quantum = longest * 1e-6f;
		if (quantum <= 0.0f) quantum = 1e-6f;
		std::map<std::array<long long, 3>, int> vmap;
		auto add_vert = [&](v4 p) -> int
		{
			std::array<long long, 3> key =
			{
				std::llround(p.x / quantum),
				std::llround(p.y / quantum),
				std::llround(p.z / quantum),
			};
			auto it = vmap.find(key);
			if (it != vmap.end())
				return it->second;

			auto idx = s_cast<int>(out_verts.size());
			vmap.emplace(key, idx);
			out_verts.push_back(p.w1());
			return idx;
		};

		// Classify and clip each grid tetra against the polytope.
		std::vector<TetPts> work, scratch;
		for (auto const& gt : grid.m_tets)
		{
			TetPts tp =
			{
				grid.m_verts[gt.m_corner[0]],
				grid.m_verts[gt.m_corner[1]],
				grid.m_verts[gt.m_corner[2]],
				grid.m_verts[gt.m_corner[3]],
			};

			// Broad classification: a single plane that all four corners are outside of separates the
			// tetra from the polytope (fully outside); if no plane has any corner outside, the tetra
			// is fully inside and is kept whole.
			auto fully_outside = false;
			auto fully_inside = true;
			for (auto const& plane : planes)
			{
				auto outside_count = 0;
				for (int i = 0; i != 4; ++i)
					outside_count += Distance(plane, tp[i]) > eps ? 1 : 0;

				if (outside_count == 4) { fully_outside = true; break; }
				if (outside_count != 0) fully_inside = false;
			}
			if (fully_outside)
				continue;

			work.assign(1, tp);
			if (!fully_inside)
			{
				for (auto const& plane : planes)
				{
					clip_by_plane(work, plane, scratch);
					std::swap(work, scratch);
					if (work.empty())
						break;
				}
			}

			// Append survivors, dropping numerically-degenerate tets and fixing orientation so every
			// emitted tetra obeys the ordering convention (positive Volume6).
			for (auto& st : work)
			{
				auto vol6 = tetramesh::Volume6(st[0], st[1], st[2], st[3]);
				if (Abs(vol6) <= eps_vol6)
					continue;
				if (vol6 < 0.0f)
					std::swap(st[2], st[3]);

				Tet tet;
				tet.m_corner[0] = add_vert(st[0]);
				tet.m_corner[1] = add_vert(st[1]);
				tet.m_corner[2] = add_vert(st[2]);
				tet.m_corner[3] = add_vert(st[3]);
				out_tets.push_back(tet);
			}
		}
	}

	// Build a ShapePolytope from a set of points using convex hull.
	// Returns the polytope packed into a byte_data<16> buffer suitable for use as a collision shape.
	// The caller owns the buffer and can access the shape via: buf.as<ShapePolytope>()
	// 'tess_resolution' requests an interior tetrahedralisation for volumetric sampling: zero omits
	// volume data, a negative value uses the exact O(face count) face fan, and a positive value uses
	// that many grid cells along the longest axis for stronger spatial sample stratification.
	inline byte_data<16> BuildPolytopeFromPoints(std::span<v4 const> points, m4x4 const& shape_to_root = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None, int tess_resolution = 0)
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

		// Optionally tetrahedralise the interior for volumetric sampling (e.g. buoyancy). This runs
		// in the final centre-of-mass-local frame (verts already shifted, planes already computed),
		// so the volume tets align with the collision shape. The tet count is not known up front, so
		// the data is computed into temporaries and then re-packed into a correctly-sized buffer.
		if (tess_resolution != 0)
		{
			std::vector<v4> vol_verts;
			std::vector<ShapePolytope::Tet> tets;
			if (tess_resolution < 0)
				TetrahedralisePolytope(poly, vol_verts, tets);
			else
				TessellatePolytope(poly, tess_resolution, vol_verts, tets);

			auto nvv = static_cast<int>(vol_verts.size());
			auto nt = static_cast<int>(tets.size());

			// The header, hull verts, faces and edges are contiguous (no padding between them), so
			// the volume verts begin exactly at edge_end(). Copy that prefix verbatim, then append.
			auto used = sizeof(ShapePolytope) + sizeof(v4) * vc + sizeof(Face) * fc + sizeof(Edge) * ec;
			auto full_size = PadTo(used + sizeof(v4) * nvv + sizeof(ShapePolytope::Tet) * nt, 16);

			byte_data<16> buf2;
			buf2.resize(full_size, std::byte{ 0 });
			memcpy(buf2.data(), buf.data(), used);

			auto& poly2 = *reinterpret_cast<ShapePolytope*>(buf2.data());
			poly2.m_volume_vert_count = nvv;
			poly2.m_tet_count = nt;
			std::copy(vol_verts.begin(), vol_verts.end(), poly2.volume_vert_beg());
			std::copy(tets.begin(), tets.end(), poly2.tet_beg());
			poly2.Complete(vc, fc, ec, nvv, nt);
			return buf2;
		}

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

		// Polytopes are not tessellated unless explicitly requested.
		PRUnitTestMethod(NotTessellatedByDefault)
		{
			v4 pts[] = {
				v4{-1, -1, -1, 1}, v4{ 1, -1, -1, 1},
				v4{-1,  1, -1, 1}, v4{ 1,  1, -1, 1},
				v4{-1, -1,  1, 1}, v4{ 1, -1,  1, 1},
				v4{-1,  1,  1, 1}, v4{ 1,  1,  1, 1},
			};

			auto buf = BuildPolytopeFromPoints(pts);
			auto& poly = buf.as<ShapePolytope>();

			PR_EXPECT(poly.m_volume_vert_count == 0);
			PR_EXPECT(poly.m_tet_count == 0);
			PR_EXPECT(poly.tet_beg() == poly.tet_end());
		}

		// A grid-aligned cube tessellates into whole interior tets that exactly recover the volume.
		PRUnitTestMethod(CubeTessellation)
		{
			v4 pts[] = {
				v4{-1, -1, -1, 1}, v4{ 1, -1, -1, 1},
				v4{-1,  1, -1, 1}, v4{ 1,  1, -1, 1},
				v4{-1, -1,  1, 1}, v4{ 1, -1,  1, 1},
				v4{-1,  1,  1, 1}, v4{ 1,  1,  1, 1},
			};

			auto buf = BuildPolytopeFromPoints(pts, m4x4::Identity(), 0, Shape::EFlags::None, 4);
			auto& poly = buf.as<ShapePolytope>();

			// The hull topology is unchanged by tessellation.
			char const* err = nullptr;
			PR_EXPECT(Validate(poly, false, &err));
			PR_EXPECT(poly.m_vert_count == 8);
			PR_EXPECT(poly.m_face_count == 12);

			PR_EXPECT(poly.m_tet_count > 0);
			PR_EXPECT(poly.m_volume_vert_count > 0);

			auto sum = 0.0f;
			for (auto const& t : poly.tets())
			{
				auto a = poly.volume_vertex(t.m_corner[0]);
				auto b = poly.volume_vertex(t.m_corner[1]);
				auto c = poly.volume_vertex(t.m_corner[2]);
				auto d = poly.volume_vertex(t.m_corner[3]);
				auto vol = tetramesh::Volume(a, b, c, d);
				PR_EXPECT(vol > 0.0f); // every emitted tetra obeys the ordering convention
				sum += vol;
			}

			// AABB == cube, so the grid aligns and the volume is recovered to float precision.
			PR_EXPECT(FEqlRelative(sum, CalcVolume(poly), 1e-4f));
			PR_EXPECT(FEqlRelative(sum, 8.0f, 1e-4f));
		}

		// A tetrahedron has a larger AABB than itself, so boundary tets are clipped; the clipped
		// tessellation must still conserve the polytope volume.
		PRUnitTestMethod(TetrahedronTessellation)
		{
			v4 pts[] = {
				v4{0, 0, 0, 1},
				v4{2, 0, 0, 1},
				v4{1, 2, 0, 1},
				v4{1, 1, 2, 1},
			};

			auto buf = BuildPolytopeFromPoints(pts, m4x4::Identity(), 0, Shape::EFlags::None, 4);
			auto& poly = buf.as<ShapePolytope>();

			PR_EXPECT(poly.m_tet_count > 0);

			auto sum = 0.0f;
			for (auto const& t : poly.tets())
			{
				auto a = poly.volume_vertex(t.m_corner[0]);
				auto b = poly.volume_vertex(t.m_corner[1]);
				auto c = poly.volume_vertex(t.m_corner[2]);
				auto d = poly.volume_vertex(t.m_corner[3]);
				auto vol = tetramesh::Volume(a, b, c, d);
				PR_EXPECT(vol > 0.0f);
				sum += vol;
			}

			// Clipping is exact in real arithmetic; allow a small tolerance for float error.
			PR_EXPECT(FEqlRelative(sum, CalcVolume(poly), 1e-2f));
		}

		// An octahedron (non grid-aligned faces) exercises multi-plane clipping; volume must conserve.
		PRUnitTestMethod(OctahedronTessellation)
		{
			v4 pts[] = {
				v4{ 1.5f, 0, 0, 1}, v4{-1.5f, 0, 0, 1},
				v4{ 0, 1.5f, 0, 1}, v4{ 0,-1.5f, 0, 1},
				v4{ 0, 0, 1.5f, 1}, v4{ 0, 0,-1.5f, 1},
			};

			auto buf = BuildPolytopeFromPoints(pts, m4x4::Identity(), 0, Shape::EFlags::None, 5);
			auto& poly = buf.as<ShapePolytope>();

			PR_EXPECT(poly.m_tet_count > 0);

			auto sum = 0.0f;
			for (auto const& t : poly.tets())
			{
				auto a = poly.volume_vertex(t.m_corner[0]);
				auto b = poly.volume_vertex(t.m_corner[1]);
				auto c = poly.volume_vertex(t.m_corner[2]);
				auto d = poly.volume_vertex(t.m_corner[3]);
				auto vol = tetramesh::Volume(a, b, c, d);
				PR_EXPECT(vol > 0.0f);
				sum += vol;
			}

			PR_EXPECT(FEqlRelative(sum, CalcVolume(poly), 2e-2f));
		}

	};
}
#endif
