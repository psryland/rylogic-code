//*********************************************
// Collision
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
#pragma once
#include "pr/container/vector.h"
#include "pr/collision/shape.h"

namespace pr::collision
{
	struct ShapePolytope
	{
		// A polytope is basically a triangle mesh with adjacency data.
		// All polytopes are assumed to be convex.

		// Mesh structure types
		using Idx = unsigned char;
		struct Face
		{
			Idx m_index[3];
			Idx pad;
		};
		struct Nbrs
		{
			// Neighbours are the vertices that share an edge connected to a vertex.
			// Neighbours also include an 'artificial' neighbour used to quickly link
			// to the other side of the polytope. The artificial neighbour is always
			// the first index in the list of neighbours.
			unsigned short m_first; // Byte offset to the first neighbour
			unsigned short m_count; // Number of neighbours

			Idx const* begin() const      { return type_ptr<Idx>(byte_ptr(this) + m_first); }
			Idx*       begin()            { return type_ptr<Idx>(byte_ptr(this) + m_first); }
			Idx const* end() const        { return begin() + m_count; }
			Idx*       end()              { return begin() + m_count; }
			Idx const& nbr(int idx) const { return begin()[idx]; }
			Idx&       nbr(int idx)       { return begin()[idx]; }
		};

		Shape m_base;
		int   m_vert_count;
		int   m_face_count;
		// v4   m_vert[m_vert_count]
		// Face m_face[m_face_count]
		// Nbrs m_nbrs[m_vert_count]
		// Idx  m_nbr[sum(m_nbrs[i].m_count)]

		explicit ShapePolytope(m4x4 const& shape_to_parent = m4x4::Identity(), MaterialId material_id = 0, Shape::EFlags flags = Shape::EFlags::None)
			:m_base(EShape::Polytope, sizeof(ShapePolytope), shape_to_parent, material_id, flags)
			,m_vert_count()
			,m_face_count()
		{
			// Careful: We can't be sure of what follows this object in memory.
			// The polytope data that belongs to this array may not be there yet.
			// Differ calculating the bounding box to the caller
		}
		void Complete(int vert_count, int face_count)
		{
			m_vert_count = vert_count;
			m_face_count = face_count;

			// Determine the size of the neighbour array
			auto nbr_count = 0;
			for (Nbrs const *n = nbrs_beg(), *n_end = nbrs_end(); n != n_end; ++n)
				nbr_count += n->m_count;

			m_base.m_size = sizeof(ShapePolytope) +
				sizeof(v4) * m_vert_count +
				sizeof(Face) * m_face_count +
				sizeof(Nbrs) * m_vert_count +
				sizeof(Idx) * nbr_count;
		
			// Find the bounding box
			m_base.m_bbox = CalcBBox(*this);
		}

		// Vertex accessors
		v4 const* vert_beg() const              { return type_ptr<v4>(this + 1); }
		v4*       vert_beg()                    { return type_ptr<v4>(this + 1); }
		v4 const* vert_end() const              { return vert_beg() + m_vert_count; }
		v4*       vert_end()                    { return vert_beg() + m_vert_count; }
		v4        vertex(std::size_t idx) const { return vert_beg()[idx]; }
		v4&       vertex(std::size_t idx)       { return vert_beg()[idx]; }

		// Face accessors
		Face const* face_beg() const    { return type_ptr<Face>(vert_end()); }
		Face*       face_beg()          { return type_ptr<Face>(vert_end()); }
		Face const* face_end() const    { return face_beg() + m_face_count; }
		Face*       face_end()          { return face_beg() + m_face_count; }
		Face const& face(int idx) const { return face_beg()[idx]; }
		Face&       face(int idx)       { return face_beg()[idx]; }

		// Neighbours accessors
		Nbrs const* nbrs_beg() const    { return type_ptr<Nbrs>(face_end()); }
		Nbrs*       nbrs_beg()          { return type_ptr<Nbrs>(face_end()); }
		Nbrs const* nbrs_end() const    { return nbrs_beg() + m_vert_count; }
		Nbrs*       nbrs_end()          { return nbrs_beg() + m_vert_count; }
		Nbrs const& nbrs(int idx) const { return nbrs_beg()[idx]; }
		Nbrs&       nbrs(int idx)       { return nbrs_beg()[idx]; }

		// Opposite side vertex
		v4        opp_vertex(int idx) const { return vert_beg()[*nbrs(idx).begin()]; }
		v4&       opp_vertex(int idx)       { return vert_beg()[*nbrs(idx).begin()]; }

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
	static_assert(is_shape_v<ShapePolytope>);
	using PolyIdx       = ShapePolytope::Idx;
	using ShapePolyFace = ShapePolytope::Face;
	using ShapePolyNbrs = ShapePolytope::Nbrs;

	// Return the bounding box for a polytope
	template <typename>
	BBox pr_vectorcall CalcBBox(ShapePolytope const& shape)
	{
		auto bb = BBox::Reset();
		for (v4 const *v = shape.vert_beg(), *vend = shape.vert_end(); v != vend; ++v)
			Grow(bb, *v);

		return bb;
	}

	// Return the volume of the polytope
	template <typename = void>
	float CalcVolume(ShapePolytope const& shape)
	{
		auto volume = 0.0f;
		for (ShapePolyFace const *f = shape.face_beg(), *f_end = shape.face_end(); f != f_end; ++f)
		{
			auto a = shape.vertex(f->m_index[0]);
			auto b	= shape.vertex(f->m_index[1]);
			auto c = shape.vertex(f->m_index[2]);
			volume += Triple(a, b, c); // Triple product is volume x 6
		}
		return volume / 6.0f;
	}

	// Return the centre of mass position of the polytope
	template <typename = void>
	v4 CalcCentreOfMass(ShapePolytope const& shape)
	{
		assert("Centre of mass is undefined for an empty polytope" && shape.m_vert_count != 0 && shape.m_face_count != 0);

		auto com = v4::Zero();
		auto volume = 0.0f;
		for (ShapePolyFace const *f = shape.face_beg(), *fend = shape.face_end(); f != fend; ++f)
		{
			auto a = shape.vertex(f->m_index[0]);
			auto b = shape.vertex(f->m_index[1]);
			auto c = shape.vertex(f->m_index[2]);
			auto vol_x6 = Triple(a, b, c); // Triple product is volume x 6
			com	+= vol_x6 * (a + b + c);    // Divide by 4 at end
			volume += vol_x6;
		}
		volume *= 4.0f;

		// If the polytope is degenerate, use the weighted average vertex positions
		if (FEql(volume, 0.f))
		{
			com = v4::Zero();
			for (v4 const *v = shape.vert_beg(), *vend = shape.vert_end(); v != vend; ++v) com += *v;
			volume = 1.0f * shape.m_vert_count;
		}

		return com.w0() / volume;
	}

	// Shift the verts of the polytope so they are centred on a new position.
	// 'shift' should be in 'shape' space. NOTE: This invalidates the inertia matrix.
	// You will need to translate the inertia matrix by the same shift.
	template <typename>
	void pr_vectorcall ShiftCentre(ShapePolytope& shape, v4 shift)
	{
		assert(shift.w == 0.0f);
		for (v4 *v = shape.vert_beg(), *vend = shape.vert_end(); v != vend; ++v) *v -= shift;
		shape.m_base.m_s2p.pos += shift;
	}

	// Return a support vertex for a polytope
	template <typename>
	v4 pr_vectorcall SupportVertex(ShapePolytope const& shape, v4 direction, int hint_vert_id, int& sup_vert_id)
	{
		assert("Invalid hint vertex index" && hint_vert_id >= 0 && hint_vert_id < shape.m_vert_count);
		assert("Direction is too short" && Length(direction) > maths::tiny<float>);

		// Find the support vertex using a 'hill-climbing' search
		// Start at the hint vertex and look for a neighbour that is more extreme in the
		// support direction. When no neighbours are closer we've found the support vertex
		sup_vert_id = hint_vert_id;
		auto support_vertex = &shape.vertex(sup_vert_id);
		auto sup_dist       = Dot3(*support_vertex, direction);

		//PR_EXPAND(PR_PH_DBG_SUPVERT, StartFile("C:/DeleteMe/collision_supverttrace.pr_script");)
		//PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::PhShape("polytope", "8000FF00", shape, m4x4::Identity());)
		//PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Line("sup_direction", "FFFFFF00", v4::Origin(), direction);)
		//PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::GroupStart("SupportVertexTrace");)
		//PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Box("start", "FF00FFFF", *support_vertex, 0.05f);)

		v4 const* nearest_vertex;
		auto skip_first_nbr = false; // Skip the artificial neighbour after the first pass
		do
		{
			nearest_vertex = support_vertex;
			auto& nbrhdr = shape.nbrs(sup_vert_id);
			for (PolyIdx const *n = nbrhdr.begin() + skip_first_nbr, *nend = nbrhdr.end(); n != nend; ++n)
			{
				// There are two possible ways we can do this, either by moving to the
				// first neighbour that is more extreme or by testing all neighbours.
				// The disadvantages are searching a non-optimal path to the support
				// vertex or searching excessive neighbours respectively.
				// Test in batches of 4 as a trade off
				if (!skip_first_nbr || nend - n < 4)
				{
	 				skip_first_nbr = true;
					auto dist = Dot3(shape.vertex(*n), direction);
					if (dist > sup_dist + maths::tiny<float>)
					{
						sup_vert_id    = *n;
						sup_dist       = dist;
						support_vertex = &shape.vertex(sup_vert_id);

						//PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Line("to", "FF0000FF", *nearest_vertex, *support_vertex);)
						//PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Box("v", "FF0000FF", *support_vertex, 0.05f);)
						break;
					}
				}
				else
				{
					m4x4 nbrs;
					nbrs.x = shape.vertex(*(n + 0));
					nbrs.y = shape.vertex(*(n + 1));
					nbrs.z = shape.vertex(*(n + 2));
					nbrs.w = shape.vertex(*(n + 3));
					nbrs = Transpose(nbrs);
					v4 dots = nbrs * direction;

					auto id = sup_vert_id;
					if (dots.x > sup_dist) { sup_dist = dots.x; sup_vert_id = *(n + 0); }
					if (dots.y > sup_dist) { sup_dist = dots.y; sup_vert_id = *(n + 1); }
					if (dots.z > sup_dist) { sup_dist = dots.z; sup_vert_id = *(n + 2); }
					if (dots.w > sup_dist) { sup_dist = dots.w; sup_vert_id = *(n + 3); }
					if (sup_vert_id != id)
					{
						support_vertex = &shape.vertex(sup_vert_id);
						//PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Line("to", "FF0000FF", *nearest_vertex, *support_vertex);)
						//PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::Box("v", "FF0000FF", *support_vertex, 0.05f);)
						break;
					}
					n += 3;
				}
			}
		}
		while (support_vertex != nearest_vertex);

		//PR_EXPAND(PR_PH_DBG_SUPVERT, ldr::GroupEnd();)
		//PR_EXPAND(PR_PH_DBG_SUPVERT, EndFile();)

		return *support_vertex;
	}

	// Returns the longest/shortest axis of a polytope in 'direction' (in polytope space)
	// Searching starts at 'hint_vert_id'. The spanning vertices are 'vert_id0' and 'vert_id1'
	// 'major' is true for the longest axis, false for the shortest axis
	template <typename>
	void GetAxis(ShapePolytope const& shape, v4& direction, int hint_vert_id, int& vert_id0, int& vert_id1, bool major)
	{
		assert(hint_vert_id  >= 0 && hint_vert_id < shape.m_vert_count);

		auto eps = major ? maths::tiny<float> : -maths::tiny<float>;

		vert_id0 = hint_vert_id;
		auto V1 = &shape.vertex(vert_id0);
		auto V2 = &shape.vertex(*shape.nbrs(vert_id0).begin());	// The first neighbour is always the most distant

		direction = *V1 - *V2;
		auto span_lensq = LengthSq(direction);
		do
		{
			hint_vert_id = vert_id0;

			// Look for a neighbour with a longer span
			auto& nbr = shape.nbrs(vert_id0);
			for (PolyIdx const *n = nbr.begin() + 1, *nend = nbr.end(); n < nend; ++n)
			{
				auto v1 = &shape.vertex(*n);
				auto v2 = &shape.vertex(*shape.nbrs(*n).begin());
				auto span = *v1 - *v2;
				auto len_sq = LengthSq(span);
				if ((len_sq > span_lensq + eps) == major)
				{
					span_lensq = len_sq;
					direction = span;
					vert_id0 = *n;
					break;
				}
			}
		}
		while (hint_vert_id != vert_id0);
		vert_id1 = *shape.nbrs(vert_id0).begin();
	}

	// Return the number of vertices in a polytope
	template <typename = void>
	int VertCount(ShapePolytope const& shape)
	{
		return shape.m_vert_count;
	}

	// Return the number of edges in a polytope
	template <typename = void>
	int EdgeCount(ShapePolytope const& shape)
	{
		// The number of edges in the polytope is the number of
		// neighbours minus the artificial neighbours over 2.
		auto nbr_count = 0;
		for (ShapePolyNbrs const* n = shape.nbrs_beg(), *nend = shape.nbrs_end(); n != nend; ++n)
			nbr_count += n->m_count;

		return (nbr_count - shape.m_vert_count) / 2;
	}

	// Return the number of faces in a polytope
	template <typename = void>
	int FaceCount(ShapePolytope const& shape)
	{
		// Use Euler's formula: F - E + V = 2. => F = 2 + E - V
		return 2 + EdgeCount(shape) - VertCount(shape);
	}

	// Generate the verts of a polytope. 'verts' should point to a buffer of v4's with
	// a length equal to the value returned from 'VertCount'
	template <typename = void>
	void GenerateVerts(ShapePolytope const& shape, v4* verts, v4* verts_end)
	{
		assert("buffer too small" && int(verts_end - verts) >= shape.m_vert_count); (void)verts_end;
		memcpy(verts, shape.vert_beg(), sizeof(v4) * VertCount(shape));
	}

	// Generate the edges of a polytope from the verts and their neighbours. 'edges' should
	// point to a buffer of 2*the number of edges returned from 'EdgeCount'
	template <typename = void>
	void GenerateEdges(ShapePolytope const& shape, v4* edges, v4* edges_end)
	{
		assert("buffer too small" && int(edges_end - edges) >= 2 * EdgeCount(shape));

		int const first_neighbour = 1; // Skip the artificial neighbour
		auto vert_index = 0;
		auto nbr_index = first_neighbour; 
		auto nbrs = &shape.nbrs(vert_index);
		while (vert_index < shape.m_vert_count && edges + 2 <= edges_end)
		{
			*edges++ = shape.vertex(vert_index);
			*edges++ = shape.vertex(nbrs->begin()[nbr_index]);

			// Increment 'vert_index' and 'nbr_index' to refer
			// to the next edge. Only consider edges for which
			// the neighbouring vertex has a higher value. This
			// ensures we only add each edge once.
			do
			{
				if (++nbr_index == nbrs->m_count)
				{
					if (++vert_index == shape.m_vert_count)
						break;

					nbrs = &shape.nbrs(vert_index);
					nbr_index = first_neighbour;
				}
			}
			while (nbrs->begin()[nbr_index] < vert_index);
		}
	}

	// Generate faces for a polytope from the verts and their neighbours.
	template <typename = void>
	void GenerateFaces(ShapePolytope const& shape, int* faces, int* faces_end, int& face_count)
	{
		// Record the start address
		auto faces_start = faces;

		// On scope exit, fill the remaining space in 'faces' with degenerates.
		// Since the verts of the polytope may not all be on the convex hull we may
		// generate less faces than 'faces_end - faces'
		auto s = Scope<void>(
			[&]{ if (faces != faces_end) *faces = 0; },
			[&]{ face_count = int(faces - faces_start); while (faces != faces_end) *faces++ = 0; });

		// Create the starting faces and handle cases for polytopes with less than 3 verts
		for (auto i = 0; i != 3; ++i)
		{
			if (faces == faces_end || i == shape.m_vert_count) return;
			*faces++ = i;
		}
		for (auto i = 3; i-- != 0; )
		{
			if (faces == faces_end) return;
			*faces++ = i;
		}

		struct Edge
		{
			int m_i0, m_i1;
			bool operator == (Edge const& rhs) const
			{
				return
					(m_i0 == rhs.m_i0 && m_i1 == rhs.m_i1) ||
					(m_i0 == rhs.m_i1 && m_i1 == rhs.m_i0);
			}
		};
		pr::vector<Edge, 50, true> edges;

		// Generate the convex hull
		for (auto i = 3; i != shape.m_vert_count; ++i)
		{
			auto& v = shape.vertex(i);
			for (int* f = faces_start, *fend = faces; f != fend; f += 3)
			{
				auto& a = shape.vertex(*(f + 0));
				auto& b = shape.vertex(*(f + 1));
				auto& c = shape.vertex(*(f + 2));

				// If 'v' is in front of this face add its edges to the edge list and remove the face
				if (Triple(v - a, b - a, c - a) >= 0.0f)
				{
					// Add the edges of this face to the edge list (remove duplicates)
					Edge ed = { *(f + 2), *(f + 0) };
					for (int j = 0; j != 3; ++j, ed.m_i0 = ed.m_i1, ed.m_i1 = *(f + j))
					{
						// Look for this edge in the list
						auto e = std::find(std::begin(edges), std::end(edges), ed);
						if (e == edges.end())
						{
							// If not found, add the edge to the list
							edges.push_back(ed);
						}
						else
						{
							// If found, then 'ed' should be the flipped edge
							*e = edges.back();
							edges.pop_back();
						}
						if (edges.size() == edges.max_size())
						{
							assert(!"Edge stack not big enough. GenerateFaces aborted early");
							return;
						}
					}

					// Remove the face
					faces -= 3;
					*(f + 0) = *(faces + 0);
					*(f + 1) = *(faces + 1);
					*(f + 2) = *(faces + 2);
					f -= 3;
					fend -= 3;
				}
			}

			// Add new faces for any edges that are in the edge list
			while (!edges.empty())
			{
				if (faces + 3 > faces_end) return;
				auto e = edges.back();
				edges.pop_back();
				*faces++ = i;
				*faces++ = e.m_i0;
				*faces++ = e.m_i1;
			}
		}
	}

	// Remove the face data from a polytope
	template <typename = void>
	void StripFaces(ShapePolytope& shape)
	{
		if (shape.m_face_count == 0)
			return;

		auto base = reinterpret_cast<char*>(&shape);
		auto src  = reinterpret_cast<char*>(&shape.nbrs(0));
		auto dst  = reinterpret_cast<char*>(shape.face_beg());
		auto size = shape.m_base.m_size;
		auto bytes_to_move = size - (src - base);
		auto bytes_removed = shape.m_face_count * sizeof(ShapePolyFace);

		// Move the remainder of the polytope data back over the face data.
		memmove(dst, src, bytes_to_move);
		shape.m_base.m_size -= bytes_removed;
		shape.m_face_count = 0;
	}

	// Validate a polytope
	template <typename = void>
	bool Validate(ShapePolytope const& shape, bool check_com, char const** err_msg = nullptr)
	{
		auto num_real_nbrs = 0;
		for (auto i = 0; i != shape.m_vert_count; ++i)
		{
			// Check the neighbours of each vertex.
			auto& nbrs = shape.nbrs(i);

			// All polytope verts should have an artificial neighbour plus >0 real neighbours
			if (nbrs.m_count <= 1)
			{
				if (err_msg) *err_msg = FmtS("Vertex %d has an invalid number of neighbours", i);
				return false;
			}

			// Count the number of real neighbours in the polytope
			num_real_nbrs += (nbrs.m_count - 1);

			// Check each neighbour
			for (PolyIdx const *j = nbrs.begin(); j != nbrs.end(); ++j)
			{
				// Check that the neighbour refers to a vert in the polytope
				if (*j >= shape.m_vert_count)
				{
					if (err_msg) *err_msg = FmtS("Vertex %d has a neighbour vertex that is out of range", i);
					return false;
				}

				// Check that the neighbour refers to a different vert in the polytope
				if (*j == i)
				{
					if (err_msg) *err_msg = FmtS("Vertex %d has itself as a neighbour", i);
					return false;
				}

				// Check that there is a neighbour in both directions between 'i' and 'j'
				auto& nbr_nbrs = shape.nbrs(*j);
				auto found = j == nbrs.begin(); // artificial neighbours don't point back
				for (auto k = nbr_nbrs.begin(); k != nbr_nbrs.end() && !found; ++k)
				{
					found = *k == i;
				}
				if (!found)
				{
					if (err_msg) *err_msg = FmtS("Vertex %d has a neighbour that does not also have vertex %d as a neighbour", i, i);
					return false;
				}

				// Check that all neighbours (apart from the artificial neighbour) are unique
				if (j != nbrs.begin())
				{
					for (auto k = j + 1; k != nbrs.end(); ++k)
					{
						if (*k == *j)
						{
							if (err_msg) *err_msg = FmtS("Vertex %d has duplicate neighbours", i);
							return false;
						}
					}
				}
			}
		}

		// Check the polytope describes a closed polyhedron
		if (shape.m_face_count != 0 && shape.m_face_count - (num_real_nbrs / 2) + shape.m_vert_count != 2)
		{
			if (err_msg) *err_msg = "The polytope is not a closed polyhedron!";
			return false;
		}

		// Check the polytope is in centre of mass frame
		if (check_com)
		{
			//m3x4 inertia = CalcInertia(shape);
		}

		return true;
	}
}