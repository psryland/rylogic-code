//**********************************************
// ConvexHull
//  Copyright (c)  Paul Ryland 2008
//**********************************************
#pragma once
#include <concepts>
#include <span>
#include <vector>
#include <iterator>
#include <algorithm>
#include <ranges>
#include <cassert>
#include "pr/math/math.h"

namespace pr::hull
{
	// An object for iterating over faces
	template <int StrideInBytes, typename VIdx>
	union Face
	{
		using VIndex = VIdx;
		static_assert(StrideInBytes >= 3 * sizeof(VIndex));

		VIndex        m_vindex[3];
		unsigned char m_stride[StrideInBytes];

		VIndex vindex(int i) const
		{
			return m_vindex[i];
		}
		VIndex& vindex(int i)
		{
			return m_vindex[i];
		}
	};

	// Get/Set the indices of a face
	template <typename Face, typename VIndex> inline void SetFace(Face& face, VIndex a, VIndex b, VIndex c)
	{
		face.vindex(0) = static_cast<Face::VIndex>(a);
		face.vindex(1) = static_cast<Face::VIndex>(b);
		face.vindex(2) = static_cast<Face::VIndex>(c);
	}
	template <typename Face, typename VIndex> inline void GetFace(Face const& face, VIndex& a, VIndex& b, VIndex& c)
	{
		a = static_cast<VIndex>(face.vindex(0));
		b = static_cast<VIndex>(face.vindex(1));
		c = static_cast<VIndex>(face.vindex(2));
	}

	// A struct containing the parameters provided to ConvexHull()
	// Prevents passing lots of variables around between functions
	template <math::VectorTypeN<4> V, std::integral VIdx, typename FaceT>
	struct HullGenerator
	{
		using Vec = V;
		using VIndex = VIdx;
		using FaceType = FaceT;

		std::span<V const> m_verts;               // Vertex container
		VIdx* m_vbeg;                              // Start of the vert index container
		VIdx* m_vhull_last;                        // One past the last vert index that is on the convex hull
		VIdx* m_vnon_hull;                         // First vert index that is definitely not on the convex hull
		VIdx* m_vend;                              // End of the vert index container
		FaceT* m_fbeg;                             // Start of the face container
		FaceT* m_flast;                            // One past the last face added to the face container
		FaceT* m_fend;                             // End of the face container
		std::vector<V> m_half_spaces;              // Face normals buffer (owned)
		V* m_hs_beg;                               // Start of the half space buffer
		V* m_hs_last;                              // Pointer into the half space buffer, equivalent position to flast
		V* m_hs_end;                               // End of the half space buffer
		float m_eps;                               // Distance within which a point counts as on a plane, scaled to the size of the point set
		std::vector<int> m_visible;                // Indices of the faces that can see the vertex being added (working space)

		HullGenerator(std::span<V const> verts, VIdx* vbeg, VIdx* vend, FaceT* fbeg, FaceT* fend)
			: m_verts(verts)
			, m_vbeg(vbeg)
			, m_vhull_last(vbeg)
			, m_vnon_hull(vend)
			, m_vend(vend)
			, m_fbeg(fbeg)
			, m_flast(fbeg)
			, m_fend(fend)
			, m_half_spaces(static_cast<size_t>(fend - fbeg))
			, m_hs_beg(m_half_spaces.data())
			, m_hs_last(m_half_spaces.data())
			, m_hs_end(m_half_spaces.data() + m_half_spaces.size())
			, m_eps(PlaneTolerance(verts, vbeg, vend))
			, m_visible()
		{}
		HullGenerator(HullGenerator&&) = delete;
		HullGenerator(HullGenerator const&) = delete;
		HullGenerator& operator=(HullGenerator&&) = delete;
		HullGenerator& operator=(HullGenerator const&) = delete;

		// Initialise the convex hull by finding a shape with volume from the bounding verts.
		// This function is set up to return false if any degenerate cases are detected.
		bool InitHull()
		{
			using std::swap;

			// A minimum of 4 verts are needed for a hull with volume
			if (m_vend - m_vbeg < 4)
				return false;

			// Check that there is room for the initial 4 faces
			if (m_fend - m_flast < 4)
				return false;

			// Scan through and find the min/max verts on the Z axis
			auto vmin = m_vbeg;
			auto vmax = m_vbeg;
			{
				auto dmin = +limits<float>::max();
				auto dmax = -limits<float>::max();
				for (auto i = m_vbeg; i != m_vend; ++i)
				{
					auto d = Dot3(v4::ZAxis(), m_verts[*i]);
					if (d < dmin) { dmin = d; vmin = i; }
					if (d > dmax) { dmax = d; vmax = i; }
				}

				// If the span is zero then all verts must lie
				// in a plane parallel to the XY plane.
				if (dmax - dmin <= m_eps)
					return false;
			}

			// Use these extreme verts as the Z axis
			auto zaxis = m_verts[*vmax] - m_verts[*vmin];

			// Move these vert indices to the convex hull end of the range.
			// Ensure vmin is less than vmax, this prevents problems with swapping.
			if (vmax < vmin) { swap(*vmin, *vmax); swap(vmin, vmax); }
			swap(*vmin, *m_vhull_last++);
			swap(*vmax, *m_vhull_last++);

			auto const& zmin = m_verts[*m_vbeg];
			auto zaxis_lensq = LengthSq(zaxis);

			// Find the most radially distant vertex from the zaxis
			{
				float dmax = 0.0f;
				for (auto i = m_vhull_last; i != m_vend; ++i)
				{
					auto vert = m_verts[*i] - zmin;
					auto d = LengthSq(vert) - Sqr(Dot3(vert, zaxis)) / zaxis_lensq;
					if (d > dmax) { dmax = d; vmax = i; }
				}

				// If all verts lie on the zaxis... ('d' is a squared distance)
				if (dmax <= Sqr(m_eps))
					return false;
			}

			// Choose a perpendicular axis
			auto axis = Cross(zaxis, m_verts[*vmax] - zmin);

			// Move 'vmax' to the convex hull end of the range.
			swap(*vmax, *m_vhull_last++);

			bool flip = false;

			// Find the vert with the greatest distance along 'axis'
			{
				float dmax = 0.0f;
				for (auto i = m_vhull_last; i != m_vend; ++i)
				{
					auto d = Dot3(axis, m_verts[*i] - zmin);
					if (Abs(d) > dmax) { dmax = Abs(d); vmax = i; flip = d < 0.0f; }
				}

				// If all verts lie on in the plane... ('axis' is not normalised, so scale the tolerance by its length)
				if (dmax <= m_eps * Length(axis))
					return false;
			}

			// Move 'vmax' to the convex hull end of the range.
			swap(*vmax, *m_vhull_last++);

			// Generate the starting shape from the four hull verts we've found.
			if (flip)
			{
				AddFace(0, 1, 2);
				AddFace(0, 2, 3);
				AddFace(0, 3, 1);
				AddFace(3, 2, 1);
			}
			else
			{
				AddFace(0, 2, 1);
				AddFace(0, 3, 2);
				AddFace(0, 1, 3);
				AddFace(1, 2, 3);
			}
			return true;
		}

		// The distance within which a point counts as lying on a face plane. Plane distances are computed in float, so their rounding error grows with the size
		// of the coordinates. Using one tolerance for every visibility test keeps those tests consistent with each other, so nearly coplanar points cannot
		// make a face look visible from one test and hidden from another.
		static float PlaneTolerance(std::span<V const> verts, VIdx const* vbeg, VIdx const* vend)
		{
			// Sum the largest coordinate magnitude on each axis, then allow a few float rounding steps of that size.
			auto max_abs = V{};
			for (auto i = vbeg; i != vend; ++i)
			{
				auto const& v = verts[*i];
				max_abs.x = Max(max_abs.x, Abs(v.x));
				max_abs.y = Max(max_abs.y, Abs(v.y));
				max_abs.z = Max(max_abs.z, Abs(v.z));
			}
			return 10.0f * limits<float>::epsilon() * (max_abs.x + max_abs.y + max_abs.z);
		}

		// Move vert indices that are inside the current hull (or within the plane tolerance of it) to the 'non-hull' end of the range.
		// Returns the vertex with the greatest distance outside any face of the hull. This must be a vertex on the final convex hull
		// because it is the extreme vertex in the direction of that face's normal. Returns 'm_vnon_hull' when no vertices remain.
		VIdx* PartitionVerts()
		{
			using std::swap;

			auto max_dist = 0.0f;
			auto max_vert = m_vnon_hull;
			for (auto v = m_vhull_last; v != m_vnon_hull;)
			{
				// Find the maximum distance of 'v' in front of the faces of the hull
				auto dist = 0.0f;
				for (auto plane = m_hs_beg; plane != m_hs_last; ++plane)
					dist = Max(dist, Dot(*plane, m_verts[*v]));

				// A vert within the tolerance of every half space is not a hull vert
				if (dist <= m_eps)
				{
					swap(*v, *(--m_vnon_hull));
					continue;
				}

				// If this vert has the greatest distance from a face, record it.
				if (dist > max_dist)
				{
					max_dist = dist;
					max_vert = v;
				}
				++v;
			}
			return max_vert;
		}

		// Expand the convex hull to include 'v', a vert that is more than the plane tolerance in front of at least one face.
		// Returns false if the face container does not have room for the new faces.
		bool GrowHull(VIdx* v)
		{
			using std::swap;

			// Find the faces that can see 'v'. The same tolerance as 'PartitionVerts' guarantees at least one.
			auto const& vert = m_verts[*v];
			m_visible.resize(0);
			for (auto plane = m_hs_beg; plane != m_hs_last; ++plane)
			{
				if (Dot(*plane, vert) > m_eps)
					m_visible.push_back(static_cast<int>(plane - m_hs_beg));
			}
			assert(!m_visible.empty() && "PartitionVerts should only return verts that can see a face");

			// Collect the horizon: the edges of the visible faces that are not shared by two visible faces. At most 3 edges per face.
			std::vector<typename Perimeter::Edge> edge_buf(3 * m_visible.size());
			Perimeter perimeter(edge_buf.data(), static_cast<int>(edge_buf.size()));
			for (auto i : m_visible)
			{
				VIndex a, b, c;
				GetFace(m_fbeg[i], a, b, c);
				perimeter.AddEdge(a, b);
				perimeter.AddEdge(b, c);
				perimeter.AddEdge(c, a);
			}

			// The new faces only close the hull if the horizon is one simple loop (the visible faces form a disc). Rounding error can break this
			// when 'v' is within a few rounding steps of the hull. Such a vert adds nothing of value, so treat it as not on the hull.
			if (!perimeter.IsSimpleLoop())
			{
				swap(*v, *(--m_vnon_hull));
				return true;
			}

			// Check there is room for the new faces. One face is added per horizon edge after the visible faces are removed.
			auto new_face_count = perimeter.m_last - perimeter.m_beg;
			if ((m_fend - m_flast) + static_cast<ptrdiff_t>(m_visible.size()) < new_face_count)
				return false;

			// Remove the visible faces. 'm_visible' is in increasing order, so erasing from the back by swapping with the last face never moves a visible face.
			for (auto i = m_visible.size(); i-- != 0; )
			{
				swap(m_fbeg[m_visible[i]], *(--m_flast));
				swap(m_hs_beg[m_visible[i]], *(--m_hs_last));
			}

			// Move 'v' into the convex hull and add a face for each horizon edge
			swap(*v, *m_vhull_last);
			auto v_idx = static_cast<VIndex>(m_vhull_last - m_vbeg);
			++m_vhull_last;
			for (auto edge = perimeter.m_beg; edge != perimeter.m_last; ++edge)
				AddFace(v_idx, edge->m_i0, edge->m_i1);

			return true;
		}

		// Add a face to the face container
		void AddFace(VIndex a, VIndex b, VIndex c)
		{
			// Set the face in the face container
			SetFace(*m_flast, a, b, c);
			++m_flast;

			// Record the half space that this face represents
			auto& A = m_verts[*(m_vbeg + a)];
			auto e0 = m_verts[*(m_vbeg + b)] - A;
			auto e1 = m_verts[*(m_vbeg + c)] - A;
			auto& plane = *m_hs_last;
			plane = Normalise(Cross(e0, e1));
			plane.w = -Dot3(plane, A);
			++m_hs_last;
		}

		// A structure used to keep track of the perimeter edges as faces are removed from the hull.
		struct Perimeter
		{
			struct Edge { VIndex m_i0, m_i1; };
			Edge* m_beg;
			Edge* m_last;
			Edge* m_end;

			Perimeter(Edge* buf, int count)
				: m_beg(buf)
				, m_last(m_beg)
				, m_end(m_beg + count)
			{}

			// Add a directed edge, or cancel it against its reverse if that is already present.
			void AddEdge(VIndex i0, VIndex i1)
			{
				for (auto e = m_beg; e != m_last; ++e)
				{
					if (!(e->m_i0 == i1 && e->m_i1 == i0))
						continue;

					// erase 'e'
					*e = *(--m_last);
					return;
				}
				assert(m_last != m_end && "Perimeter edge buffer overflow");
				m_last->m_i0 = i0;
				m_last->m_i1 = i1;
				++m_last;
			}

			// True if the edges form one closed loop that visits each vertex once.
			bool IsSimpleLoop() const
			{
				// A loop needs at least three edges, and each vertex may start only one edge.
				auto count = m_last - m_beg;
				if (count < 3)
					return false;

				for (auto e = m_beg; e != m_last; ++e)
				{
					for (auto f = e + 1; f != m_last; ++f)
					{
						if (f->m_i0 == e->m_i0)
							return false;
					}
				}

				// Follow the edges from the first one. With unique start vertices, returning to the start after 'count' steps means one loop covers every edge.
				auto edge = m_beg;
				for (auto step = 1; step != count; ++step)
				{
					auto next = std::find_if(m_beg, m_last, [&](Edge const& x) { return x.m_i0 == edge->m_i1; });
					if (next == m_last || next == m_beg)
						return false;

					edge = next;
				}
				return edge->m_i1 == m_beg->m_i0;
			}
		};
	};

	// Generate the convex hull of a point cloud.
	// 'verts' is a contiguous range of vertices (e.g. std::span<v4 const>, std::vector<v4>, v4 array, etc.)
	// 'vidx' is a span of vertex indices into 'verts' (i.e. the point cloud). This span is partitioned with hull verts at the front.
	// 'faces' is a span of faces used as working/output space
	// This function partitions the range of vert indices with those on the convex hull at the start of the range.
	// Returns true if the convex hull was generated. If false is returned and vert_count and face_count are not zero,
	// then a convex closed polytope was generated.
	// Notes:
	//  - A point cloud of N verts will have a convex hull with at most 2*(N-2) faces.
	//  - The faces output by this function are not necessarily the hull faces until the algorithm
	//    has completed. 'faces' should point to a container of faces. Faces in this
	//    container may be written to and read from several times.
	//  - The indices in the faces refer to the positions in the vert index container. This allows
	//    new or old vert indices to be used. For new indices, map the verts using the vert index
	//    buffer. For old indices use the vert index buffer as a look up map.
	//  - You may need to provide the "hull::SetFace" and "hull::GetFace" functions for your face
	//    and vert index types.
	template <std::ranges::contiguous_range VertRange, std::integral VIdx, typename FaceT>
		requires math::VectorTypeN<std::ranges::range_value_t<VertRange>, 4>
	bool ConvexHull(VertRange const& verts, std::span<VIdx> vidx, std::span<FaceT> faces, size_t& vert_count, size_t& face_count)
	{
		using V = std::ranges::range_value_t<VertRange>;

		vert_count = 0;
		face_count = 0;

		std::span<V const> vert_span(std::ranges::data(verts), std::ranges::size(verts));

		// Add all of the parameters to a struct to save on passing them around between function calls
		hull::HullGenerator<V, VIdx, FaceT> data(vert_span, vidx.data(), vidx.data() + vidx.size(), faces.data(), faces.data() + faces.size());

		// Find an initial volume
		if (!data.InitHull())
			return false;

		// Test the unclassified verts and move any that are within the convex hull to
		// the 'non-hull' end of the range. 'v' is the vert most not in the convex hull.
		auto v = data.PartitionVerts();

		// While there are unclassified verts remaining
		while (data.m_vhull_last != data.m_vnon_hull)
		{
			// Expand the convex hull to include 'v'. Stop if the face range has no room for the new faces.
			if (!data.GrowHull(v))
				break;

			// Test the unclassified verts and move any that are within the convex hull to
			// the 'non-hull' end of the range. Find a new most not in the convex hull vertex.
			v = data.PartitionVerts();
		}

		vert_count = static_cast<size_t>(data.m_vhull_last - data.m_vbeg);
		face_count = static_cast<size_t>(data.m_flast - data.m_fbeg);
		return data.m_vhull_last == data.m_vnon_hull;
	}
	
	// Overload that reorders the verts in the vertex container.
	// 'verts' is a mutable contiguous range of vertices
	// 'faces' is a span of faces used as working/output space
	// Other parameters as for the main version of this function.
	template <std::ranges::contiguous_range VertRange, typename FaceT>
		requires math::VectorTypeN<std::ranges::range_value_t<VertRange>, 4>
	bool ConvexHull(VertRange& verts, std::span<FaceT> faces, size_t& vert_count, size_t& face_count)
	{
		using V = std::ranges::range_value_t<VertRange>;
		auto num_verts = std::ranges::size(verts);

		// Create an index buffer for the verts
		std::vector<int> index(num_verts);
		for (int i = 0; i != static_cast<int>(num_verts); ++i)
			index[i] = i;

		// Find the hull
		auto result = ConvexHull(verts, std::span<int>{index}, faces, vert_count, face_count);

		// Write the indices into the 'w' component of the verts
		for (auto* i = index.data(), *i_end = i + num_verts; i != i_end; ++i)
			verts[*i].w = static_cast<float>(i - index.data());

		// Then sort on 'w'
		auto* vdata = std::ranges::data(verts);
		std::sort(vdata, vdata + num_verts, [](V const& lhs, V const& rhs) { return lhs.w < rhs.w; });
		
		// Then restore the 'w' component to '1.0f'
		for (auto* i = vdata, *i_end = i + num_verts; i != i_end; ++i)
			i->w = 1.0f;

		return result;
	}

	// Overload for faces given as an array of index triples.
	template <std::ranges::contiguous_range VertRange, std::integral VIdx>
		requires math::VectorTypeN<std::ranges::range_value_t<VertRange>, 4>
	bool ConvexHull(VertRange const& verts, std::span<VIdx> vidx, std::span<VIdx> face_indices, size_t& vert_count, size_t& face_count)
	{
		using FaceType = hull::Face<3 * sizeof(VIdx), VIdx>;

		auto face_beg = reinterpret_cast<FaceType*>(face_indices.data());
		auto face_end = reinterpret_cast<FaceType*>(face_indices.data() + face_indices.size());
		return ConvexHull(verts, vidx, std::span<FaceType>{face_beg, face_end}, vert_count, face_count);
	}

	// Overload that reorders the verts in the vertex container
	// and for faces given as an array of index triples.
	template <std::ranges::contiguous_range VertRange, std::integral VIdx>
		requires math::VectorTypeN<std::ranges::range_value_t<VertRange>, 4>
	bool ConvexHull(VertRange& verts, std::span<VIdx> face_indices, size_t& vert_count, size_t& face_count)
	{
		using FaceType = hull::Face<3 * sizeof(VIdx), VIdx>;

		auto face_beg = reinterpret_cast<FaceType*>(face_indices.data());
		auto face_end = reinterpret_cast<FaceType*>(face_indices.data() + face_indices.size());
		return ConvexHull(verts, std::span<FaceType>{face_beg, face_end}, vert_count, face_count);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"

namespace pr::geometry::tests
{
	PRUnitTestClass(ConvexHullTests)
	{
		// Helper: run convex hull on a set of points and return hull vert/face counts
		static bool RunHull(v4 const* pts, int count, size_t& vert_count, size_t& face_count)
		{
			std::vector<int> indices(count);
			for (int i = 0; i != count; ++i) indices[i] = i;

			auto max_faces = 2 * (count - 2);
			std::vector<int> faces(max_faces * 3);

			return hull::ConvexHull(
				std::span<v4 const>{pts, static_cast<size_t>(count)},
				std::span<int>{indices},
				std::span<int>{faces},
				vert_count, face_count);
		}

		// Four points forming a tetrahedron — simplest valid hull
		PRUnitTestMethod(Tetrahedron, Quick)
		{
			v4 pts[] = {
				v4{0, 0, 0, 1},
				v4{1, 0, 0, 1},
				v4{0, 1, 0, 1},
				v4{0, 0, 1, 1},
			};
			size_t vc = 0, fc = 0;
			PR_EXPECT(RunHull(pts, 4, vc, fc));
			PR_EXPECT(vc == 4);
			PR_EXPECT(fc == 4); // Tetrahedron has 4 faces
		}

		// Cube: 8 vertices, expected hull is all 8 verts with 12 triangular faces
		PRUnitTestMethod(Cube, Quick)
		{
			v4 pts[] = {
				v4{-1, -1, -1, 1}, v4{ 1, -1, -1, 1},
				v4{-1,  1, -1, 1}, v4{ 1,  1, -1, 1},
				v4{-1, -1,  1, 1}, v4{ 1, -1,  1, 1},
				v4{-1,  1,  1, 1}, v4{ 1,  1,  1, 1},
			};
			size_t vc = 0, fc = 0;
			PR_EXPECT(RunHull(pts, 8, vc, fc));
			PR_EXPECT(vc == 8);
			PR_EXPECT(fc == 12); // Cube = 6 quads = 12 triangles
		}

		// Points with interior points — only hull verts should be counted
		PRUnitTestMethod(InteriorPointsFiltered, Quick)
		{
			v4 pts[] = {
				// Cube corners (hull vertices)
				v4{-2, -2, -2, 1}, v4{ 2, -2, -2, 1},
				v4{-2,  2, -2, 1}, v4{ 2,  2, -2, 1},
				v4{-2, -2,  2, 1}, v4{ 2, -2,  2, 1},
				v4{-2,  2,  2, 1}, v4{ 2,  2,  2, 1},
				// Interior points (should be filtered out)
				v4{0, 0, 0, 1},
				v4{1, 0, 0, 1},
				v4{0, 1, 0, 1},
				v4{0, 0, 1, 1},
			};
			size_t vc = 0, fc = 0;
			PR_EXPECT(RunHull(pts, 12, vc, fc));
			PR_EXPECT(vc == 8);  // Only the 8 cube corners
			PR_EXPECT(fc == 12);
		}

		// Degenerate: coplanar points should fail (no volume)
		PRUnitTestMethod(CoplanarPointsFail, Quick)
		{
			v4 pts[] = {
				v4{0, 0, 0, 1},
				v4{1, 0, 0, 1},
				v4{0, 1, 0, 1},
				v4{1, 1, 0, 1},
				v4{0.5f, 0.5f, 0, 1},
			};
			size_t vc = 0, fc = 0;
			PR_EXPECT(!RunHull(pts, 5, vc, fc));
		}

		// Degenerate: collinear points should fail
		PRUnitTestMethod(CollinearPointsFail, Quick)
		{
			v4 pts[] = {
				v4{0, 0, 0, 1},
				v4{1, 0, 0, 1},
				v4{2, 0, 0, 1},
				v4{3, 0, 0, 1},
			};
			size_t vc = 0, fc = 0;
			PR_EXPECT(!RunHull(pts, 4, vc, fc));
		}

		// Irregular shape: 6 points forming an octahedron
		PRUnitTestMethod(Octahedron, Quick)
		{
			v4 pts[] = {
				v4{ 1, 0, 0, 1}, v4{-1, 0, 0, 1},
				v4{ 0, 1, 0, 1}, v4{ 0,-1, 0, 1},
				v4{ 0, 0, 1, 1}, v4{ 0, 0,-1, 1},
			};
			size_t vc = 0, fc = 0;
			PR_EXPECT(RunHull(pts, 6, vc, fc));
			PR_EXPECT(vc == 6);
			PR_EXPECT(fc == 8); // Octahedron has 8 triangular faces
		}

		// Many points lying exactly on shared planes (as produced by cutting a point cloud with planes) must still give a closed, manifold hull at any scale
		PRUnitTestMethod(CoplanarFacetsStayManifold, Quick)
		{
			for (auto scale : { 0.01f, 0.1f, 1.0f, 10.0f, 1000.0f })
			{
				for (auto seed = 1u; seed != 9u; ++seed)
				{
					// A lumpy sphere of points with several sides flattened by projecting the points beyond a plane onto it
					auto rng = seed * 2654435761u;
					auto next = [&] { rng = rng * 1664525u + 1013904223u; return static_cast<float>(rng >> 8) / static_cast<float>(1u << 24); };
					std::vector<v4> pts;
					for (int i = 0; i != 120; ++i)
					{
						auto dir = Normalise(v4{ next() - 0.5f, next() - 0.5f, next() - 0.5f, 0 });
						pts.push_back(dir * (1.0f - 0.2f * next()) + v4::Origin());
					}
					for (int cut = 0; cut != 6; ++cut)
					{
						auto n = Normalise(v4{ next() - 0.5f, next() - 0.5f, next() - 0.5f, 0 });
						auto offset = 0.5f + 0.3f * next();
						for (auto& p : pts)
						{
							auto excess = Dot3(p, n) - offset;
							if (excess > 0)
								p -= n * excess;
						}
					}
					for (auto& p : pts)
						p = (p - v4::Origin()) * scale + v4::Origin();

					size_t vc = 0, fc = 0;
					std::vector<int> indices(pts.size());
					for (int i = 0; i != static_cast<int>(pts.size()); ++i) indices[i] = i;
					std::vector<int> faces(2 * (pts.size() - 2) * 3);
					PR_EXPECT(hull::ConvexHull(std::span<v4 const>{pts}, std::span<int>{indices}, std::span<int>{faces}, vc, fc));
					PR_EXPECT(fc >= 4);
					PR_EXPECT(fc <= 2 * (vc - 2));

					// Every directed edge appears once, and its reverse once, so the faces close the hull without non-manifold edges
					std::vector<std::pair<int, int>> edges;
					for (size_t f = 0; f != fc; ++f)
					{
						for (int k = 0; k != 3; ++k)
							edges.emplace_back(faces[f * 3 + k], faces[f * 3 + (k + 1) % 3]);
					}
					std::ranges::sort(edges);
					PR_EXPECT(std::ranges::adjacent_find(edges) == edges.end());
					for (auto const& [a, b] : edges)
						PR_EXPECT(std::ranges::binary_search(edges, std::pair{ b, a }));
				}
			}
		}
	};
}
#endif
