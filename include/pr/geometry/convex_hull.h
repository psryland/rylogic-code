//**********************************************
// ConvexHull
//  Copyright (c)  Paul Ryland 2008
//**********************************************
#pragma once

#include <iterator>
#include <algorithm>
#include "pr/common/assert.h"
#include "pr/common/alloca.h"
#include "pr/math/math.h"

#ifndef PR_DBG_CONVEX_HULL
#define PR_DBG_CONVEX_HULL 0
#endif
#if PR_DBG_CONVEX_HULL
#include "pr/linedrawer/ldr_helper.h"
#endif

namespace pr
{
	namespace hull
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
		template <typename VertCont, typename VIter, typename FIter>
		struct HullGenerator
		{
			using VertIter = VIter;
			using FaceIter = FIter;
			using VIndex = typename std::iterator_traits<VIter>::value_type;

			static constexpr int MaxVisFaceCount = 64;
			#if PR_DBG_CONVEX_HULL
			inline static HullGenerator const* g_data = nullptr;
			#endif

			VertCont const& m_vcont;              // Vertex container
			VIter m_vbeg;                         // Start of the vert index container
			VIter m_vhull_last;                   // One past the last vert index that is on the convex hull
			VIter m_vnon_hull;                    // First vert index that is definitely not on the convex hull
			VIter m_vend;                         // End of the vert index container
			FIter m_fbeg;                         // Start of the face container
			FIter m_flast;                        // One past the last face added to the face container
			FIter m_fend;                         // End of the face container
			v4* m_hs_beg;                         // Start of a buffer of face normals with the same length as fend - fbegin
			v4* m_hs_last;                        // Pointer into the half space buffer, equilavent position to flast
			v4* m_hs_end;                         // End of the buffer of face normals
			int m_vis_face_count;                 // The number of visible faces
			int m_vis_face_buf1[MaxVisFaceCount]; // Double buffer for the
			int m_vis_face_buf2[MaxVisFaceCount]; //  indices of visible faces
			int* m_visible_face;                  // Pointer to one of the m_vis_face_buf's

			HullGenerator(VertCont const& vcont, VIter vbeg, VIter vend, FIter fbeg, FIter fend, v4* half_space)
				: m_vcont(vcont)
				, m_vbeg(vbeg)
				, m_vhull_last(vbeg)
				, m_vnon_hull(vend)
				, m_vend(vend)
				, m_fbeg(fbeg)
				, m_flast(fbeg)
				, m_fend(fend)
				, m_hs_beg(half_space)
				, m_hs_last(half_space)
				, m_hs_end(half_space + (fend - fbeg))
				, m_vis_face_count(0)
				, m_vis_face_buf1()
				, m_vis_face_buf2()
				, m_visible_face(m_vis_face_buf1)
			{
				PR_EXPAND(PR_DBG_CONVEX_HULL, g_data = this);
				PR_EXPAND(PR_DBG_CONVEX_HULL, DumpVerts());
				PR_EXPAND(PR_DBG_CONVEX_HULL, DumpFaces());
			}
			HullGenerator(HullGenerator const&) = delete;
			HullGenerator& operator=(HullGenerator const&) = delete;

			// Initialise the convex hull by finding a shape with volume from the bounding verts
			// This function is set up to return false if any degenerate cases are detected.
			bool InitHull()
			{
				PR_EXPAND(PR_DBG_CONVEX_HULL, ldr::FileOutput ldr_extm("P:\\dump\\hull_extremeverts.ldr", true); ldr_extm.clear());
				using std::swap;

				// A minimum of 4 verts are needed for a hull with volume
				if (m_vend - m_vbeg < 4)
					return false;

				// Check that there is room for the initial 4 faces
				if (m_fend - m_flast < 4)
					return false;

				// Scan through and find the min/max verts on the Z axis
				VertIter vmin = m_vbeg, vmax = m_vbeg;
				{
					auto dmin = +limits<float>::max();
					auto dmax = -limits<float>::max();
					for (auto i = m_vbeg; i != m_vend; ++i)
					{
						auto d = Dot3(v4::ZAxis(), m_vcont[*i]);
						if (d < dmin) { dmin = d; vmin = i; }
						if (d > dmax) { dmax = d; vmax = i; }
					}

					// If the span is zero then all verts must lie
					// in a plane parallel to the XY plane.
					if (dmax - dmin < math::tiny<float>)
						return false;

					PR_EXPAND(PR_DBG_CONVEX_HULL, ldr::Line("zaxis", "FF0000FF", m_vcont[*vmin], m_vcont[*vmax], ldr_extm));
				}

				// Use these extreme verts as the Z axis
				auto zaxis = m_vcont[*vmax] - m_vcont[*vmin];

				// Move these vert indices to the convex hull end of the range.
				// Ensure vmin is less than vmax, this prevents problems with swapping.
				if (vmax < vmin) { swap(*vmin, *vmax); swap(vmin, vmax); }
				swap(*vmin, *m_vhull_last++);
				swap(*vmax, *m_vhull_last++);

				auto const& zmin = m_vcont[*m_vbeg];
				auto zaxis_lensq = LengthSq(zaxis);

				// Find the most radially distant vertex from the zaxis
				{
					float dmax = 0.0f;
					for (auto i = m_vhull_last; i != m_vend; ++i)
					{
						auto vert = m_vcont[*i] - zmin;
						auto d = LengthSq(vert) - Sqr(Dot3(vert, zaxis)) / zaxis_lensq;
						if (d > dmax) { dmax = d; vmax = i; }
					}

					// If all verts lie on the zaxis...
					if (dmax < math::tiny<float>)
						return false;
				
					PR_EXPAND(PR_DBG_CONVEX_HULL, ldr::Line("yaxis", "FF00FF00", zmin, m_vcont[*vmax], ldr_extm));
				}

				// Choose a perpendicular axis
				auto axis = Cross(zaxis, m_vcont[*vmax] - zmin);

				// Move 'vmax' to the convex hull end of the range.
				swap(*vmax, *m_vhull_last++);

				bool flip = false;

				// Find the vert with the greatest distance along 'axis'
				{
					float dmax = 0.0f;
					for (auto i = m_vhull_last; i != m_vend; ++i)
					{
						auto d = Dot3(axis, m_vcont[*i] - zmin);
						if (Abs(d) > dmax) { dmax = Abs(d); vmax = i; flip = d < 0.0f; }
					}

					// If all verts lie on in the plane...
					if (dmax < math::tiny<float>)
						return false;
				
					PR_EXPAND(PR_DBG_CONVEX_HULL, ldr::Line("xaxis", "FFFF0000", zmin, m_vcont[*vmax], ldr_extm));
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

			// Move vert indices that are inside the current hull to the 'non-hull' end of the range.
			// Records the vertex with the greatest distance from any face in the hull, this must be
			// a vertex on the convex hull because it is the extreme vertex in the direction of the
			// face normal. Returns this extreme vert, also caches the indices of the faces that can
			// see this max vert.
			VertIter PartitionVerts()
			{
				using std::swap;

				m_vis_face_count = 0;
				auto max_dist = 0.0f;
				auto max_vert = m_vnon_hull;
				for (auto v = m_vhull_last; v != m_vnon_hull;)
				{
					// Find the maximum distance of 'v' from the faces of the hull
					auto dist = 0.0f;
					int vis_count = 0, face_index = 0;
					for (auto plane = m_hs_beg; plane != m_hs_last; ++plane, ++face_index)
					{
						PR_ASSERT(PR_DBG_CONVEX_HULL, m_vcont[*v].w == 1.0f, "Should be finding the convex hull of positions, not directions");
						auto d = Dot(*plane, m_vcont[*v]);
						if (d <= 0.0f)
							continue; // behind 'plane'

						dist = Max(dist, d);
						if (vis_count < MaxVisFaceCount)
							m_visible_face[vis_count] = face_index;

						++vis_count;
					}

					// If this vert is within all half spaces then it's not a hull vert
					if (dist == 0.0f)
					{
						swap(*v, *(--m_vnon_hull));
						continue;
					}

					// If this vert has the greatest distance from a face, record it.
					if (dist > max_dist)
					{
						max_dist = dist;
						max_vert = v;
						m_vis_face_count = vis_count;
						SwapVisFaceDblBuffer();
					}
					++v;
				}
				SwapVisFaceDblBuffer();
				return max_vert;
			}

			// Expand the convex hull to include 'v'.
			void GrowHull(VertIter v)
			{
				using std::swap;

				// Move 'v' into the convex hull
				swap(*v, *m_vhull_last);
				auto v_idx = static_cast<VIndex>(m_vhull_last - m_vbeg);
				auto const& vert = m_vcont[*m_vhull_last];
				++m_vhull_last;

				// Allocate stack memory for the edges remaining when the faces that can
				// see 'v' are removed. The number of perimeter edges will ultimately be
				// 2 * vis_face_count, however in the worst case we can receive the faces
				// in an order that doesn't sharing any edges so allow for 3 edges for each face.
				int max_edge_count = 3 * m_vis_face_count;
				Perimeter perimeter(PR_ALLOCA_POD(Perimeter::Edge, max_edge_count), max_edge_count);

				// If the number of visible faces is less than the size of
				// the visible face cache then we don't need to retest the faces.
				if (m_vis_face_count <= MaxVisFaceCount)
				{
					for (auto i = m_visible_face + m_vis_face_count; i-- != m_visible_face; )
					{
						auto face = m_fbeg + *i;
						auto plane = m_hs_beg + *i;

						VIndex a, b, c;
						GetFace(*face, a, b, c);

						// Add the edges of 'face' to the edge stack
						perimeter.AddEdge(a, b);
						perimeter.AddEdge(b, c);
						perimeter.AddEdge(c, a);

						// Erase 'face'
						swap(*face, *(--m_flast));
						swap(*plane, *(--m_hs_last));
						PR_EXPAND(PR_DBG_CONVEX_HULL, data.DumpFaces());
					}
				}
				// Otherwise we need to test all faces again for being able to see 'v'
				else
				{
					auto plane = m_hs_beg;
					for (auto face = m_fbeg; !(face == m_flast); )
					{
						// Ignore faces that face away or are edge on to 'v'
						if (Dot(*plane, vert) <= 0.0f)
						{
							++face;
							++plane;
							continue;
						}

						VIndex a, b, c;
						GetFace(*face, a, b, c);

						// Add the edges of 'face' to the edge stack
						perimeter.AddEdge(a, b);
						perimeter.AddEdge(b, c);
						perimeter.AddEdge(c, a);

						// Erase 'face'
						swap(*face, *(--m_flast));
						swap(*plane, *(--m_hs_last));
						PR_EXPAND(PR_DBG_CONVEX_HULL, data.DumpFaces());
					}
				}

				// Add faces for each remaining edge
				for (auto edge = perimeter.m_beg; edge != perimeter.m_last; ++edge)
				{
					AddFace(v_idx, edge->m_i0, edge->m_i1);
				}
			}

			// Add a face to the face container
			void AddFace(VIndex a, VIndex b, VIndex c)
			{
				PR_ASSERT(PR_DBG_CONVEX_HULL, a != b && b != c && c != a, "Should not be adding degenerate faces");
				PR_ASSERT(PR_DBG_CONVEX_HULL, !(m_flast == m_fend), "Shouldn't be trying to add faces if there isn't room");

				// Set the face in the face container
				SetFace(*m_flast, a, b, c);
				++m_flast;

				// Record the half space that this face represents
				auto& A = m_vcont[*(m_vbeg + a)];
				auto e0 = m_vcont[*(m_vbeg + b)] - A;
				auto e1 = m_vcont[*(m_vbeg + c)] - A;
				auto& plane = *m_hs_last;
				plane = Normalise(Cross(e0, e1));
				plane.w = -Dot3(plane, A);
				++m_hs_last;

				PR_EXPAND(PR_DBG_CONVEX_HULL, data.DumpFaces());
			}

			// Swap the visible face double buffer
			void SwapVisFaceDblBuffer()
			{
				m_visible_face = m_visible_face == m_vis_face_buf1 ? m_vis_face_buf2 : m_vis_face_buf1;
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
				void AddEdge(VIndex i0, VIndex i1)
				{
					for (auto e = m_beg; e != m_last; ++e)
					{
						if (!(e->m_i0 == i1 && e->m_i1 == i0))
							continue;

						// erase 'e'
						*e = *(--m_last);
						PR_EXPAND(PR_DBG_CONVEX_HULL, DumpEdges());
						return;
					}
					PR_ASSERT(PR_DBG_CONVEX_HULL, m_last != m_end, "");
					m_last->m_i0 = i0;
					m_last->m_i1 = i1;
					++m_last;
					PR_EXPAND(PR_DBG_CONVEX_HULL, DumpEdges());
				}

				#if PR_DBG_CONVEX_HULL
				void DumpEdges()
				{
					ldr::FileOutput file("P:\\dump\\hull_edges.ldr"); file.Open();
					ldr::GroupStart("Edges", file);
					for (auto i = m_beg; i != m_last; ++i)
					{
						ldr::Line("edge", "FFFFFFFF",
							g_data->m_vcont[g_data->m_vbeg[i->m_i0]],
							g_data->m_vcont[g_data->m_vbeg[i->m_i1]],
							file);
					}
					ldr::GroupEnd(file);
				}
				#endif
			};

			// Debugging functions
			#if PR_DBG_CONVEX_HULL
			void DumpFaces()
			{
				ldr::FileOutput file("P:\\dump\\hull_faces.ldr"); file.Open();
				ldr::GroupStart("Faces", file);
				for (auto f = m_fbeg; !(f == m_flast); ++f)
				{
					VIndex i0, i1, i2; GetFace(*f, i0, i1, i2);
					ldr::Triangle("face", "FF00FF00",
						m_vcont[*(m_vbeg + i0)],
						m_vcont[*(m_vbeg + i1)],
						m_vcont[*(m_vbeg + i2)],
						file);
				}
				ldr::GroupEnd(file);
			}
			void DumpVerts()
			{
				ldr::FileOutput file("P:\\dump\\hull_verts.ldr"); file.Open();
				ldr::GroupStart("Verts", file);
				for (auto v = m_vbeg; v != m_vend; ++v)
				{
					if      (v < m_vhull_last) ldr::Box("v", "FFA00000", m_vcont[*v], 0.05f, file);
					else if (v < m_vnon_hull)  ldr::Box("v", "FF00A000", m_vcont[*v], 0.05f, file);
					else                            ldr::Box("v", "FF0000A0", m_vcont[*v], 0.05f, file);
				}
				ldr::GroupEnd(file);
			}
			#endif
		};
	}

	// Generate the convex hull of a point cloud.
	// 'vcont' is some container of verts that supports operator [] (e.g. v4 const*)
	// 'vbeg','vend' are a range of vertex indices into 'vcont' (i.e. the point cloud)
	// 'fbeg','fend' are an output iterator range to faces
	// This function partitions the range of vert indices with those on the convex hull at the start of the range.
	// Returns true if the convex hull was generated. If false is returned and vert_count and face_count are not zero,
	// then a convex closed polytope was generated.
	// Notes:
	//  - A point cloud of N verts will have a convex hull with at most 2*(N-2) faces.
	//  - The faces output by this function are not necessarily the hull faces until the algorithm
	//    has completed. 'fbeg' and 'fend' should point to a container of faces. Faces in this
	//    container may be written to and read from several times.
	//  - The indices in the faces refer to the positions in the vert index container. This allows
	//    new or old vert indices to be used. For new indices, map the verts using the vert index
	//    buffer. For old indices use the vert index bufer as a look up map.
	//  - You may need to provide the "hull::SetFace" and "hull::GetFace" functions for your face
	//    and vert index types.
	template <typename VertCont, typename VIter, typename FIter>
	bool ConvexHull(VertCont const& vcont, VIter vbeg, VIter vend, FIter fbeg, FIter fend, size_t& vert_count, size_t& face_count)
	{
		vert_count = 0;
		face_count = 0;

		// Allocate stack or heap memory for the normal directions of the faces.
		v4* half_space = PR_MALLOCA(half_space, v4, fend - fbeg);

		// Add all of the parameters to a struct to save on passing them around between function calls
		hull::HullGenerator<VertCont, VIter, FIter> data(vcont, vbeg, vend, fbeg, fend, half_space);

		// Find an initial volume
		if (!data.InitHull())
			return false;

		// Test the unclassified verts and move any that are within the convex hull to
		// the 'non-hull' end of the range. 'v' is the vert most not in the convex hull.
		// 'vis_count' is the number of faces of the current hull that can "see" 'v'
		auto v = data.PartitionVerts();

		// While there are unclassified verts remaining
		PR_EXPAND(PR_DBG_CONVEX_HULL, int iteration = 0);
		while (data.m_vhull_last != data.m_vnon_hull)
		{
			// Check there is enough space in the face range to expand the convex hull.
			// Since the hull is convex, the faces that can see 'v' will form one connected
			// polygon. This means the number of perimeter edges will be 2 + vis_face_count.
			// Since we're also removing vis_face_count faces, the number of new faces added
			// will be 2.
			if (data.m_fend - data.m_flast < 2)
				break;

			PR_EXPAND(PR_DBG_CONVEX_HULL, data.DumpVerts());
			PR_EXPAND(PR_DBG_CONVEX_HULL, data.DumpFaces());
			PR_EXPAND(PR_DBG_CONVEX_HULL, ldr::FileOutput extm("P:\\dump\\hull_extremevert.ldr"));
			PR_EXPAND(PR_DBG_CONVEX_HULL, ldr::Box("extreme", "FFFFFFFF", data.m_vcont[*v], 0.07f, extm));

			// Expand the convex hull to include 'v'
			data.GrowHull(v);

			PR_EXPAND(PR_DBG_CONVEX_HULL, data.DumpVerts());
			PR_EXPAND(PR_DBG_CONVEX_HULL, data.DumpFaces());

			// Test the unclassified verts and move any that are within the convex hull to
			// the 'non-hull' end of the range. Find a new most not in the convex hull vertex.
			v = data.PartitionVerts();
			PR_EXPAND(PR_DBG_CONVEX_HULL, ++iteration);
		}

		vert_count = static_cast<size_t>(data.m_vhull_last - data.m_vbeg);
		face_count = static_cast<size_t>(data.m_flast - data.m_fbeg);
		PR_EXPAND(PR_DBG_CONVEX_HULL, data.DumpVerts());
		PR_EXPAND(PR_DBG_CONVEX_HULL, data.DumpFaces());
		return data.m_vhull_last == data.m_vnon_hull;
	}
	
	// Overload that reorders the verts in the vertex container.
	// 'vcont' is the container of verts supporting operator [] (e.g v4*)
	// 'num_verts' is the number of verts in 'vcont'
	// Other parameters as for the main version of this function.
	template <typename VertCont, typename FIter>
	bool ConvexHull(VertCont& vcont, size_t num_verts, FIter fbeg, FIter fend, size_t& vert_count, size_t& face_count)
	{
		// Create an index buffer for the verts
		int* index = PR_MALLOCA(index, int, num_verts);
		for (int i = 0; i != int(num_verts); ++i)
			index[i] = i;

		// Find the hull
		bool result = ConvexHull<VertCont, int*, FIter>(vcont, index, index + num_verts, fbeg, fend, vert_count, face_count);

		// Write the indices into the 'w' component of the verts
		for (int* i = index, *i_end = i + num_verts; i != i_end; ++i)
			vcont[*i].w = static_cast<float>(i - index);

		// Then sort on 'w'
		std::sort(&vcont[0], &vcont[0] + num_verts, [](v4 lhs, v4 rhs) { return lhs.w < rhs.w; });
		
		// Then restore the 'w' component to '1.0f'
		for (v4* i = &vcont[0], *i_end = i + num_verts; i != i_end; ++i)
			i->w = 1.0f;

		return result;
	}

	// Overload for faces given as an array of index triples.
	template <typename VertCont, typename VIndex>
	bool ConvexHull(VertCont const& vcont, VIndex* vbeg, VIndex* vend, VIndex* ibeg, VIndex* iend, size_t& vert_count, size_t& face_count)
	{
		using Face = typename hull::Face<3 * sizeof(VIndex), VIndex>;

		auto face_beg = reinterpret_cast<Face*>(ibeg);
		auto face_end = reinterpret_cast<Face*>(iend);
		return ConvexHull<VertCont, VIndex*, Face*>(vcont, vbeg, vend, face_beg, face_end, vert_count, face_count);
	}

	// Overload that reorders the verts in the vertex container
	// and for faces given as an array of index triples.
	template <typename VertCont, typename VIndex>
	bool ConvexHull(VertCont& vcont, size_t num_verts, VIndex* ibeg, VIndex* iend, size_t& vert_count, size_t& face_count)
	{
		using Face = typename hull::Face<3 * sizeof(VIndex), VIndex>;

		auto face_beg = reinterpret_cast<Face*>(ibeg);
		auto face_end = reinterpret_cast<Face*>(iend);
		return ConvexHull<VertCont, Face*>(vcont, num_verts, face_beg, face_end, vert_count, face_count);
	}
}
