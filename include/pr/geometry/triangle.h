//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2014
//********************************
#pragma once
#include <cassert>
#include <concepts>
#include "pr/maths/maths.h"
#include "pr/algorithm/algorithm.h"
#include "pr/container/vector.h"
#include "pr/container/ring.h"

#define LDR_OUTPUT 0
#if LDR_OUTPUT
#include "pr/linedrawer/ldr_helper.h"
#endif

namespace pr::geometry
{
	// Return the 'circum radius' of three points
	// 'centre' is only defined if the returned radius is less than float max
	inline float CircumRadius(v4 a, v4 b, v4 c, v4& centre)
	{
		v4 ab = b - a;
		v4 ac = c - a;
		float abab = LengthSq(ab);
		float acac = LengthSq(ac);
		float abac = Dot3(ab, ac);
		float e = abab * acac;
		float d = 2.0f * (e - abac * abac);
		if (Abs(d) <= maths::tiny<float>) return limits<float>::max();

		float s = (e - acac * abac) / d;
		float t = (e - abab * abac) / d;

		centre = a + s * ab + t * ac;
		return Length(centre - a);
	}

	// Returns the angles at each triangle vertex for the triangle v0,v1,v2
	inline v4 TriangleAngles(v4 v0, v4 v1, v4 v2)
	{
		// Angle at a vertex:
		// Cos(C) = a.b / |a|b|
		// Use: Cos(2C) = 2Cos²C - 1
		// Cos(2C) = 2Cos²(C) - 1 = 2*(a.b² / a²b²) - 1
		// C = 0.5 * ACos(2*(a.b² / a²b²) - 1)

		// Choose edges so that 'a' is opposite v0, and angle 'A' is the angle at v0
		auto a = v2 - v1;
		auto b = v0 - v2;
		auto c = v1 - v0;
		auto asq = LengthSq(a);
		auto bsq = LengthSq(b);
		auto csq = LengthSq(c);

		// Use acos for the two smallest angles and 'A+B+C = pi' for the largest
		v4 angles;
		if (csq > asq && csq > bsq)
		{
			auto bc = Dot3(b,c); auto d1 = bsq * csq;
			auto ca = Dot3(c,a); auto d2 = csq * asq;

			angles.x = 0.5f * Acos(Clamp(2*(bc*bc / (d1 + (d1 == 0.0f))) - 1, -1.0f, 1.0f));
			angles.y = 0.5f * Acos(Clamp(2*(ca*ca / (d2 + (d2 == 0.0f))) - 1, -1.0f, 1.0f));
			angles.z = constants<float>::tau_by_2 - angles.x - angles.y;
		}
		else if (asq > bsq && asq > csq)
		{
			auto ab = Dot3(a,b); auto d0 = asq * bsq;
			auto ca = Dot3(c,a); auto d2 = csq * asq;

			angles.y = 0.5f * Acos(Clamp(2*(ca*ca / (d2 + (d2 == 0.0f))) - 1, -1.0f, 1.0f));
			angles.z = 0.5f * Acos(Clamp(2*(ab*ab / (d0 + (d0 == 0.0f))) - 1, -1.0f, 1.0f));
			angles.x = constants<float>::tau_by_2 - angles.y - angles.z;
		}
		else
		{
			auto ab = Dot3(a,b); auto d0 = asq * bsq;
			auto bc = Dot3(b,c); auto d1 = bsq * csq;
			
			angles.x = 0.5f * Acos(Clamp(2*(bc*bc / (d1 + (d1 == 0.0f))) - 1, -1.0f, 1.0f));
			angles.z = 0.5f * Acos(Clamp(2*(ab*ab / (d0 + (d0 == 0.0f))) - 1, -1.0f, 1.0f));
			angles.y = constants<float>::tau_by_2 - angles.x - angles.z;
		}
		angles.w = 0.0f;
		return angles;
	}

	// Determine the area of a polygon
	inline float PolygonArea(std::span<v2 const> poly)
	{
		auto area = 0.0f;
		for (int i = 0; i != isize(poly)-1; ++i)
			area += Cross(poly[i+1], poly[i]);

		return area / 2.0f;
	}

	// Triangulate a 2D, non-convex, non-self-intersecting, polygon.
	// 'polygon' should have an CCW winding order.
	// 'out' is an output function: 'void out(int i0, int i1, int i2)'
	//  that receives the indices of triangles within the polygon.
	// Notes:
	//  Since the polygon is given as a list of points, it's not possible
	//  specify holes. Turn polygons with holes into a single continuous edge
	//  by inserting degenerate edges from the holes to the split/merge vertices.
	class SeidelTriangulation
	{
		// Using Seidel's algorithm
		// Sweep the polygon over one axis (Y in this case).
		// Phase 1: classify each vertex of the polygon into:
		//   start vertex - convex vertex, with neighbour Y values both > vertex.y
		//   end vertex - convex vertex, with neighbour Y values both < vertex.y
		//   split vertex - concave vertex, with neighbour Y values both > vertex.y
		//   merge vertex - concave vertex, with neighbour Y values both < vertex.y
		//   regular vertex - neighbour Y values on either side of vertex.y
		// Phase 2: create "monotone" polygons, i.e. split the polygon into smaller
		//   polygons by inserting edges at the split and merge vertices.
		//   split vertices - insert edge to nearest vertex below the split vertex
		//   merge vertices - insert edge to nearest vertex above the merge vertex
		// Phase 3: triangulate the monotone polygons.
		//   Use ear clipping of convex vertices. There is no need to test for vertices
		//   within each ear because the polygons are monotone.
		// Notes:
		//  A polygon is 'monotone' w.r.t to some line 'L' if, for any line parallel
		//  to 'L', the polygon only intersects it twice.

		using IdxCont = pr::vector<int>;
		enum class EType { Left, Right, Start, End, Split, Merge, };

		// The vertices of the polygon
		std::span<v2 const> m_verts;

		// Indices of the polygon edges (winding order CCW)
		IdxCont m_idx;

		// A list of polygon indices, sorted on Y
		IdxCont m_sorted;

		// A lookup table from polygon index to sorted list index
		IdxCont m_lookup;

	public:

		SeidelTriangulation()
			: m_verts()
			, m_idx()
			, m_sorted()
			, m_lookup()
		{}

		// Triangulate a 2d polygon
		template <std::invocable<int,int,int> TFacesOut>
		void Triangulate(std::span<v2 const> polygon, TFacesOut faces_out)
		{
			m_verts = polygon;
			m_idx.resize(m_verts.size());
			m_sorted.resize(m_idx.size());
			m_lookup.resize(m_idx.size());

			auto count = isize(polygon);
			if (count <  3) { return; }
			if (count == 3) { faces_out(0,1,2); return; }
			if (PolygonArea(polygon) < 0)
				throw std::runtime_error("Polygon winding order is incorrect");

			#if LDR_OUTPUT
			pr::ldr::LdrBuilder ldr;
			ldr.Line("", 0xFF00FF00, 1, [=](int i, v4& pt) { pt = v4(m_verts[i%count], 0, 1.0f); return i <= count; });
			ldr.ToFile(L"\\dump\\triangulate.ldr");
			#endif

			// Create a list of the polygon indices
			for (int i = 0; i != count; ++i)
				m_idx[i] = i;

			// Create a list of polygon indices sorted on Y
			m_sorted = m_idx;
			pr::sort(m_sorted, [&](int l, int r) { return Less(m_verts[l], m_verts[r]); });

			// Create a lookup table from polygon index to sorted list index
			for (int i = 0; i != count; ++i)
				m_lookup[m_sorted[i]] = i;

			// The indices of the whole polygon, as a ring buffer
			auto poly = MakeRing(m_idx.data(), m_idx.data() + m_idx.size());

			// Create monotone polygons by inserting degenerate edges
			// This will create a container with indices like this: 0,1,7,7,1,2,3,4,5,3,3,5,6,7,8,9 (10 vertex polygon)
			// The doubles represent degenerate edges in the polygon. If the inner values are
			// greater than the outer values (e.g. '1,7,7,1') then the degenerate connects to a
			// later vertex in the polygon. If the inner values are less than the inner values
			// (e.g. '5,3,3,5'), then the degenerate connects to an earlier polygon vertex.
			IdxCont mt; mt.reserve(2*count); // reserve room for a lot of degenerates
			for (int i = 0; i != count; ++i)
			{
				// Iterate around the polygon, inserting degenerate edges for Split or Merge vertices
				auto ty = Classify(poly, i);
				if (ty == EType::Split || ty == EType::Merge)
				{
					// Find the vertex to create a degenerate to. The nearest above or below, depending on type.
					auto helper = FindHelper(i, Bool2SignI(ty == EType::Merge));
					mt.push_back(i);
					mt.push_back(helper);
					mt.push_back(helper);
					mt.push_back(i);
				}
				else
				{
					mt.push_back(i);
				}
			}

			// Closing the polygon back to 0, this is needed for loop termination in 'Triangulate'
			mt.push_back(0);

			// Identify the monotone polygons in 'mt' (recursive)
			FindMonotonePolygons(mt, 0, +1, faces_out);
		}
	
	private:

		// Triangulate the monotone polygons in 'mt'
		// This is a recursive function that removes indices from 'mt'. The number of indices removed is returned
		template <std::invocable<int,int,int> TFacesOut>
		int FindMonotonePolygons(IdxCont& mt, int first, int dir, TFacesOut faces_out) const
		{
			// Look for the end of the monotone polygon by searching for
			// 'monotone[i]' == 'monotone[first]' (searching in 'dir' direction)
			// If we encounter a double value before the end, call recursively.
			// Note: this will always terminate due to the 0 added to ensure a closed polygon.
			auto monotone = MakeRing(mt.data(), mt.data() + mt.size());
			int i; for (i = first+dir; monotone[i] != monotone[first]; i += dir)
			{
				// If we encounter a double value, this marks the start or end of a
				// nested monotone polygon. We need to recursively call Triangulate.
				if (monotone[i] == monotone[i+dir])
				{
					// If the inner values are greater than the outer values, then this is the start
					// of a "forward" monotone polygon (i.e. the next occurrence of monotone[i]
					// after i+1 marks the end of the monotone polygon).
					// If the inner values are less than the outer values, then this marks the end
					// of a "backward" monotone polygon (i.e. the previous occurrence of monotone[i]
					// before i marks the start of the monotone polygon).
					auto fwd = Bool2SignI(monotone[i] > monotone[i-dir]);
					auto chg = FindMonotonePolygons(mt, i + (fwd == dir ? dir : 0), fwd, faces_out);

					// 'monotone' is invalidated when indices are removed, so recreate it.
					monotone = MakeRing(mt.data(), mt.data() + mt.size());
					i -= fwd == dir ? 0 : chg;
				}
			}

			// After the loop exits we are at the end of the monotone polygon and
			// all nested polygons have been removed. Tessellate to triangles.
			assert(monotone[i] == monotone[first]);
			auto s = dir > 0 ? first : i;
			auto e = dir > 0 ? i : first;
			return Triangulate(mt, s, e, faces_out);
		}

		// Triangulate the monotone polygon in the range [first, last] of 'mt'
		// and remove those indices from the container 'mt'
		template <std::invocable<int,int,int> TFacesOut>
		int Triangulate(IdxCont& mt, int first, int last, TFacesOut faces_out) const
		{
			// The indices of the monotone polygon
			auto poly = MakeRing(mt.data() + first, mt.data() + last);
			auto count = last - first;
			assert(count >= 3 && "polygons must have at least 3 vertices");

			// Find the index within 'poly' that has the lowest Y value.
			// This is the starting point for triangulation.
			int start = 0;
			for (int i = 1; i != count; ++i)
				if (Less(m_verts[poly[i]], m_verts[poly[start]]))
					start = i;

			// Left and Right side pointers
			auto l = start - 1;
			auto r = start + 1;

			// Triangulate.
			IdxCont queue;
			queue.push_back(poly[start]);
			for (;r - l <= count;)
			{
				// Add the next lowest vertex index
				// and look for triangles to output
				if (Less(poly[l], poly[r]))
				{
					queue.insert(queue.begin(), poly[l--]);
					for (auto i = int(queue.size()); i >= 3 && Convex(queue[0], queue[1], queue[2]); --i)
					{
						faces_out(queue[0], queue[1], queue[2]);
						queue.erase(queue.begin() + 1);
					}
				}
				else
				{
					queue.insert(queue.end(), poly[r++]);
					for (auto i = int(queue.size()); i >= 3 && Convex(queue[i-3], queue[i-2], queue[i-1]); --i)
					{
						faces_out(queue[i-3], queue[i-2], queue[i-1]);
						queue.erase(queue.begin() + (i-2));
					}
				}
			}
			assert(queue.size() == 2);

			// Remove the monotone polygon from the container
			mt.erase(mt.begin() + first, mt.begin() + last + 1);
			return last - first + 1;
		}

		// Return the index of the vertex with the nearest Y value on the 'side' of 'idx'
		int FindHelper(int idx, int side) const
		{
			auto sidx = m_lookup[idx];
			return m_sorted[sidx + side];
		}

		// Classify a vertex in a polygon
		// 'poly' is the indices that describe the polygon (or monotone polygon)
		// 'idx' is the index within 'poly' to classify
		EType Classify(Ring<int*> const& poly, int idx)
		{
			auto& prev = m_verts[poly[idx - 1]];
			auto& curr = m_verts[poly[idx    ]];
			auto& next = m_verts[poly[idx + 1]];
				
			if (Convex(prev, curr, next))
			{
				if (Less(curr, prev) && Less(curr, next)) return EType::Start;
				if (Less(prev, curr) && Less(next, curr)) return EType::End;
				return Less(prev, next) ? EType::Right : EType::Left;
			}
			else
			{
				if (Less(curr, prev) && Less(curr, next)) return EType::Split;
				if (Less(prev, curr) && Less(next, curr)) return EType::Merge;
				return Less(prev, next) ? EType::Right : EType::Left;
			}
		}

		// True if the vertex at index position 'b' is convex
		bool Convex(int a, int b, int c) const
		{
			auto& A = m_verts[a];
			auto& B = m_verts[b];
			auto& C = m_verts[c];
			return Convex(A, B, C);
		}

		// True if the vertex at 'm_verts[lhs]' has a lower Y value than 'm_verts[rhs]'
		bool Less(int lhs, int rhs) const
		{
			auto& L = m_verts[lhs];
			auto& R = m_verts[rhs];
			return Less(L, R);
		}

		// True if 'b' is a convex vertex (assuming winding order a, b, c is CCW)
		static bool Convex(v2 a, v2 b, v2 c)
		{
			return Cross(b - a, c - b) <= 0;
		}

		// True if 'lhs' is less than 'rhs'
		static bool Less(v2 lhs, v2 rhs)
		{
			// Sort on Y first, then on X
			return lhs.y != rhs.y ? lhs.y < rhs.y : lhs.x < rhs.x;
		}
	};
	template <std::invocable<int,int,int> TFacesOut>
	inline void TriangulatePolygon(std::span<v2 const> poly, TFacesOut faces_out)
	{
		SeidelTriangulation s;
		s.Triangulate<TFacesOut>(poly, faces_out);
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::geometry
{
	PRUnitTest(TriangleAnglesTests)
	{
		{//TriangleAngles
			v4 v0(+1.0f, +2.0f, 0.0f, 1.0f);
			v4 v1(-2.0f, -1.0f, 0.0f, 1.0f);
			v4 v2(+0.0f, -1.0f, 0.0f, 1.0f);
			v4 angles = TriangleAngles(v0, v1, v2);
			angles.x = pr::RadiansToDegrees(angles.x);
			angles.y = pr::RadiansToDegrees(angles.y);
			angles.z = pr::RadiansToDegrees(angles.z);
				
			PR_EXPECT(FEql(angles.x, 26.56505f));
			PR_EXPECT(FEql(angles.y, 45.0f    ));
			PR_EXPECT(FEql(angles.z, 108.4349f));
		}
	}
	PRUnitTest(TriangulatePolygonTests)
	{
		{
			v2 poly[] =
			{
				{0.0f, 0.0f},
				{0.1f, 0.9f},
				{1.0f, 1.0f},
				{0.0f, 1.0f},
			};
			pr::vector<int> tris;
			TriangulatePolygon(poly, [&](int i0, int i1, int i2)
			{
				tris.push_back(i0);
				tris.push_back(i1);
				tris.push_back(i2);

				#if LDR_OUTPUT
				pr::ldr::LdrBuilder ldr;
				ldr.Triangle("", 0xFF00FF00, v4(poly[i0],0,1), v4(poly[i1],0,1), v4(poly[i2],0,1));
				ldr.ToFile(L"\\dump\\triangulate.ldr", true);
				#endif
			});
		}
		{
			v2 poly[] =
			{
				{1.0f, 3.0f}, //0
				{1.4f, 1.7f}, //1
				{0.4f, 2.0f}, //2
				{1.5f, 1.2f}, //3
				{1.0f, 0.0f}, //4
				{1.7f, 1.0f}, //5
				{2.5f, 0.5f}, //6
				{2.0f, 1.5f}, //7
				{2.0f, 2.0f}, //8
				{1.5f, 2.5f},
			};
			pr::vector<int> tris;
			TriangulatePolygon(poly, [&](int i0, int i1, int i2)
			{
				tris.push_back(i0);
				tris.push_back(i1);
				tris.push_back(i2);

				#if LDR_OUTPUT
				pr::ldr::LdrBuilder ldr;
				ldr.Triangle("", 0xFF00FF00, v4(poly[i0],0,1), v4(poly[i1],0,1), v4(poly[i2],0,1));
				ldr.ToFile(L"\\dump\\triangulate.ldr", true);
				#endif
			});
		}
	}
}
#endif