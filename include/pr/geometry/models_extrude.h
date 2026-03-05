//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2013
//********************************
#pragma once
#include "pr/geometry/common.h"
#include "pr/geometry/triangle.h"

namespace pr::geometry
{
	// Return the model buffer requirements of an extrusion
	constexpr BufSizes ExtrudeSize(int cs_count, int path_count, bool closed, bool smooth_cs)
	{
		assert(cs_count >= 3 && "Cross section must have 3 or more points");
		assert(path_count >= 2 && "Extrusion path must have at least 2 points");

		// - 2 lots of verts at the end caps so they can have outward facing normals.
		// - 'smooth_cs' means smooth normals around the wall of the tube. If false we
		// need to double each vertex around the cross section
		auto vcount =
			(closed ? cs_count * 2 : 0) +                   // Verts for the two end caps (separate so they can have outward normals)
			(path_count * cs_count * (smooth_cs ? 1 : 2)) + // Verts around each cross section (doubled if not smooth)
			0;
		auto icount =
			(closed ? (cs_count - 2) : 0) * 3 * 2 + // The number of end cap faces x2 (two ends) * 3 (indices/face)
			(path_count - 1) * cs_count * 2 * 3 +   // The number of sections along the path * faces around each section (2 per cs vert) * 3 (indices/face)
			0;
		return { vcount, icount };
	}

	// Generate a model from an extrusion of a 2d polygon
	// 'cs' - the cross section points (expects v2 const*). CCW winding order
	// 'path' - a function that supplies a stream of transforms describing the extrusion path. Z axis should be the path tangent.
	// 'path_count' - the number of matrices in the extrusion path
	// 'num_colours - the number of colours pointed to by 'colours', can be equal to 0, 1, or path_count
	// 'colours' - the array of colours of length 'num_colours'
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Extrude(
		std::span<v2 const> cs,
		std::function<m4x4(int p, int pcount)> path,
		int path_count,
		bool closed,
		bool smooth_cs,
		std::span<Colour32 const> colours,
		VOut vout, IOut iout)
	{
		// Don't bother handling acute angles, users can just insert really small
		// line segments between acute lines within the path
		assert(path_count >= 2 && "Path must have at least 2 points");

		auto cs_count = isize(cs);
		auto [vcount, icount] = ExtrudeSize(cs_count, path_count, closed, smooth_cs);

		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | EGeom::Norm;

		// Colour iterator wrapper
		auto col = CreateRepeater(colours, path_count, Colour32White);
		auto cc = [&](Colour32 c) { props.m_has_alpha |= HasAlpha(c); return c; };

		// Bounding box
		auto bb = [&](v4 v) { Grow(props.m_bbox, v); return v; };

		// Verts - create planes of cross sections at each path point
		auto xsection = MakeRing(cs.begin(), cs.end());
		auto ori = path(0, path_count);
		if (closed)
		{
			// Starting end cap
			for (int x = 0; x != cs_count; ++x)
			{
				auto pt   = ori * v4(xsection[x], 0, 1.0f);
				auto norm = ori * -v4::ZAxis();
				vout(bb(pt), cc(*col), norm, v2::Zero());
				--vcount;
			}
		}
		for (int p = 0; p != path_count; ++p, ++col)
		{
			// Cross section verts for each segment of the path
			// Doubled if outward normals are not smooth
			v4 pt, norm;
			ori = p == 0 ? ori : path(p, path_count);
			if (smooth_cs)
			{
				for (int x = 0; x != cs_count; ++x)
				{
					pt = ori * v4(xsection[x], 0, 1.0f);
					norm = ori * v4(Normalise(Rotate90CCW(xsection[x+1] - xsection[x-1]), v2::Zero()), 0, 0);
					vout(bb(pt), cc(*col), norm, v2::Zero());
					--vcount;
				}
			}
			else
			{
				for (int x = 0; x != cs_count+1; ++x)
				{
					pt = ori * v4(xsection[x], 0, 1.0f);
					if (x != 0)
					{
						norm = ori * v4(Normalise(Rotate90CCW(xsection[x] - xsection[x-1]), v2::Zero()), 0, 0);
						vout(bb(pt), cc(*col), norm, v2::Zero());
						--vcount;
					}
					if (x != cs_count)
					{
						norm = ori * v4(Normalise(Rotate90CCW(xsection[x+1] - xsection[x]), v2::Zero()), 0, 0);
						vout(bb(pt), cc(*col), norm, v2::Zero());
						--vcount;
					}
				}
			}
		}
		if (closed)
		{
			// Closing end cap
			for (int x = 0; x != cs_count; ++x)
			{
				auto pt   = ori * v4(xsection[x], 0, 1.0f);
				auto norm = ori * +v4::ZAxis();
				vout(bb(pt), cc(*col), norm, v2::Zero());
				--vcount;
			}
		}
		assert(vcount == 0);

		// Triangulate the cross section to generate faces for the end caps.
		// Save the end cap faces so we can use them for the other end cap as well.
		pr::vector<int> cap_faces;
		if (closed)
		{
			// The cross section may not be convex.
			// Triangulate the end cap using non-convex polygon triangulation.
			TriangulatePolygon(cs, [&](int i0, int i1, int i2)
			{
				cap_faces.push_back(i0);
				cap_faces.push_back(i1);
				cap_faces.push_back(i2);
			});
		}

		// Faces
		auto v = 0; // Offset to the first vertex for each segment

		// If closed, create the starting end cap
		if (closed)
		{
			for (int i = 0, iend = int(cap_faces.size()); i != iend; i += 3)
			{
				iout(v + cap_faces[i  ]);
				iout(v + cap_faces[i+2]);
				iout(v + cap_faces[i+1]);
				icount -= 3;
			}
			v += cs_count;
		}

		// Faces along the tube walls
		auto v_per_segment = cs_count * (smooth_cs ? 1 : 2);
		for (int p = 0; p != path_count-1; ++p, v += v_per_segment)
		{
			for (int i = 0; i != v_per_segment; i += (smooth_cs ? 1 : 2))
			{
				auto j = smooth_cs ? (i + 1) % v_per_segment : (i + 1);
				iout(v + i);
				iout(v + j);
				iout(v + j + v_per_segment);
				iout(v + j + v_per_segment);
				iout(v + i + v_per_segment);
				iout(v + i);
				icount -= 6;
			}
		}

		// If closed, create the closing end cap
		if (closed)
		{
			v = cs_count + path_count * cs_count * (smooth_cs ? 1 : 2);
			for (int i = 0, iend = int(cap_faces.size()); i != iend; i += 3)
			{
				iout(v + cap_faces[i  ]);
				iout(v + cap_faces[i+1]);
				iout(v + cap_faces[i+2]);
				icount -= 3;
			}
		}
		assert(icount == 0);

		return props;
	}
}
