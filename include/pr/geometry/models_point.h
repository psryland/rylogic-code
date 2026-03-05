//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2013
//********************************
#pragma once
#include "pr/geometry/common.h"

namespace pr::geometry
{
	// Returns the number of verts and number of indices needed to hold geometry for an array of 'num_point' points.
	constexpr BufSizes PointSize(int num_points)
	{
		return
		{
			num_points,
			num_points,
		};
	}

	// Generate points from an array of points.
	// 'num_points' is the number of point in the following arrays
	// 'points' is the input array of points.
	// 'num_colours' should be either, 0, 1, or num_points
	// 'colours' is an input array of colour values, a pointer to a single colour, or null.
	// 'out_verts' is an output iterator to receive the [vert,colour] data
	// 'out_indices' is an output iterator to receive the index data
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Points(std::span<v4 const> points, std::span<Colour32 const> colours, VOut vout, IOut iout)
	{
		Props props;
		props.m_geom = EGeom::Vert | (isize(colours) ? EGeom::Colr : EGeom::None) | EGeom::Tex0; // UVs are added in the geometry shader

		// Colour iterator
		auto col = CreateRepeater(colours, isize(points), Colour32White);
		auto cc = [&](Colour32 c) { props.m_has_alpha |= HasAlpha(c); return c; };

		// Bounding box
		auto bb = [&](v4 v) { Grow(props.m_bbox, v); return v; };

		auto v_in = points.data(); int index = 0;
		for (int i = 0; i != isize(points); ++i)
		{
			vout(bb(*v_in++), cc(*col++), v4::Zero(), v2::Zero());
			iout(index++);
		}

		return props;
	}
}
