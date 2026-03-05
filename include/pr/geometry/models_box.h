//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2013
//********************************
#pragma once
#include "pr/geometry/common.h"

namespace pr::geometry
{
	// Returns the number of verts and number of indices needed to hold geometry for an array of 'num_boxes' boxes.
	constexpr BufSizes BoxSize(int num_boxes)
	{
		return
		{
			24 * num_boxes,
			36 * num_boxes,
		};
	}

	// Generate boxes from an array of corners
	// Point Order:
	//  -x, -y, -z = 0
	//  +x, -y, -z = 1
	//  -x, +y, -z = 2
	//  +x, +y, -z = 3
	//  -x, -y, +z = 4
	//  +x, -y, +z = 5
	//  -x, +y, +z = 6
	//  +x, +y, +z = 7
	// 'num_boxes' is the number of boxes given in the 'points' array. (Should be 8 * its length)
	// 'points' is the input array of corner points for the boxes
	// 'num_colours' should be either, 0, 1, num_boxes, or num_boxes*8 representing; no colour, 1 colour for all, 1 colour per box, or 1 colour per box vertex
	// 'colours' is an input array of colour values, a pointer to a single colour, or null.
	// 'out_verts' is an output iterator to receive the [vert,norm,colour,tex] data
	// 'out_indices' is an output iterator to receive the index data
	// The order of faces is +X,-X,+Y,-Y,+Z,-Z
	// The normals are outward facing
	// The texture coordinates set on the box have the 'walls' with Y as up.
	// On top (-x,+y,-z) is the top left corner, on the bottom (-x,-y,+z) is the top left corner
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Boxes(int num_boxes, VertCIter auto points, std::span<Colour32 const> colours, VOut vout, IOut iout)
	{
		int const vidx[] =
		{
			7,5,1,3, // +X
			2,0,4,6, // -X
			2,6,7,3, // +Y
			4,0,1,5, // -Y
			6,4,5,7, // +Z
			3,1,0,2, // -Z
		};
		int const vcount = sizeof(vidx)/sizeof(vidx[0]);
		int const indices[] =
		{
			0, 1, 2,  0, 2, 3,  //  0 -  6
			4, 5, 6,  4, 6, 7,  //  6 - 12
			8, 9,10,  8,10,11,  // 12 - 18
			12,13,14, 12,14,15, // 18 - 24
			16,17,18, 16,18,19, // 24 - 30
			20,21,22, 20,22,23, // 30 - 36
		};
		int const icount = sizeof(indices)/sizeof(indices[0]);

		// Texture coords
		auto t00 = v2(0.0f, 0.0f);
		auto t01 = v2(0.0f, 1.0f);
		auto t10 = v2(1.0f, 0.0f);
		auto t11 = v2(1.0f, 1.0f);

		Props props;
		props.m_geom = EGeom::Vert | (isize(colours) ? EGeom::Colr : EGeom::None) | EGeom::Norm | EGeom::Tex0;

		// Helper function for generating normals
		auto norm = [](v4 a, v4 b, v4 c) { return Normalise(Cross(c - b, a - b), v4::Zero()); };

		// Colour iterator wrapper
		auto col = CreateRepeater(colours, 8*num_boxes, Colour32White);
		auto cc = [&](Colour32 c) { props.m_has_alpha |= HasAlpha(c); return c; };

		// Bounding box
		auto bb = [&](v4 v) { Grow(props.m_bbox, v); return v; };

		auto v_in = points;
		for (int i = 0; i != num_boxes; ++i)
		{
			// Read 8 points from the vertex and colour streams
			struct { v4 pt; Colour32 cl; } vert[8];
			for (int j = 0; j != 8; ++j)
			{
				vert[j].pt = bb(*v_in++);
				vert[j].cl = cc(*col++);
			}

			// Set the verts
			auto vi = vidx;
			for (int j = 0; j != vcount/4; ++j)
			{
				auto& a = vert[*vi++];
				auto& b = vert[*vi++];
				auto& c = vert[*vi++];
				auto& d = vert[*vi++];
				vout(a.pt, a.cl, norm(d.pt, a.pt, b.pt), t00);
				vout(b.pt, b.cl, norm(a.pt, b.pt, c.pt), t01);
				vout(c.pt, c.cl, norm(b.pt, c.pt, d.pt), t11);
				vout(d.pt, d.cl, norm(c.pt, d.pt, a.pt), t10);
			}

			// Set the faces
			auto ii = indices;
			auto ibase = i * 24;
			for (int j = 0; j != icount/6; ++j)
			{
				iout(ibase + *ii++);
				iout(ibase + *ii++);
				iout(ibase + *ii++);
				iout(ibase + *ii++);
				iout(ibase + *ii++);
				iout(ibase + *ii++);
			}
		}
		return props;
	}

	// Create a transformed box
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Boxes(int num_boxes, VertCIter auto points, m4x4 const& o2w, std::span<Colour32 const> colours, VOut vout, IOut iout)
	{
		if (o2w == m4x4::Identity())
			return Boxes(num_boxes, points, colours, vout, iout);

		// An iterator wrapper for applying a transform to 'points'
		Transformer<v4 const*> tx(points, o2w);
		auto props = Boxes(num_boxes, tx, colours, vout, iout);
		return props;
	}

	// Create a box with side half lengths = rad.x,rad.y,rad.z
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Box(v4 rad, m4x4 const& o2w, Colour32 colour, VOut vout, IOut iout)
	{
		v4 const pt[8] =
		{
			{-rad.x, -rad.y, -rad.z, 1.0f},
			{+rad.x, -rad.y, -rad.z, 1.0f},
			{-rad.x, +rad.y, -rad.z, 1.0f},
			{+rad.x, +rad.y, -rad.z, 1.0f},
			{-rad.x, -rad.y,  rad.z, 1.0f},
			{+rad.x, -rad.y,  rad.z, 1.0f},
			{-rad.x, +rad.y,  rad.z, 1.0f},
			{+rad.x, +rad.y,  rad.z, 1.0f},
		};
		assert(is_aligned<v4>(&pt[0]));
		return Boxes(1, &pt[0], o2w, { &colour, 1 }, vout, iout);
	}

	// Create boxes at each point in 'positions' with side half lengths = rad.x,rad.y,rad.z
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props BoxList(int num_boxes, std::span<v4 const> positions, v4 rad, std::span<Colour32 const> colours, VOut vout, IOut iout)
	{
		vector<v4,64> points(8*num_boxes);
		auto* pt = points.data();

		auto const* pos = positions.data();
		for (int i = 0; i != num_boxes; ++i, ++pos)
		{
			*pt++ = v4(pos->x - rad.x, pos->y - rad.y, pos->z - rad.z, 1.0f);
			*pt++ = v4(pos->x + rad.x, pos->y - rad.y, pos->z - rad.z, 1.0f);
			*pt++ = v4(pos->x - rad.x, pos->y + rad.y, pos->z - rad.z, 1.0f);
			*pt++ = v4(pos->x + rad.x, pos->y + rad.y, pos->z - rad.z, 1.0f);
			*pt++ = v4(pos->x - rad.x, pos->y - rad.y, pos->z + rad.z, 1.0f);
			*pt++ = v4(pos->x + rad.x, pos->y - rad.y, pos->z + rad.z, 1.0f);
			*pt++ = v4(pos->x - rad.x, pos->y + rad.y, pos->z + rad.z, 1.0f);
			*pt++ = v4(pos->x + rad.x, pos->y + rad.y, pos->z + rad.z, 1.0f);
		}
		return Boxes(num_boxes, &points[0], colours, vout, iout);
	}

	// Create the AABB boxes in 'boxes'
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props BoxList(std::span<BBox const> boxes, std::span<Colour32 const> colours, VOut vout, IOut iout)
	{
		vector<v4,64> points(8 * boxes.size());
		auto* pt = points.data();

		for (auto const& bbox : boxes)
		{
			auto lwr = bbox.Lower();
			auto upr = bbox.Upper();
			*pt++ = v4(lwr.x, lwr.y, lwr.z, 1.0f);
			*pt++ = v4(upr.x, lwr.y, lwr.z, 1.0f);
			*pt++ = v4(lwr.x, upr.y, lwr.z, 1.0f);
			*pt++ = v4(upr.x, upr.y, lwr.z, 1.0f);
			*pt++ = v4(lwr.x, lwr.y, upr.z, 1.0f);
			*pt++ = v4(upr.x, lwr.y, upr.z, 1.0f);
			*pt++ = v4(lwr.x, upr.y, upr.z, 1.0f);
			*pt++ = v4(upr.x, upr.y, upr.z, 1.0f);
		}
		return Boxes(isize(boxes), points.data(), colours, vout, iout);
	}
}
