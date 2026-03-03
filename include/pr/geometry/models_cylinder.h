//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2013
//********************************
#pragma once
#include "pr/geometry/common.h"

namespace pr::geometry
{
	// Returns the number of verts and number of indices needed to hold geometry for a cylinder
	constexpr BufSizes CylinderSize(int wedges, int layers)
	{
		if (wedges < 3) wedges = 3;
		if (layers < 1) layers = 1;
		return
		{
			2 + (wedges + 1) * (layers + 3),
			6 * (wedges + 0) * (layers + 1),
		};
	}

	// Generate a cylinder given by a height and radius at each end, orientated with the long axis along 'Z'
	// 'radius0' is the radius of the bottom face (i.e. -z axis face) of the cylinder
	// 'radius1' is the radius of the top face (i.e. +z axis face) of the cylinder
	// 'height' is the length of the cylinder along the z axis
	// 'xscale'/'yscale' are scaling factors that can be used to make the cylinder ellipsoidal
	// 'wedges' is the number of divisions around the z axis
	// 'layers' is the number of sections along the ZAxis, must be >= 1
	// 'num_colours' should be either, 0, 1, num_boxes, num_boxes*8 representing; no colour, 1 colour for all, 1 colour per box, or 1 colour per box vertex
	// 'colours' is an input array of colour values, a pointer to a single colour, or null.
	// The texture coords assigned to the cylinder map a quad around the 'barrel' of the cylinder and a circle
	// on the ends of the cylinder since this is the most likely way it would be textured
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Cylinder(float radius0, float radius1, float height, float xscale ,float yscale ,int wedges, int layers, std::span<Colour32 const> colours, VOut vout, IOut iout)
	{
		if (wedges < 3) wedges = 3;
		if (layers < 1) layers = 1;
		auto [vcount,icount] = CylinderSize(wedges, layers);

		Props props;
		props.m_geom = EGeom::Vert | (isize(colours) ? EGeom::Colr : EGeom::None) | EGeom::Norm | EGeom::Tex0;

		// Bounding box
		float max_radius = std::max(radius0, radius1);
		Grow(props.m_bbox, v4(-max_radius * xscale, -max_radius * yscale, -height * 0.5f, 1.0f));
		Grow(props.m_bbox, v4(+max_radius * xscale, +max_radius * yscale, +height * 0.5f, 1.0f));

		// Colour iterator wrapper
		auto col = CreateRepeater(colours, vcount, Colour32White);
		auto cc = [&](Colour32 c) { props.m_has_alpha |= HasAlpha(c); return c; };

		auto z  = -height * 0.5f;
		auto dz = height / layers;
		auto da = float(maths::tau) / wedges;
		int verts_per_layer = wedges + 1;
		int ibase = 0, last = vcount - 1;

		auto pt = v4(0, 0, z, 1.0f);
		auto uv = v2(0.5f, 0.5f);
		auto nm = -v4::ZAxis();

		// Verts
		vout(pt, cc(*col++), nm, uv);
		for (int w = 0; w <= wedges; ++w) // Bottom face
		{
			auto a = da*w;
			pt = v4(cos(a) * radius0 * xscale, sin(a) * radius0 * yscale, z, 1.0f);
			uv = v2(cos(a) * 0.5f + 0.5f, sin(a) * 0.5f + 0.5f);
			vout(pt, cc(*col++), nm, uv);
		}
		for (int l = 0; l <= layers; ++l) // The walls
		{
			auto r  = Lerp(radius0, radius1, l/(float)layers);
			auto nz = radius0 - radius1;
			for (int w = 0; w <= wedges; ++w)
			{
				auto a = da*w + (l%2)*da*0.5f;
				pt = v4(cos(a) * r * xscale, sin(a) * r * yscale, z, 1.0f);
				nm = v4::Normal(height * cos(a + da*0.5f) / xscale, height * sin(a + da*0.5f) / yscale ,nz ,0.0f);
				uv = v2(a / float(maths::tau), 1.0f - (z + height*0.5f) / height);
				vout(pt, cc(*col++), nm, uv);
			}
			if (l != layers) z += dz;
		}
		nm = v4::ZAxis();
		for (int w = 0; w <= wedges; ++w) // Top face
		{
			auto a = da*w + (layers%2)*da*0.5f;
			pt = v4(cos(a) * radius1 * xscale, sin(a) * radius1 * yscale, z, 1.0f);
			uv = v2(cos(a) * 0.5f + 0.5f, sin(a) * 0.5f + 0.5f);
			vout(pt, cc(*col++), nm, uv);
		}
		pt = v4(0 ,0 ,z ,1.0f);
		uv = v2(0.5, 0.5f);
		vout(pt, cc(*col++), nm, uv);

		// Faces
		ibase = 1;
		for (int w = 0; w != wedges; ++w) // Bottom face
		{
			iout(0);
			iout(ibase + w + 1);
			iout(ibase + w);
		}
		ibase += verts_per_layer;
		for (int l = 0; l != layers; ++l) // The walls
		{
			for (int w = 0; w != wedges; ++w)
			{
				iout(ibase + w);
				iout(ibase + w + 1);
				iout(ibase + w + verts_per_layer);
				iout(ibase + w + verts_per_layer);
				iout(ibase + w + 1);
				iout(ibase + w + verts_per_layer + 1);
			}
			ibase += verts_per_layer;
		}
		ibase += verts_per_layer;
		for (int w = 0; w != wedges; ++w) // Top face
		{
			iout(ibase + w);
			iout(ibase + w + 1);
			iout(last);
		}

		return props;
	}
}
