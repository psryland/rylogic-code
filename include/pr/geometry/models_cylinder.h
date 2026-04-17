//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2013
//********************************
#pragma once
#include "pr/geometry/common.h"

namespace pr::geometry
{
	// Returns the number of verts and number of indices needed to hold geometry for a cylinder.
	// 'endcap_layers' is the number of latitude subdivisions per hemispheroidal end-cap; 0 = flat caps (default).
	constexpr BufSizes CylinderSize(int wedges, int layers, int endcap_layers = 0)
	{
		if (wedges < 3) wedges = 3;
		if (layers < 1) layers = 1;
		if (endcap_layers < 0) endcap_layers = 0;

		// Flat caps: 1 centre vert + (wedges+1) end-face ring per cap (bottom+top), plus (layers+1) wall rings.
		//   => verts:   2 + (wedges+1) * (layers + 3)
		//   => indices: 6 * wedges * (layers + 1)          // bottom fan(3W) + walls(6WL) + top fan(3W)
		//
		// Hemispheroid caps (endcap_layers = C >= 1): per cap, 'wedges' apex verts (one per fan triangle
		// so each triangle can carry its own 'u' coord without the apex pinch causing a texture wrap) + C
		// rings of (wedges+1) verts, where the last ring (lat=C) is the equator at z = +/-height/2 (same
		// positions as the flat-face ring, reused as the cap's equator to keep the mesh crack-free).
		//   => verts:   2*wedges + (wedges+1) * (layers + 2*C + 1)
		//   => indices: 6 * wedges * (layers + 2*C - 1)     // fan(3W) + strips(6W*(C-1)) per cap + walls(6WL)
		if (endcap_layers == 0)
		{
			return
			{
				2 + (wedges + 1) * (layers + 3),
				6 * wedges * (layers + 1),
			};
		}
		else
		{
			return
			{
				2 * wedges + (wedges + 1) * (layers + 2 * endcap_layers + 1),
				6 * wedges * (layers + 2 * endcap_layers - 1),
			};
		}
	}

	// Generate a cylinder given by a height and radius at each end, orientated with the long axis along 'Z'
	// 'radius0' is the radius of the bottom face (i.e. -z axis face) of the cylinder
	// 'radius1' is the radius of the top face (i.e. +z axis face) of the cylinder
	// 'height' is the length of the cylinder along the z axis (distance between the two equator rings; unaffected by end caps)
	// 'xscale'/'yscale' are scaling factors that can be used to make the cylinder ellipsoidal
	// 'wedges' is the number of divisions around the z axis
	// 'layers' is the number of sections along the ZAxis, must be >= 1
	// 'endcap_radius0' is the z-distance from the bottom equator (z=-height/2) to the tip of the bottom cap;
	//                  0 = flat, radius0 = true hemisphere, < radius0 = squashed, > radius0 = elongated
	// 'endcap_radius1' is the equivalent for the top cap (along +z from z=+height/2).
	// 'endcap_layers' is the number of latitude subdivisions per end-cap (0 = flat caps; extents are ignored).
	//                 Equator subdivisions always equal 'wedges', so the cap shares its equator with the barrel.
	// 'colours' is an input array of colour values, a pointer to a single colour, or null.
	// The texture coords assigned to the cylinder map a quad around the 'barrel' of the cylinder and a circle
	// on the ends of the cylinder since this is the most likely way it would be textured
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Cylinder(float radius0, float radius1, float height, float xscale, float yscale, int wedges, int layers, float endcap_radius0, float endcap_radius1, int endcap_layers, std::span<Colour32 const> colours, VOut vout, IOut iout)
	{
		if (wedges < 3) wedges = 3;
		if (layers < 1) layers = 1;
		if (endcap_layers < 0) endcap_layers = 0;
		auto [vcount,icount] = CylinderSize(wedges, layers, endcap_layers);

		Props props;
		props.m_geom = EGeom::Vert | (isize(colours) ? EGeom::Colr : EGeom::None) | EGeom::Norm | EGeom::Tex0;

		// Bounding box
		float max_radius = std::max(radius0, radius1);
		Grow(props.m_bbox, v4(-max_radius * xscale, -max_radius * yscale, -height * 0.5f - endcap_radius0, 1.0f));
		Grow(props.m_bbox, v4(+max_radius * xscale, +max_radius * yscale, +height * 0.5f + endcap_radius1, 1.0f));

		// Colour iterator wrapper
		auto col = CreateRepeater(colours, vcount, Colour32White);
		auto cc = [&](Colour32 c) { props.m_has_alpha |= HasAlpha(c); return c; };

		int const W = wedges;
		int const L = layers;
		int const C = endcap_layers;
		int const verts_per_layer = W + 1;

		auto z  = -height * 0.5f;
		auto dz = height / L;
		auto da = float(constants<double>::tau) / W;

		// Hemispheroid surface normal for a cap.
		// Surface:  (X/(r*sx))^2 + (Y/(r*sy))^2 + ((Z-zeq)/(z_sign*e))^2 = 1  (z_sign = -1 bottom, +1 top)
		// Outward gradient at (a,lat) where st=sin(lat), ct=cos(lat):
		//   (cos(a)*st/(r*sx), sin(a)*st/(r*sy), z_sign*ct/e)
		auto cap_normal = [](float ca, float sa, float st, float ct, float r, float sx, float sy, float e, float z_sign) -> v4
		{
			if (e == 0.0f || r == 0.0f) return v4(0.0f, 0.0f, z_sign, 0.0f);
			return v4::Normal(ca * st / (r * sx), sa * st / (r * sy), z_sign * ct / e, 0.0f);
		};

		// --------- Verts ---------

		auto pt = v4(0, 0, z, 1.0f);
		auto uv = v2(0.5f, 0.5f);
		auto nm = -v4::ZAxis();

		// Bottom cap: apex cluster (or flat centre) + intermediate rings (lat = 1 .. C-1)
		if (C == 0)
		{
			// Flat bottom centre (disk mapping, single vertex)
			vout(pt, cc(*col++), nm, uv);
		}
		else
		{
			// Apex cluster: W vertices at the same position/normal but with distinct u-coords
			// (u = (w+0.5)/W) so each fan triangle carries a narrow, non-wrapping u-range and the
			// texture pinches cleanly at the pole without a spiral.
			auto apex_pt = v4(0, 0, z - endcap_radius0, 1.0f);
			auto apex_nm = (endcap_radius0 == 0.0f || radius0 == 0.0f) ? -v4::ZAxis() : cap_normal(1.0f, 0.0f, 0.0f, 1.0f, radius0, xscale, yscale, endcap_radius0, -1.0f);
			auto apex_v = 1.0f + endcap_radius0 / height;
			for (int w = 0; w != W; ++w)
			{
				vout(apex_pt, cc(*col++), apex_nm, v2((w + 0.5f) / float(W), apex_v));
			}

			// Intermediate rings lat = 1 .. C-1 (the ring at lat=C is the equator ring emitted below as part of the bottom face ring)
			for (int lat = 1; lat < C; ++lat)
			{
				auto ang = float(lat) / float(C) * float(constants<double>::tau) * 0.25f;
				auto st = sin(ang);
				auto ct = cos(ang);
				for (int w = 0; w <= W; ++w)
				{
					auto a = da * w;
					auto ca = cos(a);
					auto sa = sin(a);
					pt = v4(ca * st * radius0 * xscale, sa * st * radius0 * yscale, z - endcap_radius0 * ct, 1.0f);
					nm = cap_normal(ca, sa, st, ct, radius0, xscale, yscale, endcap_radius0, -1.0f);
					uv = v2(a / float(constants<double>::tau), 1.0f + endcap_radius0 * ct / height);
					vout(pt, cc(*col++), nm, uv);
				}
			}
		}

		// Bottom face ring / cap equator (at z = -height/2).
		// Normal is -Z for flat caps; radial (gradient-at-equator) for hemispheroid caps.
		// UVs: disk mapping for flat caps, cylindrical mapping (matching wall l=0) for hemispheroid caps.
		for (int w = 0; w <= W; ++w)
		{
			auto a = da * w;
			auto ca = cos(a);
			auto sa = sin(a);
			pt = v4(ca * radius0 * xscale, sa * radius0 * yscale, z, 1.0f);
			nm = (C == 0) ? -v4::ZAxis() : cap_normal(ca, sa, 1.0f, 0.0f, radius0, xscale, yscale, endcap_radius0, -1.0f);
			uv = (C == 0) ? v2(ca * 0.5f + 0.5f, sa * 0.5f + 0.5f) : v2(a / float(constants<double>::tau), 1.0f);
			vout(pt, cc(*col++), nm, uv);
		}

		// The walls
		for (int l = 0; l <= L; ++l)
		{
			auto r  = Lerp(radius0, radius1, l/(float)L);
			auto nz = radius0 - radius1;
			for (int w = 0; w <= W; ++w)
			{
				auto a = da * w;
				pt = v4(cos(a) * r * xscale, sin(a) * r * yscale, z, 1.0f);

				// Smooth per-vertex radial normal (at the vertex's own angle, not the wedge midpoint).
				// Using the midpoint angle produces flat-shaded wedges which show up as a hard seam at
				// the cap/barrel equator when end-caps are enabled. 'height' and 'nz' scale the radial
				// vs axial components so the normal tilts correctly for cones.
				nm = v4::Normal(height * cos(a) / xscale, height * sin(a) / yscale, nz, 0.0f);
				uv = v2(a / float(constants<double>::tau), 1.0f - (z + height*0.5f) / height);
				vout(pt, cc(*col++), nm, uv);
			}
			if (l != L) z += dz;
		}

		// Top face ring / cap equator (at z = +height/2).
		// UVs: disk mapping for flat caps, cylindrical mapping (matching wall l=L) for hemispheroid caps.
		for (int w = 0; w <= W; ++w)
		{
			auto a = da * w;
			auto ca = cos(a);
			auto sa = sin(a);
			pt = v4(ca * radius1 * xscale, sa * radius1 * yscale, z, 1.0f);
			nm = (C == 0) ? v4::ZAxis() : cap_normal(ca, sa, 1.0f, 0.0f, radius1, xscale, yscale, endcap_radius1, +1.0f);
			uv = (C == 0) ? v2(ca * 0.5f + 0.5f, sa * 0.5f + 0.5f) : v2(a / float(constants<double>::tau), 0.0f);
			vout(pt, cc(*col++), nm, uv);
		}

		// Top cap: intermediate rings (lat = C-1 .. 1) + apex (or flat centre)
		if (C == 0)
		{
			// Flat top centre (disk mapping)
			pt = v4(0, 0, z, 1.0f);
			vout(pt, cc(*col++), v4::ZAxis(), v2(0.5f, 0.5f));
		}
		else
		{
			for (int lat = C - 1; lat >= 1; --lat)
			{
				auto ang = float(lat) / float(C) * float(constants<double>::tau) * 0.25f;
				auto st = sin(ang);
				auto ct = cos(ang);
				for (int w = 0; w <= W; ++w)
				{
					auto a = da * w;
					auto ca = cos(a);
					auto sa = sin(a);
					pt = v4(ca * st * radius1 * xscale, sa * st * radius1 * yscale, z + endcap_radius1 * ct, 1.0f);
					nm = cap_normal(ca, sa, st, ct, radius1, xscale, yscale, endcap_radius1, +1.0f);
					uv = v2(a / float(constants<double>::tau), -endcap_radius1 * ct / height);
					vout(pt, cc(*col++), nm, uv);
				}
			}

			// Apex cluster: W vertices at the same position/normal with distinct u-coords.
			auto apex_pt = v4(0, 0, z + endcap_radius1, 1.0f);
			auto apex_nm = (endcap_radius1 == 0.0f || radius1 == 0.0f) ? v4::ZAxis() : cap_normal(1.0f, 0.0f, 0.0f, 1.0f, radius1, xscale, yscale, endcap_radius1, +1.0f);
			auto apex_v = -endcap_radius1 / height;
			for (int w = 0; w != W; ++w)
			{
				vout(apex_pt, cc(*col++), apex_nm, v2((w + 0.5f) / float(W), apex_v));
			}
		}

		// --------- Indices ---------

		// Bottom cap indices
		if (C == 0)
		{
			// Flat fan: centre (0) -> bottom face ring (1..W+1)
			int ibase = 1;
			for (int w = 0; w != W; ++w)
			{
				iout(0);
				iout(ibase + w + 1);
				iout(ibase + w);
			}
		}
		else
		{
			// Apex fan: apex cluster (indices 0..W-1, one apex vert per triangle) -> first cap ring at index W.
			int ibase = W;
			for (int w = 0; w != W; ++w)
			{
				iout(w);
				iout(ibase + w + 1);
				iout(ibase + w);
			}
			// Strips between cap rings lat=k and lat=k+1 for k=1..C-1 (last strip goes to the equator ring).
			for (int lat = 1; lat < C; ++lat)
			{
				for (int w = 0; w != W; ++w)
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
		}

		// Walls indices
		// Index of the first wall-ring vertex (wall ring l=0 sits right after the bottom equator ring).
		// C==0: apex(1) + equator(W+1) = 1 + vpl.  C>0: apex cluster(W) + (C-1) intermediate rings + equator = W + C*vpl.
		int ibase = (C == 0) ? (1 + verts_per_layer) : (W + C * verts_per_layer);
		for (int l = 0; l != L; ++l)
		{
			for (int w = 0; w != W; ++w)
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

		// Top cap indices
		if (C == 0)
		{
			// Flat fan: top face ring -> top centre (last)
			int top_ring = ibase + verts_per_layer; // skip the final wall ring (l=L)
			int const last = vcount - 1;
			for (int w = 0; w != W; ++w)
			{
				iout(top_ring + w);
				iout(top_ring + w + 1);
				iout(last);
			}
		}
		else
		{
			// Strips between equator ring and each intermediate top-cap ring, then fan to the apex cluster.
			int ring = ibase + verts_per_layer; // top equator ring starts after final wall ring
			for (int lat = C; lat > 1; --lat)
			{
				for (int w = 0; w != W; ++w)
				{
					iout(ring + w);
					iout(ring + w + 1);
					iout(ring + w + verts_per_layer);
					iout(ring + w + verts_per_layer);
					iout(ring + w + 1);
					iout(ring + w + verts_per_layer + 1);
				}
				ring += verts_per_layer;
			}
			// Apex fan: last top-cap ring (indices ring..ring+W) -> top apex cluster (vcount-W .. vcount-1).
			int apex_base = vcount - W;
			for (int w = 0; w != W; ++w)
			{
				iout(ring + w);
				iout(ring + w + 1);
				iout(apex_base + w);
			}
		}

		return props;
	}
}
