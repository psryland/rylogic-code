//******************************************************************
// Shape2d
//  Copyright (c) Rylogic Ltd 2014
//******************************************************************

#pragma once

#include "pr/geometry/common.h"

namespace pr::geometry
{
	// Circle/Ellipse *************************************************************************

	// Returns the number of verts and indices needed to hold geometry for an 'Ellipse'
	constexpr BufSizes EllipseSize(bool solid, int facets)
	{
		facets = std::max(facets, 3);
		return
		{
			facets + (solid ? 1 : 0),
			solid ? 1 + 2 * facets : facets + 1,
		};
	}

	// Generate an ellipse shape
	// 'solid' - true = tristrip model, false = linestrip model
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Ellipse(float dimx, float dimy, bool solid, int facets, Colour32 colour, VOut vout, IOut iout)
	{
		facets = std::max(facets, 3);

		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | (solid ? EGeom::Norm : EGeom::None) | (solid ? EGeom::Tex0 : EGeom::None);
		props.m_bbox = BBox(v4::Origin(), v4(dimx, dimy, 0, 0));

		// Set Verts
		for (int i = 0; i != facets; ++i)
		{
			auto a = float(constants<double>::tau * i / facets);
			auto c = std::cos(a);
			auto s = std::sin(a);
			vout(v4(dimx * c, dimy * s, 0, 1), colour, v4::ZAxis(), v2(0.5f*(c + 1), 0.5f*(1 - s)));
		}
		if (solid)
			vout(v4::Origin(), colour, v4::ZAxis(), v2(0.5f, 0.5f));

		if (solid)
		{
			// Set faces
			iout(0);
			for (int i = facets; i-- != 0;)
			{
				iout(facets);
				iout(i);
			}
		}
		else // border only
		{
			// Set edges
			for (int i = 0; i != facets; ++i)
				iout(i);
			iout(0);
		}

		return props;
	}

	// Pie/Wedge ******************************************************************************

	// A description of a pie/wedge shape
	struct Wedge
	{
		float ang0, ang1;
		float radius0, radius1;
		float scalex = 1, scaley = 1;
	};

	// Returns the number of facets used for a wedge spanning [ang0,ang1] (radians).
	// The full-circle facet count is scaled by the fraction of a circle the wedge covers so the facet
	// density stays consistent regardless of the wedge's angular size, with a minimum of 3. Buffer
	// sizing and geometry generation both go through this so their facet counts always agree.
	constexpr int PieFacets(float ang0, float ang1, int facets)
	{
		auto scale = Abs(ang1 - ang0) / constants<float>::tau;
		return std::max(int(scale * facets + 0.5f), 3);
	}

	// Returns the number of verts and indices needed to hold geometry for a single-wedge 'Pie'
	constexpr BufSizes PieSize(bool solid, float ang0, float ang1, int facets)
	{
		facets = PieFacets(ang0, ang1, facets);
		return
		{
			2 * (facets + 1),
			solid ? 2 * (facets + 1) : 2 * facets + 3,
		};
	}

	// Returns the number of verts and indices needed to hold geometry for a multi-wedge 'Pie'.
	// Consecutive wedges are joined into one model and separated by a single strip-cut (primitive
	// restart) index, so the total index count includes one extra index per gap between wedges.
	constexpr BufSizes PieSize(std::span<Wedge const> wedges, bool solid, int facets)
	{
		int vcount = 0, icount = 0;
		for (auto const& w : wedges)
		{
			auto sz = PieSize(solid, w.ang0, w.ang1, facets);
			vcount += sz.vcount;
			icount += sz.icount;
		}
		icount += std::max(0, static_cast<int>(wedges.size()) - 1);
		return { vcount, icount };
	}

	// Generate a pie/wedge shape
	// 'ang0','ang1' = start/end angle in radians
	// 'solid' - true = tristrip model, false = linestrip model
	// 'facets' - the number of facets for a complete ring, scaled to the actual ang0->ang1 range
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Pie(Wedge wedge, bool solid, int facets, Colour32 colour, VOut vout, IOut iout)
	{
		facets = PieFacets(wedge.ang0, wedge.ang1, facets);
		wedge.radius0 = std::max(0.0f, wedge.radius0);
		wedge.radius1 = std::max(wedge.radius0, wedge.radius1);
			
		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | (solid ? EGeom::Norm : EGeom::None) | (solid ? EGeom::Tex0 : EGeom::None);

		// Bounding box
		auto bb = [&](v4 v) { Grow(props.m_bbox, v); return v; };

		// Tex coords
		auto tr0 = FEql(wedge.radius1, 0.f) ? 0.0f : wedge.radius0 / wedge.radius1;
		auto tr1 = 1.0f;

		// Set Verts
		for (int i = 0; i <= facets; ++i)
		{
			auto a = Lerp(wedge.ang0, wedge.ang1, float(i) / facets);
			auto c = std::cos(a);
			auto s = std::sin(a);
			vout(bb(v4(wedge.radius0 * wedge.scalex * c, wedge.radius0 * wedge.scaley * s, 0, 1)), colour, v4::ZAxis(), v2(0.5f + 0.5f*tr0*c, 0.5f - 0.5f*tr0*s));
			vout(bb(v4(wedge.radius1 * wedge.scalex * c, wedge.radius1 * wedge.scaley * s, 0, 1)), colour, v4::ZAxis(), v2(0.5f + 0.5f*tr1*c, 0.5f - 0.5f*tr1*s));
		}

		if (solid)
		{
			// Set faces
			int idx = 0;
			for (int i = 0; i <= facets; ++i)
			{
				iout(idx++);
				iout(idx++);
			}
		}
		else // border only
		{
			// Set lines
			for (int i = 0; i <= facets; ++i) iout(2 * i);
			for (int i = facets; i >= 0; --i) iout(1+2*i);
			iout(0);
		}

		return props;
	}

	// Generate a multi-wedge pie shape as a single model.
	// Each wedge is emitted as its own strip (tri-strip when 'solid', otherwise line-strip). Consecutive
	// wedges are separated by a strip-cut so they don't visually connect. The strip-cut is signalled to
	// 'iout' as the value -1; callers building an index buffer translate it to the index format's
	// primitive-restart sentinel (e.g. via IdxBuf::set_strip_cut). All wedges share one colour.
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Pie(std::span<Wedge const> wedges, bool solid, int facets, Colour32 colour, VOut vout, IOut iout)
	{
		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | (solid ? EGeom::Norm : EGeom::None) | (solid ? EGeom::Tex0 : EGeom::None);

		// Emit each wedge's strip in turn. Local wedge indices are offset by the running vertex base so
		// they address this wedge's verts within the combined buffer, and a strip-cut is inserted in the
		// gaps between wedges.
		int vbase = 0;
		for (int w = 0; w != static_cast<int>(wedges.size()); ++w)
		{
			if (w != 0)
				iout(-1);

			auto wprops = Pie(wedges[w], solid, facets, colour, vout, [&](int idx) { iout(vbase + idx); });

			props.m_geom |= wprops.m_geom;
			Grow(props.m_bbox, wprops.m_bbox);
			vbase += PieSize(solid, wedges[w].ang0, wedges[w].ang1, facets).vcount;
		}

		return props;
	}

	// Rounded Rectangle **********************************************************************

	// Returns the number of verts and indices needed to hold geometry for a 'RoundedRectangle'
	constexpr BufSizes RoundedRectangleSize(bool solid, float corner_radius, int facets)
	{
		auto verts_per_cnr = corner_radius != 0.0f ? std::max(facets / 4, 0) + 1 : 1;
		return
		{
			4 * verts_per_cnr,
			solid ? 4 * verts_per_cnr : 4 * verts_per_cnr + 1,
		};
	}

	// Generate a Rectangle shape with rounded corners
	//' 'solid' - true = tristrip model, false = linestrip model
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props RoundedRectangle(float dimx, float dimy, bool solid, float corner_radius, int facets, Colour32 colour, VOut vout, IOut iout)
	{
		if (dimx < 0) { assert(!"Rectangle model dimension X is less than zero"); dimx = 0; }
		if (dimy < 0) { assert(!"Rectangle model dimension Y is less than zero"); dimy = 0; }

		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | (solid ? EGeom::Norm : EGeom::None) | (solid ? EGeom::Tex0 : EGeom::None);
		props.m_bbox = BBox(v4::Origin(), v4(dimx, dimy, 0, 0));

		// Limit the rounding to half the smallest rectangle side length
		auto rad = corner_radius;
		if (rad > dimx) rad = dimx;
		if (rad > dimy) rad = dimy;
		auto verts_per_cnr = rad != 0.0f ? std::max(facets / 4, 0) + 1 : 1;
		auto cos = [=](int i) { return verts_per_cnr > 1 ? Cos(constants<float>::tau_by_4 * i / (verts_per_cnr - 1)) : 0.0f; };
		auto sin = [=](int i) { return verts_per_cnr > 1 ? Sin(constants<float>::tau_by_4 * i / (verts_per_cnr - 1)) : 0.0f; };
				
		// Texture coords
		auto tx = rad / (2 * dimx);
		auto ty = rad / (2 * dimy);
		auto t0 = 0.0000f;
		auto t1 = 0.9999f;

		if (solid)
		{
			// Set verts
			for (int i = 0; i != verts_per_cnr; ++i)
			{
				auto c = cos(i);
				auto s = sin(i);
				vout(v4(-dimx + rad * (1 - c), +dimy - rad * (1 - s), 0, 1), colour, v4::ZAxis(), v2(t0 + (1 - c) * tx, t0 + (1 - s) * ty));
				vout(v4(-dimx + rad * (1 - c), -dimy + rad * (1 - s), 0, 1), colour, v4::ZAxis(), v2(t0 + (1 - c) * tx, t1 - (1 - s) * ty));
			}
			for (int i = 0; i != verts_per_cnr; ++i)
			{
				auto c = cos(i);
				auto s = sin(i);
				vout(v4(+dimx - rad * (1 - s), +dimy - rad * (1 - c), 0, 1), colour, v4::ZAxis(), v2(t1 - (1 - s)*tx, t0 + (1 - c)*ty));
				vout(v4(+dimx - rad * (1 - s), -dimy + rad * (1 - c), 0, 1), colour, v4::ZAxis(), v2(t1 - (1 - s)*tx, t1 - (1 - c)*ty));
			}
		}
		else // border only
		{
			// Set verts
			for (int i = 0; i != verts_per_cnr; ++i)
			{
				auto c = cos(i);
				auto s = sin(i);
				vout(v4(-dimx + rad * (1 - c), -dimy + rad * (1 - s), 0, 1), colour, v4::ZAxis(), v2(t0 + (1 - c)*tx, t1 - (1 - s)*ty));
			}
			for (int i = 0; i != verts_per_cnr; ++i)
			{
				auto c = cos(i);
				auto s = sin(i);
				vout(v4(+dimx - rad * (1 - s), -dimy + rad * (1 - c), 0, 1), colour, v4::ZAxis(), v2(t1 - (1 - s)*tx, t1 - (1 - c)*ty));
			}
			for (int i = 0; i != verts_per_cnr; ++i)
			{
				auto c = cos(i);
				auto s = sin(i);
				vout(v4(+dimx - rad * (1 - c), +dimy - rad * (1 - s), 0, 1), colour, v4::ZAxis(), v2(t1 - (1 - c)*tx, t0 + (1 - s)*ty));
			}
			for (int i = 0; i != verts_per_cnr; ++i)
			{
				auto c = cos(i);
				auto s = sin(i);
				vout(v4(-dimx + rad * (1 - s), +dimy - rad * (1 - c), 0, 1), colour, v4::ZAxis(), v2(t0 + (1 - s)*tx, t0 + (1 - c)*ty));
			}
		}

		// Set faces/edges
		for (int i = 0; i != verts_per_cnr*4; ++i)
			iout(i);
		if (!solid)
			iout(0);

		return props;
	}

	// Polygon ********************************************************************************

	// Returns the number of verts and indices needed to hold geometry for a 'Polygon'
	constexpr BufSizes PolygonSize(int num_points, bool solid)
	{
		// Solid polygons have to be triangulated. The number of faces is (num_verts - 2)
		return
		{
			num_points,
			solid ? 3 * (num_points - 2) : num_points + 1,
		};
	}

	// Generate a Polygon shape 
	// 'num_points' - the length of the 'points' array
	// 'points' - the 2d points of the polygon. With CCW winding order.
	// 'solid' - if true, creates a TriList model. If false, creates a line strip model
	// 'num_colours' - The number of colours in the 'colours' array. Can be 0, 1, or num_points.
	// 'colours' - A array of colour values for the polygon
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Polygon(std::span<v2 const> points, bool solid, std::span<Colour32 const> colours, VOut vout, IOut iout)
	{
		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | (solid ? EGeom::Norm : EGeom::None);

		// Colour iterator
		auto col = CreateRepeater(colours, isize(points)-1, Colour32White);
		auto cc = [&](Colour32 c) { props.m_has_alpha |= HasAlpha(c); return c; };

		// Bounding box
		auto bb = [&](v4 v) { Grow(props.m_bbox, v); return v; };

		// Verts
		for (int i = 0; i != isize(points); ++i)
			vout(bb(v4(points[i], 0, 1)), cc(*col++), v4::ZAxis(), v2::Zero());

		// Faces/Lines
		if (solid)
		{
			TriangulatePolygon(points, [&](int i0, int i1, int i2)
			{
				iout(i0);
				iout(i1);
				iout(i2);
			});
		}
		else
		{
			for (int i = 0; i != isize(points); ++i)
				iout(i);
			
			iout(0);
		}

		return props;
	}
}