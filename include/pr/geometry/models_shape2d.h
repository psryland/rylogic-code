//******************************************************************
// Shape2d
//  Copyright (c) Rylogic Ltd 2014
//******************************************************************

#pragma once

#include "pr/geometry/common.h"

namespace pr::geometry
{
	// Circle/Ellipse *************************************************************************

	// A description of an ellipse/circle shape. 'cx,cy' is an optional centre offset so that several
	// ellipses can be combined into one model at independent locations.
	struct EllipseShape
	{
		float dimx, dimy;
		float cx = 0, cy = 0;
	};

	// Returns the number of verts and indices needed to hold geometry for a single 'Ellipse'
	constexpr BufSizes EllipseSize(bool solid, int facets)
	{
		facets = std::max(facets, 3);
		return
		{
			facets + (solid ? 1 : 0),
			solid ? 1 + 2 * facets : facets + 1,
		};
	}

	// Returns the number of verts and indices needed to hold geometry for a multi-ellipse model.
	// Consecutive ellipses are joined into one model and separated by a single strip-cut (primitive
	// restart) index, so the total index count includes one extra index per gap between ellipses.
	constexpr BufSizes EllipseSize(std::span<EllipseShape const> shapes, bool solid, int facets)
	{
		int vcount = 0, icount = 0;
		for ([[maybe_unused]] auto const& s : shapes)
		{
			auto sz = EllipseSize(solid, facets);
			vcount += sz.vcount;
			icount += sz.icount;
		}
		icount += std::max(0, static_cast<int>(shapes.size()) - 1);
		return { vcount, icount };
	}

	// Generate an ellipse shape
	// 'solid' - true = tristrip model, false = linestrip model
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Ellipse(EllipseShape shape, bool solid, int facets, Colour32 colour, VOut vout, IOut iout)
	{
		facets = std::max(facets, 3);

		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | (solid ? EGeom::Norm : EGeom::None) | (solid ? EGeom::Tex0 : EGeom::None);
		props.m_bbox = BBox(v4(shape.cx, shape.cy, 0, 1), v4(shape.dimx, shape.dimy, 0, 0));

		// Set Verts. Each vertex is positioned relative to the shape's centre so multiple ellipses in
		// one model can sit at independent locations.
		for (int i = 0; i != facets; ++i)
		{
			auto a = float(constants<double>::tau * i / facets);
			auto c = std::cos(a);
			auto s = std::sin(a);
			vout(v4(shape.cx + shape.dimx * c, shape.cy + shape.dimy * s, 0, 1), colour, v4::ZAxis(), v2(0.5f*(c + 1), 0.5f*(1 - s)));
		}
		if (solid)
			vout(v4(shape.cx, shape.cy, 0, 1), colour, v4::ZAxis(), v2(0.5f, 0.5f));

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

	// Generate an ellipse shape centred at the origin
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Ellipse(float dimx, float dimy, bool solid, int facets, Colour32 colour, VOut vout, IOut iout)
	{
		return Ellipse(EllipseShape{ .dimx = dimx, .dimy = dimy }, solid, facets, colour, vout, iout);
	}

	// Generate a multi-ellipse shape as a single model.
	// Each ellipse is emitted as its own strip (tri-strip when 'solid', otherwise line-strip). Consecutive
	// ellipses are separated by a strip-cut so they don't visually connect. The strip-cut is signalled to
	// 'iout' as the value -1; callers building an index buffer translate it to the index format's
	// primitive-restart sentinel (e.g. via IdxBuf::set_strip_cut). All ellipses share one colour.
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Ellipse(std::span<EllipseShape const> shapes, bool solid, int facets, Colour32 colour, VOut vout, IOut iout)
	{
		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | (solid ? EGeom::Norm : EGeom::None) | (solid ? EGeom::Tex0 : EGeom::None);

		// Emit each ellipse's strip in turn. Local indices are offset by the running vertex base so they
		// address this ellipse's verts within the combined buffer, and a strip-cut is inserted between
		// ellipses.
		int vbase = 0;
		for (int i = 0; i != static_cast<int>(shapes.size()); ++i)
		{
			if (i != 0)
				iout(-1);

			auto sprops = Ellipse(shapes[i], solid, facets, colour, vout, [&](int idx) { iout(vbase + idx); });

			props.m_geom |= sprops.m_geom;
			Grow(props.m_bbox, sprops.m_bbox);
			vbase += EllipseSize(solid, facets).vcount;
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
		float cx = 0, cy = 0;
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

		// Set Verts. Each wedge vertex is the polar position scaled non-uniformly then translated by the
		// wedge's centre, so multiple wedges in one model can sit at independent locations.
		for (int i = 0; i <= facets; ++i)
		{
			auto a = Lerp(wedge.ang0, wedge.ang1, float(i) / facets);
			auto c = std::cos(a);
			auto s = std::sin(a);
			vout(bb(v4(wedge.cx + wedge.radius0 * wedge.scalex * c, wedge.cy + wedge.radius0 * wedge.scaley * s, 0, 1)), colour, v4::ZAxis(), v2(0.5f + 0.5f*tr0*c, 0.5f - 0.5f*tr0*s));
			vout(bb(v4(wedge.cx + wedge.radius1 * wedge.scalex * c, wedge.cy + wedge.radius1 * wedge.scaley * s, 0, 1)), colour, v4::ZAxis(), v2(0.5f + 0.5f*tr1*c, 0.5f - 0.5f*tr1*s));
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

	// A description of a rectangle shape. 'dimx,dimy' are half-extents. 'cx,cy' is an optional centre
	// offset so that several rectangles can be combined into one model at independent locations. The
	// corner radius is shared across all rectangles in a model and is passed separately.
	struct RectShape
	{
		float dimx, dimy;
		float cx = 0, cy = 0;
	};

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

	// Returns the number of verts and indices needed to hold geometry for a multi-rectangle model.
	// Consecutive rectangles are joined into one model and separated by a single strip-cut (primitive
	// restart) index, so the total index count includes one extra index per gap between rectangles.
	constexpr BufSizes RoundedRectangleSize(std::span<RectShape const> shapes, bool solid, float corner_radius, int facets)
	{
		int vcount = 0, icount = 0;
		for ([[maybe_unused]] auto const& s : shapes)
		{
			auto sz = RoundedRectangleSize(solid, corner_radius, facets);
			vcount += sz.vcount;
			icount += sz.icount;
		}
		icount += std::max(0, static_cast<int>(shapes.size()) - 1);
		return { vcount, icount };
	}

	// Generate a Rectangle shape with rounded corners
	//' 'solid' - true = tristrip model, false = linestrip model
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props RoundedRectangle(RectShape shape, bool solid, float corner_radius, int facets, Colour32 colour, VOut vout_, IOut iout)
	{
		auto dimx = shape.dimx;
		auto dimy = shape.dimy;
		if (dimx < 0) { assert(!"Rectangle model dimension X is less than zero"); dimx = 0; }
		if (dimy < 0) { assert(!"Rectangle model dimension Y is less than zero"); dimy = 0; }

		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | (solid ? EGeom::Norm : EGeom::None) | (solid ? EGeom::Tex0 : EGeom::None);
		props.m_bbox = BBox(v4(shape.cx, shape.cy, 0, 1), v4(dimx, dimy, 0, 0));

		// Offset every emitted vertex by the shape's centre so multiple rectangles in one model can sit at
		// independent locations.
		auto vout = [&](v4 p, Colour32 c, v4 n, v2 t) { vout_(v4(p.x + shape.cx, p.y + shape.cy, p.z, p.w), c, n, t); };

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

	// Generate a Rectangle shape with rounded corners centred at the origin
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props RoundedRectangle(float dimx, float dimy, bool solid, float corner_radius, int facets, Colour32 colour, VOut vout, IOut iout)
	{
		return RoundedRectangle(RectShape{ .dimx = dimx, .dimy = dimy }, solid, corner_radius, facets, colour, vout, iout);
	}

	// Generate a multi-rectangle shape as a single model.
	// Each rectangle is emitted as its own strip (tri-strip when 'solid', otherwise line-strip).
	// Consecutive rectangles are separated by a strip-cut so they don't visually connect. The strip-cut is
	// signalled to 'iout' as the value -1; callers building an index buffer translate it to the index
	// format's primitive-restart sentinel (e.g. via IdxBuf::set_strip_cut). The corner radius is shared by
	// all rectangles, and all rectangles share one colour.
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props RoundedRectangle(std::span<RectShape const> shapes, bool solid, float corner_radius, int facets, Colour32 colour, VOut vout, IOut iout)
	{
		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | (solid ? EGeom::Norm : EGeom::None) | (solid ? EGeom::Tex0 : EGeom::None);

		// Emit each rectangle's strip in turn. Local indices are offset by the running vertex base so they
		// address this rectangle's verts within the combined buffer, and a strip-cut is inserted between
		// rectangles.
		int vbase = 0;
		for (int i = 0; i != static_cast<int>(shapes.size()); ++i)
		{
			if (i != 0)
				iout(-1);

			auto sprops = RoundedRectangle(shapes[i], solid, corner_radius, facets, colour, vout, [&](int idx) { iout(vbase + idx); });

			props.m_geom |= sprops.m_geom;
			Grow(props.m_bbox, sprops.m_bbox);
			vbase += RoundedRectangleSize(solid, corner_radius, facets).vcount;
		}

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

	// Returns the number of verts and indices needed to hold geometry for a multi-polygon model.
	// Solid polygons are triangle lists concatenated into one nugget with no separators. Wireframe
	// polygons are separate closed line-strips joined by a single strip-cut (primitive restart) index, so
	// the wireframe index count includes one extra index per gap between polygons.
	constexpr BufSizes PolygonSize(std::span<std::span<v2 const> const> polys, bool solid)
	{
		int vcount = 0, icount = 0;
		for (auto const& p : polys)
		{
			auto sz = PolygonSize(static_cast<int>(p.size()), solid);
			vcount += sz.vcount;
			icount += sz.icount;
		}
		if (!solid)
			icount += std::max(0, static_cast<int>(polys.size()) - 1);
		return { vcount, icount };
	}

	// Generate a multi-polygon shape as a single model.
	// 'polys' - one point list per polygon, each already in CCW winding order.
	// 'solid' - if true, creates a TriList model (triangle lists concatenated, no strip-cut); if false,
	//           creates a LineStrip model where consecutive polygons are separated by a strip-cut.
	// 'colours' - one colour list per polygon, each 0, 1, or num_points long. A polygon with no colours
	//             defaults to white. The strip-cut is signalled to 'iout' as the value -1; callers
	//             translate it to the index format's primitive-restart sentinel.
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Polygon(std::span<std::span<v2 const> const> polys, bool solid, std::span<std::span<Colour32 const> const> colours, VOut vout, IOut iout)
	{
		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | (solid ? EGeom::Norm : EGeom::None);

		// Emit each polygon in turn. Local indices are offset by the running vertex base so they address
		// this polygon's verts within the combined buffer. Wireframe polygons get a strip-cut between
		// them; solid polygons are concatenated triangle lists with no separator.
		int vbase = 0;
		for (int pi = 0; pi != static_cast<int>(polys.size()); ++pi)
		{
			auto points = polys[pi];
			auto cols = pi < static_cast<int>(colours.size()) ? colours[pi] : std::span<Colour32 const>{};

			if (!solid && pi != 0)
				iout(-1);

			auto sprops = Polygon(points, solid, cols, vout, [&](int idx) { iout(vbase + idx); });

			props.m_geom |= sprops.m_geom;
			props.m_has_alpha |= sprops.m_has_alpha;
			Grow(props.m_bbox, sprops.m_bbox);
			vbase += static_cast<int>(points.size());
		}

		return props;
	}
}