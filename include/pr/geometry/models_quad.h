//******************************************************************
// Quad
//  Copyright (c) Rylogic Ltd 2013
//******************************************************************
// Create a rectangular patch with texture coordinates
//  geometry - the geometry object to return
//  patch_origin      - the coordinate of the left, top edge of the patch
//  patch_dimension   - the overall width/height of the patch
//  patch_divisions   - how many quads to create over the width/height of the patch
//  texture_origin    - the texture coordinate to start with at the top/left corner of the patch
//  texture_dimension - the width/height of the texture on the patch.
//                      If this is less than the dimension of the patch then the texture coords will repeat
//
// The returned patch will look like:
//  0,0 ---------------------> quad_w
//   | +-----+-----+-----+-----+
//   | | 0  /| 2  /| 4  /| 6  /|
//   | |  /  |  /  |  /  |  /  |
//   | |/  1 |/  3 |/  5 |/  7 |
//   | +-----+-----+-----+-----+
//   | | 8  /| 10 /| 12 /| 14 /|
//   | |  /  |  /  |  /  |  /  |
//   | |/  9 |/ 11 |/ 13 |/ 15 |
//   V +-----+-----+-----+-----+
// quad_h
//
// Vertex order:
// 0-----1-----2----3
// |     |     |    |
// 4-----5-----6----7
// |     |     |    |
//
// Face vertex order:
//  0----2,3---
//  |   / |
//  | /   | /
//  1,4---5---
//  |     |
#pragma once
#include "pr/geometry/common.h"

namespace pr::geometry
{
	// Returns the number of verts and indices needed to hold geometry for a quad
	constexpr BufSizes QuadSize(int num_quads)
	{
		return
		{
			4 * num_quads,
			6 * num_quads,
		};
	}

	// Returns the number of verts and indices needed to hold geometry for a quad/patch
	constexpr BufSizes QuadSize(iv2 divisions)
	{
		return 
		{
			1 * (divisions.x + 2) * (divisions.y + 2),
			6 * (divisions.x + 1) * (divisions.y + 1),
		};
	}

	// Returns the number of verts and indices needed to hold geometry for a quad strip
	constexpr BufSizes QuadStripSize(int num_quads)
	{
		// A quad plus corner per quad
		return 
		{
			4 * num_quads,
			4 * num_quads,
		};
	}

	// Returns the number of verts and indices needed to hold a quad patch built from triangle strips
	constexpr BufSizes QuadPatchSize(int dimx, int dimy)
	{
		return
		{
			dimx * dimy,
			(2 * dimx + 2) * (dimy - 1),
		};
	}

	// Returns the number of verts and indices needed to hold a hex patch built from triangle strips
	constexpr BufSizes HexPatchSize(int rings)
	{
		return 
		{
			ArithmeticSum(0, 6, rings) + 1,
			ArithmeticSum(0, 12, rings) + 2 * rings,
		};
	}

	// Generate quads from sets of four points
	// Point Order: (bottom to top 'S')
	//  -x, -y = 0  uv = 00
	//  +x, -y = 1  uv = 10
	//  -x, +y = 2  uv = 01
	//  +x, +y = 3  uv = 11
	// 'num_quads' is the number of sets of 4 points pointed to by 'verts'
	// 'verts' is the input array of corner points for the quads
	// 'num_colours' should be either 0, 1, num_quads, or num_quads*4 representing; no colour, 1 colour for all, 1 colour per quad, or 1 colour per quad vertex
	// 't2q' is a transform to apply to the standard texture coordinates 0,0 -> 1,1
	// 'out_verts' is an output iterator to receive the [vert,norm,colour,tex] data
	// 'out_indices' is an output iterator to receive the index data
	template <typename TVertCIter, VertOutputFn VOut, IndexOutputFn IOut>
	Props Quad(int num_quads, TVertCIter verts, std::span<Colour32 const> colours, m4x4 const& t2q, VOut vout, IOut iout)
	{
		Props props;
		props.m_geom = EGeom::Vert | (isize(colours) ? EGeom::Colr : EGeom::None) | EGeom::Norm | EGeom::Tex0;

		// Helper function for generating normals
		auto norm = [](v4 a, v4 b, v4 c) { return Normalise(Cross3(a - b, c - b), v4::Zero()); };

		// Colour iterator wrapper
		auto col = CreateRepeater(colours, num_quads * 4, Colour32White);
		auto cc = [&](Colour32 c) { props.m_has_alpha |= HasAlpha(c); return c; };

		// Bounding box
		auto bb = [&](v4 v) { Grow(props.m_bbox, v); return v; };

		// Texture coords
		auto t00 = (t2q * v4(0.0f, 0.0f, 0.0f, 1.0f)).xy;
		auto t10 = (t2q * v4(1.0f, 0.0f, 0.0f, 1.0f)).xy;
		auto t01 = (t2q * v4(0.0f, 1.0f, 0.0f, 1.0f)).xy;
		auto t11 = (t2q * v4(1.0f, 1.0f, 0.0f, 1.0f)).xy;

		// Generate verts and faces
		for (int i = 0; i != num_quads; ++i)
		{
			auto v0 = *verts++;
			auto v1 = *verts++;
			auto v2 = *verts++;
			auto v3 = *verts++;
			auto c0 = *col++;
			auto c1 = *col++;
			auto c2 = *col++;
			auto c3 = *col++;

			// Set verts
			vout(bb(v0), cc(c0), norm(v1,v0,v2), t00);
			vout(bb(v1), cc(c1), norm(v3,v1,v0), t10);
			vout(bb(v2), cc(c2), norm(v0,v2,v3), t01);
			vout(bb(v3), cc(c3), norm(v2,v3,v1), t11);

			// Set faces
			auto ibase = i * 4;
			iout(ibase + 0);
			iout(ibase + 1);
			iout(ibase + 2);
			iout(ibase + 2);
			iout(ibase + 1);
			iout(ibase + 3);
		}
		return props;
	}
	template <typename TVertCIter, VertOutputFn VOut, IndexOutputFn IOut>
	Props Quad(int num_quads, TVertCIter verts, std::span<Colour32 const> colours, VOut vout, IOut iout)
	{
		return Quad(num_quads, verts, colours, m4x4::Identity(), vout, iout);
	}
	template <typename TVertCIter, VertOutputFn VOut, IndexOutputFn IOut>
	Props Quad(int num_quads, TVertCIter verts, VOut vout, IOut iout)
	{
		return Quad(num_quads, verts, {}, m4x4::Identity(), vout, iout);
	}

	// Generate an NxM patch of triangles
	// 'anchor' is the origin of the quad, (0,0) = centre, (-1,-1) = left,bottom, (+1,+1) = right,top, etc
	// 'quad_w' is the length and direction of the quad_w axis
	// 'quad_h' is the length and direction of the quad_h axis
	// 'divisions' is the number of times to divide the width/height of the quad. Note: num_verts_across = divisions.x + 2
	// 'colour' is a colour for the whole quad
	// 't2q' is a transform to apply to the standard texture coordinates 0,0 -> 1,1
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Quad(v2 const& anchor, v4 const& quad_w, v4 const& quad_h, iv2 const& divisions, Colour32 colour, m4x4 const& t2q, VOut vout, IOut iout)
	{
		// Set the start point so that the model origin matches 'anchor'
		auto origin = v4Origin
			- 0.5f * (1.0f + anchor.x) * quad_w
			- 0.5f * (1.0f + anchor.y) * quad_h;
		auto norm = Normalise(Cross3(quad_w, quad_h));
		auto step_x = quad_w / float(divisions.x + 1);
		auto step_y = quad_h / float(divisions.y + 1);

		// Texture coordinates
		auto uvbase = (t2q * v4Origin).xy;
		auto du     = (t2q * v4::XAxis()).xy;
		auto dv     = (t2q * v4::YAxis()).xy;

		// Create the vertices
		for (int h = 0, hend = divisions.y+2; h != hend; ++h, uvbase += dv)
		{
			auto uv = uvbase;
			auto vert = origin + float(h) * step_y;
			for (int w = 0, wend = divisions.x+2; w != wend; ++w, vert += step_x, uv += du)
				vout(vert, colour, norm, uv);
		}

		// Create the faces
		auto verts_per_row = divisions.x + 2;
		for (int h = 0, hend = divisions.y+1; h != hend; ++h)
		{
			auto row = h * verts_per_row;
			for (int w = 0, wend = divisions.x+1; w != wend; ++w)
			{
				auto col = row + w;
				iout(col);
				iout(col + 1);
				iout(col + verts_per_row);

				iout(col + verts_per_row);
				iout(col + 1);
				iout(col + 1 + verts_per_row);
			}
		}

		Props props;
		props.m_geom = EGeom::Vert|EGeom::Colr|EGeom::Norm|EGeom::Tex0;
		props.m_bbox = BBox::Make(origin, Abs(origin + quad_w + quad_h));
		props.m_has_alpha = HasAlpha(colour);
		return props;
	}

	// Create a simple quad, with a normal along 'axis_id', with a texture mapped over the whole surface
	// 'anchor' is the origin of the quad, (0,0) = centre, (-1,-1) = left,bottom, (+1,+1) = right,top, etc
	// 'width' is the length of the quad_w axis
	// 'height' is the length of the quad_h axis
	// 'divisions' is the number of times to divide the width/height of the quad. Note: num_verts_across = divisions.x + 2
	// 'colour' is a colour for the whole quad
	// 't2q' is a transform to apply to the standard texture coordinates 0,0 -> 1,1
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Quad(AxisId axis_id, v2 const& anchor, float width, float height, iv2 const& divisions, Colour32 colour, m4x4 const& t2q, VOut vout, IOut iout)
	{
		// X => Y = width, Z = Height
		// Y => Z = width, X = Height
		// Z => X = width, Y = Height
		v4 quad_w, quad_h;
		switch (axis_id) {
		case AxisId::PosX: quad_w = +width * v4::YAxis(); quad_h = +height * v4::ZAxis(); break;
		case AxisId::PosY: quad_w = +width * v4::ZAxis(); quad_h = +height * v4::XAxis(); break;
		case AxisId::PosZ: quad_w = +width * v4::XAxis(); quad_h = +height * v4::YAxis(); break;
		case AxisId::NegX: quad_w = -width * v4::YAxis(); quad_h = -height * v4::ZAxis(); break;
		case AxisId::NegY: quad_w = -width * v4::ZAxis(); quad_h = -height * v4::XAxis(); break;
		case AxisId::NegZ: quad_w = -width * v4::XAxis(); quad_h = -height * v4::YAxis(); break;
		}
		return Quad(anchor, quad_w, quad_h, divisions, colour, t2q, vout, iout);
	}

	// Create a quad centred on an arbitrary position with a normal in the given direction.
	// 'centre' is the mid-point of the quad
	// 'forward' is the normal direction of the quad (not necessarily normalised)
	// 'top' is the up direction of the quad. Can be zero (defaults to -ZAxis, then -XAxis), doesn't need to be orthogonal to 'forward'
	// 't2q' is a transform to apply to the standard texture coordinates 0,0 -> 1,1
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Quad(v4 const& centre, v4 const& forward, v4 const& top, float width, float height, iv2 const& divisions, Colour32 colour, m4x4 const& t2q, VOut vout, IOut iout)
	{
		auto fwd = forward != v4Zero ? forward : v4::YAxis();
		auto up = top != v4Zero ? top : -v4::ZAxis();
		if (Parallel(up, fwd))
			up = -v4::XAxis();

		auto quad_w = width  * Normalise(Cross(up, fwd));
		auto quad_h = height * Normalise(Cross(fwd, quad_w));
		auto origin = centre - 0.5f * quad_w - 0.5f * quad_h;
		return Quad(origin, quad_w, quad_h, divisions, colour, t2q, vout, iout);
	}

	// Generate a strip of quads centred on a line of verts.
	// 'num_quads' is the number quads in the strip (num_quads == num_verts - 1)
	// 'verts' is the input array of line verts
	// 'width' is the transverse width of the quad strip (not half width)
	// 'num_normals' should be either 0, 1, num_quads+1 representing; no norm, 1 norm for all, 1 norm per vertex pair
	// 'normals' is the normal of the first vertex. After that, normals on same side are used
	// 'num_colours' should be either 0, 1, num_quads+1 representing; no colour, 1 colour for all, 1 colour per vertex pair
	// 'v_out' is an output iterator to receive the [vert,colour,norm,tex] data
	// 'i_out' is an output iterator to receive the index data
	template <typename TVertCIter, typename TNormCIter, VertOutputFn VOut, IndexOutputFn IOut>
	Props QuadStrip(int num_quads, TVertCIter verts, float width, int num_normals, TNormCIter normals, std::span<Colour32 const> colours, VOut vout, IOut iout)
	{
		Props props;
		props.m_geom = EGeom::Vert | (isize(colours) ? EGeom::Colr : EGeom::None) | EGeom::Norm | EGeom::Tex0;

		if (num_quads < 1) return Props();
		auto num_verts = num_quads + 1;

		// Colour iterator wrapper
		auto col = CreateLerpRepeater(colours, num_verts, Colour32White);
		auto cc = [&](Colour32 c) { props.m_has_alpha |= HasAlpha(c); return c; };

		// Normal iterator wrapper
		auto norm = CreateLerpRepeater(normals, num_normals, num_verts, v4::ZAxis());

		// Bounding box
		auto lwr = +v4Max;
		auto upr = -v4Max;
		auto bb = [&](v4 v) { lwr = Min(lwr,v); upr = Max(upr,v); return v; };

		// Texture coords (note: 1D texture)
		auto t00 = v2(0.0f, 0.01f);
		auto t10 = v2(1.0f, 0.01f);

		int index = 0;
		auto hwidth = width * 0.5f;
		v4       v0, v1 = *verts++, v2 = *verts++;
		v4       n0, n1 = *norm++ , n2 = *norm++ ;
		Colour32 c0, c1 = *col++  , c2 = *col++  ;

		// Create the first pair of verts
		auto bi = Normalise(Cross3(n1, v2 - v1), Perpendicular(n1));
		vout(bb(v1 + bi*hwidth), cc(c1), n1, t00); iout(index++);
		vout(bb(v1 - bi*hwidth), cc(c1), n1, t10); iout(index++);

		for (int i = 0; i != num_quads - 1; ++i)
		{
			v0 = v1; v1 = v2; v2 = *verts++;
			n0 = n1; n1 = n2; n2 = *norm++;
			c0 = c1; c1 = c2; c2 = *col++;

			auto d0 = v1 - v0;
			auto d1 = v2 - v1;
			auto b0 = Normalise(Cross3(n1, d0), bi);
			auto b1 = Normalise(Cross3(n1, d1), bi);
			bi = Normalise(b0 + b1, bi); // The bisector at v1
			// Note: bi always points to the left of d0 and d1
			
			// Find the distance, t, along d0 to the inside corner vert
			// let t = 1 - u, where u is the distance back along d0 from v1
			// x = dot(d0,bi)/|d0| = the length of bi along d0
			// y = dot(b0,bi)      = the perpendicular distance of bi from d0
			// let w = x/|d0| = dot(d0,bi)/|d0|² => x = w*|d0|
			// x/y = |d0|/Y = similar triangles
			//   => Y = |d0|*y/x = y/w = |d0|²*dot(b0,bi)/dot(d0,bi)
			// for u >= 1; Y <= hwidth
			//   => y/w <= hwidth
			//   => y <= hwidth*w
			// u = 1 - t = X/|d0| = parametric value back from v1 where the perpendicular distance is hwidth
			//   X/hwidth = x/y => X = hwidth*w*|d0|/y
			//   => X/|d0| = hwidth*w/y
			//   => t = 1 - hwidth*w/y
			auto d0_sq = LengthSq(d0);
			auto d1_sq = LengthSq(d1);
			auto w0 = Div(Abs(Dot3(d0,bi)), d0_sq, maths::tinyf);
			auto w1 = Div(Abs(Dot3(d1,bi)), d1_sq, maths::tinyf);
			auto y = Dot3(b0,bi); // == Dot3(b1,bi);
			auto u0 = y <= hwidth*w0 ? 1.0f : hwidth*w0/y; // Cannot be a div/0 because w0,w1 are positive-semi-definite.
			auto u1 = y <= hwidth*w1 ? 1.0f : hwidth*w1/y;
			if (Dot3(d0,bi) >= 0.0f) // line turns to the right
			{
				auto inner = u0*pr::Sqrt(d0_sq) > u1*pr::Sqrt(d1_sq) // Pick the maximum distance from v1
					? v1 - u0*d0 - b0*hwidth
					: v1 + u1*d1 - b1*hwidth;

				// Finish the previous quad
				vout(bb(v1 + b0*hwidth), cc(c1), n1, t00); iout(index++);
				vout(bb(inner)         , cc(c1), n1, t10); iout(index++);

				// Start the next quad
				vout(bb(v1 + b1*hwidth), cc(c1), n1, t00); iout(index++);
				vout(bb(inner)         , cc(c1), n1, t10); iout(index++);
			}
			else // line turns to the left
			{
				auto inner = u0*pr::Sqrt(d0_sq) > u1*pr::Sqrt(d1_sq) // Pick the maximum distance from v1
					? v1 - u0*d0 + b0*hwidth
					: v1 + u1*d1 + b1*hwidth;

				// Finish the previous quad
				vout(bb(inner)         , cc(c1), n1, t10); iout(index++);
				vout(bb(v1 - b0*hwidth), cc(c1), n1, t00); iout(index++);
					
				// Start the next quad
				vout(bb(inner)         , cc(c1), n1, t10); iout(index++);
				vout(bb(v1 - b1*hwidth), cc(c1), n1, t00); iout(index++);
			}
		}

		// Finish the previous quad
		bi = Normalise(Cross3(n2, v2 - v1), Perpendicular(n2));
		vout(bb(v2 + bi*hwidth), cc(c2), n2, t00); iout(index++);
		vout(bb(v2 - bi*hwidth), cc(c2), n2, t10); iout(index++);

		props.m_bbox = BBox::Make(lwr, upr);
		return props;
	}

	// Generate a XxY patch using triangle strips.
	// The returned patch maps to a unit quad. Callers can then scale/deform as needed.
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props QuadPatch(int dimx, int dimy, VOut vout, IOut iout)
	{
		// e.g. 5x3 quad:
		//   10-11-12-13-14
		//   |\ |\ |\ |\ |
		//   | \| \| \| \|
		//   5--6--7--8--9
		//   |\ |\ |\ |\ |
		//   | \| \| \| \|
		//   0--1--2--3--4

		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | EGeom::Norm | EGeom::Tex0;
		props.m_bbox = BBox(v4(0.5f, 0.5f, 0, 1), v4(0.5f, 0.5f, 0, 0));
		
		// Make a grid of verts
		for (int j = 0; j != dimy; ++j)
		{
			auto y = static_cast<float>(j) / dimy;
			for (int i = 0; i != dimx; ++i)
			{
				auto x = static_cast<float>(i) / dimx;
				vout(v4(x,y,0,1), Colour32White, v4::ZAxis(), v2(x,y));
			}
		}

		// Generate the indices for the triangle strip
		int idx = 0;
		for (int j = 0; j != dimy; ++j)
		{
			iout(idx); // Row start degenerate
			for (int i = 0; i != dimx; ++i)
			{
				iout(idx);
				iout(idx + dimx);
				++idx;
			}
			iout(idx + dimx - 1); // Row end degenerate
		}

		return props;
	}

	// Generate a hex patch using triangle strips.
	// The radius of the patch is 1.0 with the centre at (0,0,0). Callers can then scale/deform as needed.
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props HexPatch(int rings, VOut vout, IOut iout)
	{
		// e.g. 3 rings                TriStrip Faces:
		//        m---n---o---p       | Ring 0: | Ring 1:     | Ring 2:
		//       / \ / \ / \ / \      | 1, 0,   | 7, 1, 8, 2, | j, 7, k, 8, l, 9,
		//      l---9---a---b---q     | 2, 0,   | 9, 2, a, 3, | m, 9, n, a, o, b,
		//     / \ / \ / \ / \ / \    | 3, 0,   | b, 3, c, 4, | p, b, q, c, r, d,
		//    k---8---2---3---c---r   | 4, 0,   | d, 4, e, 5, | s, d, t, e, u, f,
		//   / \ / \ / \ / \ / \ / \  | 5, 0,   | f, 5, g, 6, | v, f, w, g, x, h,
		//  j---7---1---0---4---d---s | 6, 0,   | h, 6, i, 1, | y, h ,z, i, A, 7,
		//   \ / \ / \ / \ / \ / \ /  | 1, 1,   | 7, 7        | j, j
		//    A---i---6---5---e---t   
		//     \ / \ / \ / \ / \ /    
		//      z---h---g---f---u     
		//       \ / \ / \ / \ /      
		//        y---x---w---v       

		Props props;
		props.m_geom = EGeom::Vert | EGeom::Colr | EGeom::Norm | EGeom::Tex0;
		props.m_bbox = BBox(v4Origin, v4(1, 1, 0, 0));

		auto const dx = maths::cos_60f / rings;
		auto const dy = maths::sin_60f / rings;

		// Make a grid of verts
		float x = 0.0f, y = 0.0f;
		vout(v4(x,y,0,1), Colour32White, v4::ZAxis(), v2(x,y)); // centre vert
		for (int ring = 1; ring <= rings; ++ring)
		{
			x = -2.0f * ring * dx; y = 0.0f;

			// Sextant 0 = (1,0,2)
			for (int i = 0; i != ring; ++i, x += dx, y += dy)
				vout(v4(x,y,0,1), Colour32White, v4::ZAxis(), v2(x,y));

			// Sextant 1 = (2,0,3)
			for (int i = 0; i != ring; ++i, x += 2*dx)
				vout(v4(x,y,0,1), Colour32White, v4::ZAxis(), v2(x,y));

			// Sextant 2 = (3,0,4)
			for (int i = 0; i != ring; ++i, x += dx, y -= dy)
				vout(v4(x,y,0,1), Colour32White, v4::ZAxis(), v2(x,y));

			// Sextant 3 = (4,0,5)
			for (int i = 0; i != ring; ++i, x -= dx, y -= dy)
				vout(v4(x,y,0,1), Colour32White, v4::ZAxis(), v2(x,y));

			// Sextant 4 = (5,0,6)
			for (int i = 0; i != ring; ++i, x -= 2*dx)
				vout(v4(x,y,0,1), Colour32White, v4::ZAxis(), v2(x,y));

			// Sextant 5 = (6,0,1)
			for (int i = 0; i != ring; ++i, x -= dx, y += dy)
				vout(v4(x,y,0,1), Colour32White, v4::ZAxis(), v2(x,y));
		}

		// Generate the indices for the triangle strip
		int vidx0 = 0, vidx1 = 1;
		for (int ring = 0; ring != rings; ++ring)
		{
			auto more = 6 * (ring + 1) - 1;
			for (int s = 0; s != 6; ++s)
			{
				for (int i = 0, iend = ring + 1; i != iend; ++i, --more)
				{
					iout(vidx1 + s * (ring + 1) + i);
					iout(vidx0 + (s * ring + i) * int(more != 0));
				}
			}
			iout(vidx1);
			iout(vidx1);
			vidx0 = vidx1;
			vidx1 += 6 * (ring + 1);
		}

		return props;
	}
}
