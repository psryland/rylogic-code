//********************************
// Geometry
//  Copyright (c) Rylogic Ltd 2013
//********************************
#pragma once
#include "pr/geometry/common.h"

namespace pr::geometry
{
	// Return the model buffer requirements of a mesh
	constexpr BufSizes MeshSize(int num_verts, int num_indices)
	{
		return
		{
			num_verts,
			num_indices,
		};
	}

	// Generate a model from mesh data
	// 'verts' and 'indices' are the basic model data
	// 'colours' - the array of colours. Length can be 0, 1, or verts.size()
	// 'normals' - the array of normals. Length can be 0, 1, or verts.size()
	// 'tex_coords' - the array of texture coords. Length must be 0 or verts.size()
	// Remember you can call "GenerateNormals()" to generate normals.
	template <VertOutputFn VOut, IndexOutputFn IOut>
	Props Mesh(
		std::span<v4 const> verts,
		index_cspan indices,
		std::span<Colour32 const> colours,
		std::span<v4 const> normals,
		std::span<v2 const> tex_coords,
		VOut vout, IOut iout)
	{
		Props props;
		props.m_geom =
			EGeom::Vert |
			(!colours.empty() ? EGeom::Colr : EGeom::None) |
			(!normals.empty() ? EGeom::Norm : EGeom::None) |
			(!tex_coords.empty() ? EGeom::Tex0 : EGeom::None);

		// Colour iterator wrapper
		auto col = CreateRepeater(colours, isize(verts), Colour32White);
		auto cc = [&props](Colour32 c) { props.m_has_alpha |= HasAlpha(c); return c; };

		// Normal iterator wrapper
		auto norm = CreateRepeater(normals, isize(verts), v4::Zero());

		// UV iterator wrapper
		auto uv = CreateRepeater(tex_coords, isize(verts), v2::Zero());

		// Bounding box
		auto bb = [&props](v4 v) { Grow(props.m_bbox, v); return v; };

		// Verts
		auto vptr = verts.data();
		for (auto v = 0, vend = isize(verts); v != vend; ++v)
			vout(bb(*vptr++), cc(*col++), *norm++, *uv++);

		// Faces or edges or whatever
		auto iptr = indices.begin<int>();
		for (auto i = 0, iend = isize(indices); i != iend; ++i)
			iout(*iptr++);

		return props;
	}
}
