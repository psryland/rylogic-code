//**********************************************
// P3D Graphics Tool
//  Copyright (c) Rylogic Ltd 2019
//**********************************************

#include "src/forward.h"

using namespace pr;
using namespace pr::geometry;

// Remove degenerate verts
void RemoveDegenerateVerts(p3d::Mesh& mesh_, int quantisation, float smoothing_angle, float colour_distance, float uv_distance, int verbosity)
{
	// Const mesh reference to ensure const accessors are used unless writing
	auto const& mesh = mesh_;

	// No mesh, no degenerates
	if (mesh.m_vert.size() == 0)
		return;

	if (verbosity >= 2)
		std::cout << "  Removing degenerate verts for mesh: " << mesh.m_name << std::endl;
	if (verbosity >= 3 && smoothing_angle >= 0)
		std::cout << "    Vert normals within " << smoothing_angle << " degrees are considered degenerate" << std::endl;
	if (verbosity >= 3 && colour_distance >= 0)
		std::cout << "    Vert colours within " << colour_distance << " are considered degenerate" << std::endl;
	if (verbosity >= 3 && uv_distance >= 0)
		std::cout << "    Vert texture coords within " << uv_distance << " are considered degenerate" << std::endl;

	// Quantise the verts.
	for (auto& vert : mesh_.m_vert)
		vert = Quantise(vert, 1 << quantisation);

	// A mapping from original vertex (orig) to the kept vertex from all the vertices degenerate with 'orig'
	struct VP { v4 const* orig; v4 const* kept; };
	std::vector<VP> map;
	map.resize(mesh.m_vert.size());

	// Initialise the map
	auto map_ptr = map.data();
	for (auto& vert : mesh.m_vert)
	{
		map_ptr->orig = &vert;
		map_ptr->kept = &vert;
		++map_ptr;
	}

	// Sort the mapping vector such that degenerate verts are next to each other
	pr::sort(map, [&](VP const& lhs, VP const& rhs)
	{
		auto& v0 = *lhs.orig;
		auto& v1 = *rhs.orig;
		if (v0.x != v1.x) return v0.x < v1.x;
		if (v0.y != v1.y) return v0.y < v1.y;
		if (v0.z != v1.z) return v0.z < v1.z;
		return false;
	});

	// Preserved vertex properties
	auto cos_angle_normal_diff = smoothing_angle >= 0 ? Cos(DegreesToRadians(smoothing_angle)) : 0;
	auto colour_dist_sq = colour_distance >= 0 ? Sqr(colour_distance) : 0;
	auto uv_dist_sq = uv_distance >= 0 ? Sqr(uv_distance) : 0;
	auto geom = mesh.geom();

	// The start of the vertex buffer so we can calculate indices from pointers.
	// And, the buffers of the mesh that contain values
	auto v0 = mesh.m_vert.data();

	// Set each 'kept' pointer to the first in the set of verts degenerate with 'orig'
	// Since the verts are only sorted by position, all verts need to be checked back to where the positions changes.
	auto unique_count = map.size();
	for (size_t i = 1, iend = map.size(); i != iend; ++i)
	{
		// The vert to test for degeneracy
		auto vi = *map[i].orig;
		for (auto j = i; j-- != 0;)
		{
			// A previous vertex that may be degenerate with 'vi' and, if so,
			// will already have it's 'kept' pointer pointing at the first in the set.
			auto vj = *map[j].kept;

			// If the vertex position is different, move on to the next vert
			if (vi != vj)
				break;

			// Not a degenerate if the geometry has normals and they don't match, but keep searching
			if (AllSet(geom, EGeom::Norm) && smoothing_angle >= 0)
			{
				auto ni = mesh.m_norm[s_cast<int>(map[i].orig - v0)];
				auto nj = mesh.m_norm[s_cast<int>(map[i].kept - v0)];
				if (Dot(ni, nj) <= cos_angle_normal_diff)
					continue;
			}

			// Not a degenerate if the geometry has colours and they don't match, but keep searching
			if (AllSet(geom, EGeom::Colr) && colour_distance >= 0)
			{
				auto ci = Colour(mesh.m_diff[s_cast<int>(map[i].orig - v0)]);
				auto cj = Colour(mesh.m_diff[s_cast<int>(map[j].kept - v0)]);
				if (LengthSq(ci.rgba - cj.rgba) >= colour_dist_sq)
					continue;
			}

			// Not a degenerate if the geometry has UVs and they don't match, but keep searching
			if (AllSet(geom, EGeom::Tex0) && uv_distance >= 0)
			{
				auto ti = mesh.m_tex0[s_cast<int>(map[i].orig - v0)];
				auto tj = mesh.m_tex0[s_cast<int>(map[i].kept - v0)];
				if (LengthSq(ti - tj) >= uv_dist_sq)
					continue;
			}

			// 'vi' is degenerate with 'vj'.
			// Set the 'kept' pointer for 'vi' to the same as 'vj' which will
			// be the first in the set of all verts degenerate with 'vi' and 'vj'.
			map[i].kept = map[j].kept;
			--unique_count;
			break;
		}
	}

	// Report stats
	if (verbosity >= 3)
		std::cout
			<< "    " << (map.size() - unique_count) << " degenerate verts found\n"
			<< "    " << unique_count << " verts remaining." << std::endl;

	// Returns true if 'ptr' points within 'cont'
	auto is_within = [](auto const& cont, auto const* ptr)
	{
		return ptr >= cont.data() && ptr < cont.data() + cont.size();
	};

	// Buffers to hold the unique vertices
	p3d::VCont verts(unique_count);
	p3d::CCont diffs;
	p3d::NCont norms;
	p3d::TCont tex0s;

	// Create a collection of verts without degenerates
	auto vout = verts.data();
	for (size_t i = 0, j = 0, iend = map.size(); i != iend; ++i)
	{
		// If this vert is already pointing into the new container then it's
		// degenerate with a vert that has already been copied into 'verts'
		if (is_within(verts, map[i].kept))
			continue;

		auto diff = ColourZero;
		auto norm = v4::Zero();
		auto tex0 = v2Zero;

		// 'map[i]' should be the first in the next set of degenerate verts.
		// Redirect all pointers up to the next non-degenerate vert to the new container.
		// Test all verts after 'i' until the positions differ. The preserved
		// fields mean that not all degenerate verts are contiguous in 'map'.
		auto degen_count = 1.0f;
		for (j = i + 1; j != iend; ++j)
		{
			// If the next vert has a different position, then we've reached the end of the degenerate verts
			if (*map[j].orig != *map[i].orig)
				break;

			// 'map[i]' is degenerate with 'map[j]' if their 'kept' pointers are the same.
			if (map[j].kept != map[i].kept)
				continue;

			// Set 'map[j].kept' to the location in 'verts' that will contain the vertex for this degenerate set (done after this loop)
			map[j].kept = &*vout;
			degen_count += 1.0f;

			// Perform averaging of the vertex fields that are being merged.
			if (AllSet(geom, EGeom::Colr))
				diff += Colour(mesh.m_diff[s_cast<int>(map[j].orig - v0)]);
			if (AllSet(geom, EGeom::Norm))
				norm += mesh.m_norm[s_cast<int>(map[j].orig - v0)];
			if (AllSet(geom, EGeom::Tex0))
				tex0 += mesh.m_tex0[s_cast<int>(map[j].orig - v0)];
		}

		// Assign the kept vertex of the degenerate set into the new container
		map[i].kept = &*vout;
		*vout++ = *map[i].orig;

		// Perform averaging of the vertex fields that are being merged.
		if (AllSet(geom, EGeom::Colr))
		{
			diff += Colour(mesh.m_diff[s_cast<int>(map[i].orig - v0)]);
			diffs.push_back((diff / degen_count).argb());
		}
		if (AllSet(geom, EGeom::Norm))
		{
			norm += mesh.m_norm[s_cast<int>(map[i].orig - v0)];
			norms.push_back(norm / degen_count);
		}
		if (AllSet(geom, EGeom::Tex0))
		{
			tex0 += mesh.m_tex0[s_cast<int>(map[i].orig - v0)];
			tex0s.push_back(tex0 / degen_count);
		}
	}

	// Re-sort the map back to the original order of mesh.m_vert
	pr::sort(map, [&](VP const& lhs, VP const& rhs)
	{
		return lhs.orig < rhs.orig;
	});

	// Update the mesh indices to use the non-degenerate vert indices
	for (auto& nug : mesh_.m_nugget)
	{
		auto count = isize(nug.m_vidx);
		auto iptr = nug.m_vidx.begin<int64_t>();
		for (int i = 0; i != count; ++i, ++iptr)
			*iptr = map[static_cast<size_t>(*iptr)].kept - verts.data();
	}

	// Replace the vert containers in the mesh
	mesh_.m_vert = std::move(verts);
	if (AllSet(geom, EGeom::Colr))
		mesh_.m_diff = std::move(diffs);
	if (AllSet(geom, EGeom::Norm))
		mesh_.m_norm = std::move(norms);
	if (AllSet(geom, EGeom::Tex0))
		mesh_.m_tex0 = std::move(tex0s);
}

// Remove degenerate verts. Use -1.0f to ignore normals, colours, or UVs
void RemoveDegenerateVerts(p3d::File& p3d, int quantisation, float smoothing_angle, float colour_distance, float uv_distance, int verbosity)
{
	// Apply to each mesh in the scene
	for (auto& mesh : p3d.m_scene.m_meshes)
		RemoveDegenerateVerts(mesh, quantisation, smoothing_angle, colour_distance, uv_distance, verbosity);
}
