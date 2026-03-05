//**********************************************
// P3D Graphics Tool
//  Copyright (c) Rylogic Ltd 2019
//**********************************************

#include "src/forward.h"
#include "pr/geometry/utility.h"

using namespace pr;
using namespace pr::geometry;

// Index stride independent
template <std::integral VIdx>
void DoGenNorms(p3d::Mesh& mesh, float smoothing_angle, p3d::Nugget& nug, VIdx* iptr)
{
	// Generate the normals
	GenerateNormals(
		s_cast<int>(nug.icount()),
		iptr,
		DegreesToRadians(smoothing_angle),
		s_cast<int>(mesh.vcount()),
		[&](int idx) // getv()
		{
			return mesh.m_vert[idx];
		},
		[&](int new_idx, int orig_idx, v4 normal) // vout()
		{
			// Copy the vert at 'orig_idx' to 'new_idx' and set its normal to 'normal'
			assert(s_cast<size_t>(new_idx) <= mesh.m_vert.size());
			assert(s_cast<size_t>(new_idx) <= mesh.m_norm.size());
			if (new_idx == s_cast<int>(mesh.m_vert.size())) mesh.m_vert.push_back(mesh.m_vert[orig_idx]);
			if (new_idx == s_cast<int>(mesh.m_diff.size())) mesh.m_diff.push_back(mesh.m_diff[orig_idx]);
			if (new_idx == s_cast<int>(mesh.m_norm.size())) mesh.m_norm.push_back(mesh.m_norm[orig_idx]);
			if (new_idx == s_cast<int>(mesh.m_tex0.size())) mesh.m_tex0.push_back(mesh.m_tex0[orig_idx]);
			mesh.m_norm[new_idx] = normal;
		},
		[&](int i0, int i1, int i2) // iout()
		{
			*iptr++ = s_cast<VIdx>(i0);
			*iptr++ = s_cast<VIdx>(i1);
			*iptr++ = s_cast<VIdx>(i2);
		});
}


// Generate normals for the p3d mesh
void GenerateVertNormals(p3d::Mesh& mesh, float smoothing_angle, int verbosity)
{
	// No verts, no normals
	if (mesh.m_vert.size() == 0)
		return;

	if (verbosity >= 2)
		std::cout << "  Generating normals for mesh: " << mesh.m_name << std::endl;

	// Generate normals per nugget since the topology can change per nugget
	for (auto& nug : mesh.m_nugget)
	{
		// Can only generate normals for triangle lists
		if (nug.m_topo != ETopo::TriList)
			continue;

		// Generate the normals
		switch (nug.m_vidx.stride())
		{
			case 2: DoGenNorms(mesh, smoothing_angle, nug, nug.m_vidx.data<uint16_t>()); break;
			case 4: DoGenNorms(mesh, smoothing_angle, nug, nug.m_vidx.data<uint32_t>()); break;
			default: throw std::runtime_error("Unsupported index format");
		}
	}
}

// Generate normals for the p3d file
void GenerateVertNormals(p3d::File& p3d, float smoothing_angle, int verbosity)
{
	// Generate normals for each mesh
	for (auto& mesh : p3d.m_scene.m_meshes)
		GenerateVertNormals(mesh, smoothing_angle, verbosity);
}
