//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#include "src/forward.h"
#include "src/world/terrain/terrain.h"
#include "src/world/terrain/shaders/terrain_shader.h"

namespace las
{
	using namespace cdlod;

	// Terrain
	Terrain::Terrain(Renderer& rdr)
		: m_grid_mesh()
		, m_shader()
		, m_lod_selection()
		, m_instances()
	{
		// Build a flat NxN grid mesh for CDLOD terrain patches with skirt geometry.
		// Vertices at (i/GridN, j/GridN, 0, 1) form a unit-square grid.
		// Skirt vertices duplicate edge positions with z=1 as a flag for the VS to drop them.
		// All patches share this mesh, positioned via per-instance i2w transforms.
		rdr12::ModelGenerator::Buffers<Vert> buf;
		buf.Reset(TotalVertCount, 0, 0, sizeof(uint16_t));

		// Surface vertices
		for (auto iy = 0; iy <= GridN; ++iy)
		{
			for (auto ix = 0; ix <= GridN; ++ix)
			{
				auto idx = iy * GridVerts + ix;
				auto& v = buf.m_vcont[idx];
				v.m_vert = v4(
					static_cast<float>(ix) / GridN,
					static_cast<float>(iy) / GridN,
					0, 1);
				v.m_diff = Colour(0.23f, 0.50f, 0.12f, 1.0f);
				v.m_norm = v4(0, 0, 1, 0);
				v.m_tex0 = v2(
					static_cast<float>(ix) / GridN,
					static_cast<float>(iy) / GridN);
				v.m_idx0 = iv2::Zero();
			}
		}

		// Skirt vertices: duplicate edge positions with z=1 (skirt flag for VS)
		auto skirt_base = GridVertCount;
		auto skirt_bottom = skirt_base;                  // Bottom edge (y=0): indices [skirt_base, +GridVerts)
		auto skirt_top    = skirt_base + GridVerts;       // Top edge (y=GridN)
		auto skirt_left   = skirt_base + 2 * GridVerts;   // Left edge (x=0)
		auto skirt_right  = skirt_base + 3 * GridVerts;   // Right edge (x=GridN)

		for (auto ix = 0; ix <= GridN; ++ix)
		{
			// Bottom edge skirt
			auto& vb = buf.m_vcont[skirt_bottom + ix];
			vb = buf.m_vcont[ix]; // Copy from edge vertex (iy=0, ix)
			vb.m_vert.z = 1.0f;

			// Top edge skirt
			auto& vt = buf.m_vcont[skirt_top + ix];
			vt = buf.m_vcont[GridN * GridVerts + ix]; // Copy from edge vertex (iy=GridN, ix)
			vt.m_vert.z = 1.0f;
		}
		for (auto iy = 0; iy <= GridN; ++iy)
		{
			// Left edge skirt
			auto& vl = buf.m_vcont[skirt_left + iy];
			vl = buf.m_vcont[iy * GridVerts]; // Copy from edge vertex (ix=0, iy)
			vl.m_vert.z = 1.0f;

			// Right edge skirt
			auto& vr = buf.m_vcont[skirt_right + iy];
			vr = buf.m_vcont[iy * GridVerts + GridN]; // Copy from edge vertex (ix=GridN, iy)
			vr.m_vert.z = 1.0f;
		}

		// Surface triangle list
		for (auto iy = 0; iy < GridN; ++iy)
		{
			for (auto ix = 0; ix < GridN; ++ix)
			{
				auto i00 = static_cast<uint16_t>(iy * GridVerts + ix);
				auto i10 = static_cast<uint16_t>(iy * GridVerts + ix + 1);
				auto i01 = static_cast<uint16_t>((iy + 1) * GridVerts + ix);
				auto i11 = static_cast<uint16_t>((iy + 1) * GridVerts + ix + 1);
				// CW winding for positive-Z face normal (matching D3D12 default front-face)
				buf.m_icont.push_back(i00);
				buf.m_icont.push_back(i10);
				buf.m_icont.push_back(i01);
				buf.m_icont.push_back(i10);
				buf.m_icont.push_back(i11);
				buf.m_icont.push_back(i01);
			}
		}

		// Skirt triangle strips along each edge.
		// Winding is chosen so the skirt faces outward (away from patch centre).
		auto push = [&](int a, int b, int c)
		{
			buf.m_icont.push_back(static_cast<uint16_t>(a));
			buf.m_icont.push_back(static_cast<uint16_t>(b));
			buf.m_icont.push_back(static_cast<uint16_t>(c));
		};
		for (auto i = 0; i < GridN; ++i)
		{
			// Bottom edge: normal faces -Y
			auto eb0 = i;                         // edge vertex (ix=i, iy=0)
			auto eb1 = i + 1;                     // edge vertex (ix=i+1, iy=0)
			auto sb0 = skirt_bottom + i;
			auto sb1 = skirt_bottom + i + 1;
			push(eb0, sb0, eb1);
			push(eb1, sb0, sb1);

			// Top edge: normal faces +Y
			auto et0 = GridN * GridVerts + i;      // edge vertex (ix=i, iy=GridN)
			auto et1 = GridN * GridVerts + i + 1;
			auto st0 = skirt_top + i;
			auto st1 = skirt_top + i + 1;
			push(et0, et1, st0);
			push(et1, st1, st0);

			// Left edge: normal faces -X
			auto el0 = i * GridVerts;              // edge vertex (ix=0, iy=i)
			auto el1 = (i + 1) * GridVerts;
			auto sl0 = skirt_left + i;
			auto sl1 = skirt_left + i + 1;
			push(el0, el1, sl0);
			push(el1, sl1, sl0);

			// Right edge: normal faces +X
			auto er0 = i * GridVerts + GridN;      // edge vertex (ix=GridN, iy=i)
			auto er1 = (i + 1) * GridVerts + GridN;
			auto sr0 = skirt_right + i;
			auto sr1 = skirt_right + i + 1;
			push(er0, sr0, er1);
			push(er1, sr0, sr1);
		}

		// Bbox covers the unit grid with generous height range for frustum culling.
		// Height range must cover the full noise amplitude to avoid culling tall peaks.
		// When transformed by the instance i2w (scale + translate), this gives world-space bounds.
		buf.m_bbox = BBox(v4(0.5f, 0.5f, 0, 1), v4(0.5f, 0.5f, 1000, 0));

		// Create the terrain shader
		auto shdr = Shader::Create<TerrainShader>(rdr);
		m_shader = shdr.get();

		// Configure the nugget with the custom terrain shader
		buf.m_ncont.push_back(
			NuggetDesc(ETopo::TriList, EGeom::Vert | EGeom::Colr | EGeom::Norm)
				.mat([&](MaterialSimple& m) {
					m.use_shader_overlay(ERenderStep::RenderForward, shdr);
				})
			);

		auto terrain_colour = Colour32Green;
		auto opts = ModelGenerator::CreateOptions().colours({ &terrain_colour, 1 });

		ResourceFactory factory(rdr);
		ModelGenerator::Cache cache{buf};
		m_grid_mesh = ModelGenerator::Create<Vert>(factory, cache, &opts);

		factory.FlushToGpu(EGpuFlush::Block);

		// Pre-allocate instance pool
		m_instances.resize(MaxPatches);
		for (auto& inst : m_instances)
		{
			inst.m_model = m_grid_mesh;
			inst.m_i2w = m4x4::Identity();
		}
	}

	int Terrain::PatchCount() const
	{
		return std::min(static_cast<int>(m_lod_selection.m_patches.size()), MaxPatches);
	}

	// Prepare shader constant buffers for rendering (thread-safe, no scene interaction).
	void Terrain::PrepareRender(v4 camera_world_pos, v4 sun_direction, v4 sun_colour)
	{
		if (!m_grid_mesh)
			return;

		m_lod_selection.Select(camera_world_pos, MaxDrawDist);

		auto patch_count = PatchCount();
		for (auto i = 0; i < patch_count; ++i)
		{
			auto& patch = m_lod_selection.m_patches[i];
			auto& inst = m_instances[i];
			inst.m_i2w.x = v4(patch.size, 0, 0, 0);
			inst.m_i2w.y = v4(0, patch.size, 0, 0);
			inst.m_i2w.z = v4(0, 0, 1, 0);
			inst.m_i2w.pos = v4(patch.origin_x, patch.origin_y, 0, 1);
		}

		m_shader->SetupFrame(camera_world_pos, sun_direction, sun_colour);
	}

	// Add instances to the scene drawlist (NOT thread-safe, must be called serially).
	void Terrain::AddToScene(Scene& scene)
	{
		if (!m_grid_mesh)
			return;

		auto patch_count = PatchCount();
		for (auto i = 0; i < patch_count; ++i)
			scene.AddInstance(m_instances[i]);
	}
}
