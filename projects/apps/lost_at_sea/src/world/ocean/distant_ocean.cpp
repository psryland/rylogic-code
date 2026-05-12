//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
#include "src/forward.h"
#include "src/world/ocean/distant_ocean.h"
#include "src/world/ocean/shaders/distant_ocean_shader.h"

namespace las
{
	using namespace cdlod;

	DistantOcean::DistantOcean(Renderer& rdr)
		: m_grid_mesh()
		, m_shader()
		, m_lod_selection()
		, m_instances()
	{
		// Build a flat NxN grid mesh (same dimensions as terrain patches)
		rdr12::ModelGenerator::Buffers<Vert> buf;
		buf.Reset(GridVertCount, 0, 0, sizeof(uint16_t));

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
				v.m_diff = Colour(0.05f, 0.15f, 0.30f, 1.0f);
				v.m_norm = v4(0, 0, 1, 0);
				v.m_tex0 = v2(
					static_cast<float>(ix) / GridN,
					static_cast<float>(iy) / GridN);
				v.m_idx0 = iv2::Zero();
			}
		}

		// Triangle list index buffer (CW winding for positive-Z face normal)
		for (auto iy = 0; iy < GridN; ++iy)
		{
			for (auto ix = 0; ix < GridN; ++ix)
			{
				auto i00 = static_cast<uint16_t>(iy * GridVerts + ix);
				auto i10 = static_cast<uint16_t>(iy * GridVerts + ix + 1);
				auto i01 = static_cast<uint16_t>((iy + 1) * GridVerts + ix);
				auto i11 = static_cast<uint16_t>((iy + 1) * GridVerts + ix + 1);
				buf.m_icont.push_back(i00);
				buf.m_icont.push_back(i10);
				buf.m_icont.push_back(i01);
				buf.m_icont.push_back(i10);
				buf.m_icont.push_back(i11);
				buf.m_icont.push_back(i01);
			}
		}

		// Tight bbox for flat ocean at z=0
		buf.m_bbox = BBox(v4(0.5f, 0.5f, 0, 1), v4(0.5f, 0.5f, 1, 0));

		auto shdr = Shader::Create<DistantOceanShader>(rdr);
		m_shader = shdr.get();

		buf.m_ncont.push_back(NuggetDesc(ETopo::TriList, EGeom::Vert | EGeom::Colr | EGeom::Norm)
			.mat([&](MaterialSimple& m) {
				m.use_shader_overlay(ERenderStep::RenderForward, shdr);
			})
		);

		auto colour = Colour32(0xFF401005);
		auto opts = ModelGenerator::CreateOptions().colours({ &colour, 1 });

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

	int DistantOcean::PatchCount() const
	{
		return std::min(static_cast<int>(m_lod_selection.m_patches.size()), MaxPatches);
	}

	void DistantOcean::PrepareRender(v4 camera_world_pos, bool has_env_map, v4 sun_direction, v4 sun_colour)
	{
		if (!m_grid_mesh)
			return;

		m_lod_selection.Select(camera_world_pos, MaxDrawDist, MinDrawDist);

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

		m_shader->SetupFrame(camera_world_pos, has_env_map, sun_direction, sun_colour);
	}

	void DistantOcean::AddToScene(Scene& scene)
	{
		if (!m_grid_mesh)
			return;

		auto patch_count = PatchCount();
		for (auto i = 0; i < patch_count; ++i)
			scene.AddInstance(m_instances[i]);
	}
}
