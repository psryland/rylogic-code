//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#include "src/forward.h"
#include "src/world/ocean/ocean.h"
#include "src/world/ocean/shaders/ocean_shader.h"

namespace las
{
	// The regular index lattice is mapped to a disk in the vertex shader, then radially warped to retain dense near-camera sampling without a polar singularity.
	static constexpr int GridVertexCount = 905;
	static constexpr float OuterRadius = 1000.0f;
	static constexpr float MinGridSpacing = 0.0675f;
	static constexpr float SurfaceWarpPower = 3.0f;

	// Ocean
	Ocean::Ocean(Renderer& rdr)
		: m_inst()
		, m_shader()
	{
		// The index values identify a regular Cartesian lattice. The vertex shader reconstructs and warps positions from SV_VertexID, while one placeholder vertex preserves View3D's model contract.
		auto const grid_cell_count = GridVertexCount - 1;
		auto const icount = 6 * grid_cell_count * grid_cell_count;
		rdr12::ModelGenerator::Buffers<Vert> buf;
		buf.Reset(1, icount, 0, sizeof(uint32_t));
		buf.m_vcont[0] = Vert{};
		auto iptr = buf.m_icont.begin<uint32_t>();

		// A regular grid has bounded vertex valence at the camera; no row or column converges into a centre fan.
		for (int y = 0; y != grid_cell_count; ++y)
		{
			for (int x = 0; x != grid_cell_count; ++x)
			{
				auto i0 = static_cast<uint32_t>(y * GridVertexCount + x);
				auto i1 = i0 + 1;
				auto i2 = i0 + GridVertexCount;
				auto i3 = i2 + 1;

				// Follow the square-to-disk sector diagonal so cells crossing a sector boundary do not produce a nearly collinear triangle.
				auto centred_x = 2 * x + 1 - grid_cell_count;
				auto centred_y = 2 * y + 1 - grid_cell_count;
				if (centred_x * centred_y > 0)
				{
					*iptr++ = i0;
					*iptr++ = i1;
					*iptr++ = i3;
					*iptr++ = i0;
					*iptr++ = i3;
					*iptr++ = i2;
				}
				else
				{
					*iptr++ = i0;
					*iptr++ = i1;
					*iptr++ = i2;
					*iptr++ = i1;
					*iptr++ = i3;
					*iptr++ = i2;
				}
			}
		}
		assert(iptr == buf.m_icont.end<uint32_t>());

		// Set a large bounding box since the VS will displace vertices far from their encoded positions.
		// The actual rendered extent is [-OuterRadius, +OuterRadius] in XY around the camera.
		buf.m_bbox = BBox(v4::Origin(), v4(OuterRadius, OuterRadius, 50, 0));

		// Create the ocean shader
		auto shdr = Shader::Create<OceanShader>(rdr);
		m_shader = shdr.get();

		// Configure the nugget with the custom ocean shader and alpha transparency
		buf.m_ncont.push_back(
			NuggetDesc(ETopo::TriList, EGeom::Vert | EGeom::Colr | EGeom::Norm)
				.alpha_geom()
				.flags(ENuggetFlag::ShadowCastExclude)
				.pso<EPipeState::InputLayout>(D3D12_INPUT_LAYOUT_DESC{})
				.mat([&](MaterialSimple& m) {
					m.use_shader_overlay(ERenderStep::RenderForward, shdr);
				})
			);

		auto ocean_colour = Colour32(0xFF804010);
		auto opts = ModelGenerator::CreateOptions().colours({ &ocean_colour, 1 });

		ResourceFactory factory(rdr);
		ModelGenerator::Cache cache{buf};
		m_inst.m_model = ModelGenerator::Create<Vert>(factory, cache, &opts);
		m_inst.m_i2w = m4x4::Identity(); // Instance transform: identity (the VS handles camera-relative positioning)

		factory.FlushToGpu(EGpuFlush::Block);
	}

	// Prepare shader constant buffers for rendering (thread-safe, no scene interaction).
	void Ocean::PrepareRender(water::Snapshot const& water_snapshot, v4 camera_world_pos, bool has_env_map, v4 sun_direction, v4 sun_colour)
	{
		if (!m_inst.m_model)
			return;

		m_inst.m_i2w.pos = v4(camera_world_pos.x, camera_world_pos.y, 0, 1);
		m_shader->SetupFrame(water_snapshot, camera_world_pos, OuterRadius, GridVertexCount, MinGridSpacing, SurfaceWarpPower, has_env_map, sun_direction, sun_colour);
	}

	// Add instance to the scene drawlist (NOT thread-safe, must be called serially).
	void Ocean::AddToScene(Scene& scene)
	{
		if (!m_inst.m_model)
			return;

		scene.AddInstance(m_inst);
	}

	// Return whether the near-ocean mesh is rendered as wireframe.
	bool Ocean::Wireframe() const
	{
		if (!m_inst.m_model)
			return false;

		auto const* nugget = m_inst.m_model->m_nuggets.get();
		return nugget != nullptr && nugget->FillMode() == EFillMode::Wireframe;
	}

	// Set the near-ocean mesh fill mode.
	void Ocean::Wireframe(bool enabled)
	{
		if (!m_inst.m_model)
			return;

		auto fill_mode = enabled ? EFillMode::Wireframe : EFillMode::Default;
		for (auto& nugget : Enumerate(m_inst.m_model->m_nuggets))
			nugget.FillMode(fill_mode);
	}
}
