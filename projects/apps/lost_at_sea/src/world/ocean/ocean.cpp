//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#include "src/forward.h"
#include "src/world/ocean/ocean.h"
#include "src/world/ocean/shaders/ocean_shader.h"

namespace las
{
	// Radial mesh parameters. Rings are spaced logarithmically so that triangles appear roughly the same size on screen regardless of distance from camera.
	static constexpr int NumRings = 640;
	static constexpr int NumSegments = 640;
	static constexpr float InnerRadius = 0.25f;
	static constexpr float OuterRadius = 1000.0f;
	static constexpr float MinRingSpacing = 0.25f;

	// Ocean
	Ocean::Ocean(Renderer& rdr)
		: m_inst()
		, m_shader()
	{
		// The index values identify unique radial-grid vertices. The vertex shader reconstructs all positions from SV_VertexID, while one placeholder vertex preserves View3D's model contract.
		auto icount = 3 * NumSegments + 6 * (NumRings - 1) * NumSegments;
		rdr12::ModelGenerator::Buffers<Vert> buf;
		buf.Reset(1, icount, 0, sizeof(uint32_t));
		buf.m_vcont[0] = Vert{};
		auto iptr = buf.m_icont.begin<uint32_t>();

		// Index buffer: triangle fan from centre to first ring
		for (int seg = 0; seg != NumSegments; ++seg)
		{
			auto s0 = static_cast<uint32_t>(1 + seg);
			auto s1 = static_cast<uint32_t>(1 + (seg + 1) % NumSegments);
			*iptr++ = 0;
			*iptr++ = s0;
			*iptr++ = s1;
		}

		// Quad strips between consecutive rings
		for (int ring = 0; ring != NumRings - 1; ++ring)
		{
			for (int seg = 0; seg != NumSegments; ++seg)
			{
				auto next_seg = (seg + 1) % NumSegments;
				auto i0 = static_cast<uint32_t>(1 + ring * NumSegments + seg);
				auto i1 = static_cast<uint32_t>(1 + ring * NumSegments + next_seg);
				auto i2 = static_cast<uint32_t>(1 + (ring + 1) * NumSegments + seg);
				auto i3 = static_cast<uint32_t>(1 + (ring + 1) * NumSegments + next_seg);
				*iptr++ = i0;
				*iptr++ = i2;
				*iptr++ = i1;
				*iptr++ = i1;
				*iptr++ = i2;
				*iptr++ = i3;
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
		m_shader->SetupFrame(water_snapshot, camera_world_pos, InnerRadius, OuterRadius, NumRings, NumSegments, MinRingSpacing, has_env_map, sun_direction, sun_colour);
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
