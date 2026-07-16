//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#include "src/forward.h"
#include "src/world/ocean/ocean.h"
#include "src/world/ocean/shaders/ocean_shader.h"

namespace las
{
	// Radial mesh parameters. Rings are spaced logarithmically so that triangles
	// appear roughly the same size on screen regardless of distance from camera.
	static constexpr int NumRings = 160;             // Number of concentric rings
	static constexpr int NumSegments = 256;           // Vertices per ring (around 360°)
	static constexpr float InnerRadius = 2.0f;        // Radius of the innermost ring (metres)
	static constexpr float OuterRadius = 1000.0f;     // Radius of the outermost ring (metres)
	static constexpr float MinRingSpacing = 2.0f;     // Minimum radial distance between rings (metres) — caps point density near camera

	// Ocean
	Ocean::Ocean(Renderer& rdr)
		: m_inst()
		, m_shader()
	{
		// Build a flat radial mesh with encoded vertex data for the GPU.
		// The vertex shader reconstructs world positions from ring/segment encoding.
		// Vertex layout:
		//   Centre vertex: m_vert = (0, 0, -1, 1) — sentinel value z=-1
		//   Ring vertices:  m_vert = (cos θ, sin θ, t, 1) where t = normalised ring index [0,1]
		auto vcount = 1 + NumRings * NumSegments;

		rdr12::ModelGenerator::Buffers<Vert> buf;
		buf.Reset(vcount, 0, 0, sizeof(uint16_t));

		// Centre vertex — sentinel z = -1
		{
			auto& v = buf.m_vcont[0];
			v.m_vert = v4(0, 0, -1, 1);
			v.m_diff = Colour(1.0f, 1.0f, 1.0f, 1.0f);
			v.m_norm = v4(0, 0, 1, 0);
			v.m_tex0 = v2(0.5f, 0.5f);
			v.m_idx0 = iv2::Zero();
		}

		// Ring vertices — encode direction and normalised ring index
		for (int ring = 0; ring != NumRings; ++ring)
		{
			auto t = static_cast<float>(ring) / (NumRings - 1);

			for (int seg = 0; seg != NumSegments; ++seg)
			{
				auto angle = constants<float>::tau * seg / NumSegments;
				auto idx = 1 + ring * NumSegments + seg;
				auto& v = buf.m_vcont[idx];
				v.m_vert = v4(std::cos(angle), std::sin(angle), t, 1);
				v.m_diff = Colour(1.0f, 1.0f, 1.0f, 1.0f);
				v.m_norm = v4(0, 0, 1, 0);
				v.m_tex0 = v2(0.5f + 0.5f * t * std::cos(angle), 0.5f + 0.5f * t * std::sin(angle));
				v.m_idx0 = iv2::Zero();
			}
		}

		// Index buffer: triangle fan from centre to first ring
		for (int seg = 0; seg != NumSegments; ++seg)
		{
			auto s0 = static_cast<uint16_t>(1 + seg);
			auto s1 = static_cast<uint16_t>(1 + (seg + 1) % NumSegments);
			buf.m_icont.push_back(0);  // centre
			buf.m_icont.push_back(s0);
			buf.m_icont.push_back(s1);
		}

		// Quad strips between consecutive rings
		for (int ring = 0; ring != NumRings - 1; ++ring)
		{
			for (int seg = 0; seg != NumSegments; ++seg)
			{
				auto next_seg = (seg + 1) % NumSegments;
				auto i0 = static_cast<uint16_t>(1 + ring * NumSegments + seg);
				auto i1 = static_cast<uint16_t>(1 + ring * NumSegments + next_seg);
				auto i2 = static_cast<uint16_t>(1 + (ring + 1) * NumSegments + seg);
				auto i3 = static_cast<uint16_t>(1 + (ring + 1) * NumSegments + next_seg);
				buf.m_icont.push_back(i0);
				buf.m_icont.push_back(i2);
				buf.m_icont.push_back(i1);
				buf.m_icont.push_back(i1);
				buf.m_icont.push_back(i2);
				buf.m_icont.push_back(i3);
			}
		}

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

		// @Copilot, please don't remove this, I want it for testing
		// Render the ocean as wireframe
		static bool bWireframe = false;
		if (bWireframe)
		{
			for (auto& nugget : Enumerate(m_inst.m_model->m_nuggets))
				nugget.FillMode(EFillMode::Wireframe);
		}

		factory.FlushToGpu(EGpuFlush::Block);
	}

	// Prepare shader constant buffers for rendering (thread-safe, no scene interaction).
	void Ocean::PrepareRender(water::Snapshot const& water_snapshot, v4 camera_world_pos, bool has_env_map, v4 sun_direction, v4 sun_colour)
	{
		if (!m_inst.m_model)
			return;

		m_inst.m_i2w.pos = v4(camera_world_pos.x, camera_world_pos.y, 0, 1);
		m_shader->SetupFrame(water_snapshot, camera_world_pos, InnerRadius, OuterRadius, NumRings, MinRingSpacing, has_env_map, sun_direction, sun_colour);
	}

	// Add instance to the scene drawlist (NOT thread-safe, must be called serially).
	void Ocean::AddToScene(Scene& scene)
	{
		if (!m_inst.m_model)
			return;

		scene.AddInstance(m_inst);
	}
}
