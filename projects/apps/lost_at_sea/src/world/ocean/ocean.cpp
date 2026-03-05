//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#include "src/forward.h"
#include "src/world/ocean/ocean.h"
#include "src/world/ocean/gerstner_wave.h"
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
	static constexpr float WaterDensity = 1025.0f;    // kg/m^3 (seawater)

	// Ocean
	Ocean::Ocean(Renderer& rdr)
		: m_inst()
		, m_waves()
		, m_shader()
	{
		// Initialise the ocean with a set of default wave components.
		// Scale: ~1m amplitude swell, realistic for sailing ship conditions (Beaufort 4-5).
		// Speed follows deep-water dispersion: v ≈ sqrt(g·λ/2π)
		m_waves = {
			{ Normalise(v4(1.0f, 0.3f, 0, 0)), 1.0f,  80.0f, 11.2f, 0.35f }, // Primary swell
			{ Normalise(v4(0.8f, -0.6f, 0, 0)), 0.4f,  40.0f,  7.9f, 0.30f }, // Secondary
			{ Normalise(v4(-0.3f, 1.0f, 0, 0)), 0.2f,  20.0f,  5.6f, 0.25f }, // Cross chop
			{ Normalise(v4(0.5f, 0.5f, 0, 0)), 0.08f, 10.0f,  3.9f, 0.20f }, // Small ripple
		};

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
		buf.m_ncont.push_back(NuggetDesc{ ETopo::TriList, EGeom::Vert | EGeom::Colr | EGeom::Norm }
			.use_shader_overlay(ERenderStep::RenderForward, shdr)
			.alpha_geom());

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
			for (auto& nugget : m_inst.m_model->m_nuggets)
				nugget.FillMode(EFillMode::Wireframe);
		}

		factory.FlushToGpu(EGpuFlush::Block);
	}

	// Physics queries — kept for buoyancy calculations in Phase 2

	// 
	float Ocean::HeightAt(float world_x, float world_y, float time) const
	{
		auto h = 0.0f;
		for (auto& w : m_waves)
		{
			auto k = w.WaveNumber();
			auto phase = k * (w.m_direction.x * world_x + w.m_direction.y * world_y) - w.Frequency() * time;
			h += w.m_amplitude * std::sin(phase);
		}
		return h;
	}

	v4 Ocean::DisplacedPosition(float world_x, float world_y, float time) const
	{
		auto dx = 0.0f, dy = 0.0f, dz = 0.0f;
		for (auto& w : m_waves)
		{
			auto k = w.WaveNumber();
			auto phase = k * (w.m_direction.x * world_x + w.m_direction.y * world_y) - w.Frequency() * time;
			auto c = std::cos(phase);
			auto s = std::sin(phase);
			dx -= w.m_steepness * w.m_amplitude * w.m_direction.x * c;
			dy -= w.m_steepness * w.m_amplitude * w.m_direction.y * c;
			dz += w.m_amplitude * s;
		}
		return v4(world_x + dx, world_y + dy, dz, 1.0f);
	}

	v4 Ocean::NormalAt(float world_x, float world_y, float time) const
	{
		auto nx = 0.0f, ny = 0.0f, nz = 1.0f;
		for (auto& w : m_waves)
		{
			auto k = w.WaveNumber();
			auto phase = k * (w.m_direction.x * world_x + w.m_direction.y * world_y) - w.Frequency() * time;
			auto c = std::cos(phase);
			auto s = std::sin(phase);
			nx -= w.m_direction.x * k * w.m_amplitude * c;
			ny -= w.m_direction.y * k * w.m_amplitude * c;
			nz -= w.m_steepness * k * w.m_amplitude * s;
		}
		return Normalise(v4(nx, ny, nz, 0.0f));
	}

	// Prepare shader constant buffers for rendering (thread-safe, no scene interaction).
	void Ocean::PrepareRender(v4 camera_world_pos, float time, bool has_env_map, v4 sun_direction, v4 sun_colour)
	{
		if (!m_inst.m_model)
			return;

		m_inst.m_i2w.pos = v4(camera_world_pos.x, camera_world_pos.y, 0, 1);
		m_shader->SetupFrame(m_waves, camera_world_pos, time, InnerRadius, OuterRadius, NumRings, MinRingSpacing, has_env_map, sun_direction, sun_colour);
	}

	// Add instance to the scene drawlist (NOT thread-safe, must be called serially).
	void Ocean::AddToScene(Scene& scene)
	{
		if (!m_inst.m_model)
			return;

		scene.AddInstance(m_inst);
	}
}
