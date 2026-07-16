//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#include "src/forward.h"
#include "src/world/ocean/ocean.h"
#include "src/world/ocean/shaders/ocean_shader.h"
#include "src/world/ocean/shaders/ocean_cbuf.hlsli"

namespace las
{
	enum class ERootParam : int
	{
		CBufScene = 0,  // Scene constant buffer (b0)
		CBufObject = 1, // Object constant buffer (b1)
		CBufFrame = 2,  // Frame constant buffer (b2)
		CBufOcean = 3,  // Reused by ocean shader for ocean params (b3)
	};

	OceanShader::OceanShader(Renderer& rdr)
		: Shader(rdr)
		, m_vs_bytecode()
		, m_ps_bytecode()
		, m_cbuf()
	{
		static_assert(sizeof(water::WaterFieldElement) == 64);
		static_assert(alignof(water::WaterFieldElement) == 16);
		static_assert(offsetof(water::WaterFieldElement, info) == 0);
		static_assert(offsetof(water::WaterFieldElement, position) == 16);
		static_assert(offsetof(water::WaterFieldElement, wave) == 32);
		static_assert(offsetof(water::WaterFieldElement, timing) == 48);
		static_assert((sizeof(CBufOcean) % 16) == 0);
		static_assert(sizeof(CBufOcean) < D3D12_REQ_CONSTANT_BUFFER_ELEMENT_COUNT * 16);
		auto resolver = ::pr::compute::shader_cache::ResourceSourceResolver{};

		// Compile the shader
		auto compiler = ShaderCompiler{}
			.Source("src/world/ocean/shaders/ocean.hlsl", resolver)
			.HlslVersion(EHlslVersion::Hlsl2021)
			.Define(L"SHADER_BUILD")
			.Optimise(true);

		m_vs_bytecode = compiler.ShaderModel(L"vs_6_0").EntryPoint(L"VSOcean").Compile();
		m_ps_bytecode = compiler.ShaderModel(L"ps_6_0").EntryPoint(L"PSOcean").Compile();
		m_code.VS = { m_vs_bytecode };
		m_code.PS = { m_ps_bytecode };

		// Initialise default PBR parameters
		m_cbuf = CBufOcean{
			.fresnel_f0 = 0.02f,                                    // Water at normal incidence
			.specular_power = 256.0f,                               // Sharp sun glint
			.sss_strength = 0.5f,                                   // Moderate subsurface scattering
			.colour_shallow = v4(0.10f, 0.60f, 0.55f, 1.0f),        // Turquoise
			.colour_deep = v4(0.02f, 0.08f, 0.20f, 1.0f),           // Dark ocean blue
			.colour_foam = v4(0.95f, 0.97f, 1.00f, 1.0f),           // Near-white foam
			.sun_direction = Normalise(v4(0.5f, 0.3f, 0.8f, 0.0f)), // Elevated sun, slightly NE
			.sun_colour = v4(1.0f, 0.95f, 0.85f, 1.0f),             // Warm sunlight
			.has_env_map = 0,
			.water_transparency = 0.7f,                              // Moderately clear tropical water
		};
	}

	// Called per-nugget during forward rendering to bind the ocean constant buffer
	void OceanShader::SetupElement(ID3D12GraphicsCommandList* cmd_list, ::pr::compute::GpuUploadBuffer& upload, rdr12::Scene const&, rdr12::DrawListElement const* dle)
	{
		if (dle == nullptr)
			return;

		// Upload the ocean constant buffer and bind to root parameter CBufScreenSpace (b3).
		// The ocean shader reuses this slot since it doesn't need screen-space geometry params.
		auto gpu_address = upload.Add(m_cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
		cmd_list->SetGraphicsRootConstantBufferView(static_cast<UINT>(ERootParam::CBufOcean), gpu_address);
	}

	// Copy one immutable water-field snapshot and update render-only frame parameters.
	void OceanShader::SetupFrame(water::Snapshot const& water_snapshot, v4 camera_world_pos, float outer_radius, int grid_vertex_count, float min_grid_spacing, float surface_warp_power, bool has_env_map, v4 sun_direction, v4 sun_colour)
	{
		m_cbuf.water_field_count = std::min(water_snapshot.m_element_count, water::MaxWaterFieldElementCount);
		for (int i = 0; i != m_cbuf.water_field_count; ++i)
		{
			m_cbuf.water_field[i] = water_snapshot.m_elements[i];
		}

		// Clear inactive entries so the complete snapshot remains deterministic.
		for (int i = m_cbuf.water_field_count; i != water::MaxWaterFieldElementCount; ++i)
		{
			m_cbuf.water_field[i] = water::WaterFieldElement{};
		}

		m_cbuf.camera_pos_time = v4(camera_world_pos.x, camera_world_pos.y, camera_world_pos.z, water_snapshot.m_time_s);
		m_cbuf.mesh_config = v4(outer_radius, static_cast<float>(grid_vertex_count), min_grid_spacing, surface_warp_power);
		m_cbuf.has_env_map = has_env_map ? 1 : 0;
		m_cbuf.sun_direction = sun_direction;
		m_cbuf.sun_colour = sun_colour;
	}
}
