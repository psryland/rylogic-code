//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#include "src/forward.h"
#include "src/world/ocean/ocean.h"
#include "src/world/ocean/gerstner_wave.h"
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
		static_assert((sizeof(CBufOcean) % 16) == 0);
		static_assert(sizeof(CBufOcean) == sizeof(m_cbuf), "CBufOcean exceeds m_cbuf storage");

		// Compile the shader
		auto compiler = ShaderCompiler{}
			.Source(resource::Read<char>(L"OCEAN_HLSL", L"TEXT"))
			.Includes({ new rdr12::ResourceIncludeHandler, true })
			.Define(L"SHADER_BUILD")
			.Optimise(true);

		m_vs_bytecode = compiler.ShaderModel(L"vs_6_0").EntryPoint(L"VSOcean").Compile();
		m_ps_bytecode = compiler.ShaderModel(L"ps_6_0").EntryPoint(L"PSOcean").Compile();
		m_code.VS = { m_vs_bytecode };
		m_code.PS = { m_ps_bytecode };

		// Initialise default PBR parameters
		auto& cbuf = storage_cast<CBufOcean>(m_cbuf);
		cbuf = CBufOcean{
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
	void OceanShader::SetupElement(ID3D12GraphicsCommandList* cmd_list, rdr12::GpuUploadBuffer& upload, rdr12::Scene const&, rdr12::DrawListElement const* dle)
	{
		if (dle == nullptr)
			return;

		// Upload the ocean constant buffer and bind to root parameter CBufScreenSpace (b3).
		// The ocean shader reuses this slot since it doesn't need screen-space geometry params.
		auto& cbuf = storage_cast<CBufOcean>(m_cbuf);
		auto gpu_address = upload.Add(cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
		cmd_list->SetGraphicsRootConstantBufferView(static_cast<UINT>(ERootParam::CBufOcean), gpu_address);
	}

	// Update the constant buffer data for this frame
	void OceanShader::SetupFrame(std::span<GerstnerWave const> waves, v4 camera_world_pos, float time, float inner_radius, float outer_radius, int num_rings, float min_ring_spacing, bool has_env_map, v4 sun_direction, v4 sun_colour)
	{
		auto& cbuf = storage_cast<CBufOcean>(m_cbuf);
		
		cbuf.wave_count = std::min(static_cast<int>(waves.size()), MaxOceanWaves);
		for (int i = 0; i != cbuf.wave_count; ++i)
		{
			cbuf.wave_dirs[i] = waves[i].m_direction;
			cbuf.wave_params[i] = v4(
				waves[i].m_amplitude,
				waves[i].m_wavelength,
				waves[i].m_speed,
				waves[i].m_steepness);
		}

		// Zero remaining wave slots
		for (int i = cbuf.wave_count; i != MaxOceanWaves; ++i)
		{
			cbuf.wave_dirs[i] = v4::Zero();
			cbuf.wave_params[i] = v4::Zero();
		}

		cbuf.camera_pos_time = v4(camera_world_pos.x, camera_world_pos.y, camera_world_pos.z, time);
		cbuf.mesh_config = v4(inner_radius, outer_radius, static_cast<float>(num_rings), min_ring_spacing);
		cbuf.has_env_map = has_env_map ? 1 : 0;
		cbuf.sun_direction = sun_direction;
		cbuf.sun_colour = sun_colour;
	}
}
