//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#include "src/forward.h"
#include "src/world/ocean/ocean.h"
#include "src/world/ocean/gerstner_wave.h"
#include "src/world/ocean/shaders/ocean_shader.h"
//#include "pr/view3d-12/shaders/shader_forward.h"

namespace las
{
	namespace shaders::ocean
	{
		enum class ERootParam : int
		{
			CBufScene = 0,  // Scene constant buffer (b0)
			CBufObject = 1, // Object constant buffer (b1)
			CBufFrame = 2,  // Frame constant buffer (b2)
			CBufOcean = 3,  // Reused by ocean shader for ocean params (b3)
		};

		using namespace pr::hlsl;
		#include "src/world/ocean/shaders/ocean_cbuf.hlsli"
		static_assert((sizeof(CBufOcean) % 16) == 0);
	}

	OceanShader::OceanShader(Renderer& rdr)
		: Shader(rdr)
		, m_vs_bytecode()
		, m_ps_bytecode()
		, m_cbuf()
	{
		static_assert(sizeof(shaders::ocean::CBufOcean) == sizeof(m_cbuf), "CBufOcean exceeds m_cbuf storage");

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
		auto& cbuf = storage_cast<shaders::ocean::CBufOcean>(m_cbuf);
		cbuf = shaders::ocean::CBufOcean{
			.m_fresnel_f0 = 0.02f,                                    // Water at normal incidence
			.m_specular_power = 256.0f,                               // Sharp sun glint
			.m_sss_strength = 0.5f,                                   // Moderate subsurface scattering
			.m_colour_shallow = v4(0.10f, 0.60f, 0.55f, 1.0f),        // Turquoise
			.m_colour_deep = v4(0.02f, 0.08f, 0.20f, 1.0f),           // Dark ocean blue
			.m_colour_foam = v4(0.95f, 0.97f, 1.00f, 1.0f),           // Near-white foam
			.m_sun_direction = Normalise(v4(0.5f, 0.3f, 0.8f, 0.0f)), // Elevated sun, slightly NE
			.m_sun_colour = v4(1.0f, 0.95f, 0.85f, 1.0f),             // Warm sunlight
			.m_has_env_map = 0,
			.m_water_transparency = 0.7f,                              // Moderately clear tropical water
		};
	}

	// Called per-nugget during forward rendering to bind the ocean constant buffer
	void OceanShader::SetupElement(ID3D12GraphicsCommandList* cmd_list, rdr12::GpuUploadBuffer& upload, rdr12::Scene const&, rdr12::DrawListElement const* dle)
	{
		if (dle == nullptr)
			return;

		// Upload the ocean constant buffer and bind to root parameter CBufScreenSpace (b3).
		// The ocean shader reuses this slot since it doesn't need screen-space geometry params.
		auto& cbuf = storage_cast<shaders::ocean::CBufOcean>(m_cbuf);
		auto gpu_address = upload.Add(cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
		cmd_list->SetGraphicsRootConstantBufferView(static_cast<UINT>(shaders::ocean::ERootParam::CBufOcean), gpu_address);
	}

	// Update the constant buffer data for this frame
	void OceanShader::SetupFrame(std::span<GerstnerWave const> waves, v4 camera_world_pos, float time, float inner_radius, float outer_radius, int num_rings, float min_ring_spacing, bool has_env_map, v4 sun_direction, v4 sun_colour)
	{
		auto& cbuf = storage_cast<shaders::ocean::CBufOcean>(m_cbuf);
		
		cbuf.m_wave_count = std::min(static_cast<int>(waves.size()), shaders::ocean::MaxOceanWaves);
		for (int i = 0; i != cbuf.m_wave_count; ++i)
		{
			cbuf.m_wave_dirs[i] = waves[i].m_direction;
			cbuf.m_wave_params[i] = v4(
				waves[i].m_amplitude,
				waves[i].m_wavelength,
				waves[i].m_speed,
				waves[i].m_steepness);
		}

		// Zero remaining wave slots
		for (int i = cbuf.m_wave_count; i != shaders::ocean::MaxOceanWaves; ++i)
		{
			cbuf.m_wave_dirs[i] = v4::Zero();
			cbuf.m_wave_params[i] = v4::Zero();
		}

		cbuf.m_camera_pos_time = v4(camera_world_pos.x, camera_world_pos.y, camera_world_pos.z, time);
		cbuf.m_mesh_config = v4(inner_radius, outer_radius, static_cast<float>(num_rings), min_ring_spacing);
		cbuf.m_has_env_map = has_env_map ? 1 : 0;
		cbuf.m_sun_direction = sun_direction;
		cbuf.m_sun_colour = sun_colour;
	}
}
