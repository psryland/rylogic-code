//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#include "src/forward.h"
#include "src/world/terrain/terrain.h"
#include "src/world/terrain/cdlod.h"
#include "src/world/terrain/shaders/terrain_shader.h"
#include "src/world/terrain/shaders/terrain_cbuf.hlsli"

namespace las
{
	enum class ERootParam : int
	{
		CBufScene = 0,   // Scene constant buffer (b0)
		CBufObject = 1,  // Object constant buffer (b1)
		CBufFrame = 2,   // Frame constant buffer (b2)
		CBufTerrain = 3, // Terrain params (b3)
	};

	TerrainShader::TerrainShader(Renderer& rdr)
		: Shader(rdr)
		, m_vs_bytecode()
		, m_ps_bytecode()
		, m_cbuf()
		, m_tuning()
	{
		static_assert((sizeof(CBufTerrain) % 16) == 0);
		static_assert(sizeof(CBufTerrain) == sizeof(m_cbuf), "CBufTerrain exceeds m_cbuf storage");
		auto resolver = ::pr::compute::shader_cache::ResourceSourceResolver{};

		// Compile the shader
		auto compiler = ShaderCompiler{}
			.Source("src/world/terrain/shaders/terrain.hlsl", resolver)
			.HlslVersion(EHlslVersion::Hlsl2021)
			.Define(L"SHADER_BUILD")
			.Optimise(true);

		m_vs_bytecode = compiler.ShaderModel(L"vs_6_0").EntryPoint(L"VSTerrain").Compile();
		m_ps_bytecode = compiler.ShaderModel(L"ps_6_0").EntryPoint(L"PSTerrain").Compile();
		m_code.VS = { m_vs_bytecode };
		m_code.PS = { m_ps_bytecode };

		// Initialise cbuf from tuning defaults
		auto& cbuf = storage_cast<CBufTerrain>(m_cbuf);
		cbuf = CBufTerrain {
			.camera_pos = v4::Zero(),
			.patch_config = v4(0, 0, static_cast<float>(cdlod::GridN), 0),
			.noise_params = v4(m_tuning.m_octaves, m_tuning.m_base_freq, m_tuning.m_persistence, m_tuning.m_amplitude),
			.noise_bias = v4(m_tuning.m_sea_level_bias, 0, 0, 0),
			.sun_direction = Normalise(v4(0.5f, 0.3f, 0.8f, 0.0f)),
			.sun_colour = v4(1.0f, 0.95f, 0.85f, 1.0f),
			.weather_params = v4(m_tuning.m_warp_freq, m_tuning.m_warp_strength, m_tuning.m_ridge_threshold, m_tuning.m_macro_freq),
			.beach_params = v4(m_tuning.m_beach_height, m_tuning.m_macro_scale_min, m_tuning.m_macro_scale_max, m_tuning.m_underwater_smooth_depth),
		};
	}

	// Called per-nugget during forward rendering. Copies the shared cbuf,
	// overrides per-patch morph data from the instance's i2w, then binds.
	void TerrainShader::SetupElement(ID3D12GraphicsCommandList* cmd_list, ::pr::compute::GpuUploadBuffer& upload, rdr12::Scene const&, rdr12::DrawListElement const* dle)
	{
		if (dle == nullptr)
			return;

		// Start from the shared cbuf (camera_pos, noise, sun set per-frame)
		auto cbuf = storage_cast<CBufTerrain>(m_cbuf);

		// Extract patch size from the instance's i2w (x-axis scale = patch_size)
		auto const& i2w = GetO2W(*dle->m_instance);
		auto patch_size = i2w.x.x;

		// Morph range matches LOD level boundaries:
		// morph=0 at inner edge (where this LOD's children would be used)
		// morph=1 at outer edge (where the parent LOD takes over)
		cbuf.patch_config.x = patch_size * cdlod::SubdivFactor;         // morph_start (inner edge)
		cbuf.patch_config.y = patch_size * cdlod::SubdivFactor * 2.0f;  // morph_end (outer edge = parent's threshold)
		cbuf.patch_config.z = static_cast<float>(cdlod::GridN);     // grid subdivisions

		auto gpu_address = upload.Add(cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false);
		cmd_list->SetGraphicsRootConstantBufferView(static_cast<UINT>(ERootParam::CBufTerrain), gpu_address);
	}

	// Update shared per-frame data (camera position).
	void TerrainShader::SetupFrame(v4 camera_world_pos, v4 sun_direction, v4 sun_colour)
	{
		auto& cbuf = storage_cast<CBufTerrain>(m_cbuf);
		cbuf.camera_pos = camera_world_pos;
		cbuf.sun_direction = sun_direction;
		cbuf.sun_colour = sun_colour;

		// Sync tunable parameters to cbuf each frame
		cbuf.noise_params = v4(m_tuning.m_octaves, m_tuning.m_base_freq, m_tuning.m_persistence, m_tuning.m_amplitude);
		cbuf.noise_bias = v4(m_tuning.m_sea_level_bias, 0, 0, 0);
		cbuf.weather_params = v4(m_tuning.m_warp_freq, m_tuning.m_warp_strength, m_tuning.m_ridge_threshold, m_tuning.m_macro_freq);
		cbuf.beach_params = v4(m_tuning.m_beach_height, m_tuning.m_macro_scale_min, m_tuning.m_macro_scale_max, m_tuning.m_underwater_smooth_depth);
	}
}
