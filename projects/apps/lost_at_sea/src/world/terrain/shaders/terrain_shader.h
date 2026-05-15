//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
// Custom terrain shader override: VS for CDLOD grid patches with Perlin
// noise height displacement and geomorphing. PS for height-based colouring.
#pragma once
#include "src/forward.h"

namespace las
{
	// Runtime-tunable terrain parameters.
	// Exposed via the diagnostic UI for interactive tweaking.
	struct TerrainTuning
	{
		// Noise generation
		float m_octaves = 6.0f;
		float m_base_freq = 0.001f;
		float m_persistence = 0.5f;
		float m_amplitude = 1000.0f;
		float m_sea_level_bias = -0.3f;

		// Weathering (domain warping + ridged noise)
		float m_warp_freq = 0.0004f;
		float m_warp_strength = 300.0f;
		float m_ridge_threshold = 80.0f;

		// Macro height variation (archipelago diversity)
		float m_macro_freq = 0.00008f;
		float m_macro_scale_min = 0.15f;
		float m_macro_scale_max = 1.0f;

		// Beach flattening
		float m_beach_height = 80.0f;

		// Underwater smoothing: depth (m below sea level) at which fine detail is fully removed
		float m_underwater_smooth_depth = 60.0f;
	};

	struct TerrainShader : rdr12::Shader
	{
		// Compiled shader bytecodes (populated at construction from runtime compilation).
		// The ByteCode wrappers in m_code point into these vectors, so they must outlive the shader.
		std::vector<uint8_t> m_vs_bytecode;
		std::vector<uint8_t> m_ps_bytecode;

		// Terrain constant buffer data. Shared parameters set in SetupFrame,
		// per-patch morph range overridden in SetupElement.
		alignas(16) std::byte m_cbuf[128];

		// Tunable parameters, modifiable via the diagnostic UI
		TerrainTuning m_tuning;

		explicit TerrainShader(Renderer& rdr);

		// Called per-nugget during forward rendering. Copies the shared cbuf,
		// overrides per-patch morph data from the instance's i2w, then binds.
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, ::pr::compute::GpuUploadBuffer& upload, rdr12::Scene const& scene, rdr12::DrawListElement const* dle) override;

		// Update shared per-frame data (camera position). Called once per frame.
		void SetupFrame(v4 camera_world_pos, v4 sun_direction, v4 sun_colour);
	};
}
