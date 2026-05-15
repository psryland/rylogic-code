//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Distant ocean shader overlay: flat z=0 patches with Fresnel + fog.
#pragma once
#include "src/forward.h"

namespace las
{
	struct DistantOceanShader : rdr12::Shader
	{
		std::vector<uint8_t> m_vs_bytecode;
		std::vector<uint8_t> m_ps_bytecode;

		alignas(16) std::byte m_cbuf[128];

		explicit DistantOceanShader(Renderer& rdr);

		// Called per-nugget during forward rendering to bind the constant buffer
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, ::pr::compute::GpuUploadBuffer& upload, rdr12::Scene const& scene, rdr12::DrawListElement const* dle) override;

		// Update shared per-frame data
		void SetupFrame(v4 camera_world_pos, bool has_env_map, v4 sun_direction, v4 sun_colour);
	};
}
