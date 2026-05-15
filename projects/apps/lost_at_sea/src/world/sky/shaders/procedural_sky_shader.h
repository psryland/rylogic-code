//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Procedural sky shader overlay: atmospheric scattering based on sun position.
#pragma once
#include "src/forward.h"

namespace las
{
	struct ProceduralSkyShader : rdr12::Shader
	{
		std::vector<uint8_t> m_vs_bytecode;
		std::vector<uint8_t> m_ps_bytecode;

		alignas(16) std::byte m_cbuf[48];

		explicit ProceduralSkyShader(Renderer& rdr);

		void SetupElement(ID3D12GraphicsCommandList* cmd_list, ::pr::compute::GpuUploadBuffer& upload, rdr12::Scene const& scene, rdr12::DrawListElement const* dle) override;

		void SetupFrame(v4 sun_direction, v4 sun_colour, float sun_intensity);
	};
}
