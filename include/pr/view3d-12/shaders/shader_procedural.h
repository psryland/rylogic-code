//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/shader.h"

namespace pr::rdr12
{
	// A caller-compiled procedural vertex shader with renderer-owned immutable constants.
	struct ProceduralVertexShader :Shader
	{
		static constexpr size_t ConstantsSize = 1024;

		ERenderStep m_rdr_step;
		std::vector<BYTE> m_vs_bytecode;
		std::array<std::byte, ConstantsSize> m_constants;
		string32 m_name;

		// Copy the validated vertex bytecode and exactly ConstantsSize bytes for one supported raster render step.
		ProceduralVertexShader(Renderer& rdr, ERenderStep rdr_step, std::span<BYTE const> vs_bytecode, std::span<std::byte const> constants, std::string_view name);

		// Bind the immutable procedural constants for one draw.
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const&, DrawListElement const*) override;

	protected:

		// Destroy this concrete variable-sized shader type.
		void Delete() override;
	};
}
