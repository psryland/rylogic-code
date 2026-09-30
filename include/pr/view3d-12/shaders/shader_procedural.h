//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/shader.h"

namespace pr::rdr12
{
	// A caller-compiled procedural vertex shader with renderer-owned constants. Constants may be replaced between frames; each draw uploads the current copy.
	// An optional immutable GPU buffer is bound as a raw SRV at VIEW3D_PROCEDURAL_BUFFER_REGISTER for every draw.
	struct ProceduralVertexShader :Shader
	{
		static constexpr size_t ConstantsSize = 1024;

		ERenderStep m_rdr_step;
		std::vector<BYTE> m_vs_bytecode;
		std::array<std::byte, ConstantsSize> m_constants;
		D3DPtr<ID3D12Resource> m_buffer;
		string32 m_name;

		// Copy the validated vertex bytecode and exactly ConstantsSize bytes for one supported raster render step.
		// 'buffer' is an optional GPU resource in a shader-readable state, or null for none.
		ProceduralVertexShader(Renderer& rdr, ERenderStep rdr_step, std::span<BYTE const> vs_bytecode, std::span<std::byte const> constants, D3DPtr<ID3D12Resource> buffer, std::string_view name);

		// Replace the copied constants with exactly ConstantsSize caller bytes. Draws recorded after this call use the new values.
		void Constants(std::span<std::byte const> constants);

		// Bind the current procedural constants for one draw.
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const&, CameraTransforms const&, DrawListElement const*) override;

	protected:

		// Destroy this concrete variable-sized shader type.
		void Delete() override;
	};
}
