//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/shader.h"

namespace pr::rdr12
{
	// A caller-compiled procedural shader with renderer-owned constants. Constants may be replaced between frames; each draw uploads the current copy.
	// The shader has a procedural vertex shader, a forward pixel family that replaces the stock simple-material or PBR pixel shaders, or both. Without a
	// vertex shader the stock vertex shader is used, so a pixel-only shader suits any vertex source. A procedural vertex shader requires a ProceduralVertexId model.
	// An optional immutable GPU buffer is bound as a raw SRV at VIEW3D_PROCEDURAL_BUFFER_REGISTER for every draw.
	struct ProceduralShader :Shader
	{
		static constexpr size_t ConstantsSize = 1024;

		ERenderStep m_rdr_step;
		std::vector<BYTE> m_vs_bytecode;
		std::array<std::vector<BYTE>, ForwardPixelFamily::SlotCount> m_ps_bytecode;
		ForwardPixelFamily m_pixel_family;
		std::array<std::byte, ConstantsSize> m_constants;
		D3DPtr<ID3D12Resource> m_buffer;
		string32 m_name;

		// Copy the validated bytecode and exactly ConstantsSize bytes for one supported raster render step.
		// 'vs_bytecode' is empty only when 'pixel_family' is supplied, in which case the stock vertex shader is kept.
		// 'pixel_family' is empty, or one validated pixel shader per EForwardPixelSlot in slot order; a family requires 'rdr_step' to be RenderForward.
		// 'replaces' is the stock forward family (simple or PBR) whose entry points 'pixel_family' replaces. It is unused when 'pixel_family' is empty.
		// 'buffer' is an optional GPU resource in a shader-readable state, or null for none.
		ProceduralShader(Renderer& rdr, ERenderStep rdr_step, std::span<BYTE const> vs_bytecode, std::span<std::span<BYTE const> const> pixel_family, ForwardPixelFamily const& replaces, std::span<std::byte const> constants, D3DPtr<ID3D12Resource> buffer, std::string_view name);

		// True if this shader replaces the stock vertex shader. Such shaders decode logical vertex IDs and need a ProceduralVertexId model.
		bool HasVertexShader() const;

		// True if this shader replaces the stock forward pixel shaders.
		bool HasPixelFamily() const;

		// True if this shader's forward pixel family replaces the stock PBR pixel shaders rather than the simple-material ones.
		bool HasPbrPixelFamily() const;

		// Replace the copied constants with exactly ConstantsSize caller bytes. Draws recorded after this call use the new values.
		void Constants(std::span<std::byte const> constants);

		// Bind the current procedural constants for one draw.
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const&, CameraTransforms const&, DrawListElement const*) override;

	protected:

		// Destroy this concrete variable-sized shader type.
		void Delete() override;
	};
}
