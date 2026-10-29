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
	// Up to BufferSlotCount optional GPU buffers are bound as raw SRVs, slot 'i' at VIEW3D_PROCEDURAL_BUFFER<i>_REGISTER, for every draw. The caller keeps
	// their contents valid while frames that use them are in flight; a buffer may be replaced between frames. Slot 0 is the shader's own buffer, which
	// BufferWrite and BufferDispatch update between frames. Other slots let the shader read buffers owned by other objects.
	struct ProceduralShader :Shader
	{
		static constexpr size_t ConstantsSize = 1024;
		static constexpr int BufferSlotCount = 2;

		ERenderStep m_rdr_step;
		std::vector<BYTE> m_vs_bytecode;
		std::array<std::vector<BYTE>, ForwardPixelFamily::SlotCount> m_ps_bytecode;
		ForwardPixelFamily m_pixel_family;
		std::array<std::byte, ConstantsSize> m_constants;
		std::array<D3DPtr<ID3D12Resource>, BufferSlotCount> m_buffers;
		std::unique_ptr<ResourceFactory> m_writer; // Created by the first buffer update. It persists because a temporary factory waits for the GPU when destroyed.
		std::vector<BYTE> m_dispatch_cs;           // Compute bytecode of the cached BufferDispatch pipeline. Callers usually repeat the same shader every frame.
		D3DPtr<ID3D12RootSignature> m_dispatch_sig;
		D3DPtr<ID3D12PipelineState> m_dispatch_pso;
		string32 m_name;

		// Copy the validated bytecode and exactly ConstantsSize bytes for one supported raster render step.
		// 'vs_bytecode' is empty only when 'pixel_family' is supplied, in which case the stock vertex shader is kept.
		// 'pixel_family' is empty, or one validated pixel shader per EForwardPixelSlot in slot order; a family requires 'rdr_step' to be RenderForward.
		// 'replaces' is the stock forward family (simple or PBR) whose entry points 'pixel_family' replaces. It is unused when 'pixel_family' is empty.
		// 'buffer' is an optional GPU resource for slot 0 in a shader-readable state, or null for none.
		ProceduralShader(Renderer& rdr, ERenderStep rdr_step, std::span<BYTE const> vs_bytecode, std::span<std::span<BYTE const> const> pixel_family, ForwardPixelFamily const& replaces, std::span<std::byte const> constants, D3DPtr<ID3D12Resource> buffer, std::string_view name);
		~ProceduralShader();

		// True if this shader replaces the stock vertex shader. Such shaders decode logical vertex IDs and need a ProceduralVertexId model.
		bool HasVertexShader() const;

		// True if this shader replaces the stock forward pixel shaders.
		bool HasPixelFamily() const;

		// True if this shader's forward pixel family replaces the stock PBR pixel shaders rather than the simple-material ones.
		bool HasPbrPixelFamily() const;

		// Replace the copied constants with exactly ConstantsSize caller bytes. Draws recorded after this call use the new values.
		void Constants(std::span<std::byte const> constants);

		// Replace the buffer bound at 'slot', in [0, BufferSlotCount), or unbind it with null. Draws recorded after this call use the new buffer. 'buffer'
		// must rest in D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE whenever the graphics queue is not updating it, and a shader that declares a slot must not
		// be drawn while no buffer is bound there.
		void Buffer(int slot, D3DPtr<ID3D12Resource> buffer);

		// Return the buffer bound at 'slot', in [0, BufferSlotCount), or null for none.
		D3DPtr<ID3D12Resource> const& Buffer(int slot) const;

		// Overwrite 'data' at byte 'offset' of the slot 0 buffer. The copy runs on the graphics queue after the frames already submitted and before
		// later frames, so no frame sees a partial write. 'offset' and the size of 'data' must be multiples of 4 that lie within the buffer. The buffer
		// must not be used by other queues while frames or updates that use it are in flight.
		void BufferWrite(size_t offset, std::span<std::byte const> data);

		// Run the compute shader 'cs_bytecode' over the slot 0 buffer with 'groups' thread groups. The shader sees a copy of 'constants' at b0 and the
		// whole buffer as a RWByteAddressBuffer at u0. Like BufferWrite, it runs on the graphics queue between the frames already submitted and later
		// frames. 'constants' holds 1 to 65536 bytes, and the buffer must have been created with D3D12_RESOURCE_FLAG_ALLOW_UNORDERED_ACCESS.
		void BufferDispatch(std::span<BYTE const> cs_bytecode, std::span<std::byte const> constants, iv3 groups);

		// Bind the current procedural constants for one draw.
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const&, CameraTransforms const&, DrawListElement const*) override;

	protected:

		// Destroy this concrete variable-sized shader type.
		void Delete() override;
	};
}
