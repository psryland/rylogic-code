//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/gpu_transfer_buffer.h"
#include "pr/view3d-12/utility/wrappers.h"

namespace pr::rdr12
{
	// The compiled byte code for the shader stages
	struct ShaderCode
	{
		// This is the order they appear in the pipeline state description
		ByteCode VS;
		ByteCode PS;
		ByteCode DS;
		ByteCode HS;
		ByteCode GS;
		ByteCode CS;
	};

	// A shader base class
	struct Shader :RefCounted<Shader>
	{
		// Notes:
		//  - A "shader" means the full set of VS,PS,GS,DS,HS,etc because constant buffers etc apply to all stages now.
		//  - A shader without a Signature is an 'overlay' shader, intended to replace parts of a full shader. Overlay shaders
		//    must use constant buffers that don't conflict with the base shader, and the base shader must have a signature that
		//    handles all possible overlays.
		//  - A shader does not contain a reference to a render step or window (i.e. without a GpuSync).
		//    When the shader is needed, it is "realised" in a given pool that is owned by the window/render step, etc.
		//  - The size of a shader depends on the shader type, so this type must be allocated.
		//  - The shader contains the shader specific parameters.
		//  - The realised shader is reused by the window/render step.
		//  - All shaders can share one GpuUploadBuffer
		Renderer*                   m_rdr;       // The renderer that owns this model
		ShaderCode                  m_code;      // Byte code for the shader parts
		D3DPtr<ID3D12RootSignature> m_signature; // Signature for shader, null if an overlay
		
		explicit Shader(Renderer& rdr);
		virtual ~Shader() = default;

		// Renderer access
		Renderer const& rdr() const;
		Renderer& rdr();

		// Sort id for the shader
		SortKeyId SortId() const;

		// Create a shader
		template <typename TShader, typename... Args> requires (std::is_base_of_v<Shader, TShader> && std::constructible_from<TShader, Args...>)
		static RefPtr<TShader> Create(Args&&... args)
		{
			RefPtr<TShader> shdr(rdr12::New<TShader>(std::forward<Args>(args)...), true);
			return shdr;
		}

		// Config the shader stages.
		virtual void SetupFrame(ID3D12GraphicsCommandList*, GpuUploadBuffer&, Scene const&) {}
		virtual void SetupElement(ID3D12GraphicsCommandList*, GpuUploadBuffer&, Scene const&, DrawListElement const*) {}

		// Ref counting clean up
		static void RefCountZero(RefCounted<Shader>* doomed);
		protected: virtual void Delete();
	};

	// Statically declared shader byte code
	namespace shader_code
	{
		// Not a shader
		extern ByteCode const none;

		// Forward rendering shaders
		extern ByteCode const forward_vs;
		extern ByteCode const forward_ps;
		extern ByteCode const forward_radial_fade_ps;
		extern ByteCode const forward_alpha_collect_ps;

		// Deferred rendering
		extern ByteCode const gbuffer_vs;
		extern ByteCode const gbuffer_ps;
		extern ByteCode const dslighting_vs;
		extern ByteCode const dslighting_ps;

		// Shadows
		extern ByteCode const shadow_map_vs;
		extern ByteCode const shadow_map_ps;

		// Screen Space
		extern ByteCode const kbuffer_resolve_vs;
		extern ByteCode const kbuffer_alpha_resolve_ps;
		extern ByteCode const point_sprites_gs;
		extern ByteCode const thick_line_list_gs;
		extern ByteCode const thick_line_strip_gs;
		extern ByteCode const arrow_head_gs;
		extern ByteCode const show_normals_gs;

		// Ray cast
		extern ByteCode const ray_cast_vs;
		extern ByteCode const ray_cast_vert_gs;
		extern ByteCode const ray_cast_edge_gs;
		extern ByteCode const ray_cast_face_gs;

		// MipMap generation
		extern ByteCode const mipmap_generator_cs;
	}
}
