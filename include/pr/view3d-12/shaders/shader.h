//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// The compiled byte code for the shader stages
	struct ShaderCode
	{
		using ByteCode = ::pr::compute::ByteCode;

		// This is the order they appear in the pipeline state description
		ByteCode VS;
		ByteCode PS;
		ByteCode DS;
		ByteCode HS;
		ByteCode GS;
		ByteCode CS;
	};

	// The output contract of one forward pixel-shader entry point. Each forward sub-pass, with or without far-clip fade, needs a different entry point.
	enum class EForwardPixelSlot
	{
		Opaque,
		ReflectionAttrs,
		AlphaCollect,
		FarFade,
		FarFadeReflectionAttrs,
		FarFadeAlphaCollect,
	};

	// The set of forward pixel-shader entry points that share one shading function, one per output contract.
	// See 'pr/view3d-12/shaders/forward_pixel.hlsli' for how a family is written in HLSL.
	struct ForwardPixelFamily
	{
		using ByteCode = ::pr::compute::ByteCode;
		static constexpr size_t SlotCount = static_cast<size_t>(EForwardPixelSlot::FarFadeAlphaCollect) + 1;
		std::array<ByteCode, SlotCount> m_code;

		// The stock family whose entry points this family replaces, or null for a stock family. Its pixel shader identifies the slot to replace.
		ForwardPixelFamily const* m_replaces;

		// Return the entry point for 'slot'.
		ByteCode const& operator[](EForwardPixelSlot slot) const
		{
			return m_code[static_cast<size_t>(slot)];
		}

		// Return the slot that holds 'ps', or nothing if 'ps' is not part of this family. Byte code is identified by address and length.
		std::optional<EForwardPixelSlot> Find(D3D12_SHADER_BYTECODE const& ps) const
		{
			for (size_t i = 0; i != m_code.size(); ++i)
			{
				if (m_code[i].pShaderBytecode == ps.pShaderBytecode && m_code[i].BytecodeLength == ps.BytecodeLength)
					return static_cast<EForwardPixelSlot>(i);
			}
			return std::nullopt;
		}
	};

	// A shader base class
	struct Shader :RefCounted<Shader>
	{
		using GpuUploadBuffer = ::pr::compute::GpuUploadBuffer;

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
			RefPtr<TShader> shdr(::pr::compute::New<TShader>(std::forward<Args>(args)...), true);
			return shdr;
		}

		// Config the shader stages.
		virtual void SetupFrame(ID3D12GraphicsCommandList*, GpuUploadBuffer&, Scene const&) {}
		// Configure a draw using camera transforms borrowed from the current render pass.
		virtual void SetupElement(ID3D12GraphicsCommandList*, GpuUploadBuffer&, Scene const&, CameraTransforms const&, DrawListElement const*) {}

		// Ref counting clean up
		static void RefCountZero(RefCounted<Shader>* doomed);
		protected: virtual void Delete();
	};

	// Statically declared shader byte code
	namespace shader_code
	{
		using ByteCode = ::pr::compute::ByteCode;

		// Not a shader
		extern ByteCode const none;

		// Forward rendering shaders
		extern ByteCode const forward_vs;
		extern ByteCode const forward_ps;
		extern ByteCode const forward_pbr_ps;
		extern ByteCode const forward_reflection_attrs_ps;
		extern ByteCode const forward_reflection_attrs_pbr_ps;
		extern ByteCode const forward_alpha_collect_ps;
		extern ByteCode const forward_alpha_collect_pbr_ps;
		extern ByteCode const forward_texn_pbr_vs;
		extern ByteCode const forward_texn_pbr_ps;
		extern ByteCode const forward_reflection_attrs_texn_pbr_ps;
		extern ByteCode const forward_alpha_collect_texn_pbr_ps;
		extern ByteCode const forward_radial_fade_ps;

		// Opt-in forward far-depth output variants.
		extern ByteCode const forward_far_fade_ps;
		extern ByteCode const forward_far_fade_pbr_ps;
		extern ByteCode const forward_far_fade_texn_pbr_ps;
		extern ByteCode const forward_far_fade_alpha_collect_ps;
		extern ByteCode const forward_far_fade_alpha_collect_pbr_ps;
		extern ByteCode const forward_far_fade_alpha_collect_texn_pbr_ps;
		extern ByteCode const forward_far_fade_reflection_attrs_ps;
		extern ByteCode const forward_far_fade_reflection_attrs_pbr_ps;
		extern ByteCode const forward_far_fade_reflection_attrs_texn_pbr_ps;
	    extern ByteCode const forward_detail_ps;
	    extern ByteCode const forward_reflection_attrs_detail_ps;
	    extern ByteCode const forward_alpha_collect_detail_ps;
	    extern ByteCode const forward_far_fade_detail_ps;
	    extern ByteCode const forward_far_fade_alpha_collect_detail_ps;
	    extern ByteCode const forward_far_fade_reflection_attrs_detail_ps;

		// The stock forward pixel families. Each groups the entries above that share one shading function.
		extern ForwardPixelFamily const forward_family;
		extern ForwardPixelFamily const forward_pbr_family;
		extern ForwardPixelFamily const forward_texn_pbr_family;
		extern ForwardPixelFamily const forward_detail_family;

		// Procedural atmosphere
		extern ByteCode const procedural_sky_vs;
		extern ByteCode const procedural_sky_ps;

		// Deferred rendering

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

		// Post-processing
		extern ByteCode const post_effect_vs;
		extern ByteCode const underwater_ps;

		// Ray cast
		extern ByteCode const ray_cast_vs;
		extern ByteCode const ray_cast_vert_gs;
		extern ByteCode const ray_cast_edge_gs;
		extern ByteCode const ray_cast_face_gs;

		// Ray tracing
		extern ByteCode const ray_trace_lib;
		extern ByteCode const ray_trace_present_vs;
		extern ByteCode const ray_trace_present_ps;

		// MipMap generation
		extern ByteCode const mipmap_generator_cs;

		// Skinning
		extern ByteCode const skinning_cs;

		// Environment map face distances
		extern ByteCode const env_map_distance_cs;
	}
}
