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

	// Compiler options helper
	struct ShaderCompiler
	{
		// Notes:
		//  - If you need pdb's for PIX debugging, use options like this:
		//    compiler.DebugInfo().Optimise(false).PDBOutput(L"E:\\dump\\Symbols");
		//    This will create a pdb in the specified directory. Point the PDB Search Path
		//    in PIX to this directory and you should be able to debug the shader.
		using Defines = std::unordered_map<std::wstring, std::wstring>;
		using StrList = std::vector<std::wstring>;
		using Args = std::vector<wchar_t const*>;

		D3DPtr<IDxcResult> m_result;
		D3DPtr<IDxcCompiler3> m_compiler;
		D3DPtr<IDxcBlobEncoding> m_source_blob;
		D3DPtr<IDxcIncludeHandler> m_includes;
		std::filesystem::path m_pdb_path;
		DxcBuffer m_source;
		Defines m_defines;
		std::wstring m_ep;
		std::wstring m_sm;
		bool m_optimise;
		bool m_debug_info;
		StrList m_extras;
		Args m_args;

		ShaderCompiler();
		ShaderCompiler& File(std::filesystem::path file);
		ShaderCompiler& Source(std::string_view code);
		ShaderCompiler& Includes(D3DPtr<IDxcIncludeHandler> handler);
		ShaderCompiler& EntryPoint(std::wstring_view ep);
		ShaderCompiler& ShaderModel(std::wstring_view sm);
		ShaderCompiler& Optimise(bool opt = true);
		ShaderCompiler& DebugInfo(bool dbg = true);
		ShaderCompiler& Define(std::wstring_view sym, std::wstring_view value = {});
		ShaderCompiler& PDBOutput(std::filesystem::path dir, std::string_view filename = {});
		ShaderCompiler& Arg(std::wstring_view arg);
		std::vector<uint8_t> Compile();
	};

	// Statically declared shader byte code
	namespace shader_code
	{
		// Not a shader
		extern ByteCode const none;

		// Forward rendering shaders
		extern ByteCode const forward_vs;
		extern ByteCode const forward_ps;
		extern ByteCode const forward_reflection_attrs_ps;
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

		// Ray tracing
		extern ByteCode const ray_trace_lib;
		extern ByteCode const ray_trace_present_vs;
		extern ByteCode const ray_trace_present_ps;

		// MipMap generation
		extern ByteCode const mipmap_generator_cs;

		// Skinning
		extern ByteCode const skinning_cs;
	}
}
