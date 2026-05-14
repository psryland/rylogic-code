//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/compiler/shader_cache.h"

namespace pr::rdr12
{
	// HLSL language version to request from DXC.
	enum class EHlslVersion
	{
		DxcDefault,
		Hlsl2021,
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
		shader_cache::IShaderCache* m_cache;
		shader_cache::ISourceResolver const* m_include_resolver;
		std::unique_ptr<shader_cache::ISourceResolver> m_owned_resolver;
		std::filesystem::path m_pdb_dir;
		std::wstring m_pdb_name_hint;
		std::wstring m_pdb_arg;
		std::string m_source_code;
		DxcBuffer m_source;
		shader_cache::Dependency m_source_dependency;
		std::string m_source_identity;
		Defines m_defines;
		std::wstring m_ep;
		std::wstring m_sm;
		EHlslVersion m_hlsl_version;
		bool m_optimise;
		bool m_debug_info;
		StrList m_extras;
		Args m_args;
		StrList m_canonical_args;

		explicit ShaderCompiler(shader_cache::IShaderCache* cache = nullptr);
		ShaderCompiler& Cache(shader_cache::IShaderCache* cache);
		ShaderCompiler& File(std::filesystem::path file);
		ShaderCompiler& Source(std::string_view code);
		ShaderCompiler& Source(std::string_view code, std::string_view identity);
		ShaderCompiler& Source(std::string_view source_name, shader_cache::ISourceResolver const& resolver);
		ShaderCompiler& Includes(D3DPtr<IDxcIncludeHandler> handler);
		ShaderCompiler& Includes(shader_cache::ISourceResolver const& resolver);
		ShaderCompiler& EntryPoint(std::wstring_view ep);
		ShaderCompiler& ShaderModel(std::wstring_view sm);
		ShaderCompiler& HlslVersion(EHlslVersion version);
		ShaderCompiler& Optimise(bool opt = true);
		ShaderCompiler& DebugInfo(bool dbg = true);
		ShaderCompiler& Define(std::wstring_view sym, std::wstring_view value = {});
		ShaderCompiler& PDBOutput(std::filesystem::path dir, std::string_view filename = {});
		ShaderCompiler& Arg(std::wstring_view arg);
		std::vector<uint8_t> Compile();

	private:

		// Assign the current root source text and cache metadata.
		void SetSource(shader_cache::Source source);

		// Build the final DXC argument list and the cache-key argument list.
		void BuildArgs();

		// Return true if the current compiler state can be cached safely.
		bool Cacheable() const;

		// Return the cache request for the current compiler state.
		shader_cache::CompileRequest Request() const;

		// Compile without consulting the cache.
		shader_cache::CompileResult CompileUncached();
	};
}
