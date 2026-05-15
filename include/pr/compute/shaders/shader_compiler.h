//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/shaders/shader_cache.h"

namespace pr::compute
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

		// Create a compiler with default options and no shader cache.
		ShaderCompiler();
		ShaderCompiler(ShaderCompiler&& rhs);
		ShaderCompiler(ShaderCompiler const&) = delete;
		ShaderCompiler& operator = (ShaderCompiler&& rhs);
		ShaderCompiler& operator = (ShaderCompiler const&) = delete;

		// Set the shader cache that should be consulted by Compile().
		template <typename Self> Self&& Cache(this Self&& self, shader_cache::IShaderCache* cache)
		{
			self.m_cache = cache;
			return static_cast<Self&&>(self);
		}

		// Use a file as the root source and resolve relative includes from the file's directory.
		template <typename Self> Self&& File(this Self&& self, std::filesystem::path file)
		{
			auto path = shader_cache::CanonicalPath(std::move(file));
			self.m_owned_resolver = std::make_unique<shader_cache::FileSourceResolver>(path.parent_path());

			// FileSourceResolver returns both the source text and the file metadata needed to validate cached output later.
			self.SetSource(self.m_owned_resolver->Resolve(Narrow(path.wstring())));
			self.m_include_resolver = self.m_owned_resolver.get();
			self.m_includes = nullptr;
			return static_cast<Self&&>(self);
		}

		// Use anonymous UTF-8 source text. Anonymous source is compilable but not cacheable because it has no stable identity.
		template <typename Self> Self&& Source(this Self&& self, std::string_view code)
		{
			self.m_source_blob = nullptr;
			self.m_source_code.assign(code);
			self.m_source_identity.clear();
			self.m_source_dependency = {};
			self.RebindSource();
			return static_cast<Self&&>(self);
		}

		// Use UTF-8 source text with a stable cache identity.
		template <typename Self> Self&& Source(this Self&& self, std::string_view code, std::string_view identity)
		{
			self.Source(code);
			if (!identity.empty())
			{
				// Hash the current source text so cache entries are invalidated when the caller supplies changed text under the same identity.
				self.m_source_identity = identity;
				self.m_source_dependency = shader_cache::SourceTextDependency(self.m_source_identity, self.m_source_code);
			}
			return static_cast<Self&&>(self);
		}

		// Use a caller-provided resolver for the root source and all includes.
		template <typename Self> Self&& Source(this Self&& self, std::string_view source_name, shader_cache::ISourceResolver const& resolver)
		{
			self.SetSource(resolver.Resolve(source_name));
			self.m_include_resolver = &resolver;
			self.m_includes = nullptr;
			return static_cast<Self&&>(self);
		}

		// Use a raw DXC include handler. This disables include dependency tracking because the handler has no cache-key contract.
		template <typename Self> Self&& Includes(this Self&& self, D3DPtr<IDxcIncludeHandler> handler)
		{
			self.m_includes = std::move(handler);
			self.m_include_resolver = nullptr;
			return static_cast<Self&&>(self);
		}

		// Use a cache-aware include resolver for include lookup.
		template <typename Self> Self&& Includes(this Self&& self, shader_cache::ISourceResolver const& resolver)
		{
			self.m_include_resolver = &resolver;
			self.m_includes = nullptr;
			return static_cast<Self&&>(self);
		}

		// Set the HLSL entry point passed to DXC.
		template <typename Self> Self&& EntryPoint(this Self&& self, std::wstring_view ep)
		{
			self.m_ep.clear();
			self.m_ep.append(L"-E").append(ep);
			return static_cast<Self&&>(self);
		}

		// Set the shader model target passed to DXC.
		template <typename Self> Self&& ShaderModel(this Self&& self, std::wstring_view sm)
		{
			self.m_sm.clear();
			self.m_sm.append(L"-T").append(sm);
			return static_cast<Self&&>(self);
		}

		// Select the HLSL language version requested from DXC.
		template <typename Self> Self&& HlslVersion(this Self&& self, EHlslVersion version)
		{
			self.m_hlsl_version = version;
			return static_cast<Self&&>(self);
		}

		// Enable or disable DXC optimisation.
		template <typename Self> Self&& Optimise(this Self&& self, bool opt = true)
		{
			self.m_optimise = opt;
			return static_cast<Self&&>(self);
		}

		// Enable or disable PDB/debug-info emission in the compiled shader.
		template <typename Self> Self&& DebugInfo(this Self&& self, bool dbg = true)
		{
			self.m_debug_info = dbg;
			return static_cast<Self&&>(self);
		}

		// Add or replace a preprocessor definition passed to DXC.
		template <typename Self> Self&& Define(this Self&& self, std::wstring_view sym, std::wstring_view value = {})
		{
			self.m_defines[std::wstring(sym)] = !value.empty()
				? std::format(L"-D{}={}", sym, value)
				: std::format(L"-D{}", sym);
			return static_cast<Self&&>(self);
		}

		// Request PDB output in 'dir', optionally using 'filename' as the cache-stable name hint.
		template <typename Self> Self&& PDBOutput(this Self&& self, std::filesystem::path dir, std::string_view filename = {})
		{
			self.m_pdb_dir = std::move(dir);
			self.m_pdb_name_hint = Widen(filename);
			return static_cast<Self&&>(self);
		}

		// Append a raw DXC argument after the generated arguments.
		template <typename Self> Self&& Arg(this Self&& self, std::wstring_view arg)
		{
			self.m_extras.push_back(std::wstring(arg));
			return static_cast<Self&&>(self);
		}

		std::vector<uint8_t> Compile();

	private:

		// Assign the current root source text and cache metadata.
		void SetSource(shader_cache::Source source);

		// Point the DXC source buffer at the current owned UTF-8 source text.
		void RebindSource();

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

