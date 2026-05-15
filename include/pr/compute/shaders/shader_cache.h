//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/shaders/source_resolver.h"

namespace pr::compute::shader_cache
{
	// A compiled shader plus optional side outputs such as PDB data.
	struct CompileOutput
	{
		std::vector<uint8_t> m_byte_code;
		std::vector<uint8_t> m_pdb;
		std::wstring m_pdb_name;
	};

	// The result produced by a real DXC compile on a cache miss.
	struct CompileResult
	{
		CompileOutput m_output;
		std::vector<Dependency> m_dependencies;
	};

	// A cache lookup request for one shader entry point.
	struct CompileRequest
	{
		std::string m_source_identity;
		Dependency m_source_dependency;
		std::string m_resolver_key;
		std::wstring m_compiler_key;
		std::vector<std::wstring> m_args;
		std::wstring m_pdb_name_hint;
		std::filesystem::path m_pdb_output_dir;

		// Return true if the request has enough information for a reliable cache lookup.
		bool IsCacheable() const
		{
			return !m_source_identity.empty() && m_source_dependency.IsValid() && !m_compiler_key.empty();
		}
	};

	// Silent counters for cache diagnostics.
	struct CacheStats
	{
		uint64_t m_hit_validated = 0;
		uint64_t m_miss_compiled = 0;
		uint64_t m_miss_compile_failed = 0;
		uint64_t m_hit_pdb_materialised = 0;
		uint64_t m_lock_timeout_uncached = 0;
		uint64_t m_io_error_uncached = 0;
	};

	// Interface for compiled shader caches.
	struct IShaderCache
	{
		using CompileFunc = std::function<CompileResult()>;

		virtual ~IShaderCache() = default;

		// Return cached output or call 'compile' once to produce and store it.
		virtual CompileOutput GetOrCompile(CompileRequest const& request, CompileFunc compile) = 0;

		// Return diagnostic counters for this cache instance.
		virtual CacheStats Stats() const = 0;
	};
}

