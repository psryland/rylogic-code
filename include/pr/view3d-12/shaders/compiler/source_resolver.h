//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/compiler/dependency.h"

namespace pr::rdr12::shader_cache
{
	// Source text returned by a cache-aware source resolver.
	struct Source
	{
		std::string m_identity;
		std::string m_code;
		Dependency m_dependency;
	};

	// An interface for resolving root shader sources and includes while exposing cache validation metadata.
	struct ISourceResolver
	{
		virtual ~ISourceResolver() = default;

		// Return a stable key for the resolver configuration.
		virtual std::string CacheKey() const = 0;

		// Resolve 'source_name' to source text and dependency metadata.
		virtual Source Resolve(std::string_view source_name) const = 0;
	};

	// A source resolver for files on disk.
	struct FileSourceResolver : ISourceResolver
	{
		std::vector<std::filesystem::path> m_include_dirs;

		FileSourceResolver() = default;
		explicit FileSourceResolver(std::filesystem::path include_dir);
		explicit FileSourceResolver(std::vector<std::filesystem::path> include_dirs);

		// Return a stable key for the include search paths.
		std::string CacheKey() const override;

		// Resolve a file path or include name to source text and file timestamp metadata.
		Source Resolve(std::string_view source_name) const override;
	};

	// A source resolver for embedded TEXT resources.
	struct ResourceSourceResolver : ISourceResolver
	{
		HMODULE m_module;
		std::wstring m_type;

		explicit ResourceSourceResolver(HMODULE module = nullptr, std::wstring type = L"TEXT");

		// Return a stable key for the resource module and resource type.
		std::string CacheKey() const override;

		// Resolve a resource name to source text and module timestamp metadata.
		Source Resolve(std::string_view source_name) const override;
	};

	// Return all bytes from 'path'.
	std::vector<uint8_t> ReadBytes(std::filesystem::path const& path);
}
