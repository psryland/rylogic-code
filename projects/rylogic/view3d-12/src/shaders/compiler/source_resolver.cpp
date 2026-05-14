//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#include "pr/view3d-12/shaders/compiler/source_resolver.h"

namespace pr::rdr12::shader_cache
{
	// Return all bytes from 'path'.
	std::vector<uint8_t> ReadBytes(std::filesystem::path const& path)
	{
		auto file = std::ifstream(path, std::ios::binary);
		if (!file)
			throw std::runtime_error(std::format("Failed to read shader cache file '{}'", path.string()));

		file.seekg(0, std::ios::end);
		auto size = file.tellg();
		file.seekg(0, std::ios::beg);

		std::vector<uint8_t> data(static_cast<size_t>(size));
		if (!data.empty())
			file.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));

		return data;
	}

	// --------------------------------------------------------------- 

	FileSourceResolver::FileSourceResolver(std::filesystem::path include_dir)
		: m_include_dirs{ std::move(include_dir) }
	{
	}
	FileSourceResolver::FileSourceResolver(std::vector<std::filesystem::path> include_dirs)
		: m_include_dirs(std::move(include_dirs))
	{
	}

	// Return a stable key for the include search paths.
	std::string FileSourceResolver::CacheKey() const
	{
		auto key = std::string("file:");
		for (auto const& dir : m_include_dirs)
		{
			key.append(Narrow(shader_cache::CanonicalPath(dir).wstring()));
			key.push_back(';');
		}
		return key;
	}

	// Resolve a file path or include name to source text and file timestamp metadata.
	Source FileSourceResolver::Resolve(std::string_view source_name) const
	{
		auto path = std::filesystem::path(Widen(source_name));
		auto candidates = std::vector<std::filesystem::path>{};
		if (path.is_absolute())
		{
			candidates.push_back(path);
		}
		else
		{
			for (auto const& dir : m_include_dirs)
				candidates.push_back(dir / path);
			candidates.push_back(path);
		}

		for (auto const& candidate : candidates)
		{
			std::error_code ec;
			if (!std::filesystem::exists(candidate, ec))
				continue;

			auto dep = shader_cache::FileDependency(candidate);
			auto bytes = ReadBytes(dep.m_path);
			auto code = std::string(reinterpret_cast<char const*>(bytes.data()), bytes.size());
			return shader_cache::Source{
				.m_identity = dep.m_identity,
				.m_code = std::move(code),
				.m_dependency = std::move(dep),
			};
		}

		throw std::runtime_error(std::format("Shader source file '{}' was not found", source_name));
	}

	// --------------------------------------------------------------- 

	ResourceSourceResolver::ResourceSourceResolver(HMODULE module, std::wstring type)
		: m_module(module)
		, m_type(std::move(type))
	{
	}

	// Return a stable key for the resource module and resource type.
	std::string ResourceSourceResolver::CacheKey() const
	{
		return std::format("resource:{}:{}", Narrow(shader_cache::CanonicalPath(win32::ModuleFileName(m_module)).wstring()), Narrow(m_type));
	}

	// Resolve a resource name to source text and module timestamp metadata.
	Source ResourceSourceResolver::Resolve(std::string_view source_name) const
	{
		auto name = resource::Name(Widen(source_name));
		for (;;)
		{
			if (resource::Find(name, m_type.c_str(), m_module))
				break;

			if (auto i = name.find_first_of(L"./\\"); i != std::wstring::npos)
				name = name.substr(i + 1);
			else
				throw std::runtime_error(std::format("Shader resource '{}' was not found", source_name));
		}

		auto source = resource::Read<char>(name, m_type, m_module);
		auto code = std::string(source.m_data, source.m_data + source.m_len);
		auto dep = shader_cache::ResourceDependency(Narrow(name), m_type, m_module);
		return shader_cache::Source{
			.m_identity = dep.m_identity,
			.m_code = std::move(code),
			.m_dependency = std::move(dep),
		};
	}
}