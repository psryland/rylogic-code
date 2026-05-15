//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/compute/forward.h"

namespace pr::compute::shader_cache
{
	// Metadata used to decide whether a cached shader output is still valid.
	struct Dependency
	{
		enum class EKind
		{
			SourceText,
			File,
			Resource,
		};

		EKind m_kind = EKind::SourceText;
		std::string m_identity = {};
		std::filesystem::path m_path = {};
		std::wstring m_type = {};
		uint64_t m_size = 0;
		uint64_t m_last_write = 0;
		uint64_t m_hash = 0;

		// Return true if this dependency describes a source that can be validated later.
		bool IsValid() const
		{
			return !m_identity.empty();
		}

		// Return true if this dependency describes the same content version as 'rhs'.
		bool SameContentVersion(Dependency const& rhs) const
		{
			return
				m_kind == rhs.m_kind &&
				m_identity == rhs.m_identity &&
				m_path == rhs.m_path &&
				m_type == rhs.m_type &&
				m_size == rhs.m_size &&
				m_last_write == rhs.m_last_write &&
				m_hash == rhs.m_hash;
		}
	};

	// Convert a FILETIME value to a comparable integer.
	inline uint64_t FileTime(FILETIME ft)
	{
		ULARGE_INTEGER value;
		value.LowPart = ft.dwLowDateTime;
		value.HighPart = ft.dwHighDateTime;
		return value.QuadPart;
	}

	// Return a stable absolute path when possible.
	inline std::filesystem::path CanonicalPath(std::filesystem::path path)
	{
		std::error_code ec;
		auto abs = path.is_absolute() ? path : std::filesystem::absolute(path, ec);
		if (ec)
			abs = path;

		auto canon = std::filesystem::weakly_canonical(abs, ec);
		return !ec ? canon : abs;
	}

	// Return timestamp and size metadata for a file.
	inline Dependency FileDependency(std::filesystem::path path, std::string identity = {})
	{
		path = CanonicalPath(std::move(path));

		auto file = win32::FileOpen(
			path,
			GENERIC_READ,
			FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
			OPEN_EXISTING,
			FILE_ATTRIBUTE_NORMAL,
			FILE_FLAG_SEQUENTIAL_SCAN);
		if (!file)
			throw std::runtime_error(std::format("Failed to open shader dependency '{}'", path.string()));

		BY_HANDLE_FILE_INFORMATION info;
		Check(::GetFileInformationByHandle(file, &info), "Failed to query shader dependency file information");

		ULARGE_INTEGER size;
		size.LowPart = info.nFileSizeLow;
		size.HighPart = info.nFileSizeHigh;

		return Dependency{
			.m_kind = Dependency::EKind::File,
			.m_identity = !identity.empty() ? std::move(identity) : Narrow(path.wstring()),
			.m_path = std::move(path),
			.m_type = {},
			.m_size = size.QuadPart,
			.m_last_write = FileTime(info.ftLastWriteTime),
			.m_hash = 0,
		};
	}

	// Return timestamp and size metadata for a module-backed resource.
	inline Dependency ResourceDependency(std::string identity, std::wstring type, HMODULE module = nullptr)
	{
		auto path = CanonicalPath(win32::ModuleFileName(module));
		auto dep = FileDependency(path, std::move(identity));
		dep.m_kind = Dependency::EKind::Resource;
		dep.m_type = std::move(type);
		return dep;
	}

	// Return metadata for source text that is not backed by a timestamped source.
	inline Dependency SourceTextDependency(std::string identity, std::string_view code)
	{
		auto const hash_value = !code.empty()
			? static_cast<uint64_t>(hash::HashBytes64(code.data(), code.data() + code.size()))
			: uint64_t{};

		return Dependency{
			.m_kind = Dependency::EKind::SourceText,
			.m_identity = std::move(identity),
			.m_path = {},
			.m_type = {},
			.m_size = static_cast<uint64_t>(code.size()),
			.m_last_write = 0,
			.m_hash = hash_value,
		};
	}

	// Return current metadata for a previously recorded dependency.
	inline std::optional<Dependency> CurrentDependency(Dependency const& dep)
	{
		try
		{
			switch (dep.m_kind)
			{
				case Dependency::EKind::SourceText:
				{
					return dep;
				}
				case Dependency::EKind::File:
				{
					return FileDependency(dep.m_path, dep.m_identity);
				}
				case Dependency::EKind::Resource:
				{
					auto current = FileDependency(dep.m_path, dep.m_identity);
					current.m_kind = Dependency::EKind::Resource;
					current.m_type = dep.m_type;
					return current;
				}
				default:
				{
					throw std::runtime_error("Unknown shader dependency kind");
				}
			}
		}
		catch (...)
		{
			return std::nullopt;
		}
	}
}

