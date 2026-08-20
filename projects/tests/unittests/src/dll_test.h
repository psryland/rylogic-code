//***********************************************************************
// Native DLL Contract Tests
//  Copyright (c) Rylogic Ltd 2026
//***********************************************************************
#pragma once
#include "pr/common/unittests.h"

namespace pr::unittests
{
	// Owns one exact source-built DLL module and resolves its public exports without import-library linkage.
	class DllModule
	{
		HMODULE m_module;
		std::filesystem::path m_path;

	public:
		// Load the named DLL from the selected source build's deterministic output directory.
		DllModule(char const* source_file, wchar_t const* project_output, wchar_t const* dll_name)
			: m_module()
			, m_path(BuildArtifactPath(source_file, project_output, dll_name))
		{
			m_module = LoadLibraryExW(m_path.c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
			if (m_module == nullptr)
				throw std::runtime_error(std::format("Failed to load '{}' (Win32 error {})", m_path.string(), GetLastError()));
		}
		~DllModule()
		{
			if (m_module != nullptr)
				FreeLibrary(m_module);
		}
		DllModule(DllModule const&) = delete;
		DllModule& operator=(DllModule const&) = delete;

		// Resolve one required export with its caller-supplied stdcall function-pointer type.
		template <typename FuncType>
		FuncType Proc(char const* name) const
		{
			auto proc = GetProcAddress(m_module, name);
			if (proc == nullptr)
				throw std::runtime_error(std::format("Export '{}' was not found in '{}' (Win32 error {})", name, m_path.string(), GetLastError()));

			return reinterpret_cast<FuncType>(proc);
		}

	private:
		// Locate the repository root from the owning test source, then select the current platform/configuration artifact.
		static std::filesystem::path BuildArtifactPath(char const* source_file, wchar_t const* project_output, wchar_t const* dll_name)
		{
			auto root = std::filesystem::absolute(source_file).parent_path();
			for (; root.has_parent_path() && !std::filesystem::exists(root / L"Rylogic.sln"); root = root.parent_path())
			{}

			if (!std::filesystem::exists(root / L"Rylogic.sln"))
				throw std::runtime_error(std::format("Could not locate the Rylogic repository root from '{}'", source_file));

			return root / L"obj" / project_output / Platform / Config / dll_name;
		}
	};
}
