//***********************************************************************
// Native DLL Contract Tests
//  Copyright (c) Rylogic Ltd 2026
//***********************************************************************
#pragma once
#include "pr/common/unittests.h"
#include "pr/threads/process.h"

namespace pr::unittests
{
	// The runner sets this only after selecting exactly one registered child method.
	inline std::string_view DllTestChild;

	// Select an isolated DLL control before the runner configures diagnostics or constructs fixtures.
	inline bool TakeDllTestChildOption(std::vector<char*>& args, std::ostream& err)
	{
		constexpr auto prefix = std::string_view{"-dll-test-child:"};
		auto found = args.end();
		for (auto it = args.begin(); it != args.end(); ++it)
		{
			if (!std::string_view{*it}.starts_with(prefix))
				continue;

			if (found != args.end())
			{
				err << "Repeated " << prefix << " argument\n";
				return false;
			}
			found = it;
		}
		if (found == args.end())
			return true;

		auto const selector = std::string_view{*found}.substr(prefix.size());
		auto const split = selector.rfind("::");
		if (split == selector.npos)
		{
			err << "Expected -dll-test-child:Class::Method\n";
			return false;
		}
		auto matches = [&](auto const& test)
		{
			return MatchTestFilter(test.m_class->name(), selector.substr(0, split)) && test.m_name == selector.substr(split + 2);
		};
		if (std::ranges::count_if(TestFramework::Tests, matches) != 1)
		{
			err << "Isolated DLL selector must identify exactly one registered method: " << selector << '\n';
			return false;
		}
		std::erase_if(TestFramework::Tests, [&](auto const& test) { return !matches(test); });
		DllTestChild = selector;
		args.erase(found);
		return true;
	}

	// In the parent, run this control in a fresh copy of the current runner and propagate its failure.
	// False means this is the selected child and the caller must execute its real assertions.
	inline bool RunIsolatedDllTest(std::string_view selector, std::string_view diagnostics = {})
	{
		if (DllTestChild == selector)
			return false;

		if (!DllTestChild.empty())
			throw std::logic_error("An isolated DLL control attempted to launch another control");

		// Windows executable paths cannot exceed 32767 UTF-16 code units; reject truncation explicitly.
		auto path = std::wstring(32768, L'\0');
		auto const length = GetModuleFileNameW(nullptr, path.data(), static_cast<DWORD>(path.size()));
		if (length == 0 || length == path.size())
			throw std::runtime_error("Could not obtain the isolated test executable path");

		path.resize(length);
		auto arguments = std::format("-dll-test-child:{} -asserts:stderr -verbose -threads:1", selector);
		if (!diagnostics.empty())
			arguments += std::format(" -process-diagnostics:{}", diagnostics);

		auto wide_arguments = std::wstring(arguments.begin(), arguments.end());
		auto process = Process{};
		auto& startup = process.StartupInfo.StartupInfo;
		startup.dwFlags |= STARTF_USESTDHANDLES;
		startup.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
		startup.hStdOutput = GetStdHandle(STD_OUTPUT_HANDLE);
		startup.hStdError = GetStdHandle(STD_ERROR_HANDLE);
		std::cout << "Isolated DLL control: " << selector << '\n' << std::flush;
		if (!process.Start(path.c_str(), wide_arguments.c_str()))
			throw std::runtime_error(std::format("Could not start isolated DLL control (Win32 error {})", GetLastError()));

		auto const exit_code = process.BlockTillExit();
		if (exit_code != 0)
			throw std::runtime_error(std::format("Isolated DLL control {} failed with exit code {}", selector, exit_code));

		return true;
	}

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
