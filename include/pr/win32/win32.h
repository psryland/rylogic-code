//**********************************************
// Win32 API
//  Copyright (c) Rylogic Ltd 2004
//**********************************************
// Win32 API Unicode independent
//
// _WIN32_WINNT version constants
//    #define _WIN32_WINNT_NT4          0x0400 // Windows NT 4.0
//    #define _WIN32_WINNT_WIN2K        0x0500 // Windows 2000
//    #define _WIN32_WINNT_WINXP        0x0501 // Windows XP
//    #define _WIN32_WINNT_WS03         0x0502 // Windows Server 2003
//    #define _WIN32_WINNT_WIN6         0x0600 // Windows Vista
//    #define _WIN32_WINNT_VISTA        0x0600 // Windows Vista
//    #define _WIN32_WINNT_WS08         0x0600 // Windows Server 2008
//    #define _WIN32_WINNT_LONGHORN     0x0600 // Windows Vista
//    #define _WIN32_WINNT_WIN7         0x0601 // Windows 7
//    #define _WIN32_WINNT_WIN8         0x0602 // Windows 8
//    #define _WIN32_WINNT_WINBLUE      0x0603 // Windows 8.1
//    #define _WIN32_WINNT_WINTHRESHOLD 0x0A00 // Windows 10
//    #define _WIN32_WINNT_WIN10        0x0A00 // Windows 10
#pragma once
#include <array>
#include <string>
#include <string_view>
#include <filesystem>
#include <format>
#include <sdkddkver.h>
#include <windows.h>
#include <knownfolders.h>
#include <shlobj.h>

#include "pr/common/hresult.h"
#include "pr/str/string_core.h"

namespace pr::win32
{
	// "Type traits" for win32 API functions
	template <typename Char> struct Win32;
	template <> struct Win32<char>
	{
		// GetModuleHandleEx
		static HMODULE ModuleHandleEx(DWORD flags, char const* module_name)
		{
			HMODULE library;
			Check(::GetModuleHandleExA(flags, module_name, &library), "GetModuleHandleExW failed");
			return library;
		}

		// ReplaceFile
		static bool FileReplace(char const* replacee, char const* replacer)
		{
			return ::ReplaceFileA(replacee, replacer, nullptr, REPLACEFILE_WRITE_THROUGH | REPLACEFILE_IGNORE_MERGE_ERRORS, nullptr, nullptr) != 0;
		}

		// GetWindowTextLength
		static int WindowTextLength(HWND hwnd)
		{
			return ::GetWindowTextLengthA(hwnd);
		}

		// GetWindowText - use win32::WindowText()
		static int WindowText(HWND hwnd, char* text, DWORD max_length)
		{
			return ::GetWindowTextA(hwnd, text, max_length);
		}
	};
	template <> struct Win32<wchar_t>
	{
		// GetModuleHandleEx
		static HMODULE ModuleHandleEx(DWORD flags, wchar_t const* module_name)
		{
			HMODULE library;
			Check(::GetModuleHandleExW(flags, module_name, &library), "GetModuleHandleExW failed");
			return library;
		}

		// ReplaceFile
		static bool FileReplace(wchar_t const* replacee, wchar_t const* replacer)
		{
			return ::ReplaceFileW(replacee, replacer, nullptr, REPLACEFILE_WRITE_THROUGH | REPLACEFILE_IGNORE_MERGE_ERRORS, nullptr, nullptr) != 0;
		}

		// GetWindowTextLength
		static int WindowTextLength(HWND hwnd)
		{
			return ::GetWindowTextLengthW(hwnd);
		}

		// GetWindowText - use win32::WindowText()
		static int WindowText(HWND hwnd, wchar_t* text, DWORD max_length)
		{
			return ::GetWindowTextW(hwnd, text, max_length);
		}
	};

	// RAII Windows handle
	struct Handle
	{
		struct Closer
		{
			void operator()(HANDLE h)
			{
				if (h == INVALID_HANDLE_VALUE) return;
				if (h == nullptr) return;
				::CloseHandle(h);
			}
		};
		std::unique_ptr<void, Closer> m_handle;

		Handle() noexcept
			:m_handle(nullptr)
		{
		}
		Handle(HANDLE h) noexcept
			:m_handle(h)
		{
		}
		operator HANDLE() const noexcept
		{
			return m_handle.get();
		}
		operator bool() const noexcept
		{
			return m_handle != nullptr && m_handle.get() != INVALID_HANDLE_VALUE;
		}
		bool operator == (HANDLE rhs) const noexcept
		{
			return m_handle.get() == rhs;
		}
		bool operator != (HANDLE rhs) const noexcept
		{
			return m_handle.get() != rhs;
		}
		void close()
		{
			m_handle = nullptr;
		}
	};

	// RAII WaitForSingleObject
	struct WaitForSingleObject
	{
		HANDLE m_handle;
		explicit WaitForSingleObject(HANDLE handle, DWORD timeout = INFINITE)
			:m_handle(handle)
		{
			switch (::WaitForSingleObject(m_handle, timeout))
			{
				case WAIT_OBJECT_0: break;
				case WAIT_ABANDONED: throw std::runtime_error("WaitForSingleObject on destroyed mutex");
				case WAIT_TIMEOUT: throw std::runtime_error("WaitForSingleObject timed out");
				case WAIT_FAILED: throw std::runtime_error(std::format("WaitForSingleObject failed: {}", ::GetLastError()));
			}
		}
		~WaitForSingleObject()
		{
			ReleaseMutex(m_handle);
		}
	};

	// String helper functions
	namespace impl
	{
		inline std::wstring tolower(std::wstring_view s)
		{
			std::wstring r(s);
			std::transform(r.begin(), r.end(), r.begin(), [](auto c) { return static_cast<wchar_t>(::tolower(c)); });
			return r;
		}
		inline std::wstring& replace(std::wstring& input, std::wstring_view pattern, std::wstring_view replace_with)
		{
			for (size_t pos = 0; (pos = input.find(pattern, pos)) != std::wstring::npos;)
			{
				input.replace(pos, pattern.length(), replace_with);
				pos += replace_with.length();
			}
			return input;
		}
	}

	// Windows API 'CreateFile'
	// 'dwDesiredAccess' = e.g. GENERIC_READ
	// 'dwShareMode' = e.g. FILE_SHARE_READ
	// 'dwCreationDisposition' = e.g. OPEN_EXISTING
	inline Handle FileOpen(std::filesystem::path const& filepath, DWORD dwDesiredAccess, DWORD dwShareMode, DWORD dwCreationDisposition, DWORD dwAttributes = FILE_ATTRIBUTE_NORMAL, DWORD dwFlags = 0)
	{
		// Note: don't throw on errors in this function. Leave that to the caller

		// Use LoadLibrary if you want to use CreateFile2 as it doesn't exist on Win7
		//#if (_WIN32_WINNT >= _WIN32_WINNT_WIN8)
		//CREATEFILE2_EXTENDED_PARAMETERS CreateExParams = {sizeof(CREATEFILE2_EXTENDED_PARAMETERS)};
		//CreateExParams.dwFileAttributes = dwAttributes;
		//CreateExParams.dwFileFlags = dwFlags;
		//auto h = CreateFile2(filepath.c_str(), dwDesiredAccess, dwShareMode, dwCreationDisposition, &CreateExParams);
		//#else
		auto h = CreateFileW(filepath.c_str(), dwDesiredAccess, dwShareMode, nullptr, dwCreationDisposition, dwAttributes | dwFlags, nullptr);
		//#endif
		return Handle(h);
	}

	// GetModuleFileName
	static std::filesystem::path ModuleFileName(HMODULE library = nullptr)
	{
		// Get the required buffer size
		wchar_t buf[_MAX_PATH];
		auto len = ::GetModuleFileNameW(library, &buf[0], _countof(buf));
		if (::GetLastError() != ERROR_INSUFFICIENT_BUFFER)
			return std::filesystem::path(&buf[0], &buf[0] + len);

		std::wstring name(len, 0);
		len = ::GetModuleFileNameW(library, &name[0], len);
		Check(len == name.size(), "GetModuleFileNameW failed");
		return std::move(name);
	}

	// Return the name of the currently running executable
	inline std::filesystem::path ExePath()
	{
		return ModuleFileName();
	}
	inline std::filesystem::path ExeDir()
	{
		return ExePath().parent_path();
	}

	// Return the HMODULE for the current process. Note: XP and above only
	inline HMODULE GetCurrentModule()
	{
		static_assert(_WIN32_WINNT >= _WIN32_WINNT_WINXP, "XP and above only");
		return Win32<wchar_t>::ModuleHandleEx(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT, (LPCWSTR)GetCurrentModule);
	}

	// Return the window text for a window (sends WM_GETTEXTLENGTH and WM_GETTEXT)
	template <typename String, typename = std::enable_if_t<is_string_v<String>>>
	inline bool WindowText(HWND hwnd, String& text)
	{
		using Char = typename string_traits<String>::value_type;

		auto len = Win32<Char>::WindowTextLength(hwnd);
		String text(len, 0);
		auto got = Win32<Char>::WindowText(hwnd, &text[0], DWORD(text.size()));
		return len == got;
	}
	template <typename String, typename = std::enable_if_t<is_string_v<String>>>
	inline String WindowText(HWND hwnd)
	{
		String text;
		Throw(WindowText(hwnd, text), "WindowText failed");
		return std::move(text);
	}

	// Retrieves the full path of a known folder identified by the folder's KNOWNFOLDERID.
	// The returned path does not include a trailing backslash. For example, "C:\Users" is returned rather than "C:\Users\".
	inline HRESULT FolderPath(KNOWNFOLDERID const& folder_id, DWORD flags, HANDLE token, std::filesystem::path& path)
	{
		// On return, 'path' contains the address of a pointer to a null-terminated Unicode string that specifies
		// the path of the known folder. The calling process is responsible for freeing this resource once it is no
		// longer needed by calling CoTaskMemFree. 
		wchar_t* p;
		auto hr = ::SHGetKnownFolderPath(folder_id, flags, token, &p);
		if (Failed(hr))
			return hr;

		path = p;
		::CoTaskMemFree(p);
		return hr;
	}
	inline std::filesystem::path FolderPath(KNOWNFOLDERID const& folder_id, DWORD flags, HANDLE token)
	{
		std::filesystem::path path;
		Check(FolderPath(folder_id, flags, token, path), "SHGetKnownFolderPath failed");
		return path;
	}

	// Return the filename for a user settings file
	// Look for a file in the same directory as the running module called 'portable'.
	// If found use the app directory to write settings otherwise use the users local app data directory
	// If the local app data folder is used, 'subdir' creates a subdirectory within that folder, e.g. "\\Rylogic\\MyProgram\\"
	inline std::filesystem::path AppSettingsFilepath(bool portable, std::filesystem::path const& subdir = {})
	{
		using namespace std::filesystem;

		// Determine the directory we're running in
		auto module_path = ModuleFileName();
		auto module_dir = module_path.parent_path();
		auto module_ftitle = module_path.stem();
		auto settings_filename = module_ftitle.replace_extension(L".cfg");

		// Does the file 'portable' exist? If so, return a filepath in the same dir as the module
		if (portable || exists(module_dir / L"portable"))
			return module_dir / settings_filename; // turn .\path\module.exe into .\path\module.cfg for settings

		// Otherwise, return a filepath in the local app data for the current user
		path local_app_data;
		if (Succeeded(FolderPath(FOLDERID_LocalAppData, KF_FLAG_CREATE, nullptr, local_app_data)))
		{
			if (subdir.empty()) local_app_data = local_app_data / L"Rylogic" / module_ftitle / L"";
			else                local_app_data = local_app_data / subdir;
			return local_app_data / settings_filename;
		}

		// Return a filepath in the local directory
		return module_path / settings_filename;
	}

	// Return the HWND for a window by name
	template <typename Char>
	inline HWND WindowByName(Char const* title, bool partial = false)
	{
		struct Data
		{
			HWND                    m_hwnd;
			Char const* m_title;
			size_t                  m_title_len;
			bool                    m_partial;
			std::basic_string<Char> m_name;
		};
		struct CallBacks
		{
			static BOOL CALLBACK EnumWindowsProc(HWND hwnd, LPARAM user_data)
			{
				Data& data = *reinterpret_cast<Data*>(user_data);
				data.m_name = WindowText<std::basic_string<Char>>(hwnd);

				auto match = data.m_partial
					? data.m_name.compare(0, data.m_title_len, data.m_title) == 0
					: data.m_name.compare(data.m_title) == 0;
				if (!match)
					return true;

				data.m_hwnd = hwnd;
				return false;
			}
		};
		Data data = { 0, title, char_traits<Char>::length(title), partial };
		EnumWindows(CallBacks::EnumWindowsProc, (LPARAM)&data);
		return data.m_hwnd;
	}

	// Load a dependent dll
	// 'dir' can use substitution values L".\\lib\\$(platform)\\$(config)\\"
	// 'Context' is used to ensure the module handle for the dll only exists once for each dll
	// Use it like this win32::LoadDll<struct MyDllName>(L"mydll.dll");
	template <typename Context>
	inline HMODULE LoadDll(std::filesystem::path const& dllname, std::wstring dir)
	{
		// Notes:
		//  - If you're using import libraries, you might need to set 'Delay Load Dll' in your project
		//    By default import libraries import the dll before main(). Delay load lets you load the dll
		//    manually from a lib/x64/debug directory rather than in the same directory as the exe.
		//    e.g. <DelayLoadDLLs>view3d-12.dll</DelayLoadDLLs>
		using namespace std::filesystem;

		static HMODULE library = nullptr;
		if (library != nullptr)
			return library;

		// Platform based on pointer size
		constexpr wchar_t const* platform =
			sizeof(void*) == 8 ? L"x64" :
			sizeof(void*) == 4 ? L"x86" :
			L"";

		// NDEBUG is unreliable. Seems it's not always defined in release
#if defined(_DEBUG) || defined(DEBUG)
		constexpr wchar_t const* config = L"debug";
#else
		constexpr wchar_t const* config = L"release";
#endif

		std::string searched;

		// Try the lib folder. Load the appropriate dll for the platform
		impl::replace(dir, L"$(platform)", platform);
		impl::replace(dir, L"$(config)", config);

		auto exe_dir = ExeDir();
		struct PushSearchPaths
		{
			std::vector<DLL_DIRECTORY_COOKIE> m_cookies;
			PushSearchPaths(std::filesystem::path const& dll_path)
				:m_cookies()
			{
				constexpr std::array<std::wstring_view, 5> parents = { L"debug", L"release", L"x64", L"x86", L"lib" };
				constexpr auto is_parent = [&](std::wstring_view s) { return std::find(parents.begin(), parents.end(), s) != parents.end(); };

				// If the parent directories are: debug/release, x64/x86, or lib, add them to the search paths
				auto d = dll_path.parent_path();
				for (; !d.empty() && is_parent(impl::tolower(d.filename().wstring())); d = d.parent_path())
				{
					auto cookie = ::AddDllDirectory(d.c_str());
					if (cookie)
						m_cookies.push_back(cookie);
				}

			}
			~PushSearchPaths()
			{
				for (auto cookie : m_cookies)
					::RemoveDllDirectory(cookie);
			}
		};

		// Try the EXE directory
		auto dllpath = exe_dir / dir / dllname;
		if (exists(dllpath))
		{
			PushSearchPaths users_dirs{ dllpath };
			library = ::LoadLibraryW(dllpath.c_str());
			if (library != nullptr)
				return library;
		}
		else
		{
			searched.append(dllpath.string()).append("\n");
		}

		// Try the local directory
		dllpath = exe_dir / dllname;
		if (exists(dllpath))
		{
			PushSearchPaths users_dirs{ dllpath };
			library = ::LoadLibraryW(dllpath.c_str());
			if (library != nullptr)
				return library;
		}
		else
		{
			searched.append(dllpath.string()).append("\n");
		}

		throw std::runtime_error(std::format("Failed to load dependency '{}'\nSearched:\n{}", dllname.string(), searched));
	}
	template <typename Context>
	inline HMODULE LoadDll(std::filesystem::path const& dllname)
	{
		return LoadDll<Context>(dllname, L".\\lib\\$(platform)\\$(config)");
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::win32
{
	PRUnitTest(Win32Tests)
	{
		auto a = AppSettingsFilepath(false);
	}
}
#endif