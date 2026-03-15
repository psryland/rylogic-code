//**********************************************************************************
// Resource
//  Copyright (c) Rylogic Ltd 2009
//**********************************************************************************
// Access data embedded in an EXE file
//
// To add resources to an EXE, add a '.rc' file to the project (create a text file with
//  .rc extn, and 'Add existing'. Or use the File->New->Resource->Resource File to have
//  IDE resource editor work)
//
// Add entries in the resource file like this:
//  // name         type       legacy-ignored    filename
//  IDR_EXAMPLE0    TEXT       DISCARDABLE       "test.cpp"
//  IDR_EXAMPLE1    BINARY     DISCARDABLE       "test.cpp"
//
// Call this helper function like this:
//  pr::Resource<char> res = pr::resource::Read<char>(L"IDR_EXAMPLE0", L"TEXT");
//  res.m_data;
//  res.m_size;
//
// Note: you only need a "resource.h" file containing #define's if you want to use
// 'IDR_EXAMPLE0' as an int. In this case you need to call:
//     pr::resource::Read<char>(MAKEINTRESOURCE(IDR_EXAMPLE0), "TEXT");
//
#pragma once
#include <string>
#include <string_view>
#include <windows.h>

namespace pr
{
	template <typename Type>
	struct Resource
	{
		union
		{
			Type const* m_data; // Pointer to the resource
			char const* m_buf;
		};
		size_t m_len;  // The length of the resource in 'Type's
		
		Resource()
			:m_data()
			,m_len()
		{}
		Resource(Type const* data, size_t len)
			:m_data(data)
			,m_len(len)
		{}
		size_t size() const
		{
			return m_len * sizeof(Type);
		}
		operator std::string_view() const
		{
			return std::string_view{ m_buf, m_len };
		}
	};

	namespace resource
	{
		namespace impl
		{
			inline std::string Narrow(std::wstring_view ws)
			{
				if (ws.empty()) return {};
				auto len = ::WideCharToMultiByte(CP_UTF8, 0, ws.data(), static_cast<int>(ws.size()), nullptr, 0, nullptr, nullptr);
				std::string out(len, '\0');
				::WideCharToMultiByte(CP_UTF8, 0, ws.data(), static_cast<int>(ws.size()), out.data(), len, nullptr, nullptr);
				return out;
			}
		}

		// Check for the existence of a resource named 'name'. module = 0 means 'this exe'
		// If you're resource is in a dll, you need to use the HMODULE passed to the DllMain function.
		// Note: you can use pr::GetCurrentModule() for 'module'
		inline bool Find(std::wstring_view name, wchar_t const* type, HMODULE module = nullptr)
		{
			// Notes:
			//  - Using raw pointer for 'type' because it's typically a literal. e.g. "TEXT"

			// Get a handle to the resource
			auto handle = ::FindResourceW(module, std::wstring(name).c_str(), type);
			if (handle != nullptr)
				return true;

			// No handle? Check the error was data not found
			auto last_error = GetLastError();
			if (last_error == ERROR_RESOURCE_NAME_NOT_FOUND)
				return false;

			// Throw for other errors
			throw std::runtime_error(std::format("Resource '{}' not found. (0x{:08X})", impl::Narrow(name), last_error));
		}

		// Return const access to an embedded resource
		// module = 0 means 'this exe'
		// If you're resource is in a dll, you need to use the HMODULE passed to the DllMain function
		// Note: you can use pr::GetCurrentModule() for 'module'
		template <typename Type>
		Resource<Type> Read(std::wstring_view name, wchar_t const* type, HMODULE module = nullptr)
		{
			// Get a handle to the resource
			auto handle = ::FindResourceW(module, std::wstring(name).c_str(), type);
			if (!handle)
			{
				auto last_error = ::GetLastError();
				throw std::runtime_error(std::format("Resource '{}' not found. (0x{:08X})", impl::Narrow(name), last_error));
			}

			// Get the size in bytes of the resource
			auto size = ::SizeofResource(module, handle);

			// Load the resource into memory
			// The return type of LoadResource is HGLOBAL for backward compatibility, not because the function returns
			// a handle to a global memory block. Do not pass this handle to the GlobalLock or GlobalFree function. 
			auto mem = ::LoadResource(module, handle);
			if (!mem)
			{
				auto last_error = GetLastError();
				throw std::runtime_error(std::format("Loading resource '{}' failed. (0x{:08X})", impl::Narrow(name), last_error));
			}

			// Get a pointer to the resource.
			// Valid until the module is unloaded therefore don't need to worry about unlocking (according to MSDN)
			auto data = static_cast<Type const*>(::LockResource(mem));
			return Resource<Type>(data, size / sizeof(Type));
		}
	}
}
