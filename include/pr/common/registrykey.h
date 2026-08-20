// **********************************************************************
// Registry key
//  Copyright (c) Rylogic Ltd 2009
// **********************************************************************

#pragma once
#include <string>
#include <exception>
#include <windows.h>

namespace pr
{
	namespace registry
	{
		enum EAccess
		{
			QueryValue       = KEY_QUERY_VALUE,        // (0x0001)
			SetValue         = KEY_SET_VALUE,          // (0x0002)
			CreateSubKey     = KEY_CREATE_SUB_KEY,     // (0x0004)
			EnumerateSubKeys = KEY_ENUMERATE_SUB_KEYS, // (0x0008)
			Notify           = KEY_NOTIFY,             // (0x0010)
			CreateLink       = KEY_CREATE_LINK,        // (0x0020)
			WOW64_32Key      = KEY_WOW64_32KEY,        // (0x0200)
			WOW64_64Key      = KEY_WOW64_64KEY,        // (0x0100)
			WOW64_Res        = KEY_WOW64_RES,          // (0x0300)
			KeyRead          = KEY_READ,               // ((STANDARD_RIGHTS_READ|KEY_QUERY_VALUE|KEY_ENUMERATE_SUB_KEYS|KEY_NOTIFY)&(~SYNCHRONIZE))
			KeyWrite         = KEY_WRITE,              // ((STANDARD_RIGHTS_WRITE|KEY_SET_VALUE|KEY_CREATE_SUB_KEY)&(~SYNCHRONIZE))
			KeyExecute       = KEY_EXECUTE,            // ((KEY_READ)&(~SYNCHRONIZE))
			AllAccess        = KEY_ALL_ACCESS,         // ((STANDARD_RIGHTS_ALL|KEY_QUERY_VALUE|KEY_SET_VALUE|KEY_CREATE_SUB_KEY|KEY_ENUMERATE_SUB_KEYS|KEY_NOTIFY|KEY_CREATE_LINK)&(~SYNCHRONIZE))
		};
		inline EAccess operator | (EAccess lhs, EAccess rhs) { return EAccess(int(lhs) | int(rhs)); }
		inline EAccess operator & (EAccess lhs, EAccess rhs) { return EAccess(int(lhs) & int(rhs)); }
	}

	// The Key. Note: to nest keys pass this object to the 'Open' method (3rd parameter)
	class RegistryKey
	{
		HKEY m_hkey;
		mutable DWORD m_last_error;
		bool m_was_created;

		// Check a return code and throw on error
		void Check(DWORD res, char const* error_msg) const
		{
			if (res == ERROR_SUCCESS) return;
			m_last_error = res;

			char m[256];
			FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM|FORMAT_MESSAGE_IGNORE_INSERTS, NULL, m_last_error, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), m, sizeof(m), NULL);
			auto msg = std::string(error_msg);
			if (!msg.empty() && *--msg.end() != '\n') msg.append("\n");
			msg.append(m);
			throw std::exception(msg.c_str());
		}

	public:

		~RegistryKey()
		{
			Close();
		}
		RegistryKey()
			:m_hkey(nullptr)
			,m_last_error(ERROR_SUCCESS)
			,m_was_created(false)
		{}
		RegistryKey(HKEY key, TCHAR const* subkey, registry::EAccess access, int reg_option = REG_OPTION_NON_VOLATILE)
			:RegistryKey()
		{
			if (!Open(key, subkey, access, reg_option))
				Check(m_last_error, "Registry key could not be opened");
		}
		RegistryKey(RegistryKey const&) = delete;
		RegistryKey(RegistryKey&& rhs)
			:m_hkey(rhs.m_hkey)
			,m_last_error(rhs.m_last_error)
			,m_was_created(rhs.m_was_created)
		{
			rhs.m_hkey = nullptr;
			rhs.m_last_error = 0;
			rhs.m_was_created = false;
		}
		RegistryKey& operator = (RegistryKey const&) = delete;
		RegistryKey& operator = (RegistryKey&& rhs)
		{
			if (this == &rhs) return *this;
			std::swap(m_hkey, rhs.m_hkey);
			std::swap(m_last_error, rhs.m_last_error);
			std::swap(m_was_created, rhs.m_was_created);
			return *this;
		}

		operator HKEY () { return m_hkey; }

		// Returns true if a given key exists
		static bool Exists(HKEY key, TCHAR const* subkey)
		{
			RegistryKey k;
			return k.Open(key, subkey, registry::EAccess::KeyRead);
		}

		// Open the registry key
		// 'key' = the open registry key to open, e.g HKEY_CURRENT_USER
		// 'subkey' = the "subfolders" under 'key'
		// 'access' = the desired access to the key
		// If unicode is used, 'subkey' must be aligned.
		bool Open(HKEY key, TCHAR const* subkey, registry::EAccess access, int reg_option = REG_OPTION_NON_VOLATILE)
		{
			Close();

			if ((access & registry::EAccess::SetValue) != registry::EAccess(0))
			{
				DWORD was_created;
				m_last_error = RegCreateKeyEx(key, subkey, 0, nullptr, reg_option, REGSAM(access), nullptr, &m_hkey, &was_created);
				m_was_created = was_created == REG_CREATED_NEW_KEY;
			}
			else
			{
				m_last_error = RegOpenKeyEx(key, subkey, 0, REGSAM(access), &m_hkey);
				m_was_created = false;
			}
			return m_last_error == ERROR_SUCCESS;
		}

		// Close the registry key
		void Close()
		{
			if (m_hkey) RegCloseKey(m_hkey);
			m_hkey = nullptr;
		}

		// Returns the length of a registry value in bytes. If the value type is a string,
		// this method returns the number of characters it contains (including the terminating
		// null character). Returns 0 if the value doesn't exist.
		DWORD GetKeyLength(TCHAR const* value) const
		{
			if (!m_hkey)
				throw std::exception("RegKey invalid");

			auto length = DWORD{};
			Check(RegQueryValueEx(m_hkey, value, nullptr, nullptr, nullptr, &length), "failed to read registry value data length");
			return length;
		}

		// Returns true if 'value' exists in the current key and is of type 'data_type'
		bool HasValue(TCHAR const* value, DWORD data_type) const
		{
			DWORD type;
			return RegQueryValueEx(m_hkey, value, nullptr, &type, nullptr, nullptr) == ERROR_SUCCESS && type == data_type;
		}
		bool HasValue(TCHAR const* value) const
		{
			return RegQueryValueEx(m_hkey, value, nullptr, nullptr, nullptr, nullptr) == ERROR_SUCCESS;
		}

		// Read/Write raw data from/to the registry key.
		// 'value' is the registry value to read. If null or "" then the unnamed default value is read.
		// 'data' is the location to fill with data from the registry
		// 'length' is the size in bytes of the memory pointed to by 'data'
		// 'subkey' is an option subfolder from the opened key
		// 'data_type' is the data type of the value to read (default is REG_BINARY)
		void Read(TCHAR const* value, void* data, DWORD length, DWORD data_type = REG_BINARY) const
		{
			if (!m_hkey) throw std::exception("RegKey invalid");
			Check(RegQueryValueEx(m_hkey, value, nullptr, &data_type, (BYTE*)data, &length), "failed to read registry value");
		}
		void Write(TCHAR const* value, void const* data, DWORD length, int data_type = REG_BINARY)
		{
			if (!m_hkey) throw std::exception("RegKey invalid");
			Check(RegSetValueEx(m_hkey, value, 0, data_type, (BYTE const*)data, length), "failed to read registry value");
		}

		// Read/Write a value from/to the registry as a POD type
		template <typename T, typename = std::enable_if_t<std::is_trivially_copyable_v<T>>>
		T Read(TCHAR const* value, int data_type) const
		{
			auto data = T{};
			Read(value, &data, DWORD(sizeof(data)), data_type);
			return data;
		}
		template <typename T, typename = std::enable_if_t<std::is_trivially_copyable_v<T>>>
		T Read(TCHAR const* value) const
		{
			return Read<T>(value, REG_BINARY);
		}

		// Read/Write a string from/to the registry key.
		std::string Read(TCHAR const* value) const
		{
			if (!m_hkey)
				throw std::exception("RegKey invalid");

			auto len = GetKeyLength(value);
			if (len == 0)
				return std::string{};

			auto type = DWORD{REG_SZ};
			if (len < 1024)
			{
				char str[1024];
				Check(RegQueryValueEx(m_hkey, value, nullptr, &type, (BYTE*)&str[0], &len), "failed to read registry value");
				return str;
			}
			else
			{
				auto str = std::string{};
				str.resize(len);
				Check(RegQueryValueEx(m_hkey, value, nullptr, &type, (BYTE*)&str[0], &len), "failed to read registry value");

				for (;!str.empty() && *--str.end() == 0; str.resize(str.size() - 1)) {} // rtrim null terminator
				return str;
			}
		}
		void Write(TCHAR const* value, std::string const& data)
		{
			Write(value, data.c_str(), DWORD(data.size()), REG_SZ);
		}
		void Write(TCHAR const* value, char const* data)
		{
			Write(value, data, DWORD(strlen(data)), REG_SZ);
		}

		// Read/Write a DWORD from/to the registry key.
		template <> DWORD Read<DWORD>(TCHAR const* value) const
		{
			return Read<DWORD>(value, REG_DWORD);
		}
		void Write(TCHAR const* value, unsigned long data)
		{
			Write(value, &data, sizeof(data), REG_DWORD);
		}
		void Write(TCHAR const* value, unsigned int data)
		{
			Write(value, &data, sizeof(data), REG_DWORD);
		}
		void Write(TCHAR const* value, long data)
		{
			Write(value, &data, sizeof(data), REG_DWORD);
		}
		void Write(TCHAR const* value, int data)
		{
			Write(value, &data, sizeof(data), REG_DWORD);
		}

		// Read/Write a boolean flag from/to the registry key.
		template <> bool Read<bool>(TCHAR const* value) const
		{
			return Read<DWORD>(value) != 0;
		}
		void Write(TCHAR const* value, bool data)
		{
			auto d = data ? DWORD(1) : DWORD(0);
			Write(value, &d, sizeof(d), REG_DWORD);
		}

		// Read/Write a floating point value from/to the registry key.
		template <> double Read<double>(TCHAR const* value) const
		{
			auto s = Read(value);
			return std::stod(s);
		}
		void Write(TCHAR const* value, double data)
		{
			auto str = std::to_string(data);
			Write(value, str);
		}

		// Delete a value from the currently open registry key
		void DeleteValue(TCHAR const* value)
		{
			Check(RegDeleteKeyValue(m_hkey, nullptr, value), "failed to delete registry value");
		}

		// Delete a registry key.
		static void Delete(HKEY hkey, TCHAR const* subkey)
		{
			// This is not a member because 'subkey' is a required parameter
			// and I don't want to have to store 'subkey' in this class
			// This is a better fit for typical usage as well, why would
			// you want to open a subkey only to delete it?
			if (RegDeleteKey(hkey, subkey) != ERROR_SUCCESS)
				throw std::exception("failed to delete registry key");
		}

		// Delete the currently open registry key and all subkeys
		static std::enable_if<_WIN32_WINNT >= 0x0600, void>::type DeleteTree(HKEY hkey, TCHAR const* subkey)
		{
			if (RegDeleteTree(hkey, subkey) != ERROR_SUCCESS)
				throw std::exception("failed to delete registry key and subkeys");
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(RegistryKeyTests, Quick)
	{
		char const* subkey = "Software\\Rylogic\\unittest\\";

		{// Create a dummy key
			auto rkey = pr::RegistryKey(HKEY_CURRENT_USER, subkey, pr::registry::EAccess::KeyWrite);

			// Write some values
			rkey.Write("String", "Paul Was Here");
			rkey.Write("DWord", 1234U);
			rkey.Write("Double", 3.14);
			rkey.Write("Blob", "ABCD", sizeof("ABCD"), REG_BINARY);
		}
		{// Check values exist
			auto rkey = pr::RegistryKey(HKEY_CURRENT_USER, subkey, pr::registry::EAccess::KeyRead);

			PR_EXPECT(rkey.HasValue("String"));
			PR_EXPECT(rkey.HasValue("DWord"));
			PR_EXPECT(rkey.HasValue("Double"));
			PR_EXPECT(rkey.HasValue("Blob"));

			// Read the values
			PR_EXPECT(rkey.Read("String") == "Paul Was Here");
			PR_EXPECT(rkey.Read<DWORD>("DWord") == 1234U);
			PR_EXPECT(FEql(rkey.Read<double>("Double"), 3.14));

			char blob[5];
			rkey.Read("Blob", blob, sizeof(blob), REG_BINARY);
			PR_EXPECT(memcmp(blob, "ABCD", 4) == 0);
		}
		{// Delete the values
			auto rkey = pr::RegistryKey(HKEY_CURRENT_USER, subkey, pr::registry::EAccess::KeyWrite);

			rkey.DeleteValue("String");
			rkey.DeleteValue("DWord");
			rkey.DeleteValue("Double");
			rkey.DeleteValue("Blob");
		}
		{// Check values deleted
			auto rkey = pr::RegistryKey(HKEY_CURRENT_USER, subkey, pr::registry::EAccess::KeyRead);

			PR_EXPECT(!rkey.HasValue("String"));
			PR_EXPECT(!rkey.HasValue("DWord"));
			PR_EXPECT(!rkey.HasValue("Double"));
			PR_EXPECT(!rkey.HasValue("Blob"));
		}
		{// Delete the key
			pr::RegistryKey::Delete(HKEY_CURRENT_USER, subkey);
			PR_EXPECT(!pr::RegistryKey::Exists(HKEY_CURRENT_USER, subkey));
		}
	}
}
#endif