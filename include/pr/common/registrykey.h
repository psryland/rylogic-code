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
		RegistryKey(HKEY key, char const* subkey, registry::EAccess access, int reg_option = REG_OPTION_NON_VOLATILE)
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
		static bool Exists(HKEY key, char const* subkey)
		{
			RegistryKey k;
			return k.Open(key, subkey, registry::EAccess::KeyRead);
		}

		// Open the registry key
		// 'key' = the open registry key to open, e.g HKEY_CURRENT_USER
		// 'subkey' = the "subfolders" under 'key'
		// 'access' = the desired access to the key
		bool Open(HKEY key, char const* subkey, registry::EAccess access, int reg_option = REG_OPTION_NON_VOLATILE)
		{
			Close();

			if ((access & registry::EAccess::SetValue) != registry::EAccess(0))
			{
				DWORD was_created;
				m_last_error = RegCreateKeyExA(key, subkey, 0, nullptr, reg_option, REGSAM(access), nullptr, &m_hkey, &was_created);
				m_was_created = was_created == REG_CREATED_NEW_KEY;
			}
			else
			{
				m_last_error = RegOpenKeyExA(key, subkey, 0, REGSAM(access), &m_hkey);
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
		DWORD GetKeyLength(char const* value) const
		{
			if (!m_hkey)
				throw std::exception("RegKey invalid");

			auto length = DWORD{};
			Check(RegQueryValueExA(m_hkey, value, nullptr, nullptr, nullptr, &length), "failed to read registry value data length");
			return length;
		}

		// Returns true if 'value' exists in the current key and is of type 'data_type'
		bool HasValue(char const* value, DWORD data_type) const
		{
			DWORD type;
			return RegQueryValueExA(m_hkey, value, nullptr, &type, nullptr, nullptr) == ERROR_SUCCESS && type == data_type;
		}
		bool HasValue(char const* value) const
		{
			return RegQueryValueExA(m_hkey, value, nullptr, nullptr, nullptr, nullptr) == ERROR_SUCCESS;
		}

		// Read/Write raw data from/to the registry key.
		// 'value' is the registry value to read. If null or "" then the unnamed default value is read.
		// 'data' is the location to fill with data from the registry
		// 'length' is the size in bytes of the memory pointed to by 'data'
		// 'subkey' is an option subfolder from the opened key
		// 'data_type' is the data type of the value to read (default is REG_BINARY)
		void Read(char const* value, void* data, DWORD length, DWORD data_type = REG_BINARY) const
		{
			if (!m_hkey) throw std::exception("RegKey invalid");
			Check(RegQueryValueExA(m_hkey, value, nullptr, &data_type, (BYTE*)data, &length), "failed to read registry value");
		}
		void Write(char const* value, void const* data, DWORD length, int data_type = REG_BINARY)
		{
			if (!m_hkey) throw std::exception("RegKey invalid");
			Check(RegSetValueExA(m_hkey, value, 0, data_type, (BYTE const*)data, length), "failed to write registry value");
		}

		// Read/Write a value from/to the registry as a POD type
		template <typename T, typename = std::enable_if_t<std::is_trivially_copyable_v<T>>>
		T Read(char const* value, int data_type) const
		{
			auto data = T{};
			Read(value, &data, DWORD(sizeof(data)), data_type);
			return data;
		}
		template <typename T, typename = std::enable_if_t<std::is_trivially_copyable_v<T>>>
		T Read(char const* value) const
		{
			return Read<T>(value, REG_BINARY);
		}

		// Read/Write a string from/to the registry key.
		std::string Read(char const* value) const
		{
			if (!m_hkey)
				throw std::exception("RegKey invalid");

			auto len = GetKeyLength(value);
			if (len == 0)
				return std::string{};

			// The stored data may or may not include a null terminator, so trim any trailing nulls after reading.
			auto type = DWORD{REG_SZ};
			auto str = std::string(len, '\0');
			Check(RegQueryValueExA(m_hkey, value, nullptr, &type, (BYTE*)str.data(), &len), "failed to read registry value");
			str.resize(len);

			for (;!str.empty() && str.back() == 0; str.pop_back()) {}
			return str;
		}
		void Write(char const* value, std::string const& data)
		{
			// REG_SZ data should include the null terminator
			Write(value, data.c_str(), DWORD(data.size() + 1), REG_SZ);
		}
		void Write(char const* value, char const* data)
		{
			// REG_SZ data should include the null terminator
			Write(value, data, DWORD(strlen(data) + 1), REG_SZ);
		}

		// Read/Write a DWORD from/to the registry key.
		template <> DWORD Read<DWORD>(char const* value) const
		{
			return Read<DWORD>(value, REG_DWORD);
		}
		void Write(char const* value, unsigned long data)
		{
			Write(value, &data, sizeof(data), REG_DWORD);
		}
		void Write(char const* value, unsigned int data)
		{
			Write(value, &data, sizeof(data), REG_DWORD);
		}
		void Write(char const* value, long data)
		{
			Write(value, &data, sizeof(data), REG_DWORD);
		}
		void Write(char const* value, int data)
		{
			Write(value, &data, sizeof(data), REG_DWORD);
		}

		// Read/Write a boolean flag from/to the registry key.
		template <> bool Read<bool>(char const* value) const
		{
			return Read<DWORD>(value) != 0;
		}
		void Write(char const* value, bool data)
		{
			auto d = data ? DWORD(1) : DWORD(0);
			Write(value, &d, sizeof(d), REG_DWORD);
		}

		// Read/Write a floating point value from/to the registry key.
		template <> double Read<double>(char const* value) const
		{
			auto s = Read(value);
			return std::stod(s);
		}
		void Write(char const* value, double data)
		{
			auto str = std::to_string(data);
			Write(value, str);
		}

		// Delete a value from the currently open registry key
		void DeleteValue(char const* value)
		{
			Check(RegDeleteKeyValueA(m_hkey, nullptr, value), "failed to delete registry value");
		}

		// Delete a registry key.
		static void Delete(HKEY hkey, char const* subkey)
		{
			// This is not a member because 'subkey' is a required parameter
			// and I don't want to have to store 'subkey' in this class
			// This is a better fit for typical usage as well, why would
			// you want to open a subkey only to delete it?
			if (RegDeleteKeyA(hkey, subkey) != ERROR_SUCCESS)
				throw std::exception("failed to delete registry key");
		}

		// Delete the currently open registry key and all subkeys
		static std::enable_if<_WIN32_WINNT >= 0x0600, void>::type DeleteTree(HKEY hkey, char const* subkey)
		{
			if (RegDeleteTreeA(hkey, subkey) != ERROR_SUCCESS)
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
			rkey.Write("Unterminated", "XYZ", 3, REG_SZ);
		}
		{// Check values exist
			auto rkey = pr::RegistryKey(HKEY_CURRENT_USER, subkey, pr::registry::EAccess::KeyRead);

			PR_EXPECT(rkey.HasValue("String"));
			PR_EXPECT(rkey.HasValue("DWord"));
			PR_EXPECT(rkey.HasValue("Double"));
			PR_EXPECT(rkey.HasValue("Blob"));

			// Read the values
			PR_EXPECT(rkey.Read("String") == "Paul Was Here");
			PR_EXPECT(rkey.Read("Unterminated") == "XYZ");
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
			rkey.DeleteValue("Unterminated");
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