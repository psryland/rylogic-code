//*********************************
// GUID helper functions
//  Copyright (c) Rylogic Ltd 2006
//*********************************

#pragma once

#include <type_traits>
#include <windows.h>
#include <guiddef.h>
#include <objbase.h>
#include "pr/common/to.h"
#include "pr/common/hash.h"
#include "pr/common/scope.h"
#include "pr/common/hresult.h"
#include "pr/str/string_core.h"
#include "pr/str/string.h"
#include "pr/crypt/md5.h"
#include "pr/crypt/sha1.h"

// Required lib: rpcrt4.lib
#pragma comment(lib, "rpcrt4.lib")

namespace pr
{
	using Guid = GUID;

	// Constants
	constexpr Guid GuidZero    = { 0x00000000, 0x0000, 0x0000, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	constexpr Guid GuidInvalid = { 0x00000000, 0x0000, 0x0000, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };

	// The namespace for fully-qualified domain names (from RFC 4122, Appendix C).
	constexpr Guid GuidDnsNamespace = { 0x6ba7b810, 0x9dad, 0x11d1, 0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8 };

	// The namespace for URLs (from RFC 4122, Appendix C).
	constexpr Guid GuidUrlNamespace = { 0x6ba7b811, 0x9dad, 0x11d1, 0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8 };

	// The namespace for ISO OIDs (from RFC 4122, Appendix C).
	constexpr Guid GuidIsoOidNamespace = { 0x6ba7b812, 0x9dad, 0x11d1, 0x80, 0xb4, 0x00, 0xc0, 0x4f, 0xd4, 0x30, 0xc8 };

	// GUIDs from names
	enum class EGuidVersion { MD5Hashing = 3, SHA1Hashing = 5 };

	// Create a new GUID
	inline Guid GenerateGUID()
	{
		Guid guid;
		Check(CoCreateGuid(&guid), "Failed to create GUID");
		return guid;
	}

	// Creates a name-based UUID using the algorithm from RFC 4122 section 4.3.
	// namespaceId - The ID of the namespace.
	// name - The name (within that namespace).
	// version - The version number of the UUID to create; this value must be either 3 (for MD5 hashing) or 5 (for SHA-1 hashing).
	// Returns a UUID derived from the namespace and name.
	// See <a href="http://code.logos.com/blog/2011/04/generating_a_deterministic_guid.html">Generating a deterministic GUID</a>
	inline Guid GenerateGUID(Guid namespace_id, char const* name, EGuidVersion version = EGuidVersion::SHA1Hashing)
	{
		using byte = unsigned char;

		if (name == nullptr)
			throw std::exception("name cannot be null");
		if (version != EGuidVersion::MD5Hashing && version != EGuidVersion::SHA1Hashing)
			throw std::exception("version must be either 3 or 5.");

		// Converts a GUID (expressed as a byte array) to/from network order (MSB-first).
		auto SwapByteOrder = [](byte* guid)
		{
			std::swap(guid[0], guid[3]);
			std::swap(guid[1], guid[2]);
			std::swap(guid[4], guid[5]);
			std::swap(guid[6], guid[7]);
		};

		// Convert the name to a sequence of octets (as defined by the standard or conventions of its namespace) (step 3)
		// ASSUME: UTF-8 encoding is always appropriate
		auto name_bytes = reinterpret_cast<byte const*>(name);
		auto name_bytes_len = int(strlen(name));

		// Convert the namespace UUID to network order (step 3)
		auto namespace_bytes = reinterpret_cast<unsigned char*>(&namespace_id);
		auto namespace_bytes_len = int(sizeof(GUID));
		SwapByteOrder(namespace_bytes);

		byte new_guid[sizeof(GUID)];

		// Compute the hash of the name space ID concatenated with the name (step 4)
		switch (version)
		{
			case EGuidVersion::MD5Hashing:
			{
				pr::hash::MD5 md5;
				md5.Add(namespace_bytes, namespace_bytes_len);
				md5.Add(name_bytes, name_bytes_len);
				auto hash = md5.Final();

				// Most bytes from the hash are copied straight to the bytes of the new GUID (steps 5-7, 9, 11-12)
				memcpy(&new_guid, &hash[0], 16);
				break;
			}
			case EGuidVersion::SHA1Hashing:
			{
				pr::hash::SHA1 sha1;
				sha1.Update(namespace_bytes, namespace_bytes_len);
				sha1.Update(name_bytes, name_bytes_len);
				auto hash = sha1.Final();

				// Most bytes from the hash are copied straight to the bytes of the new GUID (steps 5-7, 9, 11-12)
				memcpy(&new_guid, &hash[0], 16);
				break;
			}
			default:
			{
				throw std::runtime_error("Unknown name Guid version");
			}
		}

		// Set the four most significant bits (bits 12 through 15) of the 'time_hi_and_version' field to
		// the appropriate 4-bit version number from Section 4.1.3 (step 8)
		new_guid[6] = byte((new_guid[6] & 0x0F) | (byte(version) << 4));

		// Set the two most significant bits (bits 6 and 7) of the 'clock_seq_hi_and_reserved' to zero
		// and one, respectively (step 10)
		new_guid[8] = byte((new_guid[8] & 0x3F) | 0x80);

		// Convert the resulting UUID to local byte order (step 13)
		SwapByteOrder(new_guid);

		return *reinterpret_cast<GUID*>(&new_guid);
	}

	// Conversion
	namespace convert
	{
		// GUID to string
		template <typename Str>
		struct GuidToString
		{
			using Char = typename string_traits<Str>::value_type;
			static Str Func(GUID const& guid)
			{
				if constexpr (std::is_same_v<Char, char>)
				{
					RPC_CSTR str = nullptr;
					auto s = Scope<void>([&] { (void)::UuidToStringA(static_cast<UUID const*>(&guid), &str); }, [&] { ::RpcStringFreeA(&str); });
					return Str(reinterpret_cast<char const*>(str));
				}
				if constexpr (std::is_same_v<Char, wchar_t>)
				{
					RPC_WSTR str = nullptr;
					auto s = Scope<void>([&]{ (void)::UuidToStringW(static_cast<UUID const*>(&guid), &str); }, [&]{ ::RpcStringFreeW(&str); });
					return Str(reinterpret_cast<wchar_t const*>(str));
				}
			}
		};

		// string to GUID
		struct StringToGuid
		{
			template <typename Str, typename = std::enable_if_t<is_string_v<Str>>>
			static GUID Func(Str const& s)
			{
				using Char = typename string_traits<Str>::value_type;

				if constexpr (std::is_same_v<Char, char>)
				{
					GUID guid;
					auto r = ::UuidFromStringA(RPC_CSTR(string_traits<Str>::ptr(s)), &guid);
					if (r != RPC_S_OK) throw std::runtime_error("GUID string is invalid");
					return guid;
				}
				if constexpr (std::is_same_v<Char, wchar_t>)
				{
					GUID guid;
					auto r = ::UuidFromStringW(RPC_WSTR(string_traits<Str>::ptr(s)), &guid);
					if (r != RPC_S_OK) throw std::runtime_error("GUID string is invalid");
					return guid;
				}
			}
		};
	}

	// GUID to std::basic_string
	template <typename Char> struct Convert<std::basic_string<Char>, Guid> :convert::GuidToString<std::basic_string<Char>>
	{};

	// GUID to pr::string
	template <typename Char, int L, bool F> struct Convert<pr::string<Char, L, F>, Guid> :convert::GuidToString<pr::string<Char, L, F>>
	{};

	// Whatever to GUID
	template <typename TFrom> struct Convert<Guid, TFrom> :convert::StringToGuid
	{};
}

// Operators (==, != already defined)
namespace impl
{
	constexpr bool GuidLess(unsigned long const* lhs, unsigned long const* rhs)
	{
		return
			lhs[0] != rhs[0] ? lhs[0] < rhs[0] :
			lhs[1] != rhs[1] ? lhs[1] < rhs[1] :
			lhs[2] != rhs[2] ? lhs[2] < rhs[2] :
			lhs[3] != rhs[3] ? lhs[3] < rhs[3] :
			false;
	}
}
constexpr bool operator <  (GUID const& lhs, GUID const& rhs)
{
	return impl::GuidLess(reinterpret_cast<unsigned long const*>(&lhs), reinterpret_cast<unsigned long const*>(&rhs));
}
constexpr bool operator >  (GUID const& lhs, GUID const& rhs)
{
	return rhs < lhs;
}
constexpr bool operator <= (GUID const& lhs, GUID const& rhs)
{
	return !(rhs < lhs);
}
constexpr bool operator >= (GUID const& lhs, GUID const& rhs)
{
	return !(lhs < rhs);
}

namespace std
{
	template <> struct hash<GUID>
	{
		size_t operator()(GUID const& guid) const noexcept
		{
			static_assert(sizeof(guid) == 4 * sizeof(std::uint32_t), "GUID size != 4 * sizeof(u32)");
			static_assert(alignment_of<GUID>::value >= alignment_of<std::uint32_t>::value, "GUID alignment != u32 alignment");
			
			hash<uint32_t> h32;
			return
				h32(reinterpret_cast<uint32_t const*>(&guid)[0]) ^
				h32(reinterpret_cast<uint32_t const*>(&guid)[1]) ^
				h32(reinterpret_cast<uint32_t const*>(&guid)[2]) ^
				h32(reinterpret_cast<uint32_t const*>(&guid)[3]);
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(GuidTests)
	{
		PR_EXPECT(To<std::string>(GuidInvalid) == "00000000-0000-0000-0000-000000000000");
		PR_EXPECT(To<std::wstring>(GuidInvalid) == L"00000000-0000-0000-0000-000000000000");
		PR_EXPECT(To<Guid>("00000000-0000-0000-0000-000000000000") == GuidInvalid);
		PR_EXPECT(To<Guid>(L"00000000-0000-0000-0000-000000000000") == GuidZero);
	}
}
#endif
