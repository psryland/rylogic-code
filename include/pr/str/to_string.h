//***********************************************************************
// ToString functions
//  Copyright (c) Rylogic Ltd 2008
//***********************************************************************

#pragma once

#include <string>
#include <sstream>
#include <clocale>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <locale>
#include <vector>
#include "pr/common/to.h"
#include "pr/common/hresult.h"
#include "pr/str/string_core.h"
#include "pr/str/string.h"
#include "pr/str/string_util.h"

namespace pr
{
	// Bool
	template <StringType Str>
	struct Convert<Str, bool>
	{
		static Str Func(bool from)
		{
			using Char = typename string_traits<Str>::value_type;
			if constexpr (std::is_same_v<Char, char>)
				return from ? "true" : "false";
			if constexpr (std::is_same_v<Char, wchar_t>)
				return from ? L"true" : L"false";
		}
	};
	template <StringType Str>
	struct Convert<bool, Str>
	{
		static bool Func(Str const& from)
		{
			return str::CompareI(from, "true") == 0 || str::CompareI(from, "1") == 0;
		}
	};

	// Char
	template <StringType Str>
	struct Convert<Str, char>
	{
		static Str Func(char from)
		{
			Str str;
			string_traits<Str>::resize(str, 1);
			str[0] = from;
			return str;
		}
	};

	// Integral types
	template <StringTypeDynamic Str, std::integral Intg>
	struct Convert<Str, Intg>
	{
		static Str Func(Intg from, int radix)
		{
			using Char = typename string_traits<Str>::value_type;
			Char buf[128];
			if constexpr (std::is_unsigned_v<Intg>)
			{
				return char_traits<Char>::uitostr(from, buf, _countof(buf), radix);
			}
			else
			{
				return char_traits<Char>::itostr(from, buf, _countof(buf), radix);
			}
		}
		static Str Func(Intg from)
		{
			return Func(from, 10);
		}
	};
	template <std::integral Intg, StringType Str>
	struct Convert<Intg, Str>
	{
		using Char = typename string_traits<Str>::value_type;
		static Intg Func(Str const& s, int radix = 10, Char const** end = nullptr)
		{
			auto ptr = string_traits<Str>::ptr(s);
			errno = 0;

			if constexpr (std::is_signed_v<Intg> && sizeof(Intg) <= sizeof(long))
				return CheckErrno(static_cast<Intg>(char_traits<Char>::strtol(ptr, end, radix)));
			if constexpr (!std::is_signed_v<Intg> && sizeof(Intg) <= sizeof(long))
				return CheckErrno(static_cast<Intg>(char_traits<Char>::strtoul(ptr, end, radix) & ~Intg()));
			if constexpr (std::is_signed_v<Intg> && sizeof(Intg) > sizeof(long))
				return CheckErrno(static_cast<Intg>(char_traits<Char>::strtoll(ptr, end, radix) & ~Intg()));
			if constexpr (!std::is_signed_v<Intg> && sizeof(Intg) > sizeof(long))
				return CheckErrno(static_cast<Intg>(char_traits<Char>::strtoui64(ptr, end, radix) & ~Intg()));
		}
	};

	// Floating point types
	template <StringTypeDynamic Str, std::floating_point FP>
	struct Convert<Str, FP>
	{
		static Str Func(FP from)
		{
			using Char = typename string_traits<Str>::value_type;
			Char buf[128];

			if constexpr (!std::is_same_v<FP, long double>)
			{
				return char_traits<Char>::dtostr(from, buf, _countof(buf));
			}
			else
			{
				// careful with long double, it's non-standard
				std::basic_stringstream<Char> ss;
				ss << from;
				return ss.str().c_str();
			}
		}
	};
	template <std::floating_point FP, StringType Str>
	struct Convert<FP, Str>
	{
		using Char = typename string_traits<Str>::value_type;
		static FP Func(Str const& s, Char const** end = nullptr)
		{
			auto ptr = string_traits<Str>::ptr(s);
			errno = 0;

			return CheckErrno(static_cast<FP>(char_traits<Char>::strtod(ptr, end)));
		}
	};

	// Narrow/Wide
	template <StringType Str0, StringType Str1>
	struct Convert<Str0, Str1>
	{
		static Str0 Func(Str1 const& s)
		{
			// Notes:
			//  - Remember type deduction doesn't work for string views
			//  - If 'Str2' = 'char const* const&', then 'string_traits<Str2>::value_type' would be 'char const'
			using Char0 = std::remove_const_t<typename string_traits<Str0>::value_type>;
			using Char1 = std::remove_const_t<typename string_traits<Str1>::value_type>;

			if constexpr (std::is_convertible_v<Str1, Str0>)
			{
				return s;
			}
			else if constexpr (std::is_assignable_v<Str0, Str1>)
			{
				return Str0(std::basic_string_view<Char1>{s}); 
			}
			else if constexpr (std::is_same_v<Char0, char> && std::is_same_v<Char1, wchar_t>)
			{
				return Narrow(s);
			}
			else if constexpr (std::is_same_v<Char0, wchar_t> && std::is_same_v<Char1, char>)
			{
				return Widen(s);
			}
			else
			{
				static_assert(dependent_false<Str1>, "Cannot convert between string types");
			}
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::str
{
	PRUnitTest(ToStringTests)
	{
		using std_cstr = std::string;
		using std_wstr = std::wstring;
		using pr_cstr  = pr::string<char>;
		using pr_wstr  = pr::string<wchar_t>;

		char     narr[] =  "junk_str_junk";
		wchar_t  wide[] = L"junk_str_junk";
		std_cstr scstr  =  "junk_str_junk";
		std_wstr swstr  = L"junk_str_junk";
		pr_cstr  pcstr  =  "junk_str_junk";
		pr_wstr  pwstr  = L"junk_str_junk";

		PR_EXPECT(To<std::string>(narr) == scstr);
		PR_EXPECT(To<std::string>(wide) == scstr);
		PR_EXPECT(To<std::string>(scstr) == scstr);
		PR_EXPECT(To<std::string>(swstr) == scstr);
		PR_EXPECT(To<std::string>(pcstr) == scstr);
		PR_EXPECT(To<std::string>(pwstr) == scstr);

		PR_EXPECT(To<std::wstring>(narr) == swstr);
		PR_EXPECT(To<std::wstring>(wide) == swstr);
		PR_EXPECT(To<std::wstring>(scstr) == swstr);
		PR_EXPECT(To<std::wstring>(swstr) == swstr);
		PR_EXPECT(To<std::wstring>(pcstr) == swstr);
		PR_EXPECT(To<std::wstring>(pwstr) == swstr);

		PR_EXPECT(To<std::string>(3.14) == "3.14");
		PR_EXPECT(To<std::wstring>(42) == L"42");
		PR_EXPECT(To<std_cstr>("literal cstr") == "literal cstr");
		PR_EXPECT(To<std_wstr>("literal cstr") == L"literal cstr");
		PR_EXPECT(To<pr_cstr>("literal cstr") == "literal cstr");
		PR_EXPECT(To<pr_wstr>("literal cstr") == L"literal cstr");

		PR_EXPECT(To<int>("1234") == 1234);
		PR_EXPECT(To<int>("1234", 10) == 1234);
		PR_EXPECT(To<int>(L"1234", 10) == 1234);
		PR_EXPECT(To<unsigned short>("12345",16) == (unsigned short)0x2345);
		PR_EXPECT(To<char>(L"1") == (char)1);
		PR_EXPECT(To<int>(L"1234") == 1234);
	}
}
#endif
