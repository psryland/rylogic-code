//**********************************
// String Core
//  Copyright (c) Rylogic Ltd 2015
//**********************************
// Fundamental string functions that operate on:
//'   std::string, std::wstring, pr::string<>, etc
//'   char[], wchar_t[], etc
//'   char*, wchar_t*, etc
// Note: char-array strings are not handled as special cases because there is no
// guarantee that the entire buffer is filled by the string, the null terminator
// may be midway through the buffer
#pragma once
#include <cuchar>
#include <string>
#include <string_view>
#include <charconv>
#include <vector>
#include <concepts>
#include <functional>
#include <type_traits>
#include <system_error>
#include <locale>
#include <cstdlib>
#include <cstdarg>
#include <cassert>
#include "pr/str/char8.h"
#include "pr/str/encoding.h"
#include "pr/str/convert_utf.h"
#include "pr/meta/dep_constants.h"

// Use this define to declare a string literal in a function templated on 'tchar'
#ifndef PR_STRLITERAL
#define PR_STRLITERAL(tchar,s)  pr::char_traits<tchar>::str(s, L##s)
#endif

namespace pr
{
	// Dealing with const in template functions:
	//  - If the function only requires a const parameter, use const in the parameter list:
	//    e.g.  template <typename T> void Read(T const& t) {}
	//    Within the function, use traits<T const>::...
	//  - If the function requires const and non-const overloads, just provide the non-const
	//    version and leave type deduction to handle the const cases:
	//    e.g. template <typename T> T::value* Ptr(T& t) { return t.ptr(); }
	//         Ptr<Thing const>(thing)
	//  - Functions that end with 'C' are used to get a const property from a potentially non
	//    const object.
	//  - T&& parameters cause T to be a reference type (mostly, except when an rvalue is passed)
	//    e.g. template <typename T> void f(T&& x) {} => T will be a reference type (so you might
	//    need to use std::decay_t when using traits)

	// The largest code point defined in unicode 6.0
	inline constexpr int UnicodeMaxValue = 0x10FFFF;

	// A static instance of the locale, because this thing takes ages to construct
	inline std::locale const& locale()
	{
		static std::locale s_locale("");
		return s_locale;
	}

	// Convert an int to a byte
	constexpr char operator ""_uc(unsigned long long arg) noexcept
	{
		return static_cast<unsigned char>(arg & 0xFF);
	}

	// Convert an int to a char
	constexpr char operator ""_ch(unsigned long long arg) noexcept
	{
		return static_cast<char>(arg & 0xFF);
	}

	// Convert an int to a char8
	constexpr char8_t operator ""_c8(unsigned long long arg) noexcept
	{
		return static_cast<char8_t>(arg & 0xFF);
	}

	// Convert an int to a wchar_t
	constexpr wchar_t operator ""_wc(unsigned long long arg) noexcept
	{
		return static_cast<wchar_t>(arg & 0xFFFF);
	}

	#pragma region Char Traits

	// Extend char traits
	template <typename Char>
	struct char_traits;

	// char traits base class
	template <typename Char>
	struct char_traits_common :std::char_traits<Char>
	{
		// Return the length (in storage units, *NOT* code points) of a string
		static size_t length(Char const* str)
		{
			size_t count = 0;
			for (; *str; ++str, ++count) {}
			return count;
		}
		static size_t lengthN(Char const* str, size_t max_bytes)
		{
			size_t count = 0;
			for (; *str && max_bytes > 0; ++str, ++count, max_bytes -= sizeof(Char)) {}
			return count;
		}

		// Convert a code point to lower case. WARNING: This is broken for multi-byte encodings
		static Char lwr(Char ch)
		{
			return std::char_traits<Char>::to_char_type(std::tolower(std::char_traits<Char>::to_int_type(ch), locale()));
		}
		static Char upr(Char ch)
		{
			return std::char_traits<Char>::to_char_type(std::toupper(std::char_traits<Char>::to_int_type(ch), locale()));
		}
	};

	// 'char' traits (aka ansi string)
	template <>
	struct char_traits<char> :char_traits_common<char>
	{
		static inline constexpr char zero = '\0';

		// String literal helper
		static constexpr char const* str(char const* str, wchar_t const*)
		{
			return str;
		}

		static int strcmp(char const* lhs, char const* rhs)
		{
			return ::strcmp(lhs, rhs);
		}
		static int strncmp(char const* lhs, char const* rhs, size_t max_count) 
		{
			return ::strncmp(lhs, rhs, max_count);
		}
		static int strnicmp(char const* lhs, char const* rhs, size_t max_count)
		{
			return ::_strnicmp(lhs, rhs, max_count);
		}

		static double strtod(char const* str, char const** end)
		{
			 return ::strtod(str, (char**)end);
		}
		static long strtol(char const* str, char const** end, int radix)
		{
			 return ::strtol(str, (char**)end, radix);
		}
		static unsigned long strtoul(char const* str, char const** end, int radix)
		{
			 return ::strtoul(str, (char**)end, radix);
		}
		static long long strtoi64(char const* str, char const** end, int radix)
		{
			 return ::_strtoi64(str, (char**)end, radix);
		}
		static unsigned long long strtoui64(char const* str, char const** end, int radix)
		{
			 return ::_strtoui64(str, (char**)end, radix);
		}

		static char* itostr(long long from, char* buf, int count, int radix)
		{
			// 'buf' should be at least 65 characters long
			if (::_i64toa_s(from, buf, count, radix) == 0) return buf;
			throw std::runtime_error("conversion from integral value to string failed");
		}
		static char* uitostr(unsigned long long from, char* buf, int count, int radix)
		{
			if (::_ui64toa_s(from, buf, count, radix) == 0) return buf;
			throw std::runtime_error("conversion from unsigned integral value to string failed");
		}
		static char* dtostr(double from, char* buf, int count)
		{
			// Have to use %g otherwise converting to a string loses precision
			auto n = ::_snprintf_s(buf, count, count, "%g", from);
			if (n <= 0) throw std::runtime_error("conversion from floating point value to string failed");
			if (n >= count) throw std::runtime_error("conversion from floating point value to string was truncated");
			buf[count - 1] = 0;
			return buf;
		}
	};

	// wchar_t traits (aka utf-16/UCS-2 string)
	template <>
	struct char_traits<wchar_t> :char_traits_common<wchar_t>
	{
		static inline constexpr wchar_t zero = '\0';

		// String literal helper
		static constexpr wchar_t const* str(char const*, wchar_t const* str)
		{
			return str; 
		}

		static int strcmp(wchar_t const* lhs, wchar_t const* rhs)
		{
			return ::wcscmp(lhs, rhs);
		}
		static int strncmp(wchar_t const* lhs, wchar_t const* rhs, size_t max_count)
		{
			return ::wcsncmp(lhs, rhs, max_count);
		}
		static int strnicmp(wchar_t const* lhs, wchar_t const* rhs, size_t max_count)
		{
			return ::_wcsnicmp(lhs, rhs, max_count);
		}

		static double strtod(wchar_t const* str, wchar_t const** end)
		{
			return ::wcstod(str, (wchar_t**)end);
		}
		static long strtol(wchar_t const* str, wchar_t const** end, int radix)
		{
			return ::wcstol(str, (wchar_t**)end, radix);
		}
		static long long strtoi64(wchar_t const* str, wchar_t const** end, int radix)
		{
			return ::_wcstoi64(str, (wchar_t**)end, radix);
		}
		static unsigned long strtoul(wchar_t const* str, wchar_t const** end, int radix)
		{
			return ::wcstoul(str, (wchar_t**)end, radix);
		}
		static unsigned long long strtoui64(wchar_t const* str, wchar_t const** end, int radix)
		{
			return ::_wcstoui64(str, (wchar_t**)end, radix);
		}

		static wchar_t* itostr(long long from, wchar_t* buf, int count, int radix)
		{
			// 'buf' should be at least 65 characters long
			if (::_i64tow_s(from, buf, count, radix) == 0) return buf;
			throw std::runtime_error("conversion from integral value to string failed");
		}
		static wchar_t* uitostr(unsigned long long from, wchar_t* buf, int count, int radix)
		{
			if (::_ui64tow_s(from, buf, count, radix) == 0) return buf;
			throw std::runtime_error("conversion from unsigned integral value to string failed");
		}
		static wchar_t* dtostr(double from, wchar_t* buf, int count)
		{
			// Have to use %g otherwise converting to a string loses precision
			auto n = ::_snwprintf_s(buf, count, count, L"%g", from);
			if (n <= 0) throw std::runtime_error("conversion from floating point value to string failed");
			if (n >= count) throw std::runtime_error("conversion from floating point value to string was truncated");
			buf[count - 1] = 0;
			return buf;
		}
	};

	// char8_t traits (aka utf-8 string)
	template <>
	struct char_traits<char8_t> :char_traits_common<char8_t>
	{};

	// char16_t traits
	template <>
	struct char_traits<char16_t> :char_traits_common<char16_t>
	{};

	// char32_t traits
	template <>
	struct char_traits<char32_t> :char_traits_common<char32_t>
	{};

	// References to chars
	template <typename Char>
	struct char_traits<Char const> :char_traits<std::decay_t<Char>>
	{};
	template <typename Char>
	struct char_traits<Char&> :char_traits<std::decay_t<Char>>
	{};

	// Is 'T' a char or wchar_t
	template <typename T> struct is_char :std::false_type {};
	template <> struct is_char<char> :std::true_type {};
	template <> struct is_char<wchar_t> :std::true_type {};
	template <> struct is_char<char8_t> :std::true_type {};
	template <> struct is_char<char16_t> :std::true_type {};
	template <> struct is_char<char32_t> :std::true_type {};
	template <typename T> constexpr bool is_char_v = is_char<std::decay_t<T>>::value;

	// Concepts
	template <typename Ty> concept CharType = is_char_v<Ty>;

	// Checks
	static_assert(is_char_v<char>);
	static_assert(is_char_v<wchar_t>);
	static_assert(is_char_v<char16_t>);
	static_assert(!is_char_v<double>);

	// Casting helper to catch overflow when casting from wide to narrow characters
	template <CharType Char1, CharType Char2>
	inline Char1 char_cast(Char2 ch)
	{
		assert(static_cast<Char2>(static_cast<Char1>(ch)) == ch && "Character value overflow in cast");
		return static_cast<Char1>(ch);
	}

	#pragma endregion

	#pragma region String Traits

	// String traits
	template <typename TStr>
	struct string_traits;

	// std::basic_string
	template <typename Char>
	struct string_traits<std::basic_string<Char>> :char_traits<Char>
	{
		using value_type = Char;
		using string_type = std::basic_string<Char>;
		static bool const null_terminated = true;
		static bool const dynamic_size = true;

		static value_type const* c_str(string_type const& str) { return str.c_str(); }
		static value_type const* ptr(string_type const& str)   { return str.data(); }
		static value_type* ptr(string_type& str)               { return str.data(); }
		static size_t size(string_type const& str)             { return str.size(); }
		static bool empty(string_type const& str)              { return str.empty(); }
		static void resize(string_type& str, size_t n)         { str.resize(n); }  // note: does not guarantee to fill for all string types
	};
	template <typename Char>
	struct string_traits<std::basic_string<Char> const> :char_traits<Char>
	{
		using value_type = Char const;
		using string_type = std::basic_string<Char> const;
		static bool const null_terminated = true;
		static bool const dynamic_size = true;

		static value_type* c_str(string_type& str) { return str.c_str(); }
		static value_type* ptr(string_type& str)   { return str.data(); }
		static size_t size(string_type& str)       { return str.size(); }
		static bool empty(string_type& str)        { return str.empty(); }
		static void resize(string_type&, size_t)   { static_assert(dependent_false<Char>, "Immutable string cannot be resized"); }
	};

	// std::basic_string_view
	template <typename Char>
	struct string_traits<std::basic_string_view<Char>> :char_traits<Char>
	{
		using value_type = Char;
		using string_type = std::basic_string_view<Char>;
		static bool const null_terminated = false;
		static bool const dynamic_size = false;

		static value_type const* c_str(string_type const& str) { static_assert(dependent_false<Char>, "String views cannot provide null terminated strings"); }
		static value_type const* ptr(string_type const& str)   { return str.data(); }
		static value_type const* ptr(string_type& str)         { return str.data(); }
		static size_t size(string_type const& str)             { return str.size(); }
		static bool empty(string_type const& str)              { return str.empty(); }
		static void resize(string_type& str, size_t n)         { if (n <= size(str)) str = str.substr(0, n); else throw std::runtime_error("String views can only be made smaller"); }
	};
	template <typename Char>
	struct string_traits<std::basic_string_view<Char> const> :char_traits<Char>
	{
		using value_type = Char const;
		using string_type = std::basic_string_view<Char> const;
		static bool const null_terminated = false;
		static bool const dynamic_size = false;

		static value_type* c_str(string_type& str)     { static_assert(dependent_false<Char>, "String views cannot provide null terminated strings"); }
		static value_type* ptr(string_type& str)       { return str.data(); }
		static size_t size(string_type& str)           { return str.size(); }
		static bool empty(string_type& str)            { return str.empty(); }
		static void resize(string_type& str, size_t n) { static_assert(dependent_false<Char>, "Immutable string cannot be resized"); }
	};

	// char*
	template <typename Char>
	struct string_traits<Char*> :char_traits<Char>
	{
		using value_type = Char;
		using string_type = Char*;
		static bool const null_terminated = true;
		static bool const dynamic_size = false;

		static value_type const* c_str(string_type const str) { return str; }
		static value_type* ptr(string_type str)               { return str; }
		static size_t size(string_type const str)             { return char_traits<Char>::length(str); }
		static bool empty(string_type const str)              { return *str == 0; }
		static void resize(string_type str, size_t n)         { str[n] = 0; }
	};
	template <typename Char>
	struct string_traits<Char const*> :char_traits<Char>
	{
		using value_type = Char const;
		using string_type = Char const*;
		static bool const null_terminated = true;
		static bool const dynamic_size = false;

		static value_type* c_str(string_type str) { return str; }
		static value_type* ptr(string_type str)   { return str; }
		static size_t size(string_type str)       { return char_traits<Char>::length(str); }
		static bool empty(string_type str)        { return *str == 0; }
		static void resize(string_type, size_t)   { static_assert(dependent_false<Char>, "Immutable strings cannot be resized"); }
	};
	template <typename Char>
	struct string_traits<Char* const> :string_traits<Char*>
	{};
	template <typename Char>
	struct string_traits<Char const* const> :string_traits<Char const*>
	{};

	// char[]
	template <typename Char, size_t Len>
	struct string_traits<Char[Len]> :char_traits<Char>
	{
		using value_type = Char;
		using string_type = Char[Len];
		static bool const null_terminated = true;
		static bool const dynamic_size = false;

		static value_type const* c_str(string_type const& str) { return str; }
		static value_type const* ptr(string_type const& str)   { return str; }
		static value_type* ptr(string_type& str)               { return str; }
		static size_t size(string_type const& str)             { return length(str, Len); }
		static bool empty(string_type const& str)              { return *str == 0; }
		static void resize(string_type& str, size_t n)         { if (n < Len) str[n] = 0; else throw std::runtime_error("Resize exceeds fixed array size"); }
	};
	template <typename Char, size_t Len>
	struct string_traits<Char const[Len]> :char_traits<Char>
	{
		using value_type = Char const;
		using string_type = Char const[Len];
		static bool const null_terminated = true;
		static bool const dynamic_size = false;

		static value_type* c_str(string_type& str) { return str; }
		static value_type* ptr(string_type& str)   { return str; }
		static size_t size(string_type& str)       { return char_traits<Char>::lengthN(str, Len * sizeof(Char)); }
		static bool empty(string_type& str)        { return *str == 0; }
		static void resize(string_type&, size_t)   { static_assert(dependent_false<Char>, "Immutable string cannot be resized"); }
	};

	// std::array<char>
	template <typename Char, size_t Len>
	struct string_traits<std::array<Char, Len>> :char_traits<Char>
	{
		static_assert(Len > 0);
		using value_type = Char;
		using string_type = std::array<Char, Len>;
		static bool const null_terminated = true;
		static bool const dynamic_size = false;

		static value_type const* c_str(string_type const& str)  { return str.data(); }
		static value_type const* ptr(string_type const& str)    { return str.data(); }
		static value_type* ptr(string_type& str)                { return str.data(); }
		static size_t size(string_type const& str)              { return length(str.data(), Len); }
		static bool empty(string_type const& str)               { return str[0] == 0; }
		static void resize(std::array<Char,Len>& str, size_t n) { if (n < Len) str[n] = 0; else throw std::runtime_error("Resize exceeds fixed array size"); }
	};
	template <typename Char, size_t Len>
	struct string_traits<std::array<Char, Len> const> :char_traits<Char>
	{
		static_assert(Len > 0);
		using value_type = Char const;
		using string_type = std::array<Char, Len> const;
		static bool const null_terminated = true;
		static bool const dynamic_size = false;

		static value_type* c_str(string_type& str) { return str.data(); }
		static value_type* ptr(string_type& str)   { return str.data(); }
		static size_t size(string_type& str)       { return char_traits<Char>::length(str.data(), Len); }
		static bool empty(string_type& str)        { return str[0] == 0; }
		static void resize(string_type&, size_t)   { static_assert(dependent_false<Char>, "Immutable string cannot be resized"); }
	};

	// References to strings
	template <typename TStr>
	struct string_traits<TStr&> :string_traits<std::remove_reference_t<TStr>>
	{};

	template <typename T> concept MarkedAsString = requires(T) { typename T::is_pr_string_compatible; };

	// Is string type
	template <typename Ty> struct is_string :std::false_type {};
	template <MarkedAsString Ty> struct is_string<Ty> : std::true_type {};
	template <typename Char> struct is_string<Char*> : is_char<Char> {};
	template <typename Char> struct is_string<Char const*> : is_char<Char> {};
	template <typename Char> struct is_string<std::basic_string_view<Char>> : is_char<Char> {};
	template <typename Char> struct is_string<std::basic_string<Char>> : is_char<Char> {};
	template <typename Char, size_t Len> struct is_string<Char const[Len]> : is_char<Char> {};
	template <typename Char, size_t Len> struct is_string<Char[Len]> : is_char<Char> {};
	template <typename Ty> constexpr bool is_string_v = is_string<std::decay_t<Ty>>::value;

	// Concepts
	template <typename Ty> concept StringType = is_string_v<Ty>;
	template <typename Ty> concept StringTypeStatic = is_string_v<Ty> && string_traits<Ty>::dynamic_size == false;
	template <typename Ty> concept StringTypeDynamic = is_string_v<Ty> && string_traits<Ty>::dynamic_size == true;

	// Checks
	static_assert(std::is_same_v<string_traits<std::wstring>::value_type, wchar_t>);
	static_assert(std::is_same_v<string_traits<std::string const&>::value_type, char const>);
	static_assert(is_string_v<char*>);
	static_assert(is_string_v<wchar_t* const>);
	static_assert(is_string_v<char16_t const*>);
	static_assert(is_string_v<std::string>);
	static_assert(is_string_v<std::wstring>);
	static_assert(is_string_v<std::wstring const&>);
	static_assert(!is_string_v<double>);

	#pragma endregion

	#pragma region Encoding

	// Narrow a string to utf-8
	inline std::string Narrow(std::string const& from) // from ascii, utf8
	{
		return from;
	}
	inline std::string Narrow(std::string_view from)
	{
		// Needed because string_view is not implicitly convertible to std::string
		return Narrow(std::string(from));
	}
	inline std::string Narrow(char const* from)
	{
		// Needed to disambiguate 'string_view' and 'std::string const&' overloads
		return Narrow(std::string_view(from));
	}
	inline std::string Narrow(std::wstring_view from) // from utf16
	{
		str::convert_utf<wchar_t, char> cvt;
		return cvt.conv<std::string>(from);
	}
	inline std::u8string Narrow(std::u8string_view from)
	{
		return std::u8string(from);
	}

	// Widen string to utf16
	inline std::wstring Widen(std::wstring const& from) // from usc2, utf-16
	{
		return from;
	}
	inline std::wstring Widen(std::wstring_view from)
	{
		// Needed because string_view is not implicitly convertible to std::string
		return Widen(std::wstring(from));
	}
	inline std::wstring Widen(wchar_t const* from)
	{
		// Needed to disambiguate 'wstring_view' and 'std::wstring const&' overloads
		return Widen(std::wstring_view(from));
	}
	inline std::wstring Widen(std::string_view from) // from utf-8
	{
		str::convert_utf<char, wchar_t> cvt;
		return cvt.conv<std::wstring>(from);
	}
	inline std::wstring Widen(std::u8string_view from) // from utf-8
	{
		str::convert_utf<char8_t, wchar_t> cvt;
		return cvt.conv<std::wstring>(from);
	}

	#pragma endregion

	namespace str
	{
		#pragma region Sprintf

		// Variable arg 'sprintf', overloaded for all char type combinations
		inline int vsprintf(char* buf, size_t buf_size_in_bytes, char const* format, va_list args)
		{
			return vsprintf_s(buf, buf_size_in_bytes, format, args);
		}
		inline int vsprintf(char* buf, size_t buf_size_in_bytes, wchar_t const* format, va_list args)
		{
			return vsprintf_s(buf, buf_size_in_bytes, Narrow(format).c_str(), args);
		}
		inline int vsprintf(wchar_t* buf, size_t buf_size_in_words, wchar_t const* format, va_list args)
		{
			return vswprintf_s(buf, buf_size_in_words, format, args);
		}
		inline int vsprintf(wchar_t* buf, size_t buf_size_in_words, char const* format, va_list args)
		{
			return vswprintf_s(buf, buf_size_in_words, Widen(format).c_str(), args);
		}

		// 'sprintf' accepting any char type
		template <CharType Char1, CharType Char2>
		inline int sprintf(Char1* buf, size_t buf_size_in_words, Char2 const* format, ...)
		{
			va_list arg_list;
			va_start(arg_list, format);
			auto r = vsprintf(buf, buf_size_in_words, format, arg_list);
			va_end(arg_list);
			return r;
		}

		#pragma endregion

		#pragma region Character classes

		template <CharType Char> inline bool IsNewLine(Char ch)
		{
			return ch == '\n';
		}
		template <CharType Char> inline bool IsLineSpace(Char ch)
		{
			return ch == ' ' || ch == '\t' || ch == '\r';
		}
		template <CharType Char> inline bool IsWhiteSpace(Char ch)
		{
			return IsLineSpace(ch) || IsNewLine(ch) || ch == '\v' || ch == '\f';
		}
		template <CharType Char> inline bool IsDecDigit(Char ch)
		{
			return (ch >= '0' && ch <= '9');
		}
		template <CharType Char> inline bool IsBinDigit(Char ch)
		{
			return (ch >= '0' && ch <= '1');
		}
		template <CharType Char> inline bool IsOctDigit(Char ch)
		{
			return (ch >= '0' && ch <= '7');
		}
		template <CharType Char> inline bool IsHexDigit(Char ch)
		{
			return IsDecDigit(ch) || (ch >= 'a' && ch <= 'f') || (ch >= 'A' && ch <= 'F');
		}
		template <CharType Char> inline bool IsDigit(Char ch)
		{
			return IsDecDigit(ch);
		}
		template <CharType Char> inline bool IsAlpha(Char ch)
		{
			return (ch >= 'a' && ch <= 'z') || (ch >= 'A' && ch <= 'Z');
		}
		template <CharType Char> inline bool IsIdentifier(Char ch, bool first, bool incl_dot = false)
		{
			return ch == '_' || IsAlpha(ch) || (!first && IsDigit(ch)) || (incl_dot && ch == '.');
		}

		// Return a pointer to delimiters, either the ones provided or the default ones
		template <CharType Char> inline std::basic_string_view<Char> Delim(std::basic_string_view<Char> delim = {})
		{
			if (!delim.empty()) return delim;
			if constexpr (std::is_same_v<Char, char>)
				return " \t\n\r";
			else if constexpr (std::is_same_v<Char, wchar_t>)
				return L" \t\n\r";
		}

		#pragma endregion

		#pragma region Size

		// Return true if 'str' is an empty string
		template <StringType Str>
		inline bool Empty(Str const& str)
		{
			return string_traits<Str const>::empty(str);
		}

		// Return the size of the string, excluding the null terminator (same as 'strlen'). Not necessarily the length of a multibyte encoded string
		template <StringType Str>
		inline size_t Size(Str const& str)
		{
			return string_traits<Str const>::size(str);
		}

		#pragma endregion

		#pragma region Range

		// Return a pointer to the start of the string
		template <StringType Str, CharType Char = typename string_traits<Str>::value_type>
		inline Char* Begin(Str&& str)
		{
			// Compile error: "'return': cannot convert from 'const char *' to 'Char *'" means
			// the string_traits<Str>::value_type is missing a 'const'
			return string_traits<Str>::ptr(str);
		}
		template <StringType Str, CharType Char = typename string_traits<Str>::value_type>
		inline Char const* BeginC(Str&& str)
		{
			return Begin<Str const>(str);
		}

		// Return a pointer to the end of the string
		template <StringType Str, CharType Char = typename string_traits<Str>::value_type>
		inline Char* End(Str&& str)
		{
			return Begin(str) + Size(str);
		}
		template <StringType Str, CharType Char = typename string_traits<Str>::value_type>
		inline Char const* EndC(Str&& str)
		{
			return End<Str const>(str);
		}

		// Return a pointer to the 'N'th character or the end of the string, whichever is less
		template <StringType Str, CharType Char = typename string_traits<Str>::value_type>
		inline Char* End(Str&& str, size_t N)
		{
			auto ptr = Begin(str) + std::min(N, Size(str));
			return ptr;
		}
		template <StringType Str, CharType Char = typename string_traits<Str>::value_type>
		inline Char const* EndC(Str&& str, size_t N)
		{
			auto ptr = Begin<Str const>(str) + std::min(N, Size(str));
			return ptr;
		}

		#pragma endregion

		#pragma region Equal

		// Return true if the ranges '[i,iend)' and '[j,jend)' are equal
		template <typename Iter1, typename Iter2, typename Pred>
		inline bool Equal(Iter1 i, Iter1 iend, Iter2 j, Iter2 jend, Pred pred)
		{
			for (; !(i == iend) && !(j == jend) && pred(*i, *j); ++i, ++j) {}
			return i == iend && j == jend;
		}
		template <typename Iter1, typename Iter2> 
		inline bool Equal(Iter1 i, Iter1 iend, Iter2 j, Iter2 jend)
		{
			return Equal(i, iend, j, jend, [](auto l, auto r) { return l == r; });
		}

		// Return true if str1 and str2 are equal
		template <StringType Str1, StringType Str2, typename Pred>
		inline bool Equal(Str1 const& str1, Str2 const& str2, Pred pred)
		{
			return Equal(Begin(str1), End(str1), Begin(str2), End(str2), pred);
		}
		template <StringType Str1, StringType Str2>
		inline bool Equal(Str1 const& str1, Str2 const& str2)
		{
			return Equal(Begin(str1), End(str1), Begin(str2), End(str2));
		}

		// Return true if str1 and str2 are equal
		template <CharType Char1, CharType Char2, typename Pred>
		inline bool Equal(Char1 const* str1, Char2 const* str2, Pred pred)
		{
			auto i = Begin(str1);
			auto j = Begin(str2);
			for (; *i != 0 && *j != 0 && pred(*i, *j); ++i, ++j) {}
			return *i == 0 && *j == 0;
		}
		template <CharType Char1, CharType Char2>
		inline bool Equal(Char1 const* str1, Char2 const* str2)
		{
			return Equal(str1, str2, [](auto l, auto r) { return l == r; });
		}

		// Specialisations for char/wchar_t
		inline bool Equal(char const* str1, char const* str2)
		{
			return ::strcmp(str1, str2) == 0;
		}
		inline bool Equal(wchar_t const* str1, wchar_t const* str2)
		{
			return ::wcscmp(str1, str2) == 0;
		}

		#pragma endregion

		#pragma region EqualI

		// Return true if the ranges '[i,iend)' and '[j,jend)' are equal, ignoring case
		template <typename Iter1, typename Iter2>
		inline bool EqualI(Iter1 i, Iter1 iend, Iter2 j, Iter2 jend)
		{
			using Char1 = std::decay_t<decltype(*i)>;
			using Char2 = std::decay_t<decltype(*j)>;

			return Equal(i, iend, j, jend, [](Char1 lhs, Char2 rhs)
				{
					auto l = char_traits<Char1>::lwr(lhs);
					auto r = char_traits<Char2>::lwr(rhs);
					return l == r;
				});
		}

		// Return true if lhs and rhs are equal, ignoring case
		template <StringType Str1, StringType Str2>
		inline bool EqualI(Str1 const& str1, Str2 const& str2)
		{
			return EqualI(Begin(str1), End(str1), Begin(str2), End(str2));
		}

		// Specialisations for char/wchar_t
		inline bool EqualI(char const* str1, char const* str2)
		{
			return ::_stricmp(str1, str2) == 0;
		}
		inline bool EqualI(wchar_t const* str1, wchar_t const* str2)
		{
			return ::_wcsicmp(str1, str2) == 0;
		}

		#pragma endregion

		#pragma region EqualN

		// Return true if lhs and rhs are equal, up to the given length
		template <StringType Str1, StringType Str2, typename Pred>
		inline bool EqualN(Str1 const& str1, Str2 const& str2, size_t length, Pred pred)
		{
			return Equal(Begin(str1), End(str1, length), Begin(str2), End(str2, length), pred);
		}
		template <StringType Str1, StringType Str2>
		inline bool EqualN(Str1 const& str1, Str2 const& str2, size_t length)
		{
			return Equal(Begin(str1), End(str1, length), Begin(str2), End(str2, length));
		}
		template <CharType Char1, CharType Char2, typename Pred>
		inline bool EqualN(Char1 const* str1, Char2 const* str2, size_t length, Pred pred)
		{
			auto i = Begin(str1);
			auto j = Begin(str2);
			for (; length-- != 0 && *i != 0 && *j != 0 && pred(*i, *j); ++i, ++j) {}
			return length == size_t(-1) || (*i == 0 && *j == 0);
		}
		inline bool EqualN(char const* str1, char const* str2, size_t length)
		{
			return ::strncmp(str1, str2, length) == 0;
		}
		inline bool EqualN(wchar_t const* str1, wchar_t const* str2, size_t length)
		{
			return ::wcsncmp(str1, str2, length) == 0;
		}

		#pragma endregion

		#pragma region EqualNI

		// Return true if lhs and rhs are equal, up to the given length
		template <StringType Str1, StringType Str2>
		inline bool EqualNI(Str1 const& str1, Str2 const& str2, size_t length)
		{
			using Char1 = typename string_traits<Str1 const>::value_type;
			using Char2 = typename string_traits<Str2 const>::value_type;

			return EqualN(str1, str2, length, [](Char1 lhs, Char2 rhs)
				{
					auto l = char_traits<Char1>::lwr(lhs);
					auto r = char_traits<Char2>::lwr(rhs);
					return l == r; 
				});
		}
		template <StringType Str, CharType Char, int N>
		inline bool EqualNI(Str const& str, Char const (&str2)[N])
		{
			return EqualNI(str, str2, N);
		}
		inline bool EqualNI(char const* str1, char const* str2, size_t length)
		{
			return ::_strnicmp(str1, str2, length) == 0;
		}
		inline bool EqualNI(wchar_t const* str1, wchar_t const* str2, size_t length)
		{
			return ::_wcsnicmp(str1, str2, length) == 0;
		}

		#pragma endregion

		#pragma region FindChar

		// Return a pointer to the first occurrence of 'ch' in a string, or a pointer to '\0' if not found
		template <StringType Str1, CharType Char2, CharType Char1 = typename string_traits<Str1>::value_type>
		inline Char1 const* FindChar(Str1&& str, Char2 ch)
		{
			if constexpr (string_traits<Str1>::null_terminated)
			{
				auto ptr = string_traits<Str1>::ptr(str);
				for (; *ptr && static_cast<int>(*ptr) != static_cast<int>(ch); ++ptr) {}
				return ptr;
			}
			else
			{
				auto ptr = string_traits<Str1>::ptr(str);
				auto end = ptr + string_traits<Str1>::size(str);
				for (; ptr != end && static_cast<int>(*ptr) != static_cast<int>(ch); ++ptr) {}
				return ptr != end ? ptr : &string_traits<Str1>::zero;
			}
		}

		// Return a pointer to the first occurrence of 'ch' in a string or the string null terminator or the 'length' character
		template <StringType Str1, CharType Char2, CharType Char1 = typename string_traits<Str1>::value_type>
		inline Char1 const* FindChar(Str1&& str, Char2 ch, size_t length)
		{
			if constexpr (string_traits<Str1>::null_terminated)
			{
				auto ptr = string_traits<Str1>::ptr(str);
				for (; *ptr && length-- && static_cast<int>(*ptr) != static_cast<int>(ch); ++ptr) {}
				return ptr;
			}
			else
			{
				auto ptr = string_traits<Str1>::ptr(str);
				auto end = ptr + string_traits<Str1>::size(str, length);
				for (; ptr != end && static_cast<int>(*ptr) != static_cast<int>(ch); ++ptr) {}
				return ptr != end ? ptr : &string_traits<Str1>::zero;
			}
		}

		#pragma endregion

		#pragma region FindStr

		// Find the sub string 'what' in the given range of characters.
		// Returns an iterator to the sub string or to the end of the range.
		template <typename Iter, StringType Str, typename Pred>
		inline Iter FindStr(Iter first, Iter last, Str const& what, Pred pred)
		{
			if (Empty(what)) return last;
			auto what_len = static_cast<int>(Size(what));
			auto what_beg = Begin(what);
			for (; last - first >= what_len; ++first)
				if (pred(first, first + what_len, what_beg, what_beg + what_len))
					return first;

			return last;
		}
		template <typename Iter, StringType Str>
		inline Iter FindStr(Iter first, Iter last, Str const& what)
		{
			return FindStr(first, last, what, [](auto i, auto iend, auto j, auto jend)
			{
				return Equal(i, iend, j, jend);
			});
		}

		// Find the sub string 'what' in the given range of characters.
		// Returns an iterator to the sub string or to the end of the range.
		template <StringType Str1, StringType Str2, CharType Char1 = typename string_traits<Str1>::value_type>
		inline Char1* FindStr(Str1&& str, Str2 const& what)
		{
			return FindStr(Begin(str), End(str), what);
		}

		// Find the sub string 'what' in the range of characters provided (no case).
		// Returns an iterator to the sub string or to the end of the range.
		template <typename Iter, StringType Str>
		inline Iter FindStrI(Iter first, Iter last, Str const& what)
		{
			return FindStr(first, last, what, [](auto i, auto iend, auto j, auto jend)
			{
				return EqualI(i, iend, j, jend);
			});
		}
		template <StringType Str1, StringType Str2, CharType Char = typename string_traits<Str1>::value_type>
		inline Char* FindStrI(Str1&& str, Str2 const& what)
		{
			return FindStrI(Begin(str), End(str), what);
		}

		#pragma endregion

		#pragma region Find

		// Return an iterator to the first position that satisfies 'pred'
		template <typename Iter, typename Pred>
		inline Iter FindFirst(Iter beg, Iter end, Pred pred)
		{
			for (; beg != end && !pred(*beg); ++beg) {}
			return beg;
		}

		// Returns a pointer to the first character in '[offset, offset+count)' that satisfies 'pred', or a pointer to the end of the string or &str[offset+count]
		template <StringType Str, typename Pred, CharType Char = typename string_traits<Str>::value_type>
		inline Char* FindFirst(Str&& str, size_t offset, size_t count, Pred pred)
		{
			return FindFirst(Begin(str) + offset, End(str, offset + count), pred);
		}

		// Returns a pointer to the first character in 'str' that satisfies 'pred', or a pointer to the end of the string
		template <StringType Str, typename Pred, CharType Char = typename string_traits<Str>::value_type>
		inline Char* FindFirst(Str&& str, Pred pred)
		{
			return FindFirst(str, 0, ~size_t(), pred);
		}

		// Return an iterator to *one past* the last position that satisfies 'pred' or a pointer to the beginning of the string. Intended to be used to form a range with FindFirst/FindLast.
		template <typename Iter, typename Pred>
		inline Iter FindLast(Iter beg, Iter end, Pred pred)
		{
			for (; end != beg && !pred(*(end-1)); --end) {}
			return end;
		}

		// Returns a pointer to *one past* the last character in '[offset, offset+count)' that satisfies 'pred', or a pointer to the beginning of the string or &str[offset]
		template <StringType Str, typename Pred, CharType Char = typename string_traits<Str>::value_type>
		inline Char* FindLast(Str&& str, size_t offset, size_t count, Pred pred)
		{
			return FindLast(Begin(str) + offset, End(str, offset + count), pred);
		}

		// Returns a pointer to *one past* the last character in 'str' that satisfies 'pred', or a pointer to the beginning of the string
		template <StringType Str, typename Pred, CharType Char = typename string_traits<Str>::value_type>
		inline Char* FindLast(Str&& str, Pred pred)
		{
			return FindLast(str, 0, ~size_t(), pred);
		}

		// Find the first occurrence of one of the chars in 'delim'
		template <typename Iter, CharType Char>
		inline Iter FindFirstOf(Iter beg, Iter end, Char const* delim)
		{
			for (; beg != end && *FindChar(delim, *beg) == 0; ++beg) {}
			return beg;
		}
		template <StringType Str, CharType Char1, CharType Char2 = typename string_traits<Str>::value_type>
		inline Char2* FindFirstOf(Str&& str, Char1 const* delim)
		{
			return FindFirstOf(Begin(str), End(str), delim);
		}
		template <typename Iter, CharType Char>
		inline size_t FindFirstOfAdv(Iter& str, Char const* delim)
		{
			auto count = size_t();
			for (; *str && *FindChar(delim, *str) == 0; ++str, ++count) {}
			return count;
		}
		template <typename Iter, CharType Char>
		inline size_t FindFirstOfAdv(Iter& str, Iter end, Char const* delim)
		{
			auto count = size_t();
			for (; str != end && *FindChar(delim, *str) == 0; ++str, ++count) {}
			return count;
		}

		// Return a pointer to *one past* the last occurrence of one of the chars in 'delim'. Intended to be use with FindFirstOf to for a range
		template <typename Iter, CharType Char>
		inline Iter FindLastOf(Iter beg, Iter end, Char const* delim)
		{
			for (; end != beg && *FindChar(delim, *(end-1)) == 0; --end) {}
			return end;
		}
		template <StringType Str, CharType Char1, CharType Char2 = typename string_traits<Str>::value_type>
		inline Char2* FindLastOf(Str&& str, Char1 const* delim)
		{
			return FindLastOf(Begin(str), End(str), delim);
		}

		// Find the first character not in the set 'delim'
		template <typename Iter, CharType Char>
		inline Iter FindFirstNotOf(Iter beg, Iter end, Char const* delim)
		{
			for (; beg != end && *FindChar(delim, *beg) != 0; ++beg) {}
			return beg;
		}
		template <StringType Str, CharType Char1, CharType Char2 = typename string_traits<Str>::value_type>
		inline Char2* FindFirstNotOf(Str&& str, Char1 const* delim)
		{
			return FindFirstNotOf(Begin(str), End(str), delim);
		}
		template <typename Iter, CharType Char>
		inline size_t FindFirstNotOfAdv(Iter& str, Char const* delim)
		{
			auto count = size_t();
			for (; *str && *FindChar(delim, *str) != 0; ++str, ++count) {}
			return count;
		}
		template <typename Iter, CharType Char>
		inline size_t FindFirstNotOfAdv(Iter& str, Iter end, Char const* delim)
		{
			auto count = size_t();
			for (; str != end && *FindChar(delim, *str) != 0; ++str, ++count) {}
			return count;
		}

		// Return a pointer to *one past* the last character not in the set 'delim'
		template <typename Iter, CharType Char>
		inline Iter FindLastNotOf(Iter beg, Iter end, Char const* delim)
		{
			for (; end != beg && *FindChar(delim, *(end-1)) != 0; --end) {}
			return end;
		}
		template <StringType Str, CharType Char1, CharType Char2 = typename string_traits<Str>::value_type>
		inline Char2* FindLastNotOf(Str&& str, Char1 const* delim)
		{
			return FindLastNotOf(Begin(str), End(str), delim);
		}

		#pragma endregion

		#pragma region Resize

		// Resize a string. For pointers to fixed buffers, it's the callers responsibility to ensure sufficient space
		template <StringType Str>
		inline void Resize(Str&& str, size_t new_size)
		{
			string_traits<Str>::resize(str, new_size);
		}
		template <StringType Str, CharType Char>
		inline void Resize(Str&& str, size_t new_size, Char const ch)
		{
			auto current_size = Size(str);
			Resize(str, new_size);

			auto const fill = char_cast<typename string_traits<Str>::value_type>(ch);
			for (; current_size < new_size; ++current_size)
				str[current_size] = fill;
		}

		#pragma endregion

		#pragma region Append

		// Append a char to the end of 'str'
		// 'len' is an optimisation for pointer-like strings so that Size() doesn't need to be called for each Append
		// Returns 'str' for method chaining
		template <StringType Str1, CharType Char2>
		inline Str1&& Append(Str1&& str, Char2 ch, size_t& len)
		{
			Resize(str, len+1);
			str[len++] = char_cast<typename string_traits<Str1>::value_type>(ch);
			return std::forward<Str1>(str);
		}
		template <StringType Str1, CharType Char2>
		inline Str1&& Append(Str1&& str, Char2 ch)
		{
			auto len = Size(str);
			return Append(str, ch, len);
		}

		// Append a string to the end of 'str'
		// 'len' is an optimisation for pointer-like strings so that Size() doesn't need to be called for each Append
		// Returns 'str' for method chaining
		template <StringType Str1, StringType Str2>
		inline Str1&& Append(Str1&& str, Str2 const& s, size_t& len)
		{
			auto count = Size(s);
			auto ptr = string_traits<Str2 const>::ptr(s);
			Resize(str, len + count);
			for (; count-- != 0; )
				str[len++] = char_cast<typename string_traits<Str1>::value_type>(*ptr++);

			return std::forward<Str1>(str);
		}
		template <StringType Str1, StringType Str2>
		inline Str1&& Append(Str1&& str, Str2 const& s)
		{
			auto len = Size(str);
			return Append(str, s, len);
		}
	
		#pragma endregion

		#pragma region Assign

		// Assign a range of characters to a sub-range within a string.
		// 'dest' is the string to be assigned to
		// 'offset' is the index position of where to start copying to
		// 'count' is the maximum number of characters to copy. The capacity of dest *must* be >= offset + count
		// 'first','last' the range of characters to assign to 'dest'
		// On return, dest will be resized to 'offset + min(count, last-first)'.
		template <StringType Str, typename Iter>
		inline Str&& Assign(Str&& dest, size_t offset, size_t count, Iter first, Iter last)
		{
			// The number of characters to be copied to 'dest', clamped by 'count'
			auto size = std::min(count, size_t(last - first));

			// Set 'dest' to be the correct size. Assume 'dest' can be resized to 'offset + size'
			Resize(dest, offset + size);

			// Assign the characters
			for (auto out = Begin(dest) + offset; size--; ++out, ++first)
				*out = char_cast<typename string_traits<Str>::value_type>(*first);

			return std::forward<Str>(dest);
		}
		template <StringType Str, typename Iter>
		inline Str&& Assign(Str&& dest, Iter first, Iter last)
		{
			return Assign(dest, 0, size_t(last - first), first, last);
		}

		// Assign a null terminated string to 'dest'
		template <StringType Str, CharType Char>
		inline Str&& Assign(Str&& dest, size_t offset, size_t count, Char const* first)
		{
			Resize(dest, offset);
			for (;count-- != 0 && *first; ++first)
				Append(dest, *first, offset);

			return std::forward<Str>(dest);
		}
		template <StringType Str, CharType Char>
		inline Str&& Assign(Str&& dest, Char const* first)
		{
			return Assign(dest, 0, ~size_t(), first);
		}

		#pragma endregion

		#pragma region Upper Case

		// Convert a string to upper case
		template <StringType Str>
		inline Str& UpperCase(Str& str)
		{
			using traits = string_traits<Str>;
			auto i = Begin(str);
			auto iend = End(str);

			for (; i != iend; ++i)
				*i = traits::upr(*i);

			return str;
		}
		template <StringType Str>
		inline Str UpperCaseC(Str const& str)
		{
			auto s = str;
			return UpperCase(s);
		}
		template <CharType Char>
		inline std::basic_string<Char> UpperCaseC(Char const* str)
		{
			auto s = std::basic_string<Char>(str);
			return UpperCase(s);
		}

		#pragma endregion

		#pragma region Lower Case

		// Convert a string to lower case
		template <StringType Str>
		inline Str& LowerCase(Str& str)
		{
			using traits = string_traits<Str>;

			auto i = Begin(str);
			auto iend = End(str);
			for (; i != iend; ++i)
				*i = traits::lwr(*i);

			return str;
		}
		template <StringType Str>
		inline Str LowerCaseC(Str const& str)
		{
			auto s = str;
			return LowerCase(s);
		}
		template <CharType Char>
		inline std::basic_string<Char> LowerCaseC(Char const* str)
		{
			auto s = std::basic_string<Char>(str);
			return LowerCase(s);
		}

		#pragma endregion

		#pragma region SubStr

		// Copy a substring from within 'src' to 'out'
		template <StringType Str1, StringType Str2>
		inline Str2& SubStr(Str1 const& src, size_t offset, size_t count, Str2& out)
		{
			auto s = Begin(src) + offset;
			return Assign(out, 0, count, s, s + count);
		}
		template <StringType Str1, StringType Str2>
		inline Str2 SubStr(Str1 const& src, size_t offset, size_t count)
		{
			Str2 out = {};
			return SubStr(src, offset, count, out);
		}

		#pragma endregion

		#pragma region Split

		// Split a string at 'delims' outputting each sub string to 'out'
		// 'out' should have the signature out(tstr1 const& s, int i, int j, int n)
		// where [i,j) is the range in 's' containing the substring. 'n' is the index of the output subrange.
		// Returns the number of sub strings found.
		template <StringType Str, CharType Char, std::invocable<std::basic_string_view<typename string_traits<Str>::value_type>, int> OutCB>
		inline int Split(Str const& str, Char const* delims, OutCB out)
		{
			using StrView = std::basic_string_view<typename string_traits<Str>::value_type>;

			int i = 0, j = 0, jend = static_cast<int>(Size(str)), n = 0;
			for (; j != jend; ++j)
			{
				if (*FindChar(delims, str[j]) == 0) continue;
				out(StrView{ &str[i], s_cast<size_t>(j - i) }, n++);
				i = j + 1;
			}
			if (i != j)
			{
				out(StrView{ &str[i], s_cast<size_t>(j - i) }, n++);
			}
			return n;
		}

		#pragma endregion

		#pragma region Trim

		// Trim characters from a string
		// 'str' is the string to be trimmed
		// 'pred' should return true if the character should be trimmed
		// Returns 'str' for method chaining
		template <StringType Str, typename Pred>
		inline Str& Trim(Str& str, Pred pred, bool front, bool back)
		{
			auto beg = Begin(str);
			auto end = End(str);
			auto first = front ? FindFirst(beg  , end, [&](auto ch){ return !pred(ch); }) : beg;
			auto lastt = back  ? FindLast (first, end, [&](auto ch){ return !pred(ch); }) : end;

			// Move the non-trimmed characters to the front of the string and trim the tail
			auto out = beg;
			for (; first != lastt; ++first, ++out) { *out = *first; }
			Resize(str, out - beg);
			return str;
		}
		template <StringType Str, typename Pred>
		inline Str Trim(Str const& str, Pred pred, bool front, bool back)
		{
			auto s = str;
			return Trim(s, pred, front, back);
		}
		template <CharType Char, typename Pred>
		inline std::basic_string<Char> Trim(Char const* str, Pred pred, bool front, bool back)
		{
			auto s = std::basic_string<Char>(str);
			return Trim(s, pred, front, back);
		}
		
		// Trim leading or trailing characters in 'chars' from 'str'.
		// Returns 'str' for method chaining
		template <StringType Str, CharType Char>
		inline Str& TrimChars(Str& str, Char const* chars, bool front, bool back)
		{
			return Trim(str, [&](auto ch){ return *FindChar(chars, ch) != 0; }, front, back);
		}
		template <StringType Str, CharType Char>
		inline Str TrimChars(Str const& str, Char const* chars, bool front, bool back)
		{
			auto s = str;
			return TrimChars(s, chars, front, back);
		}
		template <CharType Char1, CharType Char2>
		inline std::basic_string<Char1> TrimChars(Char1 const* str, Char2 const* chars, bool front, bool back)
		{
			auto s = std::basic_string<Char1>(str);
			return TrimChars(s, chars, front, back);
		}

		#pragma endregion
	}
}

