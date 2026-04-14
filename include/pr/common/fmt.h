//****************************************************
// Fmt - An inline function for inserting formated text
//  Copyright (c) Rylogic Ltd 2008
//****************************************************
// Note about __declspec(thread):
//  Doesn't work in dll's on windows <= XP
//  In multithreaded situations use Fmt() or a FmtX<> per thread
#pragma once
#include <cstdio>
#include <cstdarg>
#include <cassert>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>
#include <malloc.h>

// C++11's thread_local
#if _MSC_VER < 1900
#  ifndef thread_local
#    define thread_local __declspec(thread)
#  endif
#endif

#ifdef __cplusplus
namespace pr
{
	// Implementation
	namespace impl
	{
		// Char type independent vsnprintf function
		inline int Format(char* dst, size_t max_count, char const* fmt, va_list arg_list)
		{
			return _vsprintf_p(dst, max_count, fmt, arg_list);
		}
		inline int Format(wchar_t* dst, size_t max_count, wchar_t const* fmt, va_list arg_list)
		{
			return _vswprintf_p(dst, max_count, fmt, arg_list);
		}

		template <typename TString, typename TChar> inline TString& Format(TString& dst, size_t hint_size, TChar const* fmt, va_list arg_list)
		{
			struct MallocaScope
			{
				TChar* m_ptr;
				size_t m_count;
				MallocaScope(void* mem, size_t count) :m_ptr(static_cast<TChar*>(mem)) ,m_count(count) {}
				~MallocaScope() { _freea(m_ptr); }
			};

			for (;;)
			{
				enum { sane_hint_size_limit = 32 * 1024 * 1024 };
				if (hint_size > sane_hint_size_limit)
					throw std::runtime_error("failed to format string, string too large");

				// Allocate a buffer, preferably on the stack
				MallocaScope buf(_malloca(hint_size * sizeof(TChar)), hint_size);

				// NOTE: In C99 you can only use a va_list once so you need to make a new
				// copy each time. For now, we don't need to
				int result = Format(buf.m_ptr, buf.m_count, fmt, arg_list);

				// Format error
				if (result < 0)
					throw std::runtime_error("string format error");

				// It fits, w00t!
				if (result < int(buf.m_count))
				{
					dst.append(std::basic_string_view<TChar>{ buf.m_ptr, buf.m_ptr + result });
					break;
				}

				// Otherwise, try a larger buffer
				hint_size *= 2;
			}

			return dst;
		}
	}

	// Puts a formatted string into 'str'
	template <typename TString> inline TString& Format(TString& str, typename TString::value_type const* format, ...)
	{
		va_list arg_list;
		va_start(arg_list, format);
		impl::Format(str, 1024, format, arg_list);
		va_end(arg_list);
		return str;
	}

	// Returns a formatted TString
	template <typename TString, typename TChar> inline TString FmtArgs(TChar const* format, va_list arg_list)
	{
		TString str;
		return impl::Format(str, 1024, format, arg_list);
	}
	template <typename TString, typename TChar> inline TString Fmt(TChar const* format, ...)
	{
		TString str;
		va_list arg_list;
		va_start(arg_list, format);
		impl::Format(str, 1024, format, arg_list);
		va_end(arg_list);
		return str;
	}

	// Returns a formatted std::string/std::wstring
	template <typename TChar> inline std::basic_string<TChar> Fmt(TChar const* format, ...)
	{
		std::basic_string<TChar> str;
		va_list arg_list;
		va_start(arg_list, format);
		impl::Format(str, 1024, format, arg_list);
		va_end(arg_list);
		return str;
	}
	template <typename TChar> inline std::basic_string<TChar> Fmt(size_t hint_size, TChar const* format, ...)
	{
		std::basic_string<TChar> str;
		va_list arg_list;
		va_start(arg_list, format);
		impl::Format(str, hint_size, format, arg_list);
		va_end(arg_list);
		return str;
	}

	// Returns a formatted std::string/std::wstring where 'func' translates the format codes.
	// Format codes should be '%?' where '?' is up to func to interpret.
	// 'func' should take a pointer into the format string and return something
	// that can be appended to a std::basic_string<>. 'func' is allowed to advance 's'
	// to the last character of the code e.g. "%3.3d foos", 's' = "3.3d...", func can advance 's' to the 'd'
	template <typename TFunc, typename TChar> inline std::basic_string<TChar> FmtF(TChar const* format, TFunc func)
	{
		std::basic_string<TChar> str;
		TChar const pc('%');
		for (TChar const* s = format; *s; ++s)
		{
			if (*s != pc)     { str.append(1,*s); continue; }
			if (*(++s) == pc) { str.append(1,*s); continue; }
			str.append(func(s));
		}
		return str;
	}

	// Static, use with caution, but fast string format
	template <typename Ctx, size_t Sz, typename TChar> inline TChar const* FmtArgs(TChar const* format, va_list arg_list)
	{
		static thread_local TChar buf[Sz];
		int result = impl::Format(buf, Sz-1, format, arg_list);
		assert(result >= 0 && result < Sz-1 && "formatted string truncated");
		buf[result] = 0;
		return buf;
	}
	template <typename Ctx, size_t Sz, typename TChar> inline TChar const* FmtX(TChar const* format, ...)
	{
		va_list arg_list;
		va_start(arg_list, format);
		auto s = FmtArgs<Ctx, Sz, TChar>(format, arg_list);
		va_end(arg_list);
		return s;
	}
	template <typename TChar> inline TChar const* FmtS(TChar const* format, ...)
	{
		va_list arg_list;
		va_start(arg_list, format);
		auto s = FmtArgs<struct S, 1024, TChar>(format, arg_list);
		va_end(arg_list);
		return s;
	}

	// Indent helper
	inline std::string_view Indent(int amount)
	{
		constexpr static char const space[] = "\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t\t";
		constexpr static int len = (int)_countof(space);
		return std::string_view(space, amount < len ? amount : len);
	}
}
#else
	static __inline char const* Format(char* buffer, int length, char const* format, ...)
	{
		va_list arg_list;
		va_start(arg_list, format);
		vsnprintf(buffer, length, format, arg_list);
		buffer[length - 1] = 0;
		va_end(arg_list);
		return buffer;
	}

	static __inline char const* FmtS(char const* format, ...)
	{
		static thread_local char buffer[256];
		va_list arg_list;
		va_start(arg_list, format);
		vsnprintf(buffer, sizeof(buffer), format, arg_list);
		buffer[sizeof(buffer) - 1] = 0;
		va_end(arg_list);
		return buffer;
	}
#endif

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/str/string.h"

namespace pr::common
{
	PRUnitTest(FmtTests)
	{
		using namespace pr::str;
		{// char - simple specifiers
			auto s0 = FmtS("%c %C %s %S %d %i %u %o %x %X %f %f %e %E %g %G"
				,'A' ,L'W'
				,"hello world" ,L"wide str"
				,0x01234567
				,0x55555555
				,0x89abcdef
				,01234567
				,0xdeadbeef
				,0xdeadbeef
				,6.28f
				,6.28
				,6.28f
				,6.28
				,6.28f
				,6.28
				);
			#if _MSC_VER >= 1900
			auto s1 = "A W hello world wide str 19088743 1431655765 2309737967 1234567 deadbeef DEADBEEF 6.280000 6.280000 6.280000e+00 6.280000E+00 6.28 6.28";
			#else
			auto s1 = "A W hello world wide str 19088743 1431655765 2309737967 1234567 deadbeef DEADBEEF 6.280000 6.280000 6.280000e+000 6.280000E+000 6.28 6.28";
			#endif
			PR_EXPECT(Equal(s0, s1));
		}
		{// wchar_t - simple
			auto s0 = FmtS(L"%c %C %s %S %d %i %u %o %x %X %f %f %e %E %g %G"
				,L'A' ,'W'
				,L"hello world" ,"narrow str"
				,0x01234567
				,0x55555555
				,0x89abcdef
				,01234567
				,0xdeadbeef
				,0xdeadbeef
				,6.28f
				,6.28
				,6.28f
				,6.28
				,6.28f
				,6.28
				);
			#if _MSC_VER >= 1900
			auto s1 = L"A W hello world narrow str 19088743 1431655765 2309737967 1234567 deadbeef DEADBEEF 6.280000 6.280000 6.280000e+00 6.280000E+00 6.28 6.28";
			#else
			auto s1 = L"A W hello world narrow str 19088743 1431655765 2309737967 1234567 deadbeef DEADBEEF 6.280000 6.280000 6.280000e+000 6.280000E+000 6.28 6.28";
			#endif

			PR_EXPECT(Equal(s0, s1));
		}
		{// char - length specifiers
			auto s0 = FmtS("%hhd %hd %lx %llx %Lf"
				,static_cast<signed char>(0x7f)
				,static_cast<signed short>(0x7fff)
				,static_cast<long int>(0x55555555)
				,static_cast<long long int>(0x0123456789abcdef)
				,1000000000000000000.0
				);

			PR_EXPECT(Equal(s0, "127 32767 55555555 123456789abcdef 1000000000000000000.000000"));
		}
		{// wchar_t - length specifiers
			auto s0 = FmtS(L"%hhd %hd %lx %llx %Lf"
				,static_cast<signed char>(0x7f)
				,static_cast<signed short>(0x7fff)
				,static_cast<long int>(0x55555555)
				,static_cast<long long int>(0x0123456789abcdef)
				,1000000000000000000.0
				);

			PR_EXPECT(Equal(s0, L"127 32767 55555555 123456789abcdef 1000000000000000000.000000"));
		}
		{
			std::string s0;
			pr::Format(s0, "String %d", 0);
			PR_EXPECT(s0 == "String 0");

			char const* s1 = pr::FmtS("String %d",1);
			PR_EXPECT(Equal(s1, "String 1"));

			auto s2 = pr::FmtS(L"wide string %d", 2);
			PR_EXPECT(Equal(s2, L"wide string 2"));

			auto s3 = pr::Fmt("std::string %d", 3);
			PR_EXPECT(s3.size() == 13U);

			auto s4 = pr::Fmt<pr::string<>>("pr::string %d",4);
			PR_EXPECT(s4 == "pr::string 4");
			PR_EXPECT(s4.size() == 12U);

			auto s5 = pr::FmtX<struct P, 128>("c-string %d", 5);
			PR_EXPECT(Equal(s5, "c-string 5"));
		}
	}
}
#endif
