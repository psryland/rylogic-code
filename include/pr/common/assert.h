//*****************************************************************
// Assert
//  Copyright (c) Rylogic Ltd 2012
//*****************************************************************
// Usage:
//  Each module should use a define like "PR_DBG_???" to enable asserts
//  for that module. In the main headers for the module there should be
//  a definition:
//    #ifndef PR_DBG_???
//    #define PR_DBG_??? PR_DBG
//    #endif//PR_DBG_???
//
//  Sub parts of the module can also have there own debug define as in:
//    #ifndef PR_DBG_XYZ
//    #define PR_DBG_XYZ PR_DBG_???
//    #endif//PR_DBG_XYZ
//
//  To change the assert handler define PR_ASSERT_FUNC(exp, str) to your handler
//
//  Asserts can be enabled independently of debug/release using PR_ENABLE_ASSERTS
//  By default asserts are all enabled in debug and all disabled in release
//
//  Use these to remove dependencies in standalone files:
//
//"pr/common/assert.h" should be included prior to this for pr asserts
#pragma once
#include <cstdio>
#include <string>

// Assert enabler for common headers
// If not defined, set it based on whether it's a debug build
#ifndef PR_DBG
#  ifdef NDEBUG
#    define PR_DBG 0
#  else
#    define PR_DBG 1
#  endif
#endif

#ifndef PR_STRINGISE
#  define PR_STRINGISE_IMPL(x) #x
#  define PR_STRINGISE(x) PR_STRINGISE_IMPL(x)
#endif

#ifndef PR_JOIN
#  define PR_DO_JOIN(x,y) x##y
#  define PR_JOIN(x,y)    PR_DO_JOIN(x,y)
#endif

#ifndef PR_LINK
#  define PR_LINK __FILE__ "(" PR_STRINGISE(__LINE__) ") : "
#endif

// Allow for the assert function already being defined
#ifndef PR_ASSERT_FUNC

	// Use the CRT assert dialog, if we can, otherwise use cassert
	// Note, _WIN32 is defined for both 32 and 64 bit programs
	// _CrtDgbReport is only defined when _DEBUG is defined
	#if defined(_WIN32) && defined(_DEBUG)
	#  include <crtdbg.h>
	#else
	#  include <cassert>
	#endif

	namespace pr
	{
		#if defined(_WIN32) && defined(_DEBUG)

		// This is an actual function so that a breakpoint can be put in here
		inline void AssertionFailed(char const* expr, char const* str, char const* file, int line) noexcept
		{
			std::fprintf(stderr,
				"*** ASSERTION FAILURE ***\n"
				"Expression: %s\n"
				"Comment: %s\n"
				"%s(%d)\n"
				, expr
				, str
				, file
				, line);
			std::fflush(stderr);

			(_CrtDbgReport(_CRT_WARN, file, line, nullptr,
				"*** ASSERTION FAILURE ***\n"
				"Expression: %s\n"
				"Comment: %s\n"
				"%s(%d)\n"
				,expr ,str ,file ,line) != 0) ||
			(_CrtDbgReport(_CRT_ASSERT, file, line, nullptr,
				"Expression: %s\n"
				"Comment: %s\n"
				"%s(%d)\n"
				,expr ,str ,file ,line) != 1) ||
			(_CrtDbgBreak(), 0);
		}

		#endif

		namespace impl
		{
			inline bool ConstantExpressionSink(bool value) noexcept { return value; }
		}
	}

	// Converts the assert expression to a boolean value 'var', allowing for exceptions
	#ifdef _CPPUNWIND
	#  define PR_ASSERT_EXP_AS_BOOL(var, exp) bool var; try { var = pr::impl::ConstantExpressionSink(!!(exp)); } catch (...) { var = false; }
	#else
	#  define PR_ASSERT_EXP_AS_BOOL(var, exp) bool var = pr::impl::ConstantExpressionSink(!!(exp));
	#endif

	#if defined(_WIN32) && defined(_DEBUG)

	// Define the assert function
	#define PR_ASSERT_FUNC(exp, str)\
			PR_ASSERT_EXP_AS_BOOL(assertion_valid, exp)\
			if (!assertion_valid)\
				pr::AssertionFailed(#exp, str, __FILE__, __LINE__)

	#else

	// Define the assert function
	#define PR_ASSERT_FUNC(exp, str)\
			PR_ASSERT_EXP_AS_BOOL(assertion_failure, exp)\
			assert(#exp && str && assertion_failure)

	#endif

#endif

#define PR_EXPAND0(exp)
#define PR_EXPAND1(exp)         exp
#define PR_EXPAND(grp, exp)     PR_JOIN(PR_EXPAND, grp)(exp)

#define PR_INFO0(show, str)     do {} while (0)
#define PR_INFO1(show, str)\
do\
{\
	if (pr::impl::ConstantExpressionSink(show))\
	{\
		std::string info_str(str);\
		if (!info_str.empty() && info_str.back() != '\n') info_str.append("\n");\
		OutputDebugStringA(info_str.c_str());\
	}\
}\
while (pr::impl::ConstantExpressionSink(false))

#define PR_ASSERT0(exp, str)    do {} while (pr::impl::ConstantExpressionSink(false))
#define PR_ASSERT1(exp, str)    do { PR_ASSERT_FUNC(exp, str); } while (pr::impl::ConstantExpressionSink(false))

// Assert that 'exp' is true, if not display an assertion failure message containing 'str'. Switched on 'grp'
#define PR_ASSERT(grp, exp, str)   PR_JOIN(PR_ASSERT, grp)(exp, str)

// Display an info message 'str' if 'exp' is true. Switched on 'grp'
#define PR_INFO_IF(grp, exp, str)  PR_JOIN(PR_INFO, grp)(exp, str)

// Display an info message 'str'. Switched on 'grp'
#define PR_INFO(grp, str)          PR_JOIN(PR_INFO, grp)(true, str)

// Display an info message warning when a stub function is called
#define PR_STUB_FUNC()             PR_OUTPUT_MSG("Warning: Stub function called\n")

// Support for C++0x static_assert
#if _MSC_VER < 1600
#	ifndef __cplusplus
#		define static_assert(exp,str) typedef int JOIN(static_assert_, __LINE__)[(exp) ? 1 : -1]
#	else
		namespace pr
		{
			template <int n>  struct StaticAssertDummy;
			template <bool x> struct StaticAssertFailure;
			template <>       struct StaticAssertFailure<true> { enum {StaticFailure}; };
		}
#		ifdef __GNUC__
#			define static_assert(exp,str) typedef pr::StaticAssertDummy<pr::StaticAssertFailure<!!(exp)>::StaticFailure> PR_JOIN(Static_Failure, __LINE__)
#		else
#			define static_assert(exp,str) typedef pr::StaticAssertDummy<pr::StaticAssertFailure<!!(exp)>::StaticFailure> Static_Failure
#		endif
#	endif
#endif

#ifndef PR_UNUSED
#   define PR_UNUSED(exp) sizeof(exp)
#endif
