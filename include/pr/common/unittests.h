//*******************************************************************************************
// UnitTests
//  Copyright (c) Rylogic Ltd 2009
//*******************************************************************************************
// Very lightweight unit test framework
// Use:
//  Add a block like this
/*
#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::unittests
{
	PRUnitTest(TestName, Quick)
	{
		PR_EXPECT(1+1 == 2);
	}
	PRUnitTestClass(TestClassName)
	{
		PRUnitTestMethod(MethodName, Quick)
		{
			auto tmp = temp_dir() / "file.ext";
			PR_EXPECT(tmp != "");
		}
	};
}
#endif
*/
// In any project you can define PR_UNITTESTS=1 and call pr::unittests::RunAllTests()
// to have unit tests available and executed. In the specific unittests project you
// need to add an include of the file containing the unit test, so that it gets included
// in the build.
#pragma once
#include <type_traits>
#include <concepts>
#include <span>
#include <memory>
#include <iostream>
#include <string>
#include <string_view>
#include <sstream>
#include <fstream>
#include <vector>
#include <deque>
#include <tuple>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <filesystem>
#include <algorithm>
#include <functional>
#include <execution>
#include <ranges>
#include <random>
#include <chrono>
#include <cstdarg>
#include <cmath>
#include <bitset>
#include <locale>
#include <format>
#include <Windows.h>

// Optionally use Microsoft's C++ unit test framework
// To use the Microsoft.VisualStudio.TestTools.CppUnitTestFramework framework,
//  - The test project needs to be a DLL,
//  - AdditionalLibraryDirectories include ';$(VCInstallDir)Auxiliary\VS\UnitTest\include'
//  - AdditionalIncludeDirectories includes ';$(VCInstallDir)Auxiliary\VS\UnitTest\lib'
//  - *But*, don't link to Microsoft.VisualStudio.TestTools.CppUnitTestFramework.lib, it's done by #pragma comment
//  - Make sure all necessary dependent dlls are available or the test runner will crash.
#define USE_MS_UNITTESTS 0 // Set this to 0 when compiling as an executable
#if USE_MS_UNITTESTS
	//#pragma message ("Using MS Unit Test Framework")
	#include <SDKDDKVer.h>
	#include "CppUnitTest.h"
	using namespace Microsoft::VisualStudio::CppUnitTestFramework;
	// Create tests with; TEST_CLASS(MyTest) { public: TEST_METHOD(MyTestMethod){ ... } };
	#define PRTestClass(testname)\
		ONLY_USED_AT_NAMESPACE_SCOPE class testname : public ::Microsoft::VisualStudio::CppUnitTestFramework::TestClass<testname>
	#define PRTestMethod(testname)\
		TEST_METHOD(testname)
#else
	//#pragma message ("Using Rylogic Unit Test Framework")
	#define TEST_CLASS(testname) class testname
	#define TEST_METHOD(testmethod) void unused()
	#define ONLY_USED_AT_NAMESPACE_SCOPE namespace ___CUT___ {extern int YOU_CAN_ONLY_DEFINE_TEST_CLASS_AT_NAMESPACE_SCOPE;}
#endif

// Enable this to turn on dumping to LDraw
#if PR_UNITTESTS_VISUALISE
#include "pr/common/ldraw.h"
#endif

// Cannot include pr lib headers here because they are the headers
// being unit tested. Also, this should be a standalone header

#define PR_UNITTEST_STRINGISE_IMPL(value) #value
#define PR_UNITTEST_STRINGISE(value) PR_UNITTEST_STRINGISE_IMPL(value)

namespace pr::unittests
{
	// Platform string constant
	constexpr wchar_t const* const Platform =
		sizeof(void*) == 8 ? L"x64" :
		sizeof(void*) == 4 ? L"x86" :
		L"";

	// Config string constant
	#ifdef NDEBUG
	constexpr wchar_t const* Config = L"release";
	#else
	constexpr wchar_t const* Config = L"debug";
	#endif

	// Properties controlling when a registered test should run.
	enum class EUnitTestFlags : unsigned int
	{
		None     = 0,
		Quick    = 1 << 0,
		Extended = 1 << 1,
		Stress   = 1 << 2,
	};

	// Combine test execution properties.
	constexpr EUnitTestFlags operator | (EUnitTestFlags lhs, EUnitTestFlags rhs)
	{
		return static_cast<EUnitTestFlags>(static_cast<unsigned int>(lhs) | static_cast<unsigned int>(rhs));
	}

	// Return true when a test has every requested execution property.
	constexpr bool HasAllUnitTestFlags(EUnitTestFlags flags, EUnitTestFlags required)
	{
		return (static_cast<unsigned int>(flags) & static_cast<unsigned int>(required)) == static_cast<unsigned int>(required);
	}

	// Parse a pipe-separated flag expression used by test declarations and command-line filters.
	constexpr bool TryParseUnitTestFlags(std::string_view text, EUnitTestFlags& flags)
	{
		auto is_space = [](char ch)
		{
			return ch == ' ' || ch == '\t' || ch == '\r' || ch == '\n';
		};

		flags = EUnitTestFlags::None;
		auto parsed = false;
		for (auto index = std::size_t{}; index != text.size();)
		{
			// Isolate one flag name without making declaration formatting significant.
			while (index != text.size() && is_space(text[index]))
				++index;

			auto const token_begin = index;
			while (index != text.size() && text[index] != '|' && !is_space(text[index]))
				++index;

			// Reject unknown names so misspelled declaration flags fail at compile time.
			auto const token = text.substr(token_begin, index - token_begin);
			auto flag = EUnitTestFlags::None;
			if (token == "Quick")
				flag = EUnitTestFlags::Quick;
			else if (token == "Extended")
				flag = EUnitTestFlags::Extended;
			else if (token == "Stress")
				flag = EUnitTestFlags::Stress;
			else
				return false;

			// Accumulate the property and require a separator before another name.
			flags = flags | flag;
			parsed = true;
			while (index != text.size() && is_space(text[index]))
				++index;

			if (index == text.size())
				break;
			if (text[index] != '|')
				return false;

			++index;
			if (index == text.size())
				return false;
		}
		return parsed;
	}

	// Return true when an expression contains only supported flags and separators.
	constexpr bool IsValidUnitTestFlags(std::string_view text)
	{
		auto flags = EUnitTestFlags::None;
		return TryParseUnitTestFlags(text, flags);
	}

	// Parse an expression already proven valid by a declaration-level assertion.
	constexpr EUnitTestFlags ParseUnitTestFlags(std::string_view text)
	{
		auto flags = EUnitTestFlags::None;
		TryParseUnitTestFlags(text, flags);
		return flags;
	}

	// Keep combined declarations and malformed-expression rejection compile-time checked.
	static_assert(IsValidUnitTestFlags("Extended | Stress"));
	static_assert(HasAllUnitTestFlags(ParseUnitTestFlags("Extended | Stress"), EUnitTestFlags::Extended | EUnitTestFlags::Stress));
	static_assert(!IsValidUnitTestFlags("Extended |"));

	// helpers
	namespace impl
	{
		// A static instance of the locale, because this thing takes ages to construct
		inline std::locale const& locale()
		{
			static std::locale s_locale("");
			return s_locale;
		}

		// Widen/Narrow strings
		inline std::string Narrow(char const* from, std::size_t len = 0)
		{
			if (len == 0) len = strlen(from);
			return std::string(from, from+len);
		}
		inline std::string Narrow(wchar_t const* from, std::size_t len = 0)
		{
			if (len == 0) len = wcslen(from);
			std::vector<char> buffer(len + 1);
			std::use_facet<std::ctype<wchar_t>>(locale()).narrow(from, from + len, '_', &buffer[0]);
			return std::string(&buffer[0], &buffer[len]);
		}
		inline std::string Narrow(std::string const& from)
		{
			return from;
		}
		inline std::string Narrow(std::wstring const& from)
		{
			return Narrow(from.c_str(), from.size());
		}
		inline std::wstring Widen(wchar_t const* from, std::size_t len = 0)
		{
			if (len == 0) len = wcslen(from);
			return std::wstring(from, from+len);
		}
		inline std::wstring Widen(char const* from, std::size_t len = 0)
		{
			if (len == 0) len = strlen(from);
			std::vector<wchar_t> buffer(len + 1);
			std::use_facet<std::ctype<wchar_t>>(locale()).widen(from, from + len, &buffer[0]);
			return std::wstring(&buffer[0], &buffer[len]);
		}
		inline std::wstring Widen(std::string const& from)
		{
			return Widen(from.c_str(), from.size());
		}
		inline std::wstring Widen(std::wstring const& from)
		{
			return from;
		}

		// Stream std::string into a wide stream
		inline std::basic_ostream<wchar_t>& operator <<(std::basic_ostream<wchar_t>& stream, std::string const& s)
		{
			return stream << Widen(s);
		}
	}

	// Meta data for a test case
	struct UnitTestItem
	{
		// Test function signature
		using TestFunc = std::function<void(void)>;

		char const*       m_name;  // Test name (short)
		TestFunc          m_func;  // The unit test function
		type_info const*  m_class; // Type info of the test class
		EUnitTestFlags    m_flags; // Test execution properties
		char const*       m_file;  // The file that the test is in
		int               m_line;  // Line number of the test function

		friend bool operator < (UnitTestItem const& lhs, UnitTestItem const& rhs) { return strcmp(lhs.m_name, rhs.m_name) < 0; }
	};

	// Test Framework functions
	struct TestFramework
	{
		// All registered tests
		inline static std::vector<UnitTestItem> Tests;

		// The number of calls to 'PR_EXPECT'
		inline static int TestCount;

		// A pointer to the stream that output is written to
		inline static std::ostream* ostream = &std::cout;
		static std::ostream& out() { return *ostream; }

		// Append a unit test to the Tests() collection
		template <typename T>
		static bool AddTest(char const* name, UnitTestItem::TestFunc method, EUnitTestFlags flags, char const* file, int line)
		{
			Tests.push_back(UnitTestItem{
				.m_name = name,
				.m_func = method,
				.m_class = &typeid(T),
				.m_flags = flags,
				.m_file = file,
				.m_line = line,
			});
			return true;
		}

		// Unit test check functions
		static void Fail(char const* msg, char const* file, int line)
		{
			++TestCount;
			throw std::runtime_error(std::format("{}({}): {}", file, line, msg));
		}

		// Assert that a unit test result is true
		static void IsTrue(bool result, wchar_t const* expr, char const* file, int line)
		{
			++TestCount;
			if (result) return;
			throw std::runtime_error(std::format("{}({}): '{}' failed", file, line, impl::Narrow(expr)));
		}

		// Check that 'func' throws a 'TExcept' exception
		template <typename TExcept, typename Func>
		static void Throws(Func func, wchar_t const* expr, char const* file, int line)
		{
			++TestCount;
			bool threw = false;
			bool threw_expected = false;
			try
			{
				func();
			}
			catch (TExcept const&)
			{
				threw = true;
				threw_expected = true;
			}
			catch (...)
			{
				threw = true;
			}
			if (threw_expected)
				return;

			throw std::runtime_error(std::format("{}({}): '{}' {}", file, line, impl::Narrow(expr), (threw
				? "threw an exception of an unexpected type"
				: "didn't throw when it was expected to")));
		}
	};

	// Base class for unit test classes
	template <typename Derived>
	struct UnitTestBase
	{
		// Notes:
		//  - There is no point adding the type-pack to the test class. Only methods can be
		//    invoked with different types so it only makes sense to have the types specified
		//    on the test methods.

		using base_t = UnitTestBase<Derived>;
		using test_class_type = Derived;
	
		mutable std::filesystem::path m_cached_temp_dir = {};
		int m_count = 0;

		// Test class name
		std::string_view class_name() const
		{
			// Extract class name from type name
			auto const* name = typeid(Derived).name();
			for (auto const* found = strstr(name, "::"); found != nullptr; found = strstr(name, "::"))
				name = found + 2;

			return name;
		}

		// A directory for temporary files needed by unit tests. Note: automatically cleaned
		// Use 'temp_dir' in your unit test. Also remember 'auto temp_file = temp_dir / std::filesystem::unique_path("tempfile-%%%%-%%%%-%%%%-%%%%")'
		std::filesystem::path temp_dir() const
		{
			using namespace std::filesystem;
			if (m_cached_temp_dir.empty())
			{
				m_cached_temp_dir = weakly_canonical(path(__FILE__).parent_path() / L".." / L".." / L".." / L"obj" / L"unittests" / class_name() / Platform / Config / "");

				// Removed temp files from previous test runs.
				if (std::filesystem::exists(m_cached_temp_dir))
				{
					try { std::filesystem::remove_all(m_cached_temp_dir); }
					catch (std::exception const& ex)
					{
						TestFramework::out() << "Warning: Failed to clear temp directory. " << ex.what() << std::endl;
					}
				}
				std::filesystem::create_directories(m_cached_temp_dir);
			}
			return m_cached_temp_dir;
		}

		// Return a path relative to the repo root
		std::filesystem::path repo_path(wchar_t const* repo_path)
		{
			using namespace std::filesystem;
			return weakly_canonical(path(__FILE__).parent_path() / L".." / L".." / L".." / repo_path);
		}
	};

	// Match a class-name filter as an unanchored wildcard pattern while preserving plain substring filters.
	inline bool MatchTestFilter(std::string_view value, std::string_view pattern)
	{
		if (pattern.find('*') == std::string_view::npos && pattern.find('?') == std::string_view::npos)
			return value.find(pattern) != std::string_view::npos;

		for (auto start = std::size_t{}; start <= value.size(); ++start)
		{
			auto value_index = start;
			auto pattern_index = std::size_t{};
			auto star_index = std::string_view::npos;
			auto retry_index = start;
			while (value_index < value.size())
			{
				if (pattern_index < pattern.size() && (pattern[pattern_index] == '?' || pattern[pattern_index] == value[value_index]))
				{
					++pattern_index;
					++value_index;
				}
				else if (pattern_index < pattern.size() && pattern[pattern_index] == '*')
				{
					star_index = pattern_index++;
					retry_index = value_index;
				}
				else if (star_index != std::string_view::npos)
				{
					pattern_index = star_index + 1;
					value_index = ++retry_index;
				}
				else
				{
					break;
				}

				if (pattern_index == pattern.size())
					return true;
			}

			while (pattern_index < pattern.size() && pattern[pattern_index] == '*')
				++pattern_index;
			if (pattern_index == pattern.size())
				return true;
		}
		return false;
	}

	// Run registered tests selected by optional execution-property and class-name filters.
	inline int RunAllTests(
		bool wordy,
		std::span<std::string_view const> filter = {},
		std::span<std::string_view const> exclude = {},
		std::span<EUnitTestFlags const> flags = {})
	{
		using namespace std::chrono;
		try
		{
			TestFramework::out() << " **** Begin Unit Tests **** " << std::endl;
			std::sort(std::begin(TestFramework::Tests), std::end(TestFramework::Tests));

			int passed = 0, failed = 0, skipped = 0;
			auto T0 = high_resolution_clock::now();

			// Run the tests
			for (auto const& test : TestFramework::Tests)
			{
				// A filter expression requires all its flags, while separate expressions are alternatives.
				if (!flags.empty())
				{
					auto match = std::ranges::any_of(flags, [&](auto required) { return HasAllUnitTestFlags(test.m_flags, required); });
					if (!match)
					{
						++skipped;
						continue;
					}
				}

				// Apply the class-name inclusion patterns.
				if (!filter.empty())
				{
					auto class_name = std::string_view(test.m_class->name());
					auto match = std::ranges::any_of(filter, [&](auto const& f) { return MatchTestFilter(class_name, f); });
					if (!match)
					{
						++skipped;
						continue;
					}
				}
				if (!exclude.empty())
				{
					auto class_name = std::string_view(test.m_class->name());
					auto match = std::ranges::any_of(exclude, [&](auto const& f) { return MatchTestFilter(class_name, f); });
					if (match)
					{
						++skipped;
						continue;
					}
				}

				try
				{
					if (wordy)
					{
						auto name_width = std::max(100, static_cast<int>(strlen(test.m_name)));
						auto test_name = std::format("{}.{}", test.m_class->name() + 7, test.m_name);
						TestFramework::out() << std::format("{:.<{}}", test_name, name_width) << std::flush;
					}

					TestFramework::TestCount = 0;

					// Run the test
					auto t0 = high_resolution_clock::now();
					test.m_func();
					auto t1 = high_resolution_clock::now();

					++passed;
					if (wordy)
						TestFramework::out() << std::format("success. ({:8} tests in {:4.3f} ms)\n", TestFramework::TestCount, 0.001 * duration_cast<microseconds>(t1 - t0).count());
				}
				catch (std::exception const& e)
				{
					TestFramework::out() << std::format("{}\n   {}({}): {} failed\n", e.what(), test.m_file, test.m_line, test.m_class->name());
					++failed;
				}
			}

			auto T1 = high_resolution_clock::now();

			// Print the results
			if (failed == 0)
			{
				TestFramework::out() << std::format(" **** UnitTest results: All {} unit tests passed.", passed);
				if (skipped > 0) TestFramework::out() << std::format(" ({} skipped by filter)", skipped);
				TestFramework::out() << std::format(" (taking {:1.3f} ms) ****\n", 0.001 * duration_cast<microseconds>(T1-T0).count());
			}
			else
			{
				TestFramework::out() << std::format(" **** UnitTest results: {} of {} failed.", failed, failed + passed);
				if (skipped > 0) TestFramework::out() << std::format(" ({} skipped by filter)", skipped);
				TestFramework::out() << " ****\n";
			}

			return failed == 0 ? 0 : -1;
		}
		catch (...)
		{
			TestFramework::out() << "UnitTests could not complete due to an unhandled exception" << std::endl;
			return -1;
		}
	}

	// Unit test Equal functions
	template <typename T, typename U> inline bool UTEqual(T const& lhs, U const& rhs)
	{
		return lhs == rhs;
	}
	template <typename T, size_t Len1, size_t Len2> inline bool UTEqual(T const (&lhs)[Len1], T const (&rhs)[Len2])
	{
		return Len1 == Len2 && memcmp(lhs, rhs, sizeof(T) * Len1) == 0;
	}
	template <typename T, typename U> inline bool UTEqual(T const* lhs, std::initializer_list<U> rhs)
	{
		int i = 0;
		for (auto& r : rhs)
		{
			if (lhs[i++] == r) continue;
			return false;
		}
		return true;
	}
	inline bool UTEqual(double lhs, double rhs)
	{
		if (isnan(lhs) && isnan(rhs)) return true;
		if (isinf(lhs) && isinf(rhs)) return signbit(lhs) == signbit(rhs);
		return ::abs(rhs - lhs) < DBL_EPSILON;
	}
	inline bool UTEqual(float lhs, float rhs)
	{
		if (isnan(lhs) && isnan(rhs)) return true;
		if (isinf(lhs) && isinf(rhs)) return signbit(lhs) == signbit(rhs);
		return ::fabs(rhs - lhs) < FLT_EPSILON;
	}
	inline bool UTEqual(char const* lhs, char const* rhs)
	{
		return strcmp(lhs, rhs) == 0;
	}
	inline bool UTEqual(char* lhs, char* rhs)
	{
		return strcmp(lhs, rhs) == 0;
	}
	inline bool UTEqual(wchar_t const* lhs, wchar_t const* rhs)
	{
		return wcscmp(lhs, rhs) == 0;
	}
	inline bool UTEqual(wchar_t* lhs, wchar_t* rhs)
	{
		return wcscmp(lhs, rhs) == 0;
	}
}

// Test class
#define PRUnitTestClass(classname)\
struct TestClass_##classname : pr::unittests::UnitTestBase<TestClass_##classname>

// Test method with its execution properties
#define PRUnitTestMethod(methodname, flags, ...)\
	template <typename... Types>\
	static void Test_##methodname##_()\
	{\
		if constexpr (sizeof...(Types) > 0)\
		{\
			typename base_t::test_class_type t;\
			(t.Test_##methodname<Types>(), ...);\
		}\
		else\
		{\
			typename base_t::test_class_type t;\
			t.Test_##methodname<void>();\
		}\
	}\
	static_assert(pr::unittests::IsValidUnitTestFlags(PR_UNITTEST_STRINGISE(flags)), "Unknown unit-test flag");\
	inline static bool s_registered_##methodname = pr::unittests::TestFramework::AddTest<test_class_type>(\
		#methodname,\
		+[](){ Test_##methodname##_<##__VA_ARGS__>(); },\
		pr::unittests::ParseUnitTestFlags(PR_UNITTEST_STRINGISE(flags)),\
		__FILE__,\
		__LINE__\
	);\
	template <typename T>\
	void Test_##methodname()

// Test function
#define PRUnitTest(testname, flags, ...)\
	PRUnitTestClass(testname)\
	{\
		PRUnitTestMethod(testname, flags, __VA_ARGS__);\
	};\
	template <typename T> void TestClass_##testname::Test_##testname()

#define PR_EXPECT(expr)\
	pr::unittests::TestFramework::IsTrue(expr, L#expr, __FILE__, __LINE__)

#define PR_THROWS(expr, what)\
	pr::unittests::TestFramework::Throws<what>([&]{ expr; }, L#expr, __FILE__, __LINE__)

// Redefine 'assert' so that failed asserts in tested code are reported as
// test failures (via cout) rather than triggering an abort dialog.
#undef assert
#define assert(expr) pr::unittests::TestFramework::IsTrue(!!(expr), L#expr, __FILE__, __LINE__)
