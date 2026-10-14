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
			log_stream() << "Informational output goes to '<temp_dir>/MethodName.log', not the console\n";
		}
	};
}
#endif
*/
// In any project you can define PR_UNITTESTS=1 and call pr::unittests::RunAllTests()
// to have unit tests available and executed. In the specific unittests project you
// need to add an include of the file containing the unit test, so that it gets included
// in the build.
//
// Execution:
//  Test classes run in parallel, one class at a time per worker thread, and the tests within a class run in order.
//  Use 'PRUnitTestClass(Name, Serial)' for a class that must not run at the same time as any other class.
//  Per-class shared state (e.g. a GPU device or engine) should be registered with 'TestFramework::AddClassFixture' so it is released after the class.
//  Runner arguments: '-flags:<expression>' (e.g. Quick), '-exclude:<pattern>', '-threads:<count>' (0 = one per processor), '-verbose',
//  '-asserts:stderr' (report assertion failures to stderr instead of dialogs, for unattended runs), and class name filters.
//  Each run writes the duration of every test to '<repo>/obj/unittests/<runner>/timings.csv' and lists the slowest tests.
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
#include <mutex>
#include <thread>
#include <atomic>
#include <charconv>
#include <Windows.h>
#include "pr/common/assert_reporting.h"

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

	// Properties controlling when and how a registered test should run.
	// 'Quick', 'Extended', and 'Stress' select run groups. 'Serial' applies to the whole owning test class and stops that class from running
	// at the same time as any other class. Use it for classes that share process-wide state (e.g. global singletons, fixed file paths, or windows).
	enum class EUnitTestFlags : unsigned int
	{
		None     = 0,
		Quick    = 1 << 0,
		Extended = 1 << 1,
		Stress   = 1 << 2,
		Serial   = 1 << 3,

		RunGroups = Quick | Extended | Stress,
		Invalid   = ~0u,
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
			else if (token == "Serial")
				flag = EUnitTestFlags::Serial;
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

	// Return true when a test declaration contains only supported flags and selects at least one run group.
	constexpr bool IsValidUnitTestFlags(std::string_view text)
	{
		auto flags = EUnitTestFlags::None;
		return TryParseUnitTestFlags(text, flags) && (static_cast<unsigned int>(flags) & static_cast<unsigned int>(EUnitTestFlags::RunGroups)) != 0;
	}

	// Parse an expression already proven valid by a declaration-level assertion.
	constexpr EUnitTestFlags ParseUnitTestFlags(std::string_view text)
	{
		auto flags = EUnitTestFlags::None;
		TryParseUnitTestFlags(text, flags);
		return flags;
	}

	// Parse the optional flags given to 'PRUnitTestClass'. Only 'Serial' is meaningful for a class; anything else returns 'Invalid'.
	constexpr EUnitTestFlags ParseUnitTestClassFlags(std::string_view text)
	{
		// An empty argument list means the class has no class-level flags.
		if (text.find_first_not_of(" \t\r\n") == std::string_view::npos)
			return EUnitTestFlags::None;

		auto flags = EUnitTestFlags::None;
		if (!TryParseUnitTestFlags(text, flags) || flags != EUnitTestFlags::Serial)
			return EUnitTestFlags::Invalid;

		return flags;
	}

	// Keep combined declarations and malformed-expression rejection compile-time checked.
	static_assert(IsValidUnitTestFlags("Extended | Stress"));
	static_assert(IsValidUnitTestFlags("Quick | Serial"));
	static_assert(!IsValidUnitTestFlags("Serial"));
	static_assert(HasAllUnitTestFlags(ParseUnitTestFlags("Extended | Stress"), EUnitTestFlags::Extended | EUnitTestFlags::Stress));
	static_assert(!IsValidUnitTestFlags("Extended |"));
	static_assert(ParseUnitTestClassFlags("") == EUnitTestFlags::None);
	static_assert(ParseUnitTestClassFlags("Serial") == EUnitTestFlags::Serial);
	static_assert(ParseUnitTestClassFlags("Quick") == EUnitTestFlags::Invalid);

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

	};

	// Test Framework functions
	struct TestFramework
	{
		// All registered tests
		inline static std::vector<UnitTestItem> Tests;

		// The number of calls to 'PR_EXPECT' made by the test running on the current thread
		inline static thread_local int TestCount;

		// A pointer to the stream that output is written to. Writes from the runner are serialised with 'OutputLock'.
		inline static std::ostream* ostream = &std::cout;
		inline static std::mutex OutputLock;
		static std::ostream& out() { return *ostream; }

		// Register 'release' to destroy a fixture that is shared by the tests of the class running on the current thread.
		// The runner calls 'ReleaseClassFixtures' after each class, on the thread that ran it, so a fixture with thread affinity is released on its own thread.
		// Use this rather than relying on a 'thread_local' destructor. Thread-exit destructors run while Windows holds the loader lock,
		// and releasing objects such as D3D12 devices at that point can deadlock with other threads that are loading DLLs.
		static void AddClassFixture(std::function<void()> release)
		{
			ClassFixtures.push_back(std::move(release));
		}

		// Destroy the current thread's class fixtures, newest first so fixtures built on other fixtures are released before them.
		static void ReleaseClassFixtures()
		{
			for (; !ClassFixtures.empty(); ClassFixtures.pop_back())
				ClassFixtures.back()();
		}
		inline static thread_local std::vector<std::function<void()>> ClassFixtures;

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

	// Base class for unit test classes. 'ClassFlags' holds class-level flags (only 'Serial') and is added to every test method in the class.
	template <typename Derived, EUnitTestFlags ClassFlags = EUnitTestFlags::None>
	struct UnitTestBase
	{
		// Notes:
		//  - There is no point adding the type-pack to the test class. Only methods can be
		//    invoked with different types so it only makes sense to have the types specified
		//    on the test methods.
		static_assert(ClassFlags != EUnitTestFlags::Invalid, "Only 'Serial' is a valid unit-test class flag");

		using base_t = UnitTestBase<Derived, ClassFlags>;
		using test_class_type = Derived;
		static constexpr EUnitTestFlags class_flags = ClassFlags;
	
		// True once this class's temp directory has been cleaned in this run, so its tests do not delete each other's files.
		inline static bool s_temp_dir_cleaned = false;

		mutable std::filesystem::path m_cached_temp_dir = {};
		std::ofstream m_log_stream = {};
		char const* m_test_name = "";
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

		// A directory for temporary files needed by the tests in this class. Files from previous runs are removed on first use in each run.
		// Use 'temp_dir' in your unit test. Also remember 'auto temp_file = temp_dir / std::filesystem::unique_path("tempfile-%%%%-%%%%-%%%%-%%%%")'
		std::filesystem::path temp_dir() const
		{
			// Resolve the class directory once per test instance.
			using namespace std::filesystem;
			if (m_cached_temp_dir.empty())
			{
				m_cached_temp_dir = weakly_canonical(path(__FILE__).parent_path() / L".." / L".." / L".." / L"obj" / L"unittests" / class_name() / Platform / Config / "");

				// Remove files from previous test runs, but keep files written earlier in this run.
				if (!s_temp_dir_cleaned && std::filesystem::exists(m_cached_temp_dir))
				{
					try { std::filesystem::remove_all(m_cached_temp_dir); }
					catch (std::exception const& ex)
					{
						TestFramework::out() << "Warning: Failed to clear temp directory. " << ex.what() << std::endl;
					}
				}
				s_temp_dir_cleaned = true;
				std::filesystem::create_directories(m_cached_temp_dir);
			}
			return m_cached_temp_dir;
		}

		// A stream for informational output from the running test, such as timings, sweep tables, and measured values.
		// Written to '<temp_dir>/<test name>.log' so that console output only reports test results.
		std::ostream& log_stream()
		{
			// Open the log file on first use so that tests without informational output create no file.
			if (!m_log_stream.is_open())
			{
				m_log_stream.open(temp_dir() / std::format("{}.log", m_test_name));
				if (!m_log_stream)
					throw std::runtime_error(std::format("Failed to open unit test log file for {}", m_test_name));
			}
			return m_log_stream;
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

	// Options controlling a unit test run
	struct UnitTestOptions
	{
		// Write one line per test, rather than only failures and the summary
		bool wordy = false;

		// Class-name patterns (see 'MatchTestFilter'). A test runs when its class matches any filter (or there are no filters) and no exclude.
		std::vector<std::string_view> filters = {};
		std::vector<std::string_view> excludes = {};

		// Flag expressions. A test runs when it has all the flags of any one expression (or there are no expressions).
		std::vector<EUnitTestFlags> flags = {};

		// The number of test classes that run at the same time. Zero means one per hardware thread. One runs everything on the calling thread.
		int threads = 0;

		// Report assertion failures to stderr rather than showing dialogs, so unattended runs (e.g. from a build) cannot block.
		bool asserts_to_stderr = false;
	};

	// Parse the standard unit test command line into 'options'.
	// Supports '-verbose', '-exclude:<pattern>', '-flags:<expression>', '-threads:<count>', '-asserts:stderr'. Any other argument is a class filter.
	// Returns false, after writing a message to 'err', when an argument is invalid.
	inline bool TryParseUnitTestCommandLine(std::span<char* const> args, UnitTestOptions& options, std::ostream& err)
	{
		for (auto const* a : args)
		{
			// Match each switch by its prefix, with everything else treated as a class filter.
			auto const arg = std::string_view{a};
			if (arg == "-verbose")
			{
				options.wordy = true;
			}
			else if (arg == "-asserts:stderr")
			{
				options.asserts_to_stderr = true;
			}
			else if (arg.starts_with("-exclude:"))
			{
				options.excludes.push_back(arg.substr(std::string_view{"-exclude:"}.size()));
			}
			else if (arg.starts_with("-flags:"))
			{
				// Reject invalid filters before running a success-shaped empty selection.
				auto flags = EUnitTestFlags::None;
				auto const expression = arg.substr(std::string_view{"-flags:"}.size());
				if (!TryParseUnitTestFlags(expression, flags))
				{
					err << "Invalid unit-test flags: " << expression << std::endl;
					return false;
				}
				options.flags.push_back(flags);
			}
			else if (arg.starts_with("-threads:"))
			{
				// Require the whole value to be a non-negative integer.
				auto const value = arg.substr(std::string_view{"-threads:"}.size());
				auto threads = 0;
				auto [end, ec] = std::from_chars(value.data(), value.data() + value.size(), threads);
				if (ec != std::errc{} || end != value.data() + value.size() || threads < 0)
				{
					err << "Invalid unit-test thread count: " << value << std::endl;
					return false;
				}
				options.threads = threads;
			}
			else
			{
				options.filters.push_back(arg);
			}
		}
		return true;
	}

	// The outcome of one executed test
	struct UnitTestResult
	{
		UnitTestItem const* m_test = nullptr; // The test that was run
		double m_duration_ms = 0;             // Time spent in the test function
		int m_checks = 0;                     // The number of checks the test made on its own thread
		bool m_passed = false;                // True if the test did not throw
	};

	namespace impl
	{
		// The name of a test class without the 'struct ' prefix that 'type_info::name' adds
		inline std::string_view TestClassName(std::type_info const& type)
		{
			auto name = std::string_view{type.name()};
			if (auto space = name.find(' '); space != std::string_view::npos)
				name = name.substr(space + 1);

			return name;
		}

		// Format test flags as a pipe-separated expression
		inline std::string FlagsText(EUnitTestFlags flags)
		{
			// Emit names in a fixed order so the report is stable between runs.
			auto text = std::string{};
			auto append = [&](EUnitTestFlags flag, char const* name)
			{
				// Separate each present flag from the previous one.
				if (!HasAllUnitTestFlags(flags, flag))
					return;

				if (!text.empty())
					text += '|';

				text += name;
			};
			append(EUnitTestFlags::Quick, "Quick");
			append(EUnitTestFlags::Extended, "Extended");
			append(EUnitTestFlags::Stress, "Stress");
			append(EUnitTestFlags::Serial, "Serial");
			return text;
		}

		// The timing report for this executable: '<repo>/obj/unittests/<executable name>/timings.csv'
		inline std::filesystem::path TimingReportPath()
		{
			// Name the report after the running executable so separate test runners do not overwrite each other.
			wchar_t module[MAX_PATH] = {};
			GetModuleFileNameW(nullptr, &module[0], MAX_PATH);
			auto const runner = std::filesystem::path(&module[0]).stem();
			return std::filesystem::weakly_canonical(std::filesystem::path(__FILE__).parent_path() / L".." / L".." / L".." / L"obj" / L"unittests" / runner / L"timings.csv");
		}

		// Read the total duration of each class from a previous timing report. Returns an empty map when there is no report.
		inline std::unordered_map<std::string, double> ReadClassDurations(std::filesystem::path const& report)
		{
			// The first two columns are the class name and test name. The fifth is the duration. None of these contain commas.
			auto durations = std::unordered_map<std::string, double>{};
			auto file = std::ifstream{report};
			auto line = std::string{};
			std::getline(file, line);
			while (std::getline(file, line))
			{
				// Split out the leading columns, and skip any malformed line.
				auto columns = std::vector<std::string_view>{};
				for (auto rest = std::string_view{line}; columns.size() != 5;)
				{
					auto comma = rest.find(',');
					columns.push_back(rest.substr(0, comma));
					if (comma == std::string_view::npos)
						break;

					rest = rest.substr(comma + 1);
				}
				if (columns.size() != 5)
					continue;

				// Accumulate the test duration into its class total.
				auto ms = 0.0;
				std::from_chars(columns[4].data(), columns[4].data() + columns[4].size(), ms);
				durations[std::string(columns[0])] += ms;
			}
			return durations;
		}

		// Write the per-test timing report, slowest first
		inline void WriteTimingReport(std::filesystem::path const& report, std::span<UnitTestResult const> results)
		{
			// Sort a copy so the report can be read top-down to find tests that do not belong in the 'Quick' group.
			auto sorted = std::vector<UnitTestResult>(results.begin(), results.end());
			std::ranges::stable_sort(sorted, [](auto const& lhs, auto const& rhs) { return lhs.m_duration_ms > rhs.m_duration_ms; });

			// Write one row per executed test. The file location is quoted and last because it may contain commas.
			std::filesystem::create_directories(report.parent_path());
			auto file = std::ofstream{report, std::ios::out | std::ios::trunc};
			file << "class,test,flags,result,duration_ms,checks,location\n";
			for (auto const& r : sorted)
			{
				file << std::format("{},{},{},{},{:.3f},{},\"{}({})\"\n",
					TestClassName(*r.m_test->m_class),
					r.m_test->m_name,
					FlagsText(r.m_test->m_flags),
					r.m_passed ? "pass" : "fail",
					r.m_duration_ms,
					r.m_checks,
					r.m_test->m_file,
					r.m_test->m_line);
			}
			if (!file)
				throw std::runtime_error(std::format("Failed to write unit test timing report '{}'", report.string()));
		}

		// Run one test, reporting its outcome to the output stream
		inline UnitTestResult RunTest(UnitTestItem const& test, bool wordy, bool single_threaded)
		{
			using namespace std::chrono;

			// When single-threaded, show the test name before it runs so a test that hangs or crashes can be identified.
			auto const name = std::format("{}.{}", TestClassName(*test.m_class), test.m_name);
			if (wordy && single_threaded)
			{
				std::lock_guard lock(TestFramework::OutputLock);
				TestFramework::out() << std::format("{:.<100}", name) << std::flush;
			}

			// Run the test, turning any exception into a failure.
			auto error = std::string{};
			TestFramework::TestCount = 0;
			auto const t0 = steady_clock::now();
			try
			{
				test.m_func();
			}
			catch (std::exception const& e)
			{
				error = e.what();
			}
			catch (...)
			{
				error = "Unknown exception";
			}
			auto const t1 = steady_clock::now();
			auto const result = UnitTestResult{
				.m_test = &test,
				.m_duration_ms = duration<double, std::milli>(t1 - t0).count(),
				.m_checks = TestFramework::TestCount,
				.m_passed = error.empty(),
			};

			// Write the outcome in one piece so that output from tests running at the same time does not interleave.
			auto text = std::string{};
			auto const prefix = wordy && !single_threaded ? std::format("{:.<100}", name) : std::string{};
			if (!result.m_passed)
				text = std::format("{}{}\n   {}({}): {} failed\n", prefix, error, test.m_file, test.m_line, name);
			else if (wordy)
				text = std::format("{}success. ({:8} tests in {:4.3f} ms)\n", prefix, result.m_checks, result.m_duration_ms);

			if (!text.empty())
			{
				std::lock_guard lock(TestFramework::OutputLock);
				TestFramework::out() << text << std::flush;
			}
			return result;
		}
	}

	// Run the registered tests selected by 'options'.
	// Test classes run on a pool of worker threads, one class per worker at a time, with tests in a class running in order on the same thread.
	// Classes with any 'Serial' test run afterwards on the calling thread, with nothing else running. Returns 0 if all selected tests pass.
	inline int RunAllTests(UnitTestOptions const& options)
	{
		using namespace std::chrono;
		try
		{
			// Unattended runs must report assertion failures rather than block on a dialog.
			if (options.asserts_to_stderr)
				RouteAssertsToStdErr();

			TestFramework::out() << " **** Begin Unit Tests **** " << std::endl;
			auto const T0 = steady_clock::now();

			// Sort by class, then test name, so each class's tests are next to each other and run in a stable order.
			std::ranges::sort(TestFramework::Tests, [](UnitTestItem const& lhs, UnitTestItem const& rhs)
			{
				// Order by class name, then by test name within a class.
				auto c = strcmp(lhs.m_class->name(), rhs.m_class->name());
				return c != 0 ? c < 0 : strcmp(lhs.m_name, rhs.m_name) < 0;
			});

			// Select the tests that pass the flag and class-name filters.
			auto selected = std::vector<UnitTestItem const*>{};
			auto skipped = 0;
			for (auto const& test : TestFramework::Tests)
			{
				// A filter expression requires all its flags, while separate expressions are alternatives.
				auto const class_name = std::string_view(test.m_class->name());
				auto const included =
					(options.flags.empty() || std::ranges::any_of(options.flags, [&](auto required) { return HasAllUnitTestFlags(test.m_flags, required); })) &&
					(options.filters.empty() || std::ranges::any_of(options.filters, [&](auto const& f) { return MatchTestFilter(class_name, f); })) &&
					!std::ranges::any_of(options.excludes, [&](auto const& f) { return MatchTestFilter(class_name, f); });
				if (included)
					selected.push_back(&test);
				else
					++skipped;
			}

			// Group the selected tests into class runs. A class is serial if any of its tests has the 'Serial' flag.
			struct ClassRun { std::size_t m_begin; std::size_t m_end; double m_expected_ms; };
			auto parallel_runs = std::vector<ClassRun>{};
			auto serial_runs = std::vector<ClassRun>{};
			auto const report = impl::TimingReportPath();
			auto const previous = impl::ReadClassDurations(report);
			for (std::size_t begin = 0, end = 0; begin != selected.size(); begin = end)
			{
				// Find the end of this class's tests and whether any requires serial execution.
				auto serial = false;
				for (end = begin; end != selected.size() && selected[end]->m_class == selected[begin]->m_class; ++end)
					serial |= HasAllUnitTestFlags(selected[end]->m_flags, EUnitTestFlags::Serial);

				auto const found = previous.find(std::string(impl::TestClassName(*selected[begin]->m_class)));
				auto const expected_ms = found != previous.end() ? found->second : 0.0;
				(serial ? serial_runs : parallel_runs).push_back(ClassRun{ .m_begin = begin, .m_end = end, .m_expected_ms = expected_ms });
			}

			// Start the classes that took longest last time first, so a slow class does not start near the end and extend the total run time.
			std::ranges::stable_sort(parallel_runs, [](auto const& lhs, auto const& rhs) { return lhs.m_expected_ms > rhs.m_expected_ms; });

			// Each test writes only its own result slot, so the workers do not need to synchronise access to 'results'.
			auto const hardware_threads = static_cast<int>(std::max(1U, std::thread::hardware_concurrency()));
			auto const thread_count = std::min<std::size_t>(options.threads != 0 ? options.threads : hardware_threads, parallel_runs.size());
			auto results = std::vector<UnitTestResult>(selected.size());
			auto run_class = [&](ClassRun const& run, bool single_threaded)
			{
				// Run the class's tests in order on this thread, then release the fixtures they shared.
				for (auto i = run.m_begin; i != run.m_end; ++i)
					results[i] = impl::RunTest(*selected[i], options.wordy, single_threaded);

				TestFramework::ReleaseClassFixtures();
			};

			// Run the parallel classes, using the calling thread directly when only one worker is needed.
			if (thread_count <= 1)
			{
				for (auto const& run : parallel_runs)
					run_class(run, true);
			}
			else
			{
				// Workers take the next class from a shared index until none remain. Destroying the 'jthread's waits for them to finish.
				auto next = std::atomic<std::size_t>{0};
				auto workers = std::vector<std::jthread>{};
				for (auto i = std::size_t{}; i != thread_count; ++i)
				{
					workers.emplace_back([&]
					{
						// Claim class runs one at a time.
						for (auto index = next++; index < parallel_runs.size(); index = next++)
							run_class(parallel_runs[index], false);
					});
				}
				workers.clear();
			}

			// Run the serial classes with nothing else running.
			for (auto const& run : serial_runs)
				run_class(run, true);

			auto const T1 = steady_clock::now();

			// Count the outcomes and write the timing report.
			auto const passed = static_cast<int>(std::ranges::count_if(results, [](auto const& r) { return r.m_passed; }));
			auto const failed = static_cast<int>(results.size()) - passed;
			auto test_time_ms = 0.0;
			for (auto const& r : results)
				test_time_ms += r.m_duration_ms;

			auto const wall_time_ms = duration<double, std::milli>(T1 - T0).count();
			try
			{
				impl::WriteTimingReport(report, results);
			}
			catch (std::exception const& ex)
			{
				TestFramework::out() << "Warning: " << ex.what() << std::endl;
			}

			// List the slowest tests to show which may not belong in the 'Quick' group.
			auto slowest = results;
			std::ranges::sort(slowest, [](auto const& lhs, auto const& rhs) { return lhs.m_duration_ms > rhs.m_duration_ms; });
			slowest.resize(std::min<std::size_t>(slowest.size(), 10));
			if (!slowest.empty())
			{
				TestFramework::out() << " Slowest tests:\n";
				for (auto const& r : slowest)
					TestFramework::out() << std::format("   {:10.1f} ms  {}.{}\n", r.m_duration_ms, impl::TestClassName(*r.m_test->m_class), r.m_test->m_name);

				TestFramework::out() << std::format(" Timing report: {}\n", report.string());
			}

			// Print the results
			auto const timing = std::format("(taking {:1.3f} ms, {:1.3f} ms of test time on {} thread(s))", wall_time_ms, test_time_ms, std::max<std::size_t>(thread_count, 1));
			if (failed == 0)
			{
				TestFramework::out() << std::format(" **** UnitTest results: All {} unit tests passed.", passed);
				if (skipped > 0) TestFramework::out() << std::format(" ({} skipped by filter)", skipped);
				TestFramework::out() << std::format(" {} ****\n", timing);
			}
			else
			{
				TestFramework::out() << std::format(" **** UnitTest results: {} of {} failed.", failed, failed + passed);
				if (skipped > 0) TestFramework::out() << std::format(" ({} skipped by filter)", skipped);
				TestFramework::out() << std::format(" {} ****\n", timing);
			}
			TestFramework::out() << std::flush;
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

// Test class. Optionally pass 'Serial' to stop the class running at the same time as any other test class.
#define PRUnitTestClass(classname, ...)\
struct TestClass_##classname : pr::unittests::UnitTestBase<TestClass_##classname, pr::unittests::ParseUnitTestClassFlags("" #__VA_ARGS__)>

// Test method with its execution properties
#define PRUnitTestMethod(methodname, flags, ...)\
	template <typename... Types>\
	static void Test_##methodname##_()\
	{\
		/* Run on a fresh instance named after the method, so each test gets its own log file. */\
		typename base_t::test_class_type t;\
		t.m_test_name = #methodname;\
		if constexpr (sizeof...(Types) > 0)\
			(t.Test_##methodname<Types>(), ...);\
		else\
			t.Test_##methodname<void>();\
	}\
	static_assert(pr::unittests::IsValidUnitTestFlags(PR_UNITTEST_STRINGISE(flags)), "Unknown unit-test flag");\
	inline static bool s_registered_##methodname = pr::unittests::TestFramework::AddTest<test_class_type>(\
		#methodname,\
		+[](){ Test_##methodname##_<##__VA_ARGS__>(); },\
		pr::unittests::ParseUnitTestFlags(PR_UNITTEST_STRINGISE(flags)) | base_t::class_flags,\
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
