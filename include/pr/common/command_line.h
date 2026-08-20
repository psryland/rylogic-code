//***********************************************************************
// Command line parser
//  Copyright (c) Rylogic Ltd 2006
//***********************************************************************
// Usage:
//	struct Thing : pr::cmdline::IOptionReceiver
//	{
//		std::string m_input_filename;
//		std::string m_output_filename;
//		bool CmdLineData(TArgIter& arg, TArgIter arg_end) { ++arg; return true; }
//		bool CmdLineOption(std::string const& option, TArgIter& arg, TArgIter arg_end)
//		{
//			if      (pr::str::EqualI(option, "-I") && arg != arg_end) { m_input_filename  = *arg++; return true; }
//			else if (pr::str::EqualI(option, "-O") && arg != arg_end) { m_output_filename = *arg++; return true; }
//			else if (pr::str::EqualI(option, "-h"))                   { ShowHelp(); return true; }
//			printf("Error: Unknown option '%s'\n", option.c_str());
//			return false;
//		}
//		void ShowHelp() { printf("help\n"); }
//	}
//	Thing thing;
//	if (!pr::EnumCommandLine(argc, argv, thing)) return false;
//	//or
//	if (!pr::EnumCommandLine(GetCommandLine(), thing)) return false;
//
#pragma once
#include <concepts>
#include <type_traits>
#include <string_view>
#include <string>
#include <vector>
#include <ranges>
#include <stdexcept>

namespace pr
{
	namespace cmdline
	{
		// Helper to test if 'str' is of the form '-xyz'
		inline bool IsOption(std::string_view str)
		{
			return str.size() >= 2 && (str[0] == '-' || str[0] == '/');
		}

		inline bool IsOption(std::wstring_view str)
		{
			return str.size() >= 2 && (str[0] == '-' || str[0] == '/');
		}

		// Convert a command line string in to argv format by inserting '\0' at delimiters
		template <typename Char> requires (std::is_same_v<Char, char> || std::is_same_v<Char, wchar_t>)
		inline std::vector<Char const*> Tokenize(Char* str, size_t len)
		{
			std::vector<Char const*> argv;

			auto s = str;
			auto e = s + len;
			auto is_delim = [](Char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v'; };
			for (; s != e; s += int(s != e))
			{
				if (*s == '\"' || *s == '\'')
				{
					auto quote = *s;
					argv.push_back(s + 1);

					++s;
					for (; s != e && *s != quote; ++s) {}
					if (s != e) *s = '\0'; // Null-terminate inside the closing quote
					continue;
				}
				if (!is_delim(*s))
				{
					argv.push_back(s);

					for (; s != e && !is_delim(*s); ++s) {}
					if (s != e) *s = '\0'; // Null-terminate at the delimiter
					continue;
				}
			}

			return argv;
		}
	}

	// --------------------------------------------------------------------------------------------

	struct CmdLine
	{
		// Expected format:
		//   arg0 cmd ... --arg value value ... --arg value ... -a value value ...
		//
		//  - First argument (arg0) is the program name
		//  - "cmd ..." are N sequential arguments (commands) stored as 'key' values with no 'values'
		//  - "--arg value ..." are key to value(s) pairs. After an "--arg", all values that don't start with "-" are part of the argument's data
		struct Arg
		{
			std::string key;
			std::vector<std::string> values;

			// The number of values associated with this argument
			int num_values() const
			{
				return static_cast<int>(values.size());
			}

			// Interpret a value as type 'T'
			template <typename T> T as(int idx = 0) const
			{
				if constexpr (std::floating_point<T>)
				{
					return static_cast<T>(std::stod(values[idx]));
				}
				else if constexpr (std::integral<T>)
				{
					return static_cast<T>(std::stoll(values[idx]));
				}
				else if constexpr (std::is_convertible_v<T, std::filesystem::path>)
				{
					return T(values[idx]);
				}
				else
				{
					static_assert(std::is_same_v<T, std::false_type>, "Unsupported type conversion");
					throw std::runtime_error("Unsupported type conversion");
				}
			}
		
			// Interpret a value as a key=value pair.
			Arg kv(int idx = 0) const
			{
				auto const& value = values[idx];
				auto const eq = value.find('=');
				if (eq == std::string::npos)
					throw std::runtime_error("Key/value pair missing '=' delimiter");

				auto k = value.substr(0, eq);
				auto v = value.substr(eq + 1);
				return Arg{ std::move(k), {std::move(v)} };
			}

			// Ranged-for helper for access to all values as key=value pairs
			auto kv_pairs() const
			{
				struct I
				{
					Arg const* m_this;
					int m_index;
					I& operator++() { ++m_index; return *this; }
					Arg operator*() const { return m_this->kv(m_index); }
					bool operator == (I const& rhs) const { return m_this == rhs.m_this && m_index == rhs.m_index; }
				};
				struct R
				{
					Arg const* m_this;
					auto begin() const { return I{m_this, 0}; }
					auto end() const { return I{ m_this, m_this->num_values() }; }
				};
				return R{ this };
			}
		};

		std::string arg0;
		std::vector<Arg> args;

		CmdLine()
			: arg0()
			, args()
		{
		}
		CmdLine(int argc, char const* argv[])
			: arg0(argc > 0 ? argv[0] : "")
			, args()
		{
			for (int i = 1; i < argc; )
			{
				Arg arg = {};

				std::string_view token(argv[i++]);
				if (token.starts_with("-"))
				{
					// Trim the '--' from the key
					for (; token.empty() || token[0] == '-'; token.remove_prefix(1)) {}
					arg.key = token;

					// Read the next arguments as data, up to the next arg that starts with '-'
					for (; i < argc; ++i)
					{
						std::string_view value(argv[i]);
						if (value.starts_with("-")) break;
						arg.values.push_back(std::string(value));
					}
				}
				else
				{
					// Otherwise, this is a command argument with no key
					arg.values.push_back(std::string(token));
				}
				args.push_back(arg);
			}
		}
		CmdLine(int argc, char* argv[])
			: CmdLine(argc, const_cast<char const**>(argv))
		{}
		CmdLine(std::string_view command_line)
			: arg0()
			, args()
		{
			auto buf = std::string(command_line);
			auto argv = cmdline::Tokenize(buf.data(), buf.size());
			*this = CmdLine(int(argv.size()), argv.data());
		}

		// Count the number of occurrances of the given key
		int count(std::string_view key) const
		{
			int count = 0;
			for (auto& a : args) count += a.key == key;
			return count;
		}

		// Access an argument by key value
		Arg const& operator()(std::string_view key, int start_index = 0) const
		{
			for (; start_index < ssize(args); ++start_index)
			{
				if (std::ranges::equal(args[start_index].key, key, [](char a, char b) { return std::tolower(a) == std::tolower(b); }))
					return args[start_index];
			}
			throw std::runtime_error(std::format("Argument {} not found", key));
		}

		// Check that 'key' is provided exactly 'occurrances' times
		bool check(std::string_view key, int min_occurrances = 1, int max_occurrances = 1) const
		{
			auto n = count(key);
			if (n == 0 && min_occurrances != 0)
				throw std::runtime_error(std::format("Required parameter '--{}' not provided", key));
			if (n != min_occurrances && min_occurrances == max_occurrances)
				throw std::runtime_error(std::format("Parameter '--{}' expected {} times", key, min_occurrances));
			if (n < min_occurrances || n > max_occurrances)
				throw std::runtime_error(std::format("Parameter '--{}' expected {}-{} times", key, min_occurrances, max_occurrances));
			return true;
		}
	};

	// --------------------------------------------------------------------------------------------

	namespace cmdline
	{
		// Interface for receiving command line options
		template <typename Char = char>
		struct IOptionReceiver
		{
			using OptionString = std::basic_string<Char>;
			using TArgs        = std::vector<OptionString>;
			using TArgIter     = typename TArgs::const_iterator;

			virtual ~IOptionReceiver() {}

			// Helper to test if 'str' is of the form '-xyz'
			virtual bool IsOption(OptionString const& str) const { return cmdline::IsOption(str); }

			// Called for anything not preceded by '-'.
			// The caller should advance 'arg' for each argument read.
			// Return true to continue parsing, false to abort parsing, or
			// set arg = arg_end to end parsing and have true returned.
			virtual bool CmdLineData(TArgIter& arg, TArgIter /*arg_end*/) { ++arg; return true; }

			// Called when an option is found. An option is anything preceded by a '-'.
			// 'option' is the name of the option, including the '-'.
			// 'arg' is an iterator to the next command line value after 'option'
			// 'arg_end' is the end of the argument vector
			// e.g. -Option arg0 arg1 arg2
			// The caller should advance 'arg' for each argument read.
			// Return true to continue parsing, false to abort parsing, or
			// set arg = arg_end to end parsing and have true returned.
			virtual bool CmdLineOption(OptionString const& /*option*/, TArgIter& /*arg*/, TArgIter /*arg_end*/) { return true; }
		};
	}

	// Parse a range of command line arguments
	// Returns true if all command line parameters were parsed
	template <typename Char = char, typename TArgIter = cmdline::IOptionReceiver<Char>::TArgIter>
	inline bool EnumCommandLine(TArgIter arg, TArgIter arg_end, cmdline::IOptionReceiver<Char>& receiver)
	{
		for (;arg != arg_end;)
		{
			if (cmdline::IsOption(*arg))
			{
				auto option = arg++;
				if (!receiver.CmdLineOption(*option, arg, arg_end))
					return false;
			}
			else
			{
				if (!receiver.CmdLineData(arg, arg_end))
					return false;
			}
		}
		return true;
	}
	
	// Parse console program style command line arguments
	// Returns true if all command line parameters were parsed
	template <typename Char = char>
	inline bool EnumCommandLine(int argc, Char* argv[], cmdline::IOptionReceiver<Char>& receiver)
	{
		// Note: ignoring the argv[0] parameter to make both versions of 'EnumCommandLine'
		// consistent. Programs that want the executable name should use:
		//  char exepath[1024]; GetModuleFileName(0, exepath, sizeof(exepath));
		typename cmdline::IOptionReceiver<Char>::TArgs args;
		for (int i = 1; i < argc; ++i) args.push_back(argv[i]);
		return argc == 0 || EnumCommandLine(args.begin(), args.end(), receiver);
	}
	
	// Parse windows program style command line arguments
	// Returns true if all command line parameters were parsed
	template <typename Char = char>
	inline bool EnumCommandLine(Char const* cmd_line, cmdline::IOptionReceiver<Char>& receiver)
	{
		auto buf = std::basic_string<Char>(cmd_line);
		auto tokens = cmdline::Tokenize(buf.data(), buf.size());

		// Convert to strings for the IOptionReceiver interface
		typename cmdline::IOptionReceiver<Char>::TArgs args;
		for (auto t : tokens) args.push_back(t);
		return EnumCommandLine(args.begin(), args.end(), receiver);
	}

	// --------------------------------------------------------------------------------------------
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTest(CmdLineSinglePositional, Quick)
	{
		char const* argv[] =
		{
			"app",
			"one",
		};

		auto cmd = pr::CmdLine(int(sizeof(argv) / sizeof(argv[0])), argv);

		PR_EXPECT(cmd.args.size() == 1);
		PR_EXPECT(cmd.args[0].key.empty());
		PR_EXPECT(cmd.args[0].values.size() == 1);
		PR_EXPECT(cmd.args[0].values[0] == "one");
	}

	PRUnitTest(CmdLineMultiplePositionals, Quick)
	{
		char const* argv[] =
		{
			"app",
			"one",
			"two",
			"three",
		};

		auto cmd = pr::CmdLine(int(sizeof(argv) / sizeof(argv[0])), argv);

		PR_EXPECT(cmd.args.size() == 3);
		PR_EXPECT(cmd.args[0].key.empty() && cmd.args[0].values.size() == 1 && cmd.args[0].values[0] == "one");
		PR_EXPECT(cmd.args[1].key.empty() && cmd.args[1].values.size() == 1 && cmd.args[1].values[0] == "two");
		PR_EXPECT(cmd.args[2].key.empty() && cmd.args[2].values.size() == 1 && cmd.args[2].values[0] == "three");
	}

	PRUnitTest(CmdLinePositionalsAndOptions, Quick)
	{
		char const* argv[] =
		{
			"app",
			"one",
			"two",
			"-opt",
			"value",
		};

		auto cmd = pr::CmdLine(int(sizeof(argv) / sizeof(argv[0])), argv);

		PR_EXPECT(cmd.args.size() == 3);
		PR_EXPECT(cmd.args[0].key.empty() && cmd.args[0].values.size() == 1 && cmd.args[0].values[0] == "one");
		PR_EXPECT(cmd.args[1].key.empty() && cmd.args[1].values.size() == 1 && cmd.args[1].values[0] == "two");
		PR_EXPECT(cmd.args[2].key == "opt" && cmd.args[2].values.size() == 1 && cmd.args[2].values[0] == "value");
	}
}
#endif