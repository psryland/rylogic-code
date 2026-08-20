//********************************
// Name of (Type)
//  Copyright (c) Rylogic Ltd 2025
//********************************
// Don't work :-/
#pragma once

#include <iostream>
#include <type_traits>
#include <typeinfo>
#include <string_view>

namespace pr
{
	// Helper to convert a type name to a string at compile time (requires C++20)
	template <typename T>
	constexpr std::string_view nameof()
	{
		#if defined(__clang__)
		constexpr std::string_view prefix = "std::string_view type_name() [T = ";
		constexpr std::string_view suffix = "]";
		constexpr std::string_view function = __PRETTY_FUNCTION__;
		#elif defined(__GNUC__)
		constexpr std::string_view prefix = "constexpr std::string_view type_name() [with T = ";
		constexpr std::string_view suffix = "]";
		constexpr std::string_view function = __PRETTY_FUNCTION__;
		#elif defined(_MSC_VER)
		constexpr std::string_view prefix = "type_name<";
		constexpr std::string_view suffix = ">(void)";
		constexpr std::string_view function = __FUNCSIG__;
		#else
		constexpr std::string_view function = "Unknown compiler";
		#endif
		return function.substr(prefix.size(), function.size() - prefix.size() - suffix.size());
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	struct NameofTestType;
	PRUnitTest(NameofTests, Quick)
	{
	}
}
#endif
