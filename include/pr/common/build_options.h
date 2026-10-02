//*****************************************************************************************
// Build Options
//  Copyright (c) Rylogic Ltd 2016
//*****************************************************************************************
#pragma once

#include <string>
#include <format>
#include <iterator>
#include <stdexcept>
#include <windows.h>

namespace pr
{
	// Records the build options used to compile.
	// Use 'int' for each member so the template function can output the values
	// Ensure this object is created in user code and passed into a compiled library function.
	// In the library, call 'CheckBuildOptions'.
	// *Don't* sub-class, use composition.
	struct StdBuildOptions
	{
		int WinVer;
		int MscVer;
		int LeanAndMean;
		int ExtraLean;
		int IteratorDebugLevel;

		StdBuildOptions()
			:WinVer(_WIN32_WINNT)
			,MscVer(_MSC_VER)
			#ifdef WIN32_LEAN_AND_MEAN
			,LeanAndMean(1)
			#else
			,LeanAndMean(0)
			#endif
			#ifdef VC_EXTRALEAN
			,ExtraLean(1)
			#else
			,ExtraLean(0)
			#endif
			,IteratorDebugLevel(_ITERATOR_DEBUG_LEVEL)
		{}
	};

	// Call this from a compile library function.
	// Note: you can call it at various points in the code to catch #defines being changed.
	// 'TBuildOptions' should be a type with 'Size' as the first member, and composed of
	// other BuildOption structures and members
	template <typename TBuildOptions> static void CheckBuildOptions(TBuildOptions const& lhs, TBuildOptions const& rhs)
	{
		if (memcmp(&lhs, &rhs, sizeof(TBuildOptions)) != 0)
		{
			auto* lptr = reinterpret_cast<int const*>(&lhs);
			auto* rptr = reinterpret_cast<int const*>(&rhs);

			std::string msg = "Build option values don't match.\nCheck all projects are compiled with the same settings.\n";

			// List every value pair so the mismatched options can be identified.
			for (int i = 0, iend = sizeof(TBuildOptions)/sizeof(int); i != iend; ++i)
				std::format_to(std::back_inserter(msg), "\t{} - {}\n", lptr[i], rptr[i]);

			OutputDebugStringA(msg.c_str());
			throw std::runtime_error(msg);
		}
	}
}
