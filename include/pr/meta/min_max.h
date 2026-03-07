//******************************************
// min<> max<>
//  Copyright (c) Rylogic Ltd 2013
//******************************************

#pragma once
#include "pr/common/min_max_fix.h"

namespace pr
{
	namespace meta
	{
		// Compile time min of N integral types
		template <typename Type, Type L, Type R=L, Type... U> struct min
		{
			static Type const value = min<Type, L, min<Type, R, U...>::value>::value;
		};
		template <typename Type, Type L, Type R> struct min<Type,L,R>
		{
			static Type const value = L < R ? L : R;
		};
		template <typename Type, Type L> struct min<Type,L>
		{
			static Type const value = L;
		};

		// Compile time max of N integral types
		template <typename Type, Type L, Type R=L, Type... U> struct max
		{
			static Type const value = max<Type, L, max<Type, R, U...>::value>::value;
		};
		template <typename Type, Type L, Type R> struct max<Type,L,R>
		{
			static Type const value = L < R ? R : L;
		};
		template <typename Type, Type L> struct max<Type,L>
		{
			static Type const value = L;
		};
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::meta
{
	PRUnitTest(MinMaxTests)
	{
		static_assert(meta::min<int,+5,+2>::value == +2, "");
		static_assert(meta::min<int,-5,+2>::value == -5, "");
		static_assert(meta::min<int,+5,-2>::value == -2, "");
		static_assert(meta::min<int,-5,-2>::value == -5, "");

		static_assert(meta::max<int,+5,+2>::value == +5, "");
		static_assert(meta::max<int,-5,+2>::value == +2, "");
		static_assert(meta::max<int,+5,-2>::value == +5, "");
		static_assert(meta::max<int,-5,-2>::value == -2, "");

		static_assert(meta::max<int,-2,3,-1,4>::value == 4, "");
		static_assert(meta::min<char,-2,3,-1,4>::value == -2, "");
		static_assert(meta::max<int,-2>::value == -2, "");
		static_assert(meta::min<char,-2>::value == -2, "");

		enum class EE :__int64
		{
			A = 0x10000000000LL,
			B = -0x10000000000LL,
		};
		static_assert(meta::max<EE, EE::A, EE::B>::value == EE::A, "");
	}
}
#endif