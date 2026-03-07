//******************************************
// abs<>
//  Copyright (c) Rylogic Ltd 2013
//******************************************

#pragma once

namespace pr
{
	namespace meta
	{
		namespace impl
		{
			template <typename Type, Type X, bool Neg> struct abs0              { static const Type value =  X; };
			template <typename Type, Type X>           struct abs0<Type,X,true> { static const Type value = -X; };
		}
	
		// absolute value of 'X'
		template <typename Type, Type X> struct abs :impl::abs0<Type,X,X<0> {};
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::meta
{
	PRUnitTest(AbsTests)
	{
		static_assert(meta::abs<int,+5>::value == +5, "");
		static_assert(meta::abs<int,-5>::value == +5, "");
	}
}
#endif