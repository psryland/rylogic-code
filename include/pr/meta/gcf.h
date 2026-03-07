#pragma once

#include "pr/meta/abs.h"
#include "pr/meta/min_max.h"

namespace pr
{
	namespace meta
	{
		namespace impl
		{
			template <typename Type, Type Max, Type Min>            struct gcf0;
			template <typename Type, Type Max, Type Min, bool Zero> struct gcf1 :gcf0<Type, Min, Max % Min> {};
			template <typename Type, Type Max, Type Min>            struct gcf1<Type, Max, Min, true> { static const Type value = Max; };
			template <typename Type, Type Max, Type Min>            struct gcf0 :gcf1<Type, Max, Min, Min == 0> {};
		}
	
		// Compile time greatest common factor
		template <typename Type, Type N1, Type N2>
		struct gcf :impl::gcf0<
			Type,
			max<Type, abs<Type,N1>::value, abs<Type,N2>::value>::value,
			min<Type, abs<Type,N1>::value, abs<Type,N2>::value>::value>
		{};
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::meta
{
	PRUnitTest(GcfTests)
	{
		static_assert(meta::gcf<int,+20,+12>::value == +4, "");
		static_assert(meta::gcf<int,+20,-12>::value == +4, "");
		static_assert(meta::gcf<int,-20,+12>::value == +4, "");
		static_assert(meta::gcf<int,-20,-12>::value == +4, "");

		static_assert(meta::gcf<int,+12,+20>::value == +4, "");
		static_assert(meta::gcf<int,-12,+20>::value == +4, "");
		static_assert(meta::gcf<int,+12,-20>::value == +4, "");
		static_assert(meta::gcf<int,-12,-20>::value == +4, "");
	}
}
#endif
