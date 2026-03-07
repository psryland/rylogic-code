// Shamelessly lifted from Steve Robb's fuz library

#pragma once

#include "pr/meta/square_root.h"

namespace pr
{
	namespace meta
	{
		namespace impl
		{
			namespace is_prime
			{
				template <std::size_t N, std::size_t SqrtN, std::size_t D, bool DGreaterThanSqrtN, bool NModDEqualsZero>
				struct impl2
				{
					enum { value = impl2<N, SqrtN, D + 2, (D + 2 > SqrtN), !(N % (D + 2))>::value };
				};

				template <std::size_t N, std::size_t SqrtN, std::size_t D, bool NModDEqualsZero>
				struct impl2<N, SqrtN, D, true, NModDEqualsZero>
				{
					enum { value = true };
				};

				template <std::size_t N, std::size_t SqrtN, std::size_t D>
				struct impl2<N, SqrtN, D, false, true>
				{
					enum { value = false };
				};

				template <std::size_t N, bool ObviousNonPrime>
				struct impl
				{
					enum { value = impl2<N, meta::square_root<N>::value, 1, false, false>::value };
				};

				template <std::size_t N>
				struct impl<N, true>
				{
					enum { value = false };
				};
			}
		}

		template <std::size_t N>
		struct is_prime
		{
			enum { value = impl::is_prime::impl<N, (N < 2) || (N != 2 && !(N % 2))>::value };
		};
	}
}