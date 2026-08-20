// Algorithm from this page: http://www.embedded.com/98/9802fe2.htm
// This is the classic digit-by-digit (base-4) integer square root algorithm: each iteration
// pulls the next 2 bits off the top of N and appends one bit to Root, so it needs Bits/2
// iterations to consume all of N's bits, where Bits is the width of std::size_t. Each
// iteration exposes the next 2 bits by right-shifting N by Bits-2, so that shift amount
// must also scale with Bits rather than being a fixed constant - hardcoding both values to
// the width of a 32-bit size_t silently truncated the result on platforms (e.g. x64) where
// std::size_t is wider than 32 bits.

#pragma once
#ifndef PR_META_SQRT_H
#define PR_META_SQRT_H

#include <cstddef>
#include "pr/meta/constants.h"

namespace pr
{
	namespace meta
	{
		namespace impl
		{
			template <std::size_t Iter, std::size_t Shift, std::size_t N, std::size_t Root = 0, std::size_t Rem = 0>
			struct square_root_iter
			{
				static const std::size_t divisor = (Root << 2) + 1;
				static const std::size_t nextrem = (Rem  << 2) + (N >> Shift);
				static const std::size_t mult	 = !!(divisor <= nextrem);
				static const std::size_t value	 = square_root_iter<Iter - 1, Shift, N << 2, (Root << 1) + mult, nextrem - mult * divisor>::value;
			};

			template <std::size_t Shift, std::size_t N, std::size_t Root, std::size_t Rem>
			struct square_root_iter<0, Shift, N, Root, Rem>
			{
				static const std::size_t value = Root;
			};
		}

		template <std::size_t N>
		struct square_root
		{
			// std::size_t's bit width determines how many 2-bit digits of N need to be consumed,
			// and where in N the top 2 bits sit before each iteration's left-shift by 2.
			static const std::size_t bits  = sizeof(std::size_t) * 8;
			static const std::size_t value = impl::square_root_iter<bits / 2, bits - 2, N>::value;
		};

	}
}

#endif PR_META_SQRT_H

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/meta/is_prime.h"
#include "pr/meta/prime_gtreq.h"
namespace pr::meta
{
	PRUnitTest(SquareRootTests, Quick)
	{
		// 0/1 boundaries, and exact vs non-exact squares
		static_assert(square_root<0>::value == 0, "");
		static_assert(square_root<1>::value == 1, "");
		static_assert(square_root<2>::value == 1, "");
		static_assert(square_root<3>::value == 1, "");
		static_assert(square_root<4>::value == 2, "");
		static_assert(square_root<5>::value == 2, "");
		static_assert(square_root<8>::value == 2, "");
		static_assert(square_root<9>::value == 3, "");
		static_assert(square_root<15>::value == 3, "");
		static_assert(square_root<16>::value == 4, "");
		static_assert(square_root<17>::value == 4, "");
		static_assert(square_root<24>::value == 4, "");
		static_assert(square_root<25>::value == 5, "");
		static_assert(square_root<26>::value == 5, "");
		static_assert(square_root<65535>::value == 255, "");
		static_assert(square_root<65536>::value == 256, "");
		static_assert(square_root<65537>::value == 256, "");
		static_assert(square_root<999999>::value == 999, "");
		static_assert(square_root<1000000>::value == 1000, "");
		static_assert(square_root<1000001>::value == 1000, "");

		// Values that straddle the 32-bit boundary, and the largest representable size_t
		static_assert(square_root<4294967295ULL>::value == 65535, "");                        // 2^32 - 1
		static_assert(square_root<4294967296ULL>::value == 65536, "");                        // 2^32
		static_assert(square_root<4294967297ULL>::value == 65536, "");                        // 2^32 + 1
		static_assert(square_root<18446744065119617025ULL>::value == 4294967295ULL, "");       // (2^32 - 1)^2
		static_assert(square_root<18446744073709551615ULL>::value == 4294967295ULL, "");       // 2^64 - 1 (SIZE_MAX)

		// is_prime/prime_gtreq use square_root as an upper bound for trial division, so an
		// underestimated square root would let them stop before ruling out a real factor.
		static_assert(is_prime<2>::value == true, "");
		static_assert(is_prime<3>::value == true, "");
		static_assert(is_prime<4>::value == false, "");
		static_assert(is_prime<17>::value == true, "");
		static_assert(is_prime<25>::value == false, ""); // 5*5, exactly at sqrt(25)
		static_assert(is_prime<49>::value == false, ""); // 7*7
		static_assert(is_prime<997>::value == true, "");
		static_assert(is_prime<999983>::value == true, "");
		static_assert(is_prime<999999>::value == false, "");
		static_assert(prime_gtreq<10>::value == 11, "");
		static_assert(prime_gtreq<1000000>::value == 1000003, "");
	}
}
#endif
