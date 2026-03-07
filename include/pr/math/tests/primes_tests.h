//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/math/math.h"

namespace pr::math::tests
{
	PRUnitTestClass(PrimesTests)
	{
		PRUnitTestMethod(PrimeTable)
		{
			// Verify the table count
			auto count = std::size(primes);
			PR_EXPECT(count == 1000);

			// First prime is 2
			PR_EXPECT(primes[0] == 2);

			// Last prime in the table is 7919 (the 1000th prime)
			PR_EXPECT(primes[count - 1] == 7919);

			// Table is sorted
			for (size_t i = 1; i != count; ++i)
			{
				PR_EXPECT(primes[i] > primes[i - 1]);
			}
		}

		PRUnitTestMethod(IsPrimeBasic)
		{
			// Known non-primes
			PR_EXPECT(!IsPrime(0));
			PR_EXPECT(!IsPrime(1));
			PR_EXPECT(!IsPrime(4));
			PR_EXPECT(!IsPrime(6));
			PR_EXPECT(!IsPrime(9));
			PR_EXPECT(!IsPrime(15));
			PR_EXPECT(!IsPrime(100));

			// Known primes
			PR_EXPECT(IsPrime(2));
			PR_EXPECT(IsPrime(3));
			PR_EXPECT(IsPrime(5));
			PR_EXPECT(IsPrime(7));
			PR_EXPECT(IsPrime(11));
			PR_EXPECT(IsPrime(13));
			PR_EXPECT(IsPrime(97));
			PR_EXPECT(IsPrime(7919));

			// Negative values
			PR_EXPECT(!IsPrime(-1));
			PR_EXPECT(!IsPrime(-7));
		}

		PRUnitTestMethod(IsPrimeAgainstTable)
		{
			// Cross-check IsPrime against the prime table
			auto count = std::size(primes);
			for (int i = 0, j = 0; i <= primes[count - 1]; ++i)
			{
				auto is_prime = (j < static_cast<int>(count)) && (i == primes[j]);
				PR_EXPECT(IsPrime(i) == is_prime);
				j += int(is_prime);
			}
		}

		PRUnitTestMethod(IsPrimeConstexpr)
		{
			// Verify IsPrime works at compile time
			static_assert(IsPrime(2));
			static_assert(IsPrime(7919));
			static_assert(!IsPrime(0));
			static_assert(!IsPrime(1));
			static_assert(!IsPrime(4));
		}

		PRUnitTestMethod(PrimeGtrThan)
		{
			// Edge cases
			PR_EXPECT(PrimeGtrThan(0) == 2);
			PR_EXPECT(PrimeGtrThan(1) == 2);
			PR_EXPECT(PrimeGtrThan(2) == 3);
			PR_EXPECT(PrimeGtrThan(3) == 5);
			PR_EXPECT(PrimeGtrThan(4) == 5);

			// Sequential primes
			PR_EXPECT(PrimeGtrThan(10) == 11);
			PR_EXPECT(PrimeGtrThan(11) == 13);
			PR_EXPECT(PrimeGtrThan(96) == 97);
			PR_EXPECT(PrimeGtrThan(97) == 101);

			// Find the 10,000th prime by iterating
			auto p = 2;
			for (int i = 0; i != 10000; ++i)
			{
				p = PrimeGtrThan(p);
			}
			PR_EXPECT(p == 104743);
			PR_EXPECT(IsPrime(p));
		}

		PRUnitTestMethod(PrimeLessThan)
		{
			// Edge cases
			PR_EXPECT(PrimeLessThan(3) == 2);
			PR_EXPECT(PrimeLessThan(4) == 3);
			PR_EXPECT(PrimeLessThan(5) == 3);
			PR_EXPECT(PrimeLessThan(6) == 5);

			// Sequential primes
			PR_EXPECT(PrimeLessThan(12) == 11);
			PR_EXPECT(PrimeLessThan(100) == 97);
			PR_EXPECT(PrimeLessThan(97) == 89);

			// Walk backwards through the entire prime table
			auto count = std::size(primes);
			auto p = primes[count - 1] + 1;
			for (int i = static_cast<int>(count); i-- != 0; )
			{
				p = PrimeLessThan(p);
				PR_EXPECT(p == primes[i]);
				PR_EXPECT(IsPrime(p));
			}
		}

		PRUnitTestMethod(PrimeGtrLessRoundTrip)
		{
			// For several primes, verify that PrimeLessThan(PrimeGtrThan(p)) == p
			for (auto p : {2, 3, 5, 7, 11, 97, 997, 7919})
			{
				auto next = PrimeGtrThan(p);
				auto prev = PrimeLessThan(next);
				PR_EXPECT(prev == p);
			}
		}
	};
}
#endif
