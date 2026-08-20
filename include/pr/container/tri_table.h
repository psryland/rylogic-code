//**************************************
// Triangular Table
//  Copyright (c) Rylogic Ltd 2007
//**************************************
//
// Helper functions for triangular tables
// Inclusive = A table with entries for [0,N)   vs. [0,N)
// Exclusive = A table with entries for [0,N-1) vs. [1,N)
//
// Lower triangular Inclusive table:
//  ____|_0_|_1_|_2_|_3_|_4_|
//  __0_|_+_                |
//  __1_|_X_|_+_            |
//  __2_|_X_|_X_|_+_        |
//  __3_|_X_|_X_|_X_|_+_    |
//  __4_|_X_|_X_|_X_|_X_|_+_|
//  Size = n * (n + 1) / 2
//
// Lower triangular Exclusive table of size 5 looks like this:
//  ____|_0_|_1_|_2_|_3_|
//  __1_|_X_            |
//  __2_|_X_|_X_        |
//  __3_|_X_|_X_|_X_    |
//  __4_|_X_|_X_|_X_|_X_|
//  Size = n * (n - 1) / 2
//
// Note:
//  It's not possible to have an upper triangular table 'Index' function without
//  knowing the dimension of the table. This is why only lower triangular is supported.
//  i.e.,
//   Upper triangular exclusive table:
//   ____|_1_|_2_|_3_|_4_|
//   __0_|_X_|_X_|_X_|_X_|
//   __1_|   |_X_|_X_|_X_|
//   __2_|       |_X_|_X_|
//   __3_|           |_X_|
//   
//   Upper triangular inclusive table:
//   ____|_0_|_1_|_2_|_3_|_4_|
//   __0_|_+_|_X_|_X_|_X_|_X_|
//   __1_|   |_+_|_X_|_X_|_X_|
//   __3_|       |_+_|_X_|_X_|
//   __2_|           |_+_|_X_|
//   __4_|               |_+_|
//
// To convert an upper triangular table to a lower triangular table
// you need to transpose the data when creating the table.
//
// Also note:
//   n(n - 1)/2 = m(m + 1)/2, if m = n+1:
//   (n+1)((n+1) - 1)/2
//   (n + 1)n/2
//  So sizes of the exclusive table are just sizes of the inclusive table shifted by 1
//
// The easiest way to think about this is to solve it for the inclusive table, then if it's
// actually the exclusive type, just +1 to the largest index.
#pragma once
#include <cstdint>
#include <numbers>
#include <compare>
#include <limits>
#include <cassert>

namespace pr::tri_table
{
	// Exclusive max. Ensure index < MaxIndex
	constexpr int64_t MaxIndex = 1'518'500'250LL;
	
	// Table types,
	enum class EType : int
	{
		Inclusive = +1, // Table includes i-vs-i
		Exclusive = -1, // Table excludes i-vs-i
	};

	// The index for each table dimension
	struct index_pair_t
	{
		int64_t indexA, indexB;
		friend std::strong_ordering operator <=>(index_pair_t, index_pair_t) = default;
	};

	namespace impl
	{
		// Integer square root
		constexpr int64_t ISqrt(int64_t x)
		{
			// Compile time version of the square root.
			//  - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
			//  - This method always converges or oscillates about the answer with a difference of 1.
			//  - returns 0 for x < 0
			struct L {
				constexpr static int64_t NewtonRaphson(int64_t x, int64_t curr, int64_t prev, int64_t pprev) {
					if (curr != prev && curr != pprev)
						return NewtonRaphson(x, (curr + x / curr) >> 1, curr, prev);

					return Abs(x - curr * curr) < Abs(x - prev * prev) ? curr : prev;
				}
				constexpr static int64_t Abs(int64_t x) {
					return x >= 0 ? x : -x;
				}
			};
			return x >= 0 ? L::NewtonRaphson(x, x, 0, 0) : std::numeric_limits<int64_t>::quiet_NaN();
		}
	}

	// Returns the required array size for a 'num_elements' tri-table
	constexpr int64_t Size(EType type, int64_t num_elements)
	{
		return num_elements * (num_elements + int(type)) / 2;
	}

	// Returns the square dimension of the tri-table. (i.e. the inverse of the 'Size' function)
	constexpr int64_t Dimension(EType type, int64_t array_size)
	{
		// Size = S = n(n+t)/2, t = +1 (incl), -1 (excl)
		//'   => n² + n.t - 2S = 0
		//'   => n = (-t +/- sqrt(t² + 8S)) / 2  (quadratic formula)
		// However:
		//  sqrt isn't constexpr
		//  sqrt(1 + 8S) overflows for large S (and doesn't work on GPU with only sqrt(float)
		// But:
		//'   sqrt(1 + 8S) = sqrt(A²*(1 + 8S)/A²) = A*sqrt((1 + 8S)/A²)
		//' let A = 2*sqrt(S) then
		//'   sqrt(1 + 8S) = 2*sqrt(S)*sqrt((1 + 8S)/4S) = 2*sqrt(S)*sqrt(1/(4*S) + 2))
		//'   ~= 2*sqrt(2)*sqrt(S) if S >> 1.
		//'   => n = sqrt(2)*sqrt(S) - t/2. Since n must be positive

		// Remember this is the number of full rows that fit into 'array_size'
		constexpr int64_t small_sizes[] = {
			0,
			1, 1,
			2, 2, 2,
			3, 3, 3, 3,
			4, 4, 4, 4, 4,
		};
		if (array_size + 1 < sizeof(small_sizes)/sizeof(small_sizes[0]))
			return small_sizes[array_size] + int(type == EType::Exclusive);

		auto sqrt_array_size = impl::ISqrt(array_size);
		auto num_elements = static_cast<int64_t>(std::numbers::sqrt2 * sqrt_array_size - int(type) / 2.0);

		// Ensure 'num_elements' is the maximum possible value.
		for (; Size(type, num_elements + 0) > array_size; --num_elements) {}
		for (; Size(type, num_elements + 1) <= array_size; ++num_elements) {}
		return num_elements;
	}

	// Returns the index into a tri-table array for the element (a,b)|(b,a)
	constexpr int64_t Index(EType type, int64_t indexA, int64_t indexB)
	{
		assert(indexA >= 0 && indexB >= 0 && indexA < MaxIndex && indexB < MaxIndex);
		assert((type != EType::Exclusive || indexA != indexB) && "indexA == indexB is invalid for an exclusive table");
		return (indexA < indexB)
			? indexB * (indexB + int(type)) / 2 + indexA
			: indexA * (indexA + int(type)) / 2 + indexB;
	}

	// Inverse of the 'Index' function. Get the indices A and B for a given table index
	constexpr index_pair_t FromIndex(EType type, int64_t tri_index)
	{
		// If 'tri_index' is valid, then the array must have at least 'tri_index+1' elements.
		// The dimension of a table with 'tri_index+1' elements is the length of each side of the table, N.
		// Since 'indexL' is the first item in the last row, it's index value is N - 1.
		auto indexL = Dimension(EType::Inclusive, tri_index); // large

		// The array size of a table with 'indexL' values is the sum of all rows < 'indexL'.
		// Since 'tri_index' equals the required array size - 1, subtract gives the position
		// along the row.
		auto indexS = tri_index - Size(EType::Inclusive, indexL); // small

		// Exclusive tables are the same shape as inclusive tables, except the row index is +1
		return { static_cast<int>(indexS), static_cast<int>(indexL) + int(type == EType::Exclusive)};
	}

	// Type for using statements
	template <EType Type> struct TriTable
	{
		static constexpr int64_t Size(int64_t num_elements)
		{
			return tri_table::Size(Type, num_elements);
		}

		static constexpr int64_t Dimension(int64_t array_size)
		{
			return tri_table::Dimension(Type, array_size);
		}

		static constexpr int64_t Index(int64_t indexA, int64_t indexB)
		{
			return tri_table::Index(Type, indexA, indexB);
		}

		static constexpr index_pair_t FromIndex(int64_t index)
		{
			return tri_table::FromIndex(Type, index);
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::common
{
	PRUnitTestFamily(TriangularTableTests, Slow)
	{
		using namespace pr::tri_table;

		using IT = TriTable<EType::Inclusive>;
		using ET = TriTable<EType::Exclusive>;

		static_assert(IT::Size(0) == 0);
		static_assert(IT::Size(1) == 1);
		static_assert(IT::Size(2) == 3);
		static_assert(IT::Size(3) == 6);
		static_assert(IT::Size(4) == 10);
		static_assert(IT::Size(5) == 15);

		static_assert(ET::Size(0) == 0);
		static_assert(ET::Size(1) == 0);
		static_assert(ET::Size(2) == 1);
		static_assert(ET::Size(3) == 3);
		static_assert(ET::Size(4) == 6);
		static_assert(ET::Size(5) == 10);

		// Note that ET is just IT shifted by 1
		PR_EXPECT(IT::Dimension(0) == 0);
		PR_EXPECT(IT::Dimension(1) == 1);
		PR_EXPECT(IT::Dimension(3) == 2);
		PR_EXPECT(IT::Dimension(6) == 3);
		PR_EXPECT(IT::Dimension(10) == 4);
		PR_EXPECT(IT::Dimension(15) == 5);

		PR_EXPECT(ET::Dimension(0) == 1); // not zero, check the graph of n(n-1)/2
		PR_EXPECT(ET::Dimension(1) == 2);
		PR_EXPECT(ET::Dimension(3) == 3);
		PR_EXPECT(ET::Dimension(6) == 4);
		PR_EXPECT(ET::Dimension(10) == 5);
		PR_EXPECT(ET::Dimension(15) == 6);

		PR_EXPECT(IT::Dimension(IT::Size(371890)) == 371890);

		PR_EXPECT(IT::FromIndex(IT::Index(0, 5)) == index_pair_t(0, 5));
		PR_EXPECT(IT::FromIndex(IT::Index(2, 2)) == index_pair_t(2, 2));

		PR_EXPECT(ET::FromIndex(ET::Index(0, 2)) == index_pair_t(0, 2));
		PR_EXPECT(ET::FromIndex(ET::Index(1, 2)) == index_pair_t(1, 2));

		// Round-trip Size and Dimension
		for (double f = 2.0; f < 1'000'000'000.0; f = 1.2 * f + 0.7)
		{
			// Start with size 2, because exclusive is ambiguous for size=1
			auto i = static_cast<int64_t>(f);
			PR_EXPECT(IT::Dimension(IT::Size(i)) == i);
			PR_EXPECT(ET::Dimension(ET::Size(i)) == i);
		}

		// Check at limits
		auto big_index = MaxIndex - 1;
		PR_EXPECT(IT::Index(big_index, big_index) == 1152921505384281374LL);
		PR_EXPECT(ET::Index(big_index, big_index - 1) == 1152921503865781124LL);
		
		PR_EXPECT(IT::FromIndex(1152921505384281374LL) == index_pair_t(big_index, big_index));
		PR_EXPECT(ET::FromIndex(1152921503865781124LL) == index_pair_t(big_index - 1, big_index));

		// Round-trip Index and A/B
		for (int a = 0; a != 1000; ++a)
		{
			for (int b = 0; b != 1000; ++b)
			{
				auto index_i = IT::Index(a, b);
				auto [a_i, b_i] = IT::FromIndex(index_i);
				PR_EXPECT(IT::Index(a_i, b_i) == index_i);

				if (a != b)
				{
					auto index_e = ET::Index(a, b);
					auto [a_e, b_e] = ET::FromIndex(index_e);
					PR_EXPECT(ET::Index(a_e, b_e) == index_e);
				}
			}
		}

		#if 0 // this is a little expensive
		// Round-trip Index and A/B
		constexpr int N = 1'000'000;
		auto ints = std::ranges::views::iota(0, N);
		std::for_each(std::execution::par, ints.begin(), ints.end(), [](int a)
		{
			for (int b = a; b != N; ++b)
			{
				auto index_i = IT::Index(a, b);
				auto [a_i, b_i] = IT::FromIndex(index_i);
				if (IT::Index(a_i, b_i) != index_i)
				{
					auto msg = std::format("TriTable Inclusive round trip failed: Index({}, {}) == {}.  FromIndex({}) == {}, {}", a, b, index_i, index_i, a_i, b_i);
					OutputDebugStringA(msg.c_str());
					return;
				}

				if (a != b)
				{
					auto index_e = ET::Index(a, b);
					auto [a_e, b_e] = ET::FromIndex(index_e);
					if (ET::Index(a_e, b_e) != index_e)
					{
						auto msg = std::format("TriTable Exclusive round trip failed: Index({}, {}) == {}.  FromIndex({}) == {}, {}", a, b, index_e, index_e, a_e, b_e);
						OutputDebugStringA(msg.c_str());
						return;
					}
				}
			}
		});
		#endif

		// Last index + 1 == table size
		static_assert(IT::Index(2, 2) + 1 == static_cast<int>(IT::Size(3)));
		static_assert(ET::Index(2, 1) + 1 == static_cast<int>(ET::Size(3)));
		static_assert(IT::Index(4, 4) + 1 == static_cast<int>(IT::Size(5)));
		static_assert(ET::Index(3, 4) + 1 == static_cast<int>(ET::Size(5)));

		// Index order (table size = 3, inclusive)
		static_assert(IT::Index(0, 0) == 0); static_assert(IT::Index(0, 0) == 0);
		static_assert(IT::Index(1, 0) == 1); static_assert(IT::Index(0, 1) == 1);
		static_assert(IT::Index(1, 1) == 2); static_assert(IT::Index(1, 1) == 2);
		static_assert(IT::Index(2, 0) == 3); static_assert(IT::Index(0, 2) == 3);
		static_assert(IT::Index(2, 1) == 4); static_assert(IT::Index(1, 2) == 4);
		static_assert(IT::Index(2, 2) == 5); static_assert(IT::Index(2, 2) == 5);

		// Index order (table size = 4, exclusive)
		static_assert(ET::Index(1, 0) == 0); static_assert(ET::Index(0, 1) == 0);
		static_assert(ET::Index(2, 0) == 1); static_assert(ET::Index(0, 2) == 1);
		static_assert(ET::Index(2, 1) == 2); static_assert(ET::Index(1, 2) == 2);
		static_assert(ET::Index(3, 0) == 3); static_assert(ET::Index(0, 3) == 3);
		static_assert(ET::Index(3, 1) == 4); static_assert(ET::Index(1, 3) == 4);
		static_assert(ET::Index(3, 2) == 5); static_assert(ET::Index(2, 3) == 5);
	}
}
#endif
