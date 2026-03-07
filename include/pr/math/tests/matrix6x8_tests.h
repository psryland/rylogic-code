//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/math.h"

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	PRUnitTestClass(Matrix6x8Tests)
	{
		PRUnitTestMethod(Construction, float, double)
		{
			using mat6_t = Mat6x8<T, void, void>;
			using mat3_t = Mat3x4<T>;
			using vec8_t = Vec8<T, void>;
			using vec4_t = Vec4<T>;

			// Default construction
			auto m0 = mat6_t{};

			// Identity
			auto I = mat6_t::Identity();
			auto v = vec8_t{ 1, 2, 3, 0, 4, 5, 6, 0 };
			auto r = I * v;
			PR_EXPECT(FEql(r, v));
		}

		PRUnitTestMethod(MultiplyVector, float, double)
		{
			using mat6_t = Mat6x8<T, void, void>;
			using mat3_t = Mat3x4<T>;
			using vec8_t = Vec8<T, void>;
			using vec4_t = Vec4<T>;

			auto M = mat6_t
			{
				vec8_t{1, 0, 0, 0, 0, 0},
				vec8_t{0, 1, 0, 0, 0, 0},
				vec8_t{0, 0, 1, 0, 0, 0},
				vec8_t{0, 0, 0, 1, 0, 0},
				vec8_t{0, 0, 0, 0, 1, 0},
				vec8_t{0, 0, 0, 0, 0, 1},
			};
			auto V = vec8_t{ 1, 2, 3, 0, 4, 5, 6, 0 };
			auto R = M * V;
			PR_EXPECT(FEql(R, V));
		}

		PRUnitTestMethod(Transpose, float, double)
		{
			using mat6_t = Mat6x8<T, void, void>;
			using vec8_t = Vec8<T, void>;

			auto M = mat6_t
			{
				vec8_t{1, 0, 0, 0, 0, 0},
				vec8_t{0, 2, 0, 0, 0, 0},
				vec8_t{0, 0, 3, 0, 0, 0},
				vec8_t{0, 0, 0, 4, 0, 0},
				vec8_t{0, 0, 0, 0, 5, 0},
				vec8_t{0, 0, 0, 0, 0, 6},
			};
			auto Mt = Transpose(M);

			// Diagonal matrix transpose is itself
			PR_EXPECT(FEql(M, Mt));
		}

		PRUnitTestMethod(Inverse, float, double)
		{
			using mat6_t = Mat6x8<T, void, void>;
			using vec8_t = Vec8<T, void>;

			auto M = mat6_t
			{
				vec8_t{+1, +1, +2, -1, +6, +2},
				vec8_t{-2, +2, +4, -3, +5, -4},
				vec8_t{+1, +3, -2, -5, +4, +6},
				vec8_t{+1, +4, +3, -7, +3, -5},
				vec8_t{+1, +2, +3, -2, +2, +3},
				vec8_t{+1, -1, -2, -3, +6, -1}
			};
			auto M_inv = Invert(M);
			auto I = M * M_inv;
			PR_EXPECT(FEql(I, mat6_t::Identity()));
		}
	};
}
#endif
