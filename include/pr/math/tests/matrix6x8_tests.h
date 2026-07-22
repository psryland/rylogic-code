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
			using mat3_t = Mat3x3<T>;
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
			using mat3_t = Mat3x3<T>;
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
			struct SpaceA {};
			struct SpaceB {};
			using mat3_t = Mat3x3<T>;
			using mat6_t = Mat6x8<T, void, void>;
			using tagged_mat_t = Mat6x8<T, SpaceA, SpaceB>;

			// The transpose of an A->B mapping must be typed as a B->A mapping.
			static_assert(std::is_same_v<decltype(Transpose(tagged_mat_t{})), Mat6x8<T, SpaceB, SpaceA>>);

			auto M = mat6_t
			{
				mat3_t{Vec3<T>{T( 1), T( 2), T( 3)}, Vec3<T>{T( 4), T( 5), T( 6)}, Vec3<T>{T( 7), T( 8), T( 9)}},
				mat3_t{Vec3<T>{T(10), T(11), T(12)}, Vec3<T>{T(13), T(14), T(15)}, Vec3<T>{T(16), T(17), T(18)}},
				mat3_t{Vec3<T>{T(19), T(20), T(21)}, Vec3<T>{T(22), T(23), T(24)}, Vec3<T>{T(25), T(26), T(27)}},
				mat3_t{Vec3<T>{T(28), T(29), T(30)}, Vec3<T>{T(31), T(32), T(33)}, Vec3<T>{T(34), T(35), T(36)}},
			};
			auto Mt = Transpose(M);

			// Check that the transpose swaps the off-diagonal blocks without changing their element layout.
			PR_EXPECT(FEql(Mt.m00, Transpose(M.m00)));
			PR_EXPECT(FEql(Mt.m01, Transpose(M.m10)));
			PR_EXPECT(FEql(Mt.m10, Transpose(M.m01)));
			PR_EXPECT(FEql(Mt.m11, Transpose(M.m11)));
			PR_EXPECT(FEql(Transpose(Mt), M));
		}

		// Exercise the dense case and check both product orders land on the identity.
		PRUnitTestMethod(InverseDense, float, double)
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
			auto I0 = M * M_inv;
			auto I1 = M_inv * M;
			PR_EXPECT(FEql(I0, mat6_t::Identity()));
			PR_EXPECT(FEql(I1, mat6_t::Identity()));
		}

		// Exercise the block-swap case where the diagonal 3x3 blocks are zero.
		PRUnitTestMethod(InverseSwappedBlocks, float, double)
		{
			struct SpaceA {};
			struct SpaceB {};
			using mat3_t = Mat3x3<T>;
			using mat6_t = Mat6x8<T, void, void>;
			using tagged_mat_t = Mat6x8<T, SpaceA, SpaceB>;

			auto M = tagged_mat_t{
				mat3_t::Zero(), mat3_t::Identity(),
				mat3_t::Identity(), mat3_t::Zero()
			};
			static_assert(std::is_same_v<decltype(Invert(M)), Mat6x8<T, SpaceB, SpaceA>>);

			auto M_inv = Invert(M);
			auto left = M * M_inv;
			auto right = M_inv * M;
			PR_EXPECT(FEql(static_cast<mat6_t const&>(left), mat6_t::Identity()));
			PR_EXPECT(FEql(static_cast<mat6_t const&>(right), mat6_t::Identity()));
		}

		// Exercise a full permutation that makes every 3x3 block singular on its own.
		PRUnitTestMethod(InversePermutation, float, double)
		{
			struct SpaceA {};
			struct SpaceB {};
			using mat6_t = Mat6x8<T, void, void>;
			using tagged_mat_t = Mat6x8<T, SpaceA, SpaceB>;
			using vec8_t = Vec8<T, void>;

			auto M = tagged_mat_t{};
			int const row_of_col[6] = { 0, 3, 4, 1, 2, 5 };
			int const scalar_of_row[6] = { 0, 1, 2, 4, 5, 6 };
			for (int col = 0; col != 6; ++col)
			{
				auto column = vec8_t{ T(0) };
				// The first three matrix rows live in the angular half of Vec8; the last three live in the linear half.
				column[scalar_of_row[row_of_col[col]]] = T(1);
				M.col(col, column);
			}
			static_assert(std::is_same_v<decltype(Invert(M)), Mat6x8<T, SpaceB, SpaceA>>);

			auto M_inv = Invert(M);
			auto left = M * M_inv;
			auto right = M_inv * M;
			PR_EXPECT(FEql(static_cast<mat6_t const&>(left), mat6_t::Identity()));
			PR_EXPECT(FEql(static_cast<mat6_t const&>(right), mat6_t::Identity()));
		}

		// A zero matrix never finds a nonzero pivot, so the singular-matrix throw must remain intact.
		PRUnitTestMethod(InverseSingularThrows, float, double)
		{
			using mat6_t = Mat6x8<T, void, void>;

			PR_THROWS(Invert(mat6_t::Zero()), std::runtime_error);
		}

		// Verify that Mat6x8 combines its Mat3x3 blocks with the same any/all NaN contract as the generic matrix helpers.
		PRUnitTestMethod(IsNaNAggregation, float, double)
		{
			using mat3_t = Mat3x3<T>;
			using mat6_t = Mat6x8<T, void, void>;

			auto const nan = std::numeric_limits<T>::quiet_NaN();

			auto const finite = mat6_t::Identity();
			auto const one_component_nan = mat6_t{
				mat3_t{
					Vec3<T>{nan, T(0), T(0)},
					Vec3<T>{T(0), T(1), T(0)},
					Vec3<T>{T(0), T(0), T(1)}
				},
				mat3_t::Zero(),
				mat3_t::Zero(),
				mat3_t::Identity()
			};
			auto const mixed_blocks = mat6_t{
				mat3_t{nan},
				mat3_t{
					Vec3<T>{nan, T(0), T(0)},
					Vec3<T>{T(0), T(1), T(0)},
					Vec3<T>{T(0), T(0), T(1)}
				},
				mat3_t::Zero(),
				mat3_t::Identity()
			};
			auto const all_nan = mat6_t{
				mat3_t{nan},
				mat3_t{nan},
				mat3_t{nan},
				mat3_t{nan}
			};

			// 'any=true' succeeds when any block contains at least one NaN.
			PR_EXPECT(!IsNaN(finite, true));
			PR_EXPECT(IsNaN(one_component_nan, true));
			PR_EXPECT(IsNaN(mixed_blocks, true));

			// 'any=false' succeeds only when every block reports that all of its elements are NaN.
			PR_EXPECT(!IsNaN(finite, false));
			PR_EXPECT(!IsNaN(one_component_nan, false));
			PR_EXPECT(!IsNaN(mixed_blocks, false));
			PR_EXPECT(IsNaN(all_nan, false));
		}
	};
}
#endif
