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
	PRUnitTestClass(Vector8)
	{
		std::default_random_engine rng = {};

		PRUnitTestMethod(Construction, Quick, float, double)
		{
			using vec8_t = Vec8<T, void>;
			using vec4_t = Vec4<T>;

			// Default construction
			auto v0 = vec8_t{};

			// From 6 scalars
			auto v1 = vec8_t{ T(1), T(2), T(3), T(4), T(5), T(6) };
			PR_EXPECT(v1.ang.x == T(1));
			PR_EXPECT(v1.ang.y == T(2));
			PR_EXPECT(v1.ang.z == T(3));
			PR_EXPECT(v1.lin.x == T(4));
			PR_EXPECT(v1.lin.y == T(5));
			PR_EXPECT(v1.lin.z == T(6));

			// From two Vec4
			auto v2 = vec8_t{ vec4_t(1, 2, 3, 0), vec4_t(4, 5, 6, 0) };
			PR_EXPECT(FEql(v1, v2));

			// Zero
			auto z = vec8_t{};
			PR_EXPECT(FEql(z.ang, vec4_t(0, 0, 0, 0)));
			PR_EXPECT(FEql(z.lin, vec4_t(0, 0, 0, 0)));
		}
		// Constant-evaluation indexing must stay on the active union view.
		PRUnitTestMethod(ConstexprIndexing, Quick, float, double, int32_t, int64_t)
		{
			using vec8_t = Vec8<T, void>;
			static_assert(std::same_as<decltype(std::declval<vec8_t const&>()[0]), T const&>);

			// Read every indexed component at compile time so both sub-vectors are exercised.
			static_assert([]() constexpr
			{
				constexpr vec8_t v = vec8_t{
					T(1), T(2), T(3), T(4),
					T(5), T(6), T(7), T(8)
				};

				return
					v[0] == T(1) &&
					v[1] == T(2) &&
					v[2] == T(3) &&
					v[3] == T(4) &&
					v[4] == T(5) &&
					v[5] == T(6) &&
					v[6] == T(7) &&
					v[7] == T(8);
			}());

			// Write through the indexed setter in constant evaluation so the mutable overload
			// follows the same active-member path.
			static_assert([]() constexpr
			{
				auto v = vec8_t{
					T(1), T(2), T(3), T(4),
					T(5), T(6), T(7), T(8)
				};

				v[5] = T(42);
				return v[5] == T(42) && v[4] == T(5) && v[6] == T(7);
			}());
		}

		PRUnitTestMethod(LinAt_AngAt, Quick, float, double)
		{
			using vec8_t = Vec8<T, void>;
			using vec4_t = Vec4<T>;
			{
				auto v = vec8_t{
					Random<vec4_t>(rng, T(0), T(10)),
					Random<vec4_t>(rng, T(0), T(10))
				};
				auto lin = v.LinAt(vec4_t::Origin());
				auto ang = v.AngAt(vec4_t::Origin());
				auto V = vec8_t{ ang, lin };
				PR_EXPECT(FEql(v, V));
			}
			{// LinAt, AngAt
				auto v = vec8_t{ 0, 0, 1, 0, 1, 0 };

				auto lin0 = v.LinAt(vec4_t{ -1,0,0,0 });
				auto ang0 = v.AngAt(vec4_t{ -1,0,0,0 });
				PR_EXPECT(FEql(lin0, vec4_t{ 0,0,0,0 }));
				PR_EXPECT(FEql(ang0, vec4_t{ 0,0,2,0 }));

				auto lin1 = v.LinAt(vec4_t{ 0,0,0,0 });
				auto ang1 = v.AngAt(vec4_t{ 0,0,0,0 });
				PR_EXPECT(FEql(lin1, vec4_t{ 0,1,0,0 }));
				PR_EXPECT(FEql(ang1, vec4_t{ 0,0,1,0 }));

				auto lin2 = v.LinAt(vec4_t{ +1,0,0,0 });
				auto ang2 = v.AngAt(vec4_t{ +1,0,0,0 });
				PR_EXPECT(FEql(lin2, vec4_t{ 0,2,0,0 }));
				PR_EXPECT(FEql(ang2, vec4_t{ 0,0,0,0 }));

				auto lin3 = v.LinAt(vec4_t{ +2,0,0,0 });
				auto ang3 = v.AngAt(vec4_t{ +2,0,0,0 });
				PR_EXPECT(FEql(lin3, vec4_t{ 0,3,0,0 }));
				PR_EXPECT(FEql(ang3, vec4_t{ 0,0,-1,0 }));

				auto lin4 = v.LinAt(vec4_t{ +3,0,0,0 });
				auto ang4 = v.AngAt(vec4_t{ +3,0,0,0 });
				PR_EXPECT(FEql(lin4, vec4_t{ 0,4,0,0 }));
				PR_EXPECT(FEql(ang4, vec4_t{ 0,0,-2,0 }));
			}
		}
		PRUnitTestMethod(Projection, Quick, float, double)
		{
			using vec8_t = Vec8<T, void>;
			using vec4_t = Vec4<T>;
			
			auto v = vec8_t{ 1,-2,3,-3,2,-1 };
			auto vn = Proj(v, vec4_t::ZAxis());
			auto vt = v - vn;
			auto r = vn + vt;
			PR_EXPECT(FEql(vn, vec8_t{ 0,0,3,0,0,-1 }));
			PR_EXPECT(FEql(vt, vec8_t{ 1,-2,0,-3,2,0 }));
			PR_EXPECT(FEql(r, v));
		}
		PRUnitTestMethod(Reflection, Quick, float, double)
		{
			using vec8_t = Vec8<T, void>;
			using vec4_t = Vec4<T>;
			
			// Projection/Reflect
			auto v = vec8_t{ 0, 0, 1, 0, 1, 0 };
			auto n = vec4_t::Normal(-1, -1, -1, 0);
			auto r = vec8_t{ T(-0.6666666666666), T(-0.6666666666666), T(0.3333333333333), T(-0.6666666666666), T(0.33333333333333), T(-0.6666666666666) };
			auto R = Reflect(v, n);
			PR_EXPECT(FEql(r, R));
		}

		// Verify that Vec8 preserves the underlying Vec4 any/all NaN contract across both sub-vectors.
		PRUnitTestMethod(IsNaNAggregation, Quick, float, double)
		{
			using vec8_t = Vec8<T, void>;
			using vec4_t = Vec4<T>;

			auto const nan = std::numeric_limits<T>::quiet_NaN();

			auto const finite = vec8_t{
				vec4_t{T(1), T(2), T(3), T(4)},
				vec4_t{T(5), T(6), T(7), T(8)}
			};
			auto const angular_nan = vec8_t{
				vec4_t{nan, nan, nan, nan},
				finite.lin
			};
			auto const mixed_blocks = vec8_t{
				vec4_t{nan, T(2), T(3), T(4)},
				vec4_t{nan, nan, nan, nan}
			};
			auto const all_nan = vec8_t{
				vec4_t{nan, nan, nan, nan},
				vec4_t{nan, nan, nan, nan}
			};

			// 'any=true' succeeds when either sub-vector contains a NaN.
			PR_EXPECT(!IsNaN(finite, true));
			PR_EXPECT(IsNaN(angular_nan, true));
			PR_EXPECT(IsNaN(mixed_blocks, true));

			// 'any=false' succeeds only when both sub-vectors report that all of their elements are NaN.
			PR_EXPECT(!IsNaN(finite, false));
			PR_EXPECT(!IsNaN(angular_nan, false));
			PR_EXPECT(!IsNaN(mixed_blocks, false));
			PR_EXPECT(IsNaN(all_nan, false));
		}
	};
}
#endif