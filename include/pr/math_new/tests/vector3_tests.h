//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math_new/math.h"

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	PRUnitTestClass(Vector3)
	{
		PRUnitTestMethod(Construction, float, double, int32_t, int64_t)
		{
			using vec3_t = Vec3<T>;

			// Scalar broadcast
			constexpr auto V0 = vec3_t(T(1));
			static_assert(V0.x == T(1));
			static_assert(V0.y == T(1));
			static_assert(V0.z == T(1));

			// Element-wise
			constexpr auto V1 = vec3_t(T(1), T(2), T(3));
			static_assert(V1.x == T(1));
			static_assert(V1.y == T(2));
			static_assert(V1.z == T(3));

			// From initialiser list
			constexpr auto V2 = vec3_t({ T(3), T(4), T(5) });
			static_assert(V2.x == T(3));
			static_assert(V2.y == T(4));
			static_assert(V2.z == T(5));

			// From array
			constexpr T arr[] = { T(5), T(6), T(7) };
			constexpr auto V3 = vec3_t(arr);
			static_assert(V3.x == T(5));
			static_assert(V3.y == T(6));
			static_assert(V3.z == T(7));

			// From Vec2 + z
			constexpr auto V4 = vec3_t(Vec2<T>(T(10), T(20)), T(30));
			static_assert(V4.x == T(10));
			static_assert(V4.y == T(20));
			static_assert(V4.z == T(30));

			// From AxisId
			auto V5 = vec3_t(AxisId{AxisId::PosX});
			PR_EXPECT(V5.x == T(1) && V5.y == T(0) && V5.z == T(0));
			auto V6 = vec3_t(AxisId{AxisId::NegY});
			PR_EXPECT(V6.x == T(0) && V6.y == T(-1) && V6.z == T(0));

			// Array access
			static_assert(V1[0] == T(1));
			static_assert(V1[1] == T(2));
			static_assert(V1[2] == T(3));
		}
		PRUnitTestMethod(Normal, float, double)
		{
			using vec3_t = Vec3<T>;

			auto V0 = vec3_t::Normal(T(3), T(4), T(5));
			auto expected_len = Sqrt(T(3*3 + 4*4 + 5*5));
			PR_EXPECT(FEql(V0, vec3_t(T(3) / expected_len, T(4) / expected_len, T(5) / expected_len)));
			PR_EXPECT(FEql(Length(V0), T(1)));
		}
		PRUnitTestMethod(SubVectors, float, double, int32_t, int64_t)
		{
			using vec3_t = Vec3<T>;

			constexpr auto V0 = vec3_t(T(1), T(2), T(3));

			// w0: creates Vec4 with w=0 (direction vector)
			auto V1 = V0.w0();
			PR_EXPECT(V1.x == T(1) && V1.y == T(2) && V1.z == T(3) && V1.w == T(0));

			// w1: creates Vec4 with w=1 (position vector)
			auto V2 = V0.w1();
			PR_EXPECT(V2.x == T(1) && V2.y == T(2) && V2.z == T(3) && V2.w == T(1));

			// vec2: extract two components
			auto V3 = V0.vec2(0, 2);
			PR_EXPECT(V3.x == T(1) && V3.y == T(3));
			auto V4 = V0.vec2(1, 0);
			PR_EXPECT(V4.x == T(2) && V4.y == T(1));
		}
		PRUnitTestMethod(Constants, float, double, int32_t, int64_t)
		{
			using vec3_t = Vec3<T>;

			static_assert(Zero<vec3_t>() == vec3_t(T(0), T(0), T(0)));
			static_assert(One<vec3_t>() == vec3_t(T(1), T(1), T(1)));
			static_assert(XAxis<vec3_t>() == vec3_t(T(1), T(0), T(0)));
			static_assert(YAxis<vec3_t>() == vec3_t(T(0), T(1), T(0)));
			static_assert(ZAxis<vec3_t>() == vec3_t(T(0), T(0), T(1)));
			static_assert(Origin<vec3_t>() == vec3_t(T(0), T(0), T(0)));
		}
		PRUnitTestMethod(Operators, float, double, int32_t, int64_t)
		{
			using vec3_t = Vec3<T>;
			static constexpr auto Eql = [](auto lhs, auto rhs) constexpr
			{
				if constexpr (std::floating_point<T>)
					return FEql(lhs, rhs);
				else
					return lhs == rhs;
			};

			constexpr auto V0 = vec3_t(T(10), T(8), T(6));
			constexpr auto V1 = vec3_t(T(2), T(4), T(3));

			static_assert(Eql(V0 + V1, vec3_t(T(12), T(12), T(9))));
			static_assert(Eql(V0 - V1, vec3_t(T(8), T(4), T(3))));
			static_assert(Eql(V0 * V1, vec3_t(T(20), T(32), T(18))));
			static_assert(Eql(V0 / V1, vec3_t(T(5), T(2), T(2))));

			static_assert(Eql(V0 * T(3), vec3_t(T(30), T(24), T(18))));
			static_assert(Eql(T(3) * V0, vec3_t(T(30), T(24), T(18))));
			static_assert(Eql(V0 / T(2), vec3_t(T(5), T(4), T(3))));

			static_assert(Eql(+V0, vec3_t(T(+10), T(+8), T(+6))));
			static_assert(Eql(-V0, vec3_t(T(-10), T(-8), T(-6))));

			static_assert(V0 == vec3_t(T(10), T(8), T(6)));
			static_assert(V0 != vec3_t(T(2), T(1), T(3)));
		}
	};
}
#endif