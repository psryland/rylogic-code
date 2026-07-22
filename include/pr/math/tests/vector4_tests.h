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
	PRUnitTestClass(Vector4)
	{
		PRUnitTestMethod(Construction, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;

			// Scalar broadcast
			constexpr auto V0 = vec4_t(T(1));
			static_assert(V0.x == T(1) && V0.y == T(1) && V0.z == T(1) && V0.w == T(1));

			// Element-wise
			constexpr auto V1 = vec4_t(T(1), T(2), T(3), T(4));
			static_assert(V1.x == T(1) && V1.y == T(2) && V1.z == T(3) && V1.w == T(4));

			// xyz with w=0
			constexpr auto V2 = vec4_t(T(1), T(2), T(3));
			static_assert(V2.x == T(1) && V2.y == T(2) && V2.z == T(3) && V2.w == T(0));

			// From Vec3 + w
			constexpr auto V3 = vec4_t(Vec3<T>(T(10), T(20), T(30)), T(1));
			static_assert(V3.x == T(10) && V3.y == T(20) && V3.z == T(30) && V3.w == T(1));

			// From Vec2 + z + w
			constexpr auto V4 = vec4_t(Vec2<T>(T(10), T(20)), T(30), T(40));
			static_assert(V4.x == T(10) && V4.y == T(20) && V4.z == T(30) && V4.w == T(40));

			// From two Vec2s
			auto V5 = vec4_t(Vec2<T>(T(1), T(2)), Vec2<T>(T(3), T(4)));
			PR_EXPECT(V5.x == T(1) && V5.y == T(2) && V5.z == T(3) && V5.w == T(4));

			// From AxisId
			auto V6 = vec4_t(AxisId{AxisId::PosZ});
			PR_EXPECT(V6.x == T(0) && V6.y == T(0) && V6.z == T(1) && V6.w == T(0));
			auto V7 = vec4_t(AxisId{AxisId::NegZ});
			PR_EXPECT(V7.x == T(0) && V7.y == T(0) && V7.z == T(-1) && V7.w == T(0));

			// From array
			constexpr T arr[] = { T(5), T(6), T(7), T(8) };
			constexpr auto V8 = vec4_t(arr);
			static_assert(V8.x == T(5) && V8.y == T(6) && V8.z == T(7) && V8.w == T(8));

			// Array access
			static_assert(V1[0] == T(1));
			static_assert(V1[1] == T(2));
			static_assert(V1[2] == T(3));
			static_assert(V1[3] == T(4));
		}
		PRUnitTestMethod(Normal, float, double)
		{
			using vec4_t = Vec4<T>;

			auto V0 = vec4_t::Normal(T(1), T(2), T(3), T(4));
			PR_EXPECT(FEql(Length(V0), T(1)));
		}
		PRUnitTestMethod(SubVectors, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;

			constexpr auto V0 = vec4_t(T(1), T(2), T(3), T(4));

			// w0: creates Vec4 with w=0 (direction vector)
			auto V1 = V0.w0();
			PR_EXPECT(V1.x == T(1) && V1.y == T(2) && V1.z == T(3) && V1.w == T(0));

			// w1: creates Vec4 with w=1 (position vector)
			auto V2 = V0.w1();
			PR_EXPECT(V2.x == T(1) && V2.y == T(2) && V2.z == T(3) && V2.w == T(1));

			// vec2: extract two components
			auto V3 = V0.vec2(0, 3);
			PR_EXPECT(V3.x == T(1) && V3.y == T(4));

			// vec3: extract three components
			auto V4 = V0.vec3(2, 1, 0);
			PR_EXPECT(V4.x == T(3) && V4.y == T(2) && V4.z == T(1));
		}
		PRUnitTestMethod(Constants, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;

			static_assert(All(Zero<vec4_t>() == vec4_t(T(0), T(0), T(0), T(0))));
			static_assert(All(One<vec4_t>() == vec4_t(T(1), T(1), T(1), T(1))));
			static_assert(All(XAxis<vec4_t>() == vec4_t(T(1), T(0), T(0), T(0))));
			static_assert(All(YAxis<vec4_t>() == vec4_t(T(0), T(1), T(0), T(0))));
			static_assert(All(ZAxis<vec4_t>() == vec4_t(T(0), T(0), T(1), T(0))));
			static_assert(All(WAxis<vec4_t>() == vec4_t(T(0), T(0), T(0), T(1))));
			static_assert(All(Origin<vec4_t>() == vec4_t(T(0), T(0), T(0), T(1))));
		}
		PRUnitTestMethod(Operators, float, double, int32_t, int64_t)
		{
			// Vec4 arithmetic operators are not constexpr-evaluable due to the use of intrinsics.
			using vec4_t = Vec4<T>;

			auto a = vec4_t(T(10), T(20), T(30), T(40));
			auto b = vec4_t(T(-40), T(-30), T(-20), T(-10));

			PR_EXPECT(All(a + b == vec4_t(T(-30), T(-10), T(+10), T(+30))));
			PR_EXPECT(All(a - b == vec4_t(T(+50), T(+50), T(+50), T(+50))));
			PR_EXPECT(All(a * T(3) == vec4_t(T(+30), T(+60), T(+90), T(+120))));
			PR_EXPECT(All(T(3) * a == vec4_t(T(+30), T(+60), T(+90), T(+120))));
			PR_EXPECT(All(a / T(2) == vec4_t(T(5), T(10), T(15), T(20))));

			PR_EXPECT(All(+a == vec4_t(T(+10), T(+20), T(+30), T(+40))));
			PR_EXPECT(All(-a == vec4_t(T(-10), T(-20), T(-30), T(-40))));

			auto c = vec4_t(T(+1), T(-2), T(+3), T(-4));
			auto d = vec4_t(T(-1), T(+2), T(-3), T(+4));
			PR_EXPECT(All((c == d) == !(c != d)));
			PR_EXPECT(All((c != d) == !(c == d)));
		}
		PRUnitTestMethod(Dot4, float, double, int32_t, int64_t)
		{
			// Dot3 vs Dot4 (type-specific because Dot3 is Vec4-only)
			using vec4_t = Vec4<T>;

			auto a = vec4_t(T(-2), T(4), T(2), T(6));
			auto b = vec4_t(T(3), T(-5), T(2), T(-4));
			PR_EXPECT(Dot(a, b) == T(-46));  // Full 4-component dot
			PR_EXPECT(Dot3(a, b) == T(-22)); // 3-component dot (ignores w)
		}

		// Component-wise modulus for Vec%Vec, Vec%scalar, and scalar%Vec across all element types.
		PRUnitTestMethod(Modulus, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;

			if constexpr (std::is_floating_point_v<T>)
			{
				// Vec % Vec — ordinary positive values
				{
					auto lhs = vec4_t(T(7), T(10), T(3.5), T(0));
					auto rhs = vec4_t(T(3), T( 4), T(1.5), T(2));
					auto got = lhs % rhs;
					PR_EXPECT(FEql(got.x, std::fmod(T(7),   T(3))));
					PR_EXPECT(FEql(got.y, std::fmod(T(10),  T(4))));
					PR_EXPECT(FEql(got.z, std::fmod(T(3.5), T(1.5))));
					PR_EXPECT(FEql(got.w, std::fmod(T(0),   T(2))));
				}

				// Vec % Vec — signed dividend: sign of result matches dividend sign
				{
					auto lhs = vec4_t(T(-7), T( 7), T(-7), T( 7));
					auto rhs = vec4_t(T( 3), T( 3), T(-3), T(-3));
					auto got = lhs % rhs;
					PR_EXPECT(FEql(got.x, std::fmod(T(-7), T( 3))));
					PR_EXPECT(FEql(got.y, std::fmod(T( 7), T( 3))));
					PR_EXPECT(FEql(got.z, std::fmod(T(-7), T(-3))));
					PR_EXPECT(FEql(got.w, std::fmod(T( 7), T(-3))));
				}

				// Vec % Vec — large quotient: |a/b| exceeds the mantissa's exact integer range
				{
					auto lhs = vec4_t(T(1e20), T(1e15), T(1e7), T(123456789));
					auto rhs = vec4_t(T(3),    T(7),    T(11),  T(97));
					auto got = lhs % rhs;
					PR_EXPECT(FEql(got.x, std::fmod(T(1e20),      T(3))));
					PR_EXPECT(FEql(got.y, std::fmod(T(1e15),      T(7))));
					PR_EXPECT(FEql(got.z, std::fmod(T(1e7),       T(11))));
					PR_EXPECT(FEql(got.w, std::fmod(T(123456789), T(97))));
				}

				// Vec % scalar — exercises the Vec % element_t generic overload
				{
					auto lhs = vec4_t(T(7), T(-7), T(1e20), T(0));
					auto got = lhs % T(3);
					PR_EXPECT(FEql(got.x, std::fmod(T(7),    T(3))));
					PR_EXPECT(FEql(got.y, std::fmod(T(-7),   T(3))));
					PR_EXPECT(FEql(got.z, std::fmod(T(1e20), T(3))));
					PR_EXPECT(FEql(got.w, std::fmod(T(0),    T(3))));
				}

				// scalar % Vec — exercises the element_t % Vec generic overload (compile fix)
				{
					auto rhs = vec4_t(T(3), T(-3), T(1e20), T(7));
					auto got = T(10) % rhs;
					PR_EXPECT(FEql(got.x, std::fmod(T(10), T(3))));
					PR_EXPECT(FEql(got.y, std::fmod(T(10), T(-3))));
					PR_EXPECT(FEql(got.z, std::fmod(T(10), T(1e20))));
					PR_EXPECT(FEql(got.w, std::fmod(T(10), T(7))));
				}

				// IEEE 754 special cases — all three forms delegate to std::fmod per component
				{
					auto inf = std::numeric_limits<T>::infinity();
					auto nan = std::numeric_limits<T>::quiet_NaN();
					T three = T(3);

					// infinity dividend → NaN
					auto r_inf_lhs = Vec4<T>(inf, T(5), T(5), T(5)) % Vec4<T>(three, three, three, three);
					PR_EXPECT(std::isnan(r_inf_lhs.x));
					PR_EXPECT(!std::isnan(r_inf_lhs.y));

					// finite % infinity → finite (remainder equals dividend)
					auto r_inf_rhs = Vec4<T>(T(5), inf, T(5), T(5)) % Vec4<T>(inf, three, three, three);
					PR_EXPECT(FEql(r_inf_rhs.x, std::fmod(T(5), inf)));
					PR_EXPECT(std::isnan(r_inf_rhs.y));

					// zero divisor → NaN
					auto r_zero = Vec4<T>(T(5), T(5), T(5), T(5)) % Vec4<T>(T(0), three, three, three);
					PR_EXPECT(std::isnan(r_zero.x));
					PR_EXPECT(!std::isnan(r_zero.y));

					// NaN propagates
					auto r_nan = Vec4<T>(nan, T(5), T(5), T(5)) % Vec4<T>(three, three, three, three);
					PR_EXPECT(std::isnan(r_nan.x));
					PR_EXPECT(!std::isnan(r_nan.y));

					// signed-zero dividend: sign of zero is preserved by fmod
					T neg_zero = -T(0);
					auto r_neg_zero = Vec4<T>(neg_zero, T(0), T(5), T(5)) % Vec4<T>(three, three, three, three);
					PR_EXPECT(std::signbit(r_neg_zero.x));   // -0 % 3 == -0
					PR_EXPECT(!std::signbit(r_neg_zero.y));  // +0 % 3 == +0
				}
			}
			else
			{
				// Vec % Vec — integer modulus truncates toward zero (C++ standard)
				{
					auto a = vec4_t(T(10), T(-10), T(10), T(-10));
					auto b = vec4_t(T( 3), T(  3), T(-3), T( -3));
					auto got = a % b;
					PR_EXPECT(got.x == T(10) % T( 3));   //  1
					PR_EXPECT(got.y == T(-10) % T( 3));  // -1
					PR_EXPECT(got.z == T(10) % T(-3));   //  1
					PR_EXPECT(got.w == T(-10) % T(-3));  // -1
				}

				// Vec % scalar
				{
					auto got = vec4_t(T(10), T(-10), T(10), T(-10)) % T(3);
					PR_EXPECT(got.x == T( 10) % T(3));
					PR_EXPECT(got.y == T(-10) % T(3));
					PR_EXPECT(got.z == T( 10) % T(3));
					PR_EXPECT(got.w == T(-10) % T(3));
				}

				// scalar % Vec
				{
					auto got = T(10) % vec4_t(T(3), T(-3), T(7), T(-7));
					PR_EXPECT(got.x == T(10) % T( 3));
					PR_EXPECT(got.y == T(10) % T(-3));
					PR_EXPECT(got.z == T(10) % T( 7));
					PR_EXPECT(got.w == T(10) % T(-7));
				}
			}
		}
	};
}
#endif
