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
	PRUnitTestClass(Vector2)
	{
		PRUnitTestMethod(Construction, float, double, int32_t, int64_t)
		{
			using vec2_t = Vec2<T>;

			constexpr auto V0 = vec2_t(T(1));
			static_assert(V0.x == T(1));
			static_assert(V0.y == T(1));

			constexpr auto V1 = vec2_t(T(1), T(2));
			static_assert(V1.x == T(1));
			static_assert(V1.y == T(2));

			constexpr auto V2 = vec2_t({ T(3), T(4) });
			static_assert(V2.x == T(3));
			static_assert(V2.y == T(4));

			if constexpr (std::floating_point<T>)
			{
				constexpr auto V3 = vec2_t::Normal(T(3), T(4));
				static_assert(FEql(V3.x, T(0.6)));
				static_assert(FEql(V3.y, T(0.8)));
			}

			constexpr T arr[] = { T(5), T(6) };
			constexpr auto V4 = vec2_t(arr);
			static_assert(V4.x == T(5));
			static_assert(V4.y == T(6));

			// Array access
			static_assert(V1[0] == T(1));
			static_assert(V1[1] == T(2));
		}
		PRUnitTestMethod(Operators, float, double, int32_t, int64_t)
		{
			// Test operators with non-uniform values for thorough coverage
			using vec2_t = Vec2<T>;
			static constexpr auto Eql = [](auto lhs, auto rhs) constexpr
			{
				if constexpr (std::floating_point<T>)
					return FEql(lhs, rhs);
				else
					return lhs == rhs;
			};

			constexpr auto V0 = vec2_t(T(10), T(8));
			constexpr auto V1 = vec2_t(T(2), T(12));

			static_assert(Eql(V0 + V1, vec2_t(T(+12), T(+20))));
			static_assert(Eql(V0 - V1, vec2_t(T(+8), T(-4))));
			static_assert(Eql(V0 * V1, vec2_t(T(+20), T(+96))));
			static_assert(Eql(V0 / V1, vec2_t(T(+5), T(8) / T(12))));
			PR_EXPECT(Eql(V0 % V1, vec2_t(T(+0), T(8))));

			static_assert(Eql(V0 * T(3), vec2_t(T(30), T(24))));
			static_assert(Eql(V0 / T(2), vec2_t(T(5), T(4))));
			PR_EXPECT(Eql(V0 % T(2), vec2_t(T(0), T(0))));

			static_assert(Eql(T(3) * V0, vec2_t(T(30), T(24))));

			static_assert(Eql(+V0, vec2_t(T(+10), T(+8))));
			static_assert(Eql(-V0, vec2_t(T(-10), T(-8))));

			static_assert(V0 == vec2_t(T(10), T(8)));
			static_assert(V0 != vec2_t(T(2), T(1)));
		}
		PRUnitTestMethod(DotProduct, float, double, int32_t, int64_t)
		{
			using vec2_t = Vec2<T>;

			constexpr auto V0 = vec2_t(T(3), T(4));
			constexpr auto V1 = vec2_t(T(1), T(2));

			// Dot(V0, V1) = 3*1 + 4*2 = 11
			static_assert(Dot(V0, V1) == T(11));

			// Dot product with self = LengthSq
			static_assert(LengthSq(V0) == T(25));
		}
		PRUnitTestMethod(CrossProduct, float, double, int32_t, int64_t)
		{
			using vec2_t = Vec2<T>;

			constexpr auto V0 = vec2_t(T(3), T(4));
			constexpr auto V1 = vec2_t(T(1), T(2));

			// 2D Cross(V0, V1) = V0.x * V1.y - V0.y * V1.x = 3*2 - 4*1 = 2
			static_assert(Cross(V0, V1) == T(2));

			// Parallel vectors have zero cross product
			static_assert(Cross(V0, V0) == T(0));

			// Anti-symmetric: Cross(a,b) == -Cross(b,a)
			static_assert(Cross(V0, V1) == -Cross(V1, V0));
		}
		PRUnitTestMethod(Length, float, double)
		{
			using vec2_t = Vec2<T>;

			// 3-4-5 right triangle
			constexpr auto V0 = vec2_t(T(3), T(4));
			static_assert(LengthSq(V0) == T(25));
			static_assert(FEql(Length(V0), T(5)));

			// Unit vectors have length 1
			static_assert(FEql(Length(XAxis<vec2_t>()), T(1)));
			static_assert(FEql(Length(YAxis<vec2_t>()), T(1)));
		}
		PRUnitTestMethod(Permute, float, double, int32_t, int64_t)
		{
			using vec2_t = Vec2<T>;

			constexpr auto V0 = vec2_t(T(1), T(2));

			// Permute by 0 is identity
			static_assert(Permute(V0, 0) == vec2_t(T(1), T(2)));

			// Permute by 1 swaps x and y
			static_assert(Permute(V0, 1) == vec2_t(T(2), T(1)));

			// Permute by 2 is back to identity
			static_assert(Permute(V0, 2) == vec2_t(T(1), T(2)));
		}
		PRUnitTestMethod(Orthant, float, double, int32_t, int64_t)
		{
			using vec2_t = Vec2<T>;

			// Quadrant bitmask: bit 0 = (x >= 0), bit 1 = (y >= 0)
			static_assert(Orthant(vec2_t(T(+1), T(+1))) == 3); // +x, +y
			static_assert(Orthant(vec2_t(T(-1), T(+1))) == 2); // -x, +y
			static_assert(Orthant(vec2_t(T(+1), T(-1))) == 1); // +x, -y
			static_assert(Orthant(vec2_t(T(-1), T(-1))) == 0); // -x, -y
		}

		#if 0 // todo: CosAngle and Angle functions not yet ported to math_new
		PRUnitTestMethod(CosAngle, float, double)
		{
			using vec2_t = Vec2<T>;

			constexpr auto V0 = vec2_t(T(1), T(0));
			constexpr auto V1 = vec2_t(T(0), T(1));

			// CosAngle between orthogonal vectors should be cos(90deg) = 0
			static_assert(FEql(CosAngle(V0, V1), Cos(DegreesToRadians(T(90)))));

			// Angle between orthogonal vectors should be 90deg
			static_assert(FEql(Angle(V0, V1), DegreesToRadians(T(90))));
		}
		#endif

		PRUnitTestMethod(Rotate90, float, double, int32_t, int64_t)
		{
			using vec2_t = Vec2<T>;

			constexpr auto V0 = vec2_t(T(1), T(0));

			// Rotate CW: (1,0) -> (0,1). CW rotates +X toward +Y.
			static_assert(Rotate90CW(V0) == vec2_t(T(0), T(1)));

			// Rotate CCW: (1,0) -> (0,-1). CCW rotates +X toward -Y.
			static_assert(Rotate90CCW(V0) == vec2_t(T(0), T(-1)));

			// Full rotation: 4x CW should return to original
			constexpr auto V1 = Rotate90CW(Rotate90CW(Rotate90CW(Rotate90CW(V0))));
			static_assert(V1 == V0);

			// Full rotation: 4x CCW should return to original
			constexpr auto V2 = Rotate90CCW(Rotate90CCW(Rotate90CCW(Rotate90CCW(V0))));
			static_assert(V2 == V0);

			// CW then CCW is identity
			constexpr auto V3 = Rotate90CCW(Rotate90CW(V0));
			static_assert(V3 == V0);
		}
	};
}
#endif
