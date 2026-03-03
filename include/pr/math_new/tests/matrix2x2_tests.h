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
	PRUnitTestClass(Matrix2x2)
	{
		PRUnitTestMethod(Construction, float, double, int32_t, int64_t)
		{
			using vec2_t = Vec2<T>;
			using mat2_t = Mat2x2<T>;

			// From scalars (column-major)
			constexpr auto V0 = mat2_t(T(1), T(2), T(3), T(4));
			static_assert(V0.x == vec2_t(T(1), T(2)));
			static_assert(V0.y == vec2_t(T(3), T(4)));

			// From column vectors
			constexpr auto V1 = mat2_t(vec2_t(T(1), T(2)), vec2_t(T(3), T(4)));
			static_assert(V1.x == vec2_t(T(1), T(2)));
			static_assert(V1.y == vec2_t(T(3), T(4)));

			// From range
			constexpr auto V2 = mat2_t({ T(5), T(6), T(7), T(8) });
			static_assert(V2.x == vec2_t(T(5), T(6)));
			static_assert(V2.y == vec2_t(T(7), T(8)));

			// Array access
			static_assert(V0[0] == vec2_t(T(1), T(2)));
			static_assert(V0[1] == vec2_t(T(3), T(4)));
		}
		PRUnitTestMethod(Constants, float, double, int32_t, int64_t)
		{
			using vec2_t = Vec2<T>;
			using mat2_t = Mat2x2<T>;

			static_assert(Zero<mat2_t>() == mat2_t(T(0), T(0), T(0), T(0)));
			static_assert(Identity<mat2_t>() == mat2_t(T(1), T(0), T(0), T(1)));

			// Identity * Identity = Identity
			auto I = mat2_t::Identity();
			PR_EXPECT(I * I == I);
		}
		PRUnitTestMethod(Rotation, float, double)
		{
			using mat2_t = Mat2x2<T>;
			using vec2_t = Vec2<T>;

			// 90 degree rotation (pi/2)
			auto rot = mat2_t::Rotation(DegreesToRadians(T(90)));

			// Rotating (1,0) by 90deg should give (0,1)
			auto v = vec2_t(T(1), T(0));
			auto r = rot * v;
			PR_EXPECT(FEql(r, vec2_t(T(0), T(1))));

			// Should be orthogonal
			PR_EXPECT(IsOrthogonal(rot));
		}
		PRUnitTestMethod(ScaleFactory, float, double, int32_t, int64_t)
		{
			using mat2_t = Mat2x2<T>;
			using vec2_t = Vec2<T>;

			// Uniform scale
			auto s1 = mat2_t::Scale(T(3));
			PR_EXPECT(s1 * vec2_t(T(1), T(2)) == vec2_t(T(3), T(6)));

			// Non-uniform scale
			auto s2 = mat2_t::Scale(T(2), T(3));
			PR_EXPECT(s2 * vec2_t(T(1), T(1)) == vec2_t(T(2), T(3)));
		}
		PRUnitTestMethod(ShearFactory, float, double, int32_t, int64_t)
		{
			using mat2_t = Mat2x2<T>;
			using vec2_t = Vec2<T>;

			// sxy shears y by x-amount, syx shears x by y-amount
			auto sh = mat2_t::Shear(T(0), T(1));
			auto v = vec2_t(T(0), T(1));
			auto r = sh * v;

			// Shearing (0,1) with syx=1 adds y to x: (0+1, 1) = (1,1)
			PR_EXPECT(r == vec2_t(T(1), T(1)));
		}
	};
}
#endif