//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math_new/math.h"

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::spatial::tests
{
	PRUnitTestClass(SpatialAlgebraTests)
	{
		PRUnitTestMethod(CrossProduct, float, double)
		{
			using V4 = Vec4<T>;
			using V8 = Vec8<T, ::pr::math::spatial::Motion>;

			// Cross of motion vectors: coriolis-like term
			auto a = V8(V4(0, 0, 1, 0), V4(0, 0, 0, 0)); // pure rotation about z
			auto b = V8(V4(0, 0, 0, 0), V4(1, 0, 0, 0)); // pure translation along x
			auto c = Cross(a, b);

			// w x v = (0,0,1) x (1,0,0) = (0,1,0)
			PR_EXPECT(FEql(c.lin, V4(0, 1, 0, 0)));
		}

		PRUnitTestMethod(DotProduct, float, double)
		{
			using V4 = Vec4<T>;
			using VM = Vec8<T, ::pr::math::spatial::Motion>;
			using VF = Vec8<T, ::pr::math::spatial::Force>;

			// Dot of motion and force vectors gives power
			auto m = VM(V4(0, 0, 1, 0), V4(1, 0, 0, 0)); // angular z + linear x
			auto f = VF(V4(0, 0, 2, 0), V4(3, 0, 0, 0)); // torque z + force x

			// Power = w·τ + v·f = 1*2 + 1*3 = 5
			auto power = Dot(m, f);
			PR_EXPECT(FEql(power, T(5)));
		}

		PRUnitTestMethod(ShiftMotion, float, double)
		{
			using V4 = Vec4<T>;
			using V8 = Vec8<T, ::pr::math::spatial::Motion>;

			// Shifting a pure rotation about z by offset along x
			// v_new = v_old + w x r
			auto v = V8(V4(0, 0, 1, 0), V4(0, 0, 0, 0)); // pure angular velocity about z
			auto r = V4(1, 0, 0, 0); // offset along x
			auto shifted = Shift(v, r);

			// Linear velocity at offset: w x r = (0,0,1) x (1,0,0) = (0,1,0)
			PR_EXPECT(FEql(shifted.ang, V4(0, 0, 1, 0))); // angular unchanged
			PR_EXPECT(FEql(shifted.lin, V4(0, 1, 0, 0)));
		}

		PRUnitTestMethod(ShiftForce, float, double)
		{
			using V4 = Vec4<T>;
			using V8 = Vec8<T, ::pr::math::spatial::Force>;

			// Shifting a pure force along x by offset along y
			// τ_new = τ_old + r x f
			auto f = V8(V4(0, 0, 0, 0), V4(1, 0, 0, 0)); // pure force along x
			auto r = V4(0, 1, 0, 0); // offset along y
			auto shifted = Shift(f, r);

			// Torque at offset: r x f = Cross(lin, ofs) = (1,0,0) x (0,1,0) = (0,0,1)
			PR_EXPECT(FEql(shifted.lin, V4(1, 0, 0, 0))); // force unchanged
			PR_EXPECT(FEql(shifted.ang, V4(0, 0, 1, 0)));
		}

		PRUnitTestMethod(InertiaTest, float, double)
		{
			using V4 = Vec4<T>;
			using M3 = Mat3x4<T>;

			// Solid sphere inertia at CoM: I = 2/5 * m * r²
			auto mass = T(10);
			auto radius = T(1);
			auto I_scalar = T(0.4) * mass * radius * radius; // 4.0
			auto I_mat = M3(
				V4(I_scalar, 0, 0, 0),
				V4(0, I_scalar, 0, 0),
				V4(0, 0, I_scalar, 0)
			);
			auto com = V4(0, 0, 0, 0); // at origin
			auto I = Inertia(I_mat, com, mass);

			// At CoM, CPM(0) = 0, so m00 = mass * unit_inertia = I_mat * mass
			// m00.x = column 0 of m00
			PR_EXPECT(FEql(I.m00.x.x, mass * I_scalar));
			PR_EXPECT(FEql(I.m00.y.y, mass * I_scalar));
			PR_EXPECT(FEql(I.m00.z.z, mass * I_scalar));

			// m11 = mass * Identity
			PR_EXPECT(FEql(I.m11.x.x, mass));
			PR_EXPECT(FEql(I.m11.y.y, mass));
			PR_EXPECT(FEql(I.m11.z.z, mass));
		}
	};
}
#endif
