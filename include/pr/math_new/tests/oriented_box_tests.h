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
	PRUnitTestClass(OrientedBoxTests)
	{
		PRUnitTestMethod(Construction, float, double)
		{
			using V4 = Vec4<T>;
			using M3 = Mat3x4<T>;
			using M4 = Mat4x4<T>;
			using OB = OrientedBox<T>;

			// Default construction
			auto ob0 = OB{};

			// From centre, radius, and orientation
			auto ob1 = OB(V4(1, 2, 3, 1), V4(4, 5, 6, 0), Identity<M3>());
			PR_EXPECT(FEql(ob1.Centre(), V4(1, 2, 3, 1)));
			PR_EXPECT(FEql(ob1.m_radius, V4(4, 5, 6, 0)));

			// Unit and Reset constants
			auto unit = OB::Unit();
			auto reset = OB::Reset();
		}

		PRUnitTestMethod(SizeAndVolume, float, double)
		{
			using V4 = Vec4<T>;
			using M3 = Mat3x4<T>;
			using OB = OrientedBox<T>;

			auto ob = OB(V4(0, 0, 0, 1), V4(2, 3, 4, 0), Identity<M3>());
			PR_EXPECT(ob.SizeX() == T(4));
			PR_EXPECT(ob.SizeY() == T(6));
			PR_EXPECT(ob.SizeZ() == T(8));
			PR_EXPECT(Volume(ob) == T(192));
		}

		PRUnitTestMethod(DiametreTest, float, double)
		{
			using V4 = Vec4<T>;
			using M3 = Mat3x4<T>;
			using OB = OrientedBox<T>;

			auto ob = OB(V4(0, 0, 0, 1), V4(1, 1, 1, 0), Identity<M3>());
			PR_EXPECT(FEql(ob.DiametreSq(), T(12)));
			PR_EXPECT(FEql(ob.Diametre(), Sqrt(T(12))));
		}

		PRUnitTestMethod(GetBSphereTest, float, double)
		{
			using V4 = Vec4<T>;
			using M3 = Mat3x4<T>;
			using OB = OrientedBox<T>;
			using BS = BoundingSphere<T>;

			auto ob = OB(V4(0, 0, 0, 1), V4(1, 1, 1, 0), Identity<M3>());
			auto bs = GetBSphere(ob);
			PR_EXPECT(FEql(bs.Centre(), V4(0, 0, 0, 1)));
			PR_EXPECT(FEql(bs.Radius(), Sqrt(T(3))));
		}

		PRUnitTestMethod(TranslationOps, float, double)
		{
			using V4 = Vec4<T>;
			using M3 = Mat3x4<T>;
			using OB = OrientedBox<T>;

			auto ob = OB(V4(0, 0, 0, 1), V4(1, 1, 1, 0), Identity<M3>());
			auto shifted = ob + V4(5, 0, 0, 0);
			PR_EXPECT(FEql(shifted.Centre(), V4(5, 0, 0, 1)));
			PR_EXPECT(FEql(shifted.m_radius, ob.m_radius));
		}
	};
}
#endif
