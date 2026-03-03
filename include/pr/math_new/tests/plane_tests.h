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
	PRUnitTestClass(PlaneTests)
	{
		PRUnitTestMethod(Construction, float, double)
		{
			using V4 = Vec4<T>;
			using P = Plane<T>;

			// From direction and distance
			auto p1 = P(V4(0, 1, 0, 0), T(5));
			PR_EXPECT(FEql(p1.direction(), V4(0, 1, 0, 0)));
			PR_EXPECT(p1.distance() == T(5));

			// From components
			auto p2 = P(T(0), T(0), T(1), T(-3));
			PR_EXPECT(FEql(p2.direction(), V4(0, 0, 1, 0)));
			PR_EXPECT(p2.distance() == T(3));

			// From point and direction
			auto p3 = P(V4(0, 5, 0, 1), V4(0, 1, 0, 0));
			PR_EXPECT(FEql(p3.direction(), V4(0, 1, 0, 0)));
		}

		PRUnitTestMethod(FromTriangle, float, double)
		{
			using V4 = Vec4<T>;
			using P = Plane<T>;

			auto p = P::FromTriangle(V4(0, 0, 0, 1), V4(1, 0, 0, 1), V4(0, 1, 0, 1));
			auto norm = Normalise(p);
			PR_EXPECT(FEql(norm.direction(), V4(0, 0, 1, 0)));
		}

		PRUnitTestMethod(NormaliseTest, float, double)
		{
			using V4 = Vec4<T>;
			using P = Plane<T>;

			auto p = P(T(0), T(0), T(2), T(-6));
			auto n = Normalise(p);
			PR_EXPECT(FEql(n.direction(), V4(0, 0, 1, 0)));
			PR_EXPECT(FEql(n.distance(), T(3)));
		}

		PRUnitTestMethod(DistanceTest, float, double)
		{
			using V4 = Vec4<T>;
			using P = Plane<T>;

			// XY plane at z=0
			auto p = P(V4(0, 0, 1, 0), T(0));
			PR_EXPECT(FEql(Distance(p, V4(0, 0, 5, 1)), T(5)));
			PR_EXPECT(FEql(Distance(p, V4(0, 0, -3, 1)), T(-3)));
			PR_EXPECT(FEql(Distance(p, V4(0, 0, 0, 1)), T(0)));

			// Offset plane
			auto p2 = P(V4(0, 1, 0, 0), T(2));
			PR_EXPECT(FEql(Distance(p2, V4(0, 5, 0, 1)), T(3)));
			PR_EXPECT(FEql(Distance(p2, V4(0, 0, 0, 1)), T(-2)));
		}

		PRUnitTestMethod(ProjectTest, float, double)
		{
			using V4 = Vec4<T>;
			using P = Plane<T>;

			// Project point onto XY plane
			auto p = P(V4(0, 0, 1, 0), T(0));
			auto proj = Project(p, V4(3, 4, 5, 1));
			PR_EXPECT(FEql(proj, V4(3, 4, 0, 1)));

			// Project onto offset plane
			auto p2 = P(V4(0, 1, 0, 0), T(2));
			auto proj2 = Project(p2, V4(3, 5, 7, 1));
			PR_EXPECT(FEql(proj2, V4(3, 2, 7, 1)));
		}
	};
}
#endif
