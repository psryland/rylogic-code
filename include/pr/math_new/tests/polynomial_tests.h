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
	PRUnitTestClass(PolynomialUtilTests)
	{
		PRUnitTestMethod(MonicRoots)
		{
			// Ax + B = 0 => x = -B/A
			auto m = Monic{ 2.0, -6.0 }; // 2x - 6 = 0 => x = 3
			auto roots = FindRoots(m);
			PR_EXPECT(roots.m_count == 1);
			PR_EXPECT(FEql(roots[0], 3.0));
			PR_EXPECT(FEql(m.F(3.0), 0.0));
		}

		PRUnitTestMethod(MonicEvaluation)
		{
			auto m = Monic{ 3.0, -1.0 }; // 3x - 1
			PR_EXPECT(FEql(m.F(0.0), -1.0));
			PR_EXPECT(FEql(m.F(1.0), 2.0));
			PR_EXPECT(FEql(m.dF(5.0), 3.0)); // derivative is constant
			PR_EXPECT(FEql(m.ddF(5.0), 0.0));
		}

		PRUnitTestMethod(QuadraticRoots)
		{
			// x² - 5x + 6 = 0 => x = 2, 3
			auto q = Quadratic{ 1.0, -5.0, 6.0 };
			auto roots = FindRoots(q);
			PR_EXPECT(roots.m_count == 2);

			// Sort roots for deterministic comparison
			auto r0 = roots[0] < roots[1] ? roots[0] : roots[1];
			auto r1 = roots[0] < roots[1] ? roots[1] : roots[0];
			PR_EXPECT(FEql(r0, 2.0));
			PR_EXPECT(FEql(r1, 3.0));
		}

		PRUnitTestMethod(QuadraticNoRealRoots)
		{
			// x² + 1 = 0 => no real roots
			auto q = Quadratic{ 1.0, 0.0, 1.0 };
			auto roots = FindRoots(q);
			PR_EXPECT(roots.m_count == 0);
		}

		PRUnitTestMethod(QuadraticRepeatedRoot)
		{
			// x² - 4x + 4 = 0 => x = 2 (repeated)
			auto q = Quadratic{ 1.0, -4.0, 4.0 };
			auto roots = FindRoots(q);
			PR_EXPECT(roots.m_count >= 1);
			PR_EXPECT(FEql(roots[0], 2.0));
		}

		PRUnitTestMethod(QuadraticEvaluation)
		{
			auto q = Quadratic{ 2.0, -3.0, 1.0 }; // 2x² - 3x + 1
			PR_EXPECT(FEql(q.F(0.0), 1.0));
			PR_EXPECT(FEql(q.F(1.0), 0.0));
			PR_EXPECT(FEql(q.dF(0.0), -3.0)); // 4x - 3
			PR_EXPECT(FEql(q.dF(1.0), 1.0));
			PR_EXPECT(FEql(q.ddF(0.0), 4.0)); // 4
		}

		PRUnitTestMethod(CubicRoots)
		{
			// x³ - 6x² + 11x - 6 = 0 => x = 1, 2, 3
			auto c = Cubic{ 1.0, -6.0, 11.0, -6.0 };
			auto roots = FindRoots(c);
			PR_EXPECT(roots.m_count == 3);

			// Verify all roots evaluate to ~0
			for (int i = 0; i != roots.m_count; ++i)
				PR_EXPECT(Abs(c.F(roots[i])) < 1e-6);
		}

		PRUnitTestMethod(CubicEvaluation)
		{
			auto c = Cubic{ 1.0, 0.0, 0.0, -8.0 }; // x³ - 8
			PR_EXPECT(FEql(c.F(2.0), 0.0));
			PR_EXPECT(FEql(c.dF(2.0), 12.0)); // 3x²
			PR_EXPECT(FEql(c.ddF(2.0), 12.0)); // 6x
		}

		PRUnitTestMethod(QuarticRoots)
		{
			// Simple quartic: x⁴ = 0 has root x = 0 (multiplicity 4)
			auto q1 = Quartic{ 1.0, 0.0, 0.0, 0.0, 0.0 };
			auto r1 = FindRoots(q1);
			PR_EXPECT(r1.m_count >= 1);
			for (int i = 0; i != r1.m_count; ++i)
				PR_EXPECT(Abs(r1[i]) < 0.01);
		}

		PRUnitTestMethod(StationaryPointsTest)
		{
			// x² - 4x + 3 has stationary point at x = 2
			auto q = Quadratic{ 1.0, -4.0, 3.0 };
			auto sp = StationaryPoints(q);
			PR_EXPECT(sp.m_count == 1);
			PR_EXPECT(FEql(sp[0], 2.0));

			// x³ - 3x has stationary points at x = ±1
			auto c = Cubic{ 1.0, 0.0, -3.0, 0.0 };
			auto sp2 = StationaryPoints(c);
			PR_EXPECT(sp2.m_count == 2);
		}

		PRUnitTestMethod(FromPointsTests)
		{
			using V2 = Vec2<double>;

			// Monic from two points
			auto m = Monic::FromPoints(V2(0, 1), V2(1, 3)); // y = 2x + 1
			PR_EXPECT(FEql(m.A, 2.0));
			PR_EXPECT(FEql(m.B, 1.0));

			// Quadratic from three points
			auto q = Quadratic::FromPoints(V2(0, 0), V2(1, 1), V2(2, 4)); // y = x²
			PR_EXPECT(FEql(q.F(0.0), 0.0));
			PR_EXPECT(FEql(q.F(1.0), 1.0));
			PR_EXPECT(FEql(q.F(2.0), 4.0));
		}
	};
}
#endif
