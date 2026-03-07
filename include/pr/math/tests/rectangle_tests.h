//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/math/math.h"

namespace pr::math::tests
{
	PRUnitTestClass(RectangleTests)
	{
		PRUnitTestMethod(Construction, float, double)
		{
			using Rect = Rectangle<T>;
			using V2 = Vec2<T>;

			// Default constructible
			[[maybe_unused]] Rect r0;

			// From min/max vectors
			auto r1 = Rect(V2(1, 2), V2(3, 4));
			PR_EXPECT(r1.m_min.x == T(1));
			PR_EXPECT(r1.m_min.y == T(2));
			PR_EXPECT(r1.m_max.x == T(3));
			PR_EXPECT(r1.m_max.y == T(4));

			// From scalars
			auto r2 = Rect(T(5), T(6), T(7), T(8));
			PR_EXPECT(r2.m_min.x == T(5));
			PR_EXPECT(r2.m_min.y == T(6));
			PR_EXPECT(r2.m_max.x == T(7));
			PR_EXPECT(r2.m_max.y == T(8));

			// Explicit conversion from another scalar type
			auto r3 = Rectangle<int>(1, 2, 3, 4);
			auto r4 = Rect(r3);
			PR_EXPECT(r4.m_min.x == T(1));
			PR_EXPECT(r4.m_max.y == T(4));
		}

		PRUnitTestMethod(Constants, float, double)
		{
			using Rect = Rectangle<T>;
			using V2 = Vec2<T>;

			auto zero = Rect::Zero();
			PR_EXPECT(zero.m_min == Zero<V2>());
			PR_EXPECT(zero.m_max == Zero<V2>());

			auto unit = Rect::Unit();
			PR_EXPECT(unit.m_min == Zero<V2>());
			PR_EXPECT(unit.m_max == One<V2>());

			auto reset = Rect::Reset();
			PR_EXPECT(reset.empty());
		}

		PRUnitTestMethod(EmptyAndReset, float, double)
		{
			using Rect = Rectangle<T>;
			using V2 = Vec2<T>;

			// Reset() creates an empty rectangle
			auto r = Rect::Reset();
			PR_EXPECT(r.empty());

			// A valid rectangle is not empty
			auto r2 = Rect(V2(0, 0), V2(1, 1));
			PR_EXPECT(!r2.empty());

			// Reset the rectangle
			r2.reset();
			PR_EXPECT(r2.empty());

			// Degenerate rectangle with m_min == m_max is not empty (zero area but valid)
			auto r3 = Rect(V2(1, 1), V2(1, 1));
			PR_EXPECT(!r3.empty());
		}

		PRUnitTestMethod(SizeAndEdges, float, double)
		{
			using Rect = Rectangle<T>;
			using V2 = Vec2<T>;

			auto r = Rect(T(1), T(2), T(5), T(8));

			// Size
			PR_EXPECT(FEql(r.Size(), V2(4, 6)));
			PR_EXPECT(r.SizeX() == T(4));
			PR_EXPECT(r.SizeY() == T(6));

			// Edges
			PR_EXPECT(r.Left() == T(1));
			PR_EXPECT(r.Top() == T(2));
			PR_EXPECT(r.Right() == T(5));
			PR_EXPECT(r.Bottom() == T(8));

			// Centre
			PR_EXPECT(FEql(r.Centre(), V2(3, 5)));
		}

		PRUnitTestMethod(DiametreAndArea, float, double)
		{
			using Rect = Rectangle<T>;

			auto r = Rect(T(0), T(0), T(3), T(4));

			// DiametreSq = 3^2 + 4^2 = 25
			PR_EXPECT(FEql(r.DiametreSq(), T(25)));

			// Diametre = 5
			PR_EXPECT(FEql(r.Diametre(), T(5)));

			// Area = 3 * 4 = 12
			PR_EXPECT(r.Area() == T(12));

			// Aspect = 3/4 = 0.75
			PR_EXPECT(FEql(r.Aspect(), 0.75));
		}

		PRUnitTestMethod(Operators, float, double)
		{
			using Rect = Rectangle<T>;
			using V2 = Vec2<T>;

			auto r = Rect(T(1), T(2), T(5), T(8));

			// Offset by +=
			auto r1 = r;
			r1 += V2(10, 20);
			PR_EXPECT(FEql(r1.m_min, V2(11, 22)));
			PR_EXPECT(FEql(r1.m_max, V2(15, 28)));

			// Offset by -=
			auto r2 = r;
			r2 -= V2(1, 2);
			PR_EXPECT(FEql(r2.m_min, V2(0, 0)));
			PR_EXPECT(FEql(r2.m_max, V2(4, 6)));

			// Offset by + and -
			auto r3 = r + V2(3, 3);
			PR_EXPECT(FEql(r3.m_min, V2(4, 5)));

			auto r4 = r - V2(1, 2);
			PR_EXPECT(FEql(r4.m_min, V2(0, 0)));

			// FEql
			auto r5 = Rect(T(1), T(2), T(5), T(8));
			PR_EXPECT(FEql(r, r5));

			// ==, !=
			PR_EXPECT(r == r5);
			PR_EXPECT(!(r != r5));
		}

		PRUnitTestMethod(ShiftedAndInflated, float, double)
		{
			using Rect = Rectangle<T>;

			auto r = Rect(T(10), T(20), T(30), T(40));

			// Shifted
			auto s = Shifted(r, T(5), T(-5));
			PR_EXPECT(s.m_min.x == T(15));
			PR_EXPECT(s.m_min.y == T(15));
			PR_EXPECT(s.m_max.x == T(35));
			PR_EXPECT(s.m_max.y == T(35));

			// Inflated by uniform amount
			auto i1 = Inflated(r, T(2));
			PR_EXPECT(i1.m_min.x == T(8));
			PR_EXPECT(i1.m_min.y == T(18));
			PR_EXPECT(i1.m_max.x == T(32));
			PR_EXPECT(i1.m_max.y == T(42));

			// Inflated by dx, dy
			auto i2 = Inflated(r, T(1), T(3));
			PR_EXPECT(i2.m_min.x == T(9));
			PR_EXPECT(i2.m_min.y == T(17));
			PR_EXPECT(i2.m_max.x == T(31));
			PR_EXPECT(i2.m_max.y == T(43));

			// Inflated by individual values
			auto i3 = Inflated(r, T(1), T(2), T(3), T(4));
			PR_EXPECT(i3.m_min.x == T(9));
			PR_EXPECT(i3.m_min.y == T(18));
			PR_EXPECT(i3.m_max.x == T(33));
			PR_EXPECT(i3.m_max.y == T(44));
		}

		PRUnitTestMethod(Scale, float, double)
		{
			using Rect = Rectangle<T>;

			// Rect with size 20x20, centred at (10,10) -> (30,30)
			auto r = Rect(T(10), T(10), T(30), T(30));

			// Scale by 1 = inflate by half-size (10 on each side)
			auto s1 = Scale(r, T(1));
			PR_EXPECT(s1.m_min.x == T(0));
			PR_EXPECT(s1.m_min.y == T(0));
			PR_EXPECT(s1.m_max.x == T(40));
			PR_EXPECT(s1.m_max.y == T(40));

			// Scale by 0 = no change
			auto s0 = Scale(r, T(0));
			PR_EXPECT(FEql(s0, r));
		}

		PRUnitTestMethod(GrowWithPoint, float, double)
		{
			using Rect = Rectangle<T>;
			using V2 = Vec2<T>;

			// Start with a reset rect and grow with a point
			auto r = Rect::Reset();
			Grow(r, V2(5, 3));
			PR_EXPECT(FEql(r.m_min, V2(5, 3)));
			PR_EXPECT(FEql(r.m_max, V2(5, 3)));

			// Grow to include another point
			Grow(r, V2(1, 7));
			PR_EXPECT(FEql(r.m_min, V2(1, 3)));
			PR_EXPECT(FEql(r.m_max, V2(5, 7)));

			// Grow with interior point (no change)
			Grow(r, V2(3, 5));
			PR_EXPECT(FEql(r.m_min, V2(1, 3)));
			PR_EXPECT(FEql(r.m_max, V2(5, 7)));

			// Union (non-mutating version)
			auto r2 = Rect(T(0), T(0), T(2), T(2));
			auto r3 = Union(r2, V2(5, 5));
			PR_EXPECT(FEql(r3.m_min, V2(0, 0)));
			PR_EXPECT(FEql(r3.m_max, V2(5, 5)));

			// Original unchanged
			PR_EXPECT(FEql(r2.m_max, V2(2, 2)));
		}

		PRUnitTestMethod(GrowWithRect, float, double)
		{
			using Rect = Rectangle<T>;
			using V2 = Vec2<T>;

			auto r1 = Rect(T(0), T(0), T(5), T(5));
			auto r2 = Rect(T(3), T(3), T(8), T(8));
			Grow(r1, r2);
			PR_EXPECT(FEql(r1.m_min, V2(0, 0)));
			PR_EXPECT(FEql(r1.m_max, V2(8, 8)));

			// Union (non-mutating)
			auto r3 = Rect(T(0), T(0), T(2), T(2));
			auto r4 = Rect(T(5), T(5), T(7), T(7));
			auto r5 = Union(r3, r4);
			PR_EXPECT(FEql(r5.m_min, V2(0, 0)));
			PR_EXPECT(FEql(r5.m_max, V2(7, 7)));
		}

		PRUnitTestMethod(IsWithin, float, double)
		{
			using Rect = Rectangle<T>;
			using V2 = Vec2<T>;

			auto r = Rect(T(0), T(0), T(10), T(10));

			// Interior point
			PR_EXPECT(IsWithin(r, V2(5, 5)));

			// On min edge (inclusive)
			PR_EXPECT(IsWithin(r, V2(0, 0)));

			// On max edge (exclusive)
			PR_EXPECT(!IsWithin(r, V2(10, 10)));
			PR_EXPECT(!IsWithin(r, V2(10, 5)));
			PR_EXPECT(!IsWithin(r, V2(5, 10)));

			// Outside
			PR_EXPECT(!IsWithin(r, V2(-1, 5)));
			PR_EXPECT(!IsWithin(r, V2(5, -1)));
			PR_EXPECT(!IsWithin(r, V2(11, 5)));
			PR_EXPECT(!IsWithin(r, V2(5, 11)));
		}

		PRUnitTestMethod(IsIntersection, float, double)
		{
			using Rect = Rectangle<T>;

			auto r = Rect(T(0), T(0), T(10), T(10));

			// Overlapping
			PR_EXPECT(IsIntersection(r, Rect(T(5), T(5), T(15), T(15))));

			// Contained
			PR_EXPECT(IsIntersection(r, Rect(T(2), T(2), T(5), T(5))));

			// Same rect
			PR_EXPECT(IsIntersection(r, r));

			// Non-overlapping
			PR_EXPECT(!IsIntersection(r, Rect(T(11), T(0), T(20), T(10))));
			PR_EXPECT(!IsIntersection(r, Rect(T(0), T(11), T(10), T(20))));
			PR_EXPECT(!IsIntersection(r, Rect(T(-5), T(0), T(-1), T(10))));
		}

		PRUnitTestMethod(NormaliseScalePoint, float, double)
		{
			using Rect = Rectangle<T>;
			using V2 = Vec2<T>;

			auto pt = V2(200, 300);
			auto rt = Rect(50, 50, 200, 300);
			auto nss = NormalisePoint(rt, pt, T(1), T(1));
			auto ss = ScalePoint(rt, nss, T(1), T(1));
			PR_EXPECT(FEql(nss, V2(T(1), T(1))));
			PR_EXPECT(FEql(pt, ss));

			pt = V2(200, 300);
			rt = Rect(50, 50, 200, 300);
			nss = NormalisePoint(rt, pt, T(1), -T(1));
			ss = ScalePoint(rt, nss, T(1), -T(1));
			PR_EXPECT(FEql(nss, V2{ 1, -1 }));
			PR_EXPECT(FEql(pt, ss));

			pt = V2{ 75, 130 };
			rt = Rect(50, 50, 200, 300);
			nss = NormalisePoint(rt, pt, T(1), -T(1));
			ss = ScalePoint(rt, nss, T(1), -T(1));
			PR_EXPECT(FEql(nss, V2(T(-2) / T(3), T(0.36))));
			PR_EXPECT(FEql(pt, ss));

			// Centre of rect normalises to origin
			auto centre = rt.Centre();
			auto nc = NormalisePoint(rt, centre, T(1), T(1));
			PR_EXPECT(FEql(nc, V2(T(0), T(0))));
		}

		PRUnitTestMethod(SizeSetters, float, double)
		{
			using Rect = Rectangle<T>;

			// SizeX with anchor left
			auto r = Rect(T(10), T(20), T(30), T(40));
			r.SizeX(T(40), -1);
			PR_EXPECT(r.m_min.x == T(10));
			PR_EXPECT(r.m_max.x == T(50));

			// SizeX with anchor centre
			r = Rect(T(10), T(20), T(30), T(40));
			r.SizeX(T(40), 0);
			PR_EXPECT(r.m_min.x == T(0));
			PR_EXPECT(r.m_max.x == T(40));

			// SizeX with anchor right
			r = Rect(T(10), T(20), T(30), T(40));
			r.SizeX(T(40), +1);
			PR_EXPECT(r.m_min.x == T(-10));
			PR_EXPECT(r.m_max.x == T(30));

			// SizeY with anchor top
			r = Rect(T(10), T(20), T(30), T(40));
			r.SizeY(T(40), -1);
			PR_EXPECT(r.m_min.y == T(20));
			PR_EXPECT(r.m_max.y == T(60));

			// SizeY with anchor centre
			r = Rect(T(10), T(20), T(30), T(40));
			r.SizeY(T(40), 0);
			PR_EXPECT(r.m_min.y == T(10));
			PR_EXPECT(r.m_max.y == T(50));

			// SizeY with anchor bottom
			r = Rect(T(10), T(20), T(30), T(40));
			r.SizeY(T(40), +1);
			PR_EXPECT(r.m_min.y == T(0));
			PR_EXPECT(r.m_max.y == T(40));
		}
	};
}
#endif
