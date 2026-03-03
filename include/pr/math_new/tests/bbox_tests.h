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
	PRUnitTestClass(BBoxTests)
	{
		PRUnitTestMethod(Construction, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			// Default construction
			auto b0 = BB{};

			// From centre and radius
			auto b1 = BB(V4(1, 2, 3, 1), V4(4, 5, 6, 0));
			PR_EXPECT(FEql(b1.Centre(), V4(1, 2, 3, 1)));
			PR_EXPECT(FEql(b1.Radius(), V4(4, 5, 6, 0)));

			// Unit/Reset/Infinity constants
			auto unit = BB::Unit();
			PR_EXPECT(unit.valid());
			PR_EXPECT(FEql(unit.Centre(), V4(0, 0, 0, 1)));
			PR_EXPECT(FEql(unit.Radius(), V4(T(0.5), T(0.5), T(0.5), 0)));

			auto reset = BB::Reset();
			PR_EXPECT(!reset.valid());

			auto inf = BB::Infinity();
			PR_EXPECT(inf.valid());
			PR_EXPECT(FEql(inf.Centre(), V4(0, 0, 0, 1)));
			PR_EXPECT(inf.Radius().x > T(1e30));
		}

		PRUnitTestMethod(MakeFromCorners, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			auto b = BB::Make(V4(-1, -2, -3, 1), V4(3, 4, 5, 1));
			PR_EXPECT(FEql(b.Centre(), V4(1, 1, 1, 1)));
			PR_EXPECT(FEql(b.Radius(), V4(2, 3, 4, 0)));
		}

		PRUnitTestMethod(LowerUpper, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			auto b = BB(V4(1, 2, 3, 1), V4(4, 5, 6, 0));
			PR_EXPECT(FEql(b.Lower(), V4(-3, -3, -3, 1)));
			PR_EXPECT(FEql(b.Upper(), V4(5, 7, 9, 1)));
			PR_EXPECT(b.LowerX() == T(-3));
			PR_EXPECT(b.LowerY() == T(-3));
			PR_EXPECT(b.LowerZ() == T(-3));
			PR_EXPECT(b.UpperX() == T(5));
			PR_EXPECT(b.UpperY() == T(7));
			PR_EXPECT(b.UpperZ() == T(9));
		}

		PRUnitTestMethod(SizeAndVolume, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			auto b = BB(V4(0, 0, 0, 1), V4(2, 3, 4, 0));
			PR_EXPECT(b.SizeX() == T(4));
			PR_EXPECT(b.SizeY() == T(6));
			PR_EXPECT(b.SizeZ() == T(8));
			PR_EXPECT(Volume(b) == T(192));
		}

		PRUnitTestMethod(IsPointAndHasVolume, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			auto point = BB(V4(1, 2, 3, 1), V4(0, 0, 0, 0));
			PR_EXPECT(point.is_point());
			PR_EXPECT(!point.has_volume());

			auto vol = BB(V4(0, 0, 0, 1), V4(1, 1, 1, 0));
			PR_EXPECT(!vol.is_point());
			PR_EXPECT(vol.has_volume());
		}

		PRUnitTestMethod(CornersTest, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			auto b = BB(V4(0, 0, 0, 1), V4(1, 1, 1, 0));
			auto corners = Corners(b);
			PR_EXPECT(corners.size() == 8);

			// All corners should be at distance sqrt(3) from centre
			for (auto& c : corners)
			{
				PR_EXPECT(FEql(Length(c - b.Centre()), Sqrt(T(3))));
			}
		}

		PRUnitTestMethod(UnionAndGrow, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			// Union with point
			auto b = BB(V4(0, 0, 0, 1), V4(1, 1, 1, 0));
			auto b2 = Union(b, V4(5, 0, 0, 1));
			PR_EXPECT(b2.UpperX() == T(5));
			PR_EXPECT(b2.LowerX() == T(-1));

			// Union of two boxes
			auto a = BB(V4(0, 0, 0, 1), V4(1, 1, 1, 0));
			auto c = BB(V4(5, 5, 5, 1), V4(1, 1, 1, 0));
			auto u = Union(a, c);
			PR_EXPECT(FEql(u.Lower(), V4(-1, -1, -1, 1)));
			PR_EXPECT(FEql(u.Upper(), V4(6, 6, 6, 1)));

			// Grow (mutating)
			auto g = BB::Reset();
			Grow(g, V4(1, 2, 3, 1));
			Grow(g, V4(-1, -2, -3, 1));
			PR_EXPECT(FEql(g.Centre(), V4(0, 0, 0, 1)));
			PR_EXPECT(FEql(g.Radius(), V4(1, 2, 3, 0)));
		}

		PRUnitTestMethod(IsWithinTests, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			auto b = BB(V4(0, 0, 0, 1), V4(2, 2, 2, 0));

			// Point inside
			PR_EXPECT(IsWithin(b, V4(1, 1, 1, 1)));
			PR_EXPECT(IsWithin(b, V4(0, 0, 0, 1)));

			// Point outside
			PR_EXPECT(!IsWithin(b, V4(3, 0, 0, 1)));

			// Point on boundary (with tolerance)
			PR_EXPECT(IsWithin(b, V4(2, 0, 0, 1)));

			// BBox within BBox
			auto inner = BB(V4(0, 0, 0, 1), V4(1, 1, 1, 0));
			PR_EXPECT(IsWithin(b, inner));
			auto outer = BB(V4(0, 0, 0, 1), V4(3, 3, 3, 0));
			PR_EXPECT(!IsWithin(b, outer));
		}

		PRUnitTestMethod(TranslationOps, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			auto b = BB(V4(0, 0, 0, 1), V4(1, 1, 1, 0));
			auto shifted = b + V4(5, 0, 0, 0);
			PR_EXPECT(FEql(shifted.Centre(), V4(5, 0, 0, 1)));
			PR_EXPECT(FEql(shifted.Radius(), b.Radius()));

			auto b2 = b;
			b2 += V4(0, 3, 0, 0);
			PR_EXPECT(FEql(b2.Centre(), V4(0, 3, 0, 1)));
		}

		PRUnitTestMethod(ScaleOps, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			auto b = BB(V4(0, 0, 0, 1), V4(1, 2, 3, 0));
			b *= T(2);
			PR_EXPECT(FEql(b.Radius(), V4(2, 4, 6, 0)));
			PR_EXPECT(FEql(b.Centre(), V4(0, 0, 0, 1)));
		}

		PRUnitTestMethod(GetBSphereTest, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			auto b = BB(V4(0, 0, 0, 1), V4(1, 1, 1, 0));
			auto s = GetBSphere(b);
			PR_EXPECT(FEql(s.Centre(), b.Centre()));
			PR_EXPECT(FEql(s.Radius(), Sqrt(T(3))));
		}

		PRUnitTestMethod(SupportPointTest, float, double)
		{
			using V4 = Vec4<T>;
			using BB = BBox<T>;

			auto b = BB(V4(0, 0, 0, 1), V4(1, 1, 1, 0));
			auto sp = SupportPoint(b, V4(1, 0, 0, 0));
			PR_EXPECT(FEql(sp, V4(1, 0, 0, 1)));
		}
	};
}
#endif
