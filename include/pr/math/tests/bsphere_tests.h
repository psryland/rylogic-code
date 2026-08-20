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
	PRUnitTestClass(BSphereTests)
	{
		PRUnitTestMethod(Construction, Quick, float, double)
		{
			using V4 = Vec4<T>;
			using BS = BoundingSphere<T>;

			// From centre and radius
			auto s = BS(V4(1, 2, 3, 1), T(5));
			PR_EXPECT(FEql(s.Centre(), V4(1, 2, 3, 1)));
			PR_EXPECT(s.Radius() == T(5));
			PR_EXPECT(s.RadiusSq() == T(25));
			PR_EXPECT(s.Diametre() == T(10));
			PR_EXPECT(s.DiametreSq() == T(100));

			// Unit and Reset constants
			auto unit = BS::Unit();
			PR_EXPECT(unit.valid());
			PR_EXPECT(FEql(unit.Centre(), V4(0, 0, 0, 1)));
			PR_EXPECT(unit.Radius() == T(1));

			auto reset = BS::Reset();
			PR_EXPECT(!reset.valid());

			// is_point
			auto point = BS(V4(1, 2, 3, 1), T(0));
			PR_EXPECT(point.is_point());
			PR_EXPECT(!unit.is_point());
		}

		PRUnitTestMethod(VolumeTest, Quick, float, double)
		{
			using V4 = Vec4<T>;
			using BS = BoundingSphere<T>;

			auto s = BS(V4(0, 0, 0, 1), T(1));
			auto vol = Volume(s);
			auto expected = T(4.188790) * T(1) * T(1) * T(1); // (2/3)*tau*r^3
			PR_EXPECT(Abs(vol - expected) < T(0.001));
		}

		PRUnitTestMethod(GrowTests, Quick, float, double)
		{
			using V4 = Vec4<T>;
			using BS = BoundingSphere<T>;

			// Grow to include a point
			auto s = BS(V4(0, 0, 0, 1), T(1));
			Grow(s, V4(3, 0, 0, 1));
			PR_EXPECT(IsWithin(s, V4(3, 0, 0, 1)));
			PR_EXPECT(IsWithin(s, V4(0, 0, 0, 1)));

			// GrowLoose (doesn't move centre)
			auto s2 = BS(V4(0, 0, 0, 1), T(1));
			GrowLoose(s2, V4(3, 0, 0, 1));
			PR_EXPECT(FEql(s2.Centre(), V4(0, 0, 0, 1)));
			PR_EXPECT(s2.Radius() >= T(3));

			// Grow to include another sphere
			auto a = BS(V4(0, 0, 0, 1), T(1));
			auto b = BS(V4(5, 0, 0, 1), T(1));
			Grow(a, b);
			PR_EXPECT(IsWithin(a, V4(-1, 0, 0, 1)));
			PR_EXPECT(IsWithin(a, V4(6, 0, 0, 1)));
		}

		PRUnitTestMethod(UnionTests, Quick, float, double)
		{
			using V4 = Vec4<T>;
			using BS = BoundingSphere<T>;

			auto a = BS(V4(0, 0, 0, 1), T(1));
			auto b = BS(V4(4, 0, 0, 1), T(1));
			auto u = Union(a, b);
			PR_EXPECT(IsWithin(u, V4(-1, 0, 0, 1)));
			PR_EXPECT(IsWithin(u, V4(5, 0, 0, 1)));

			// Union with point
			auto s = BS(V4(0, 0, 0, 1), T(1));
			auto s2 = Union(s, V4(5, 0, 0, 1));
			PR_EXPECT(IsWithin(s2, V4(5, 0, 0, 1)));
			PR_EXPECT(IsWithin(s2, V4(0, 0, -1, 1)));
		}

		PRUnitTestMethod(GrowUnionCoincidentCentres, Quick, float, double)
		{
			using V4 = Vec4<T>;
			using BS = BoundingSphere<T>;

			// Co-located spheres with differing radii must not divide by a zero separation between
			// centres. The merged result should be the larger of the two, co-centred sphere, regardless
			// of argument order.
			auto tiny = BS(V4(1, 2, 3, 1), T(1));
			auto huge = BS(V4(1, 2, 3, 1), T(5));

			auto grow_tiny_by_huge = tiny;
			Grow(grow_tiny_by_huge, huge);
			PR_EXPECT(FEql(grow_tiny_by_huge.Centre(), V4(1, 2, 3, 1)));
			PR_EXPECT(grow_tiny_by_huge.Radius() == T(5));

			auto grow_huge_by_tiny = huge;
			Grow(grow_huge_by_tiny, tiny);
			PR_EXPECT(FEql(grow_huge_by_tiny.Centre(), V4(1, 2, 3, 1)));
			PR_EXPECT(grow_huge_by_tiny.Radius() == T(5));

			auto union_tiny_huge = Union(tiny, huge);
			PR_EXPECT(FEql(union_tiny_huge.Centre(), V4(1, 2, 3, 1)));
			PR_EXPECT(union_tiny_huge.Radius() == T(5));

			auto union_huge_tiny = Union(huge, tiny);
			PR_EXPECT(FEql(union_huge_tiny.Centre(), V4(1, 2, 3, 1)));
			PR_EXPECT(union_huge_tiny.Radius() == T(5));

			// Co-located spheres with equal radii should merge to the same sphere, unchanged.
			auto a = BS(V4(1, 2, 3, 1), T(4));
			auto b = BS(V4(1, 2, 3, 1), T(4));

			auto grown_equal = a;
			Grow(grown_equal, b);
			PR_EXPECT(FEql(grown_equal.Centre(), V4(1, 2, 3, 1)));
			PR_EXPECT(grown_equal.Radius() == T(4));

			auto unioned_equal = Union(a, b);
			PR_EXPECT(FEql(unioned_equal.Centre(), V4(1, 2, 3, 1)));
			PR_EXPECT(unioned_equal.Radius() == T(4));
		}

		PRUnitTestMethod(IsWithinTests, Quick, float, double)
		{
			using V4 = Vec4<T>;
			using BS = BoundingSphere<T>;

			auto s = BS(V4(0, 0, 0, 1), T(2));

			// Point inside
			auto w1 = IsWithin(s, V4(1, 0, 0, 1));
			auto w2 = IsWithin(s, V4(0, 0, 0, 1));
			PR_EXPECT(w1);
			PR_EXPECT(w2);

			// Point outside
			auto w3 = IsWithin(s, V4(3, 0, 0, 1));
			PR_EXPECT(!w3);

			// Point on boundary
			auto w4 = IsWithin(s, V4(2, 0, 0, 1));
			PR_EXPECT(w4);

			// Sphere within sphere
			auto inner = BS(V4(0, 0, 0, 1), T(1));
			auto w5 = IsWithin(s, inner);
			PR_EXPECT(w5);

			// Clearly non-contained case
			auto distant = BS(V4(10, 0, 0, 1), T(3));
			auto w6 = IsWithin(s, distant);
			PR_EXPECT(!w6);
		}

		PRUnitTestMethod(IsIntersectionTests, Quick, float, double)
		{
			using V4 = Vec4<T>;
			using BS = BoundingSphere<T>;

			auto a = BS(V4(0, 0, 0, 1), T(2));
			auto b = BS(V4(3, 0, 0, 1), T(2));
			PR_EXPECT(IsIntersection(a, b));

			auto c = BS(V4(5, 0, 0, 1), T(2));
			PR_EXPECT(!IsIntersection(a, c));
		}

		PRUnitTestMethod(TranslationOps, Quick, float, double)
		{
			using V4 = Vec4<T>;
			using BS = BoundingSphere<T>;

			auto s = BS(V4(0, 0, 0, 1), T(1));
			auto shifted = s + V4(5, 0, 0, 0);
			PR_EXPECT(FEql(shifted.Centre(), V4(5, 0, 0, 1)));
			PR_EXPECT(shifted.Radius() == T(1));
		}

		PRUnitTestMethod(SupportPointTest, Quick, float, double)
		{
			using V4 = Vec4<T>;
			using BS = BoundingSphere<T>;

			auto s = BS(V4(1, 0, 0, 1), T(2));
			auto sp = SupportPoint(s, V4(1, 0, 0, 0));
			PR_EXPECT(FEql(sp, V4(3, 0, 0, 1)));
		}
	};
}
#endif
