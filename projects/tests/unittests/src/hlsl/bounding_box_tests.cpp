//*********************************************
// HLSL BoundingBoxTests
//  Copyright (c) Rylogic Ltd 2025
//*********************************************
#include "pr/common/unittests.h"
#include "pr/math/math.h"
#include "pr/hlsl/interop.h"
#include "pr/hlsl/bounding_box.hlsli"

namespace pr::hlsl::tests
{
	using namespace pr::math;

	PRUnitTestClass(BoundingBoxTests)
	{
		PRUnitTestMethod(ResetIsEmpty)
		{
			auto bbox = BBox_Reset();
			PR_EXPECT(bbox.IsEmpty());
		}
		PRUnitTestMethod(CreateNotEmpty)
		{
			auto bbox = BBox_Create(float4(0, 0, 0, 1), float4(1, 1, 1, 0));
			PR_EXPECT(!bbox.IsEmpty());
		}
		PRUnitTestMethod(MinMax)
		{
			auto bbox = BBox_Create(float4(1, 2, 3, 1), float4(0.5f, 1, 1.5f, 0));
			auto mn = bbox.Min();
			auto mx = bbox.Max();
			PR_EXPECT(FEql(mn, float4(0.5f, 1, 1.5f, 1)));
			PR_EXPECT(FEql(mx, float4(1.5f, 3, 4.5f, 1)));
		}
		PRUnitTestMethod(FromMinMax)
		{
			auto bbox = BBox_FromMinMax(float4(-1, -2, -3, 1), float4(1, 2, 3, 1));
			PR_EXPECT(FEql(bbox.centre, float4(0, 0, 0, 1)));
			PR_EXPECT(FEql(bbox.radius, float4(1, 2, 3, 0)));
		}
		PRUnitTestMethod(GrowFromEmpty)
		{
			auto bbox = BBox_Reset();
			bbox = bbox.Grow(float4(1, 2, 3, 1));
			PR_EXPECT(!bbox.IsEmpty());
			PR_EXPECT(FEql(bbox.centre, float4(1, 2, 3, 1)));
			PR_EXPECT(FEql(bbox.radius, float4(0, 0, 0, 0)));
		}
		PRUnitTestMethod(GrowExpands)
		{
			auto bbox = BBox_Create(float4(0, 0, 0, 1), float4(1, 1, 1, 0));
			bbox = bbox.Grow(float4(3, 0, 0, 1));
			auto mx = bbox.Max();
			PR_EXPECT(FEql(mx.x, 3.0f));
		}
		PRUnitTestMethod(UnionTwoBoxes)
		{
			auto a = BBox_Create(float4(0, 0, 0, 1), float4(1, 1, 1, 0));
			auto b = BBox_Create(float4(3, 0, 0, 1), float4(1, 1, 1, 0));
			auto u = a.Union(b);
			auto mn = u.Min();
			auto mx = u.Max();
			PR_EXPECT(FEql(mn.x, -1.0f));
			PR_EXPECT(FEql(mx.x, 4.0f));
		}
		PRUnitTestMethod(UnionWithEmpty)
		{
			auto a = BBox_Create(float4(1, 2, 3, 1), float4(0.5f, 0.5f, 0.5f, 0));
			auto empty = BBox_Reset();

			auto u1 = a.Union(empty);
			PR_EXPECT(FEql(u1.centre, a.centre));
			PR_EXPECT(FEql(u1.radius, a.radius));

			auto u2 = empty.Union(a);
			PR_EXPECT(FEql(u2.centre, a.centre));
			PR_EXPECT(FEql(u2.radius, a.radius));
		}
		PRUnitTestMethod(IsIntersectionOverlap)
		{
			auto a = BBox_Create(float4(0, 0, 0, 1), float4(1, 1, 1, 0));
			auto b = BBox_Create(float4(1.5f, 0, 0, 1), float4(1, 1, 1, 0));
			PR_EXPECT(a.IsIntersection(b));
		}
		PRUnitTestMethod(IsIntersectionNoOverlap)
		{
			auto a = BBox_Create(float4(0, 0, 0, 1), float4(1, 1, 1, 0));
			auto b = BBox_Create(float4(5, 0, 0, 1), float4(1, 1, 1, 0));
			PR_EXPECT(!a.IsIntersection(b));
		}
		PRUnitTestMethod(TransformIdentity)
		{
			auto bbox = BBox_Create(float4(1, 2, 3, 1), float4(0.5f, 1, 1.5f, 0));
			auto I = float4x4(
				float4(1, 0, 0, 0),
				float4(0, 1, 0, 0),
				float4(0, 0, 1, 0),
				float4(0, 0, 0, 1));
			auto result = bbox.Transform(I);
			PR_EXPECT(FEql(result.centre, bbox.centre));
			PR_EXPECT(FEql(result.radius, bbox.radius));
		}
		PRUnitTestMethod(TransformTranslation)
		{
			auto bbox = BBox_Create(float4(0, 0, 0, 1), float4(1, 1, 1, 0));
			auto T = float4x4(
				float4(1, 0, 0, 0),
				float4(0, 1, 0, 0),
				float4(0, 0, 1, 0),
				float4(5, 0, 0, 1));
			auto result = bbox.Transform(T);

			// Centre should be translated, radius unchanged
			PR_EXPECT(FEql(result.centre.x, 5.0f));
			PR_EXPECT(FEql(result.radius.x, 1.0f));
			PR_EXPECT(FEql(result.radius.y, 1.0f));
			PR_EXPECT(FEql(result.radius.z, 1.0f));
		}
	};
}
