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
	PRUnitTestClass(FrustumPrimitiveTests)
	{
		PRUnitTestMethod(WHFrustum, float, double)
		{
			using V2 = Vec2<T>;
			using V4 = Vec4<T>;
			using FR = Frustum3<T>;

			// Create a frustum from width/height
			auto f = FR::MakeWH(T(2), T(1), T(1), T(100));
			PR_EXPECT(f.zfar() == T(100));

			// Check orthographic flag
			PR_EXPECT(!f.orthographic());

			// Check fov
			auto fovx = f.fovX();
			auto fovy = f.fovY();
			PR_EXPECT(fovx > T(0));
			PR_EXPECT(fovy > T(0));
			PR_EXPECT(fovx > fovy); // wider than tall

			// Aspect ratio
			auto aspect = f.aspect();
			PR_EXPECT(FEql(aspect, T(2)));
		}

		PRUnitTestMethod(Corners, float, double)
		{
			using V4 = Vec4<T>;
			using V2 = Vec2<T>;
			using M4 = Mat4x4<T>;
			using FR = Frustum3<T>;

			auto f = FR::MakeWH(T(2), T(2), T(1), T(10));
			auto corners = Corners(f, T(10)); // returns Mat4x4 (4 corners at given z)

			// All 4 corners should be at z = -10
			PR_EXPECT(FEql(corners.x.z, T(-10)));
			PR_EXPECT(FEql(corners.y.z, T(-10)));
			PR_EXPECT(FEql(corners.z.z, T(-10)));
			PR_EXPECT(FEql(corners.w.z, T(-10)));
		}

		PRUnitTestMethod(IsWithinPoint, float, double)
		{
			using V4 = Vec4<T>;
			using V2 = Vec2<T>;
			using FR = Frustum3<T>;

			auto f = FR::MakeWH(T(2), T(2), T(1), T(100));

			// Point inside frustum (near the centre)
			auto inside = IsWithin(f, V4(0, 0, T(-10), 1), T(0), V2(T(1), T(100)));
			PR_EXPECT(inside);

			// Point outside (behind the frustum)
			auto behind = IsWithin(f, V4(0, 0, T(1), 1), T(0), V2(T(1), T(100)));
			PR_EXPECT(!behind);

			// Point outside (far beyond)
			auto beyond = IsWithin(f, V4(0, 0, T(-200), 1), T(0), V2(T(1), T(100)));
			PR_EXPECT(!beyond);
		}

		PRUnitTestMethod(ProjectionMatrix, float, double)
		{
			using V2 = Vec2<T>;
			using M4 = Mat4x4<T>;
			using FR = Frustum3<T>;

			auto f = FR::MakeWH(T(2), T(2), T(1), T(100));
			auto proj = f.projection(T(1), T(100));

			// Projection matrix should be non-zero
			PR_EXPECT(!FEql(proj, M4{}));
		}

		PRUnitTestMethod(OrthographicFrustum, float, double)
		{
			using V2 = Vec2<T>;
			using FR = Frustum3<T>;

			auto f = FR::MakeOrtho(T(10), T(8));
			PR_EXPECT(f.orthographic());
		}

		PRUnitTestMethod(Planes, float, double)
		{
			using V4 = Vec4<T>;
			using FR = Frustum3<T>;
			using P = Plane3<T>;

			auto f = FR::MakeWH(T(2), T(2), T(1), T(100));

			// Each plane should be valid
			auto p0 = f.plane(FR::EPlane::XPos);
			auto p1 = f.plane(FR::EPlane::XNeg);
			auto p2 = f.plane(FR::EPlane::YPos);
			auto p3 = f.plane(FR::EPlane::YNeg);
			auto p4 = f.plane(FR::EPlane::ZFar);

			// XPos and XNeg directions should oppose each other in x
			PR_EXPECT(p0.direction().x * p1.direction().x < T(0));

			// YPos and YNeg directions should oppose each other in y
			PR_EXPECT(p2.direction().y * p3.direction().y < T(0));
		}

		PRUnitTestMethod(MakeFromProjection, float, double)
		{
			using V2 = Vec2<T>;
			using M4 = Mat4x4<T>;
			using FR = Frustum3<T>;

			auto f1 = FR::MakeWH(T(4), T(3), T(1), T(50));
			auto proj = f1.projection(T(1), T(50));
			auto f2 = FR::MakeFromProjection(proj);
			PR_EXPECT(FEql(f1, f2));
		}
	};
}
#endif
