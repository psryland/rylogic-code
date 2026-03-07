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
	PRUnitTestClass(XformTests)
	{
		std::default_random_engine rng;
		TestClass_XformTests()
			: rng(1u)
		{}

		PRUnitTestMethod(ConstructionRoundTrip, float, double)
		{
			using xform_t = Xform<T>;
			using m4x4_t = Mat4x4<T>;
			using v4_t = Vec4<T>;
			using v2_t = Vec2<T>;

			auto xf1 = Random<xform_t>(rng, v4_t(1, 1, 1, 1), T(2), v2_t(T(0.2), T(1.5)));
			auto m = m4x4_t(xf1);
			auto xf2 = xform_t(m);

			PR_EXPECT(FEql(xf1, xf2));
		}
		PRUnitTestMethod(Multiply, float, double)
		{
			using xform_t = Xform<T>;
			using m4x4_t = Mat4x4<T>;

			// Scale must be uniform, or multiply would result in shear
			auto xf1 = Random<xform_t>(rng, Vec2<T>{ T(2), T(2) });
			auto xf2 = Random<xform_t>(rng, Vec2<T>{ T(3), T(3) });
			auto xf3 = xf1 * xf2;

			auto m1 = m4x4_t(xf1);
			auto m2 = m4x4_t(xf2);
			auto m3 = m1 * m2;

			auto xf3_from_m = xform_t(m3);
			auto m3_from_xf = m4x4_t(xf3);

			// Note: ** This is only true if scale is uniform **
			PR_EXPECT(FEql(xf3, xf3_from_m));
			PR_EXPECT(FEql(m3, m3_from_xf));
		}
		PRUnitTestMethod(MultiplyVector, float, double)
		{
			using xform_t = Xform<T>;
			using m4x4_t = Mat4x4<T>;
			using v4_t = Vec4<T>;

			auto v0 = v4_t(1, 2, 3, 1);
			auto v1 = v4_t(1, 2, 3, 0);

			auto xf = Random<xform_t>(rng);
			auto m = m4x4_t(xf);

			v4_t r0 = xf * v0;
			v4_t r1 = xf * v1;

			v4_t R0 = m * v0;
			v4_t R1 = m * v1;

			PR_EXPECT(FEql(r0, R0));
			PR_EXPECT(FEql(r1, R1));
		}
		PRUnitTestMethod(Inversion, float, double)
		{
			using xform_t = Xform<T>;
			using m4x4_t = Mat4x4<T>;

			auto xf = Random<xform_t>(rng);
			auto m = m4x4_t(xf);

			auto xf_inv = Invert(xf);
			auto m_inv = Invert(m);

			auto r = m4x4_t(xf_inv);
			auto R = m_inv;
			PR_EXPECT(FEql(r, R));

			auto xf2 = Invert(xf_inv);
			auto m2 = Invert(m_inv);
			PR_EXPECT(FEql(xf, xf2));
			PR_EXPECT(FEql(m, m2));
		}
		PRUnitTestMethod(Identity, float, double)
		{
			using xform_t = Xform<T>;
			using m4x4_t = Mat4x4<T>;

			auto xf = xform_t::Identity();
			auto m = m4x4_t(xf);
			PR_EXPECT(FEql(m, m4x4_t::Identity()));
		}
		PRUnitTestMethod(S1, float, double)
		{
			using xform_t = Xform<T>;
			using v4_t = Vec4<T>;

			auto xf = Random<xform_t>(rng, Vec2<T>{ T(0.5), T(2.0) });
			auto xf_s1 = xf.s1();
			PR_EXPECT(FEql(xf_s1.scl, v4_t::One()));
			PR_EXPECT(FEqlOrientation(xf_s1.rot, xf.rot));
			PR_EXPECT(FEql(xf_s1.pos, xf.pos));
		}
	};
}
#endif
