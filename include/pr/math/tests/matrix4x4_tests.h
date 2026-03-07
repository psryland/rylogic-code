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
	PRUnitTestClass(Matrix4x4)
	{
		std::default_random_engine rng = std::default_random_engine(1u);

		PRUnitTestMethod(Construction, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x4<T>;
			using mat4_t = Mat4x4<T>;

			// From scalar broadcast
			auto M0 = mat4_t(T(1));
			PR_EXPECT(M0.x == vec4_t(T(1)));
			PR_EXPECT(M0.y == vec4_t(T(1)));
			PR_EXPECT(M0.z == vec4_t(T(1)));
			PR_EXPECT(M0.w == vec4_t(T(1)));

			// From Vec4 columns
			auto M1 = mat4_t(
				vec4_t(T(1), T(2), T(3), T(4)),
				vec4_t(T(5), T(6), T(7), T(8)),
				vec4_t(T(9), T(10), T(11), T(12)),
				vec4_t(T(13), T(14), T(15), T(16)));
			PR_EXPECT(M1.x == vec4_t(T(1), T(2), T(3), T(4)));
			PR_EXPECT(M1.w == vec4_t(T(13), T(14), T(15), T(16)));

			// From Mat3x4 + position
			auto rot = mat3_t::Identity();
			auto pos = vec4_t(T(1), T(2), T(3), T(1));
			auto M2 = mat4_t(rot, pos);
			PR_EXPECT(M2.x == rot.x);
			PR_EXPECT(M2.y == rot.y);
			PR_EXPECT(M2.z == rot.z);
			PR_EXPECT(M2.w == pos);

			// Array access
			PR_EXPECT(M1[0] == M1.x);
			PR_EXPECT(M1[1] == M1.y);
			PR_EXPECT(M1[2] == M1.z);
			PR_EXPECT(M1[3] == M1.w);
		}
		PRUnitTestMethod(ColRow, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;
			using mat4_t = Mat4x4<T>;

			auto M = mat4_t(
				vec4_t(T(1), T(2), T(3), T(4)),
				vec4_t(T(5), T(6), T(7), T(8)),
				vec4_t(T(9), T(10), T(11), T(12)),
				vec4_t(T(13), T(14), T(15), T(16)));

			// col(i) returns the i-th column
			PR_EXPECT(M.col(0) == vec4_t(T(1), T(2), T(3), T(4)));
			PR_EXPECT(M.col(3) == vec4_t(T(13), T(14), T(15), T(16)));

			// row(i) returns the i-th row
			PR_EXPECT(M.row(0) == vec4_t(T(1), T(5), T(9), T(13)));
			PR_EXPECT(M.row(3) == vec4_t(T(4), T(8), T(12), T(16)));

			// Set col/row
			auto M2 = M;
			M2.col(2, vec4_t(T(90), T(100), T(110), T(120)));
			PR_EXPECT(M2.col(2) == vec4_t(T(90), T(100), T(110), T(120)));

			auto M3 = M;
			M3.row(0, vec4_t(T(10), T(50), T(90), T(130)));
			PR_EXPECT(M3.x[0] == T(10) && M3.y[0] == T(50) && M3.z[0] == T(90) && M3.w[0] == T(130));
		}
		PRUnitTestMethod(Constants, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;
			using mat4_t = Mat4x4<T>;

			PR_EXPECT(mat4_t::Zero() == mat4_t(T(0)));

			auto I = mat4_t::Identity();
			PR_EXPECT(I.x == vec4_t(T(1), T(0), T(0), T(0)));
			PR_EXPECT(I.y == vec4_t(T(0), T(1), T(0), T(0)));
			PR_EXPECT(I.z == vec4_t(T(0), T(0), T(1), T(0)));
			PR_EXPECT(I.w == vec4_t(T(0), T(0), T(0), T(1)));

			// Identity * Identity = Identity
			PR_EXPECT(I * I == I);
		}
		PRUnitTestMethod(TraceScaleUnscaled, float, double)
		{
			using vec4_t = Vec4<T>;
			using mat4_t = Mat4x4<T>;

			auto M = mat4_t(
				vec4_t(T(2), T(0), T(0), T(0)),
				vec4_t(T(0), T(3), T(0), T(0)),
				vec4_t(T(0), T(0), T(4), T(0)),
				vec4_t(T(0), T(0), T(0), T(1)));

			// trace returns diagonal elements
			auto tr = M.diagonal();
			PR_EXPECT(tr.x == T(2) && tr.y == T(3) && tr.z == T(4) && tr.w == T(1));
		}
		PRUnitTestMethod(TranslationFactory, float, double)
		{
			using vec4_t = Vec4<T>;
			using mat4_t = Mat4x4<T>;

			auto pos = vec4_t(T(1), T(2), T(3), T(1));
			auto M1 = mat4_t::Translation(pos);
			auto M2 = mat4_t::Translation(T(1), T(2), T(3));
			PR_EXPECT(M1 == M2);

			// Translating a position should add the translation
			auto p = vec4_t(T(0), T(0), T(0), T(1));
			auto r = M1 * p;
			PR_EXPECT(r == vec4_t(T(1), T(2), T(3), T(1)));

			// Translating a direction should not change it
			auto d = vec4_t(T(1), T(0), T(0), T(0));
			PR_EXPECT(M1 * d == d);
		}
		PRUnitTestMethod(TransformComposition, float, double)
		{
			// Composing transforms: (B*A)*v should equal B*(A*v)
			using vec4_t = Vec4<T>;
			using mat4_t = Mat4x4<T>;

			auto V1 = vec4_t(T(1), T(2), T(3), T(1));
			auto a2b = mat4_t::Transform(vec4_t::Normal(T(3), T(-2), T(-1), T(0)), T(1.23), vec4_t(T(4.4), T(-3.3), T(2.2), T(1)));
			auto b2c = mat4_t::Transform(vec4_t::Normal(T(-1), T(2), T(-3), T(0)), T(-3.21), vec4_t(T(-1.1), T(2.2), T(-3.3), T(1)));
			PR_EXPECT(IsOrthonormal(a2b));
			PR_EXPECT(IsOrthonormal(b2c));

			auto V2 = a2b * V1;
			auto V3 = b2c * V2;
			auto a2c = b2c * a2b;
			auto V4 = a2c * V1;
			PR_EXPECT(FEql(V3, V4));
		}
		PRUnitTestMethod(TransformFromQuat, float, double)
		{
			// Transform from quaternion should match transform from euler angles
			using vec4_t = Vec4<T>;
			using quat_t = Quat<T>;
			using mat4_t = Mat4x4<T>;

			auto q = quat_t(T(1.0), T(0.5), T(0.7));
			auto m1 = mat4_t::Transform(vec4_t::Normal(T(1), T(0), T(0), T(0)), T(1.0), vec4_t::Origin());
			auto m2 = mat4_t::Transform(q, vec4_t::Origin());
			PR_EXPECT(IsOrthonormal(m1));
			PR_EXPECT(IsOrthonormal(m2));

			// Random axis-angle round-trip
			std::uniform_real_distribution<T> dist(T(-1), T(1));
			auto ang = dist(rng);
			auto axis = vec4_t::Normal(T(1), T(2), T(3), T(0));
			auto m3 = mat4_t::Transform(axis, ang, vec4_t::Origin());
			auto m4 = mat4_t::Transform(quat_t(axis, ang), vec4_t::Origin());
			PR_EXPECT(IsOrthonormal(m3));
			PR_EXPECT(IsOrthonormal(m4));
			PR_EXPECT(FEql(m3, m4));
		}
		PRUnitTestMethod(W0, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;
			using mat4_t = Mat4x4<T>;

			auto M = mat4_t(
				vec4_t(T(1), T(0), T(0), T(0)),
				vec4_t(T(0), T(1), T(0), T(0)),
				vec4_t(T(0), T(0), T(1), T(0)),
				vec4_t(T(5), T(6), T(7), T(1)));

			// w0 strips the translation, setting w column to Origin
			auto M0 = M.w0();
			PR_EXPECT(M0.w == vec4_t::Origin());
			PR_EXPECT(M0.x == M.x && M0.y == M.y && M0.z == M.z);
		}
	};
}
#endif