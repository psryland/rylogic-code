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
	PRUnitTestClass(Matrix3x4)
	{
		std::default_random_engine rng = std::default_random_engine(1u);

		PRUnitTestMethod(Construction, float, double, int32_t, int64_t)
		{
			using vec3_t = Vec3<T>;
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x4<T>;

			// From scalar broadcast
			auto M0 = mat3_t(T(1));
			PR_EXPECT(M0.x == vec4_t(T(1)));
			PR_EXPECT(M0.y == vec4_t(T(1)));
			PR_EXPECT(M0.z == vec4_t(T(1)));

			// From Vec4 columns
			auto M1 = mat3_t(vec4_t(T(1), T(2), T(3), T(4)), vec4_t(T(5), T(6), T(7), T(8)), vec4_t(T(9), T(10), T(11), T(12)));
			PR_EXPECT(M1.x == vec4_t(T(1), T(2), T(3), T(4)));
			PR_EXPECT(M1.y == vec4_t(T(5), T(6), T(7), T(8)));
			PR_EXPECT(M1.z == vec4_t(T(9), T(10), T(11), T(12)));

			// From Vec3 columns (w components set to 0)
			auto M2 = mat3_t(vec3_t(T(1), T(2), T(3)), vec3_t(T(4), T(5), T(6)), vec3_t(T(7), T(8), T(9)));
			PR_EXPECT(M2.x == vec4_t(T(1), T(2), T(3), T(0)));
			PR_EXPECT(M2.y == vec4_t(T(4), T(5), T(6), T(0)));
			PR_EXPECT(M2.z == vec4_t(T(7), T(8), T(9), T(0)));

			// From range (12 scalars)
			T arr[] = { T(1),T(2),T(3),T(4), T(5),T(6),T(7),T(8), T(9),T(10),T(11),T(12) };
			auto M3 = mat3_t(arr);
			PR_EXPECT(M3.x == M1.x && M3.y == M1.y && M3.z == M1.z);

			// Array access
			PR_EXPECT(M1[0] == M1.x);
			PR_EXPECT(M1[1] == M1.y);
			PR_EXPECT(M1[2] == M1.z);
		}
		PRUnitTestMethod(ColRow, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x4<T>;

			auto M = mat3_t(
				vec4_t(T(1), T(2), T(3), T(0)),
				vec4_t(T(4), T(5), T(6), T(0)),
				vec4_t(T(7), T(8), T(9), T(0)));

			// col(i) returns the i-th column
			PR_EXPECT(M.col(0) == vec4_t(T(1), T(2), T(3), T(0)));
			PR_EXPECT(M.col(1) == vec4_t(T(4), T(5), T(6), T(0)));
			PR_EXPECT(M.col(2) == vec4_t(T(7), T(8), T(9), T(0)));

			// row(i) returns the i-th row (w component is always 0 for Mat3x4)
			PR_EXPECT(M.row(0) == vec4_t(T(1), T(4), T(7), T(0)));
			PR_EXPECT(M.row(1) == vec4_t(T(2), T(5), T(8), T(0)));
			PR_EXPECT(M.row(2) == vec4_t(T(3), T(6), T(9), T(0)));

			// Set col/row
			auto M2 = M;
			M2.col(1, vec4_t(T(40), T(50), T(60), T(0)));
			PR_EXPECT(M2.col(1) == vec4_t(T(40), T(50), T(60), T(0)));

			auto M3 = M;
			M3.row(1, vec4_t(T(20), T(50), T(80), T(0)));
			PR_EXPECT(M3.x[1] == T(20) && M3.y[1] == T(50) && M3.z[1] == T(80));
		}
		PRUnitTestMethod(Constants, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x4<T>;

			PR_EXPECT(mat3_t::Zero() == mat3_t(T(0)));

			auto I = mat3_t::Identity();
			PR_EXPECT(I.x == vec4_t(T(1), T(0), T(0), T(0)));
			PR_EXPECT(I.y == vec4_t(T(0), T(1), T(0), T(0)));
			PR_EXPECT(I.z == vec4_t(T(0), T(0), T(1), T(0)));

			// Identity * Identity = Identity
			PR_EXPECT(I * I == I);
		}
		PRUnitTestMethod(W1Factory, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x4<T>;
			using mat4_t = Mat4x4<T>;

			auto rot = mat3_t::Identity();
			auto pos = vec4_t(T(1), T(2), T(3), T(1));

			// w1() creates a 4x4 matrix from this 3x4
			auto M = rot.w1(pos);
			PR_EXPECT(M.x == rot.x);
			PR_EXPECT(M.y == rot.y);
			PR_EXPECT(M.z == rot.z);
			PR_EXPECT(M.w == pos);
		}
		PRUnitTestMethod(TraceScaleUnscaled, float, double)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x4<T>;

			auto M = mat3_t(
				vec4_t(T(2), T(0), T(0), T(0)),
				vec4_t(T(0), T(3), T(0), T(0)),
				vec4_t(T(0), T(0), T(4), T(0)));

			// trace returns diagonal elements
			auto tr = M.diagonal();
			PR_EXPECT(tr.x == T(2) && tr.y == T(3) && tr.z == T(4));

			// scale returns the column lengths as a diagonal matrix
			auto sc = M.scale();
			PR_EXPECT(FEql(sc.x.x, T(2)));
			PR_EXPECT(FEql(sc.y.y, T(3)));
			PR_EXPECT(FEql(sc.z.z, T(4)));

			// unscaled returns the matrix with unit-length columns
			auto us = M.unscaled();
			PR_EXPECT(FEql(us, mat3_t::Identity()));
		}
		PRUnitTestMethod(MultiplyVector, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x4<T>;

			auto m = mat3_t(vec4_t(T(1), T(2), T(3), T(4)), vec4_t(T(1), T(1), T(1), T(1)), vec4_t(T(4), T(3), T(2), T(1)));
			auto v4 = vec4_t(T(-3), T(4), T(2), T(-2));

			// Mat3x4 * Vec4: result.i = Dot(row_i, v4), row_i formed by transposing columns
			auto R4 = vec4_t(T(1), T(2), T(-3), T(0));
			PR_EXPECT(m * v4 == R4);
		}
		PRUnitTestMethod(RotationFactories, float, double)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x4<T>;

			// Rotation from axis + angle
			auto axis = vec4_t::Normal(T(0), T(0), T(1), T(0));
			auto rot = mat3_t::Rotation(axis, DegreesToRadians(T(90)));
			PR_EXPECT(IsOrthonormal(rot));

			// Should rotate (1,0,0,0) to (0,1,0,0)
			auto v = vec4_t::XAxis();
			auto r = rot * v;
			PR_EXPECT(FEql(r, vec4_t(T(0), T(1), T(0), T(0))));

			// Rotation from euler angles
			auto euler_rot = mat3_t::RotationRad(T(0), T(0), DegreesToRadians(T(90)));
			PR_EXPECT(IsOrthonormal(euler_rot));

			// Rotation from one vector to another
			auto from = vec4_t::XAxis();
			auto to = vec4_t::YAxis();
			auto r2r = mat3_t::Rotation(from, to);
			PR_EXPECT(IsOrthonormal(r2r));
			PR_EXPECT(FEql(r2r * from, to));
		}
		PRUnitTestMethod(ScaleFactory, float, double)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x4<T>;

			// Uniform scale
			auto s1 = mat3_t::Scale(T(3));
			PR_EXPECT(s1 * vec4_t(T(1), T(1), T(1), T(0)) == vec4_t(T(3), T(3), T(3), T(0)));

			// Non-uniform scale
			auto s2 = mat3_t::Scale(T(2), T(3), T(4));
			PR_EXPECT(s2 * vec4_t(T(1), T(1), T(1), T(0)) == vec4_t(T(2), T(3), T(4), T(0)));
		}
	};
}
#endif