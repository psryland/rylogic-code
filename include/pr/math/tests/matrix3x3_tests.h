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
	PRUnitTestClass(Matrix3x3)
	{
		std::default_random_engine rng = std::default_random_engine(1u);

		PRUnitTestMethod(Construction, float, double, int32_t, int64_t)
		{
			using vec3_t = Vec3<T>;
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x3<T>;

			// From scalar broadcast (w padding is always 0)
			auto M0 = mat3_t(T(1));
			PR_EXPECT(All(M0.x == vec3_t(T(1), T(1), T(1))));
			PR_EXPECT(All(M0.y == vec3_t(T(1), T(1), T(1))));
			PR_EXPECT(All(M0.z == vec3_t(T(1), T(1), T(1))));
			PR_EXPECT(M0.xw == T(0) && M0.yw == T(0) && M0.zw == T(0));

			// From Vec4 columns (w components are zeroed)
			auto M1 = mat3_t(vec4_t(T(1), T(2), T(3), T(99)), vec4_t(T(4), T(5), T(6), T(99)), vec4_t(T(7), T(8), T(9), T(99)));
			PR_EXPECT(All(M1.x == vec3_t(T(1), T(2), T(3))));
			PR_EXPECT(All(M1.y == vec3_t(T(4), T(5), T(6))));
			PR_EXPECT(All(M1.z == vec3_t(T(7), T(8), T(9))));
			PR_EXPECT(M1.xw == T(0) && M1.yw == T(0) && M1.zw == T(0));

			// From Vec3 columns
			auto M2 = mat3_t(vec3_t(T(1), T(2), T(3)), vec3_t(T(4), T(5), T(6)), vec3_t(T(7), T(8), T(9)));
			PR_EXPECT(All(M2.x == M1.x) && All(M2.y == M1.y) && All(M2.z == M1.z));

			// From range (9 scalars, 3x3)
			T arr[] = { T(1),T(2),T(3), T(4),T(5),T(6), T(7),T(8),T(9) };
			auto M3 = mat3_t(arr);
			PR_EXPECT(All(M3.x == M1.x) && All(M3.y == M1.y) && All(M3.z == M1.z));

			// From std::array and std::span
			constexpr std::array<T, 9> array_values{ T(10), T(11), T(12), T(13), T(14), T(15), T(16), T(17), T(18) };
			constexpr mat3_t from_array(array_values);
			static_assert(All(from_array.x == vec3_t(T(10), T(11), T(12))));
			static_assert(All(from_array.y == vec3_t(T(13), T(14), T(15))));
			static_assert(All(from_array.z == vec3_t(T(16), T(17), T(18))));

			std::span<T const> span_values{ array_values };
			mat3_t from_span(span_values);
			PR_EXPECT(All(from_span.x == vec3_t(T(10), T(11), T(12))));
			PR_EXPECT(All(from_span.y == vec3_t(T(13), T(14), T(15))));
			PR_EXPECT(All(from_span.z == vec3_t(T(16), T(17), T(18))));

			// PtrRange comes from generic_constructor_tests.h and has no range-level operator[].
			T const ptr_data[] = { T(19), T(20), T(21), T(22), T(23), T(24), T(25), T(26), T(27), T(99) };
			PtrRange<T> ptr_range{ ptr_data, ptr_data + 9 };
			mat3_t from_ptr_range(ptr_range);
			PR_EXPECT(All(from_ptr_range.x == vec3_t(T(19), T(20), T(21))));
			PR_EXPECT(All(from_ptr_range.y == vec3_t(T(22), T(23), T(24))));
			PR_EXPECT(All(from_ptr_range.z == vec3_t(T(25), T(26), T(27))));

			// Unbounded iota is not sized, so the constructor must not ask for distance.
			auto iota_range = std::views::iota(0);
			mat3_t from_iota(iota_range);
			PR_EXPECT(All(from_iota.x == vec3_t(T(0), T(1), T(2))));
			PR_EXPECT(All(from_iota.y == vec3_t(T(3), T(4), T(5))));
			PR_EXPECT(All(from_iota.z == vec3_t(T(6), T(7), T(8))));

			// Non-convertible references are rejected at compile time.
			static_assert(!std::is_constructible_v<mat3_t, std::array<char const*, 9> const&>);

			// Array access (returns Vec3)
			PR_EXPECT(All(M1[0] == M1.x));
			PR_EXPECT(All(M1[1] == M1.y));
			PR_EXPECT(All(M1[2] == M1.z));
		}
		PRUnitTestMethod(ColRow, float, double, int32_t, int64_t)
		{
			using vec3_t = Vec3<T>;
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x3<T>;

			auto M = mat3_t(
				vec3_t(T(1), T(2), T(3)),
				vec3_t(T(4), T(5), T(6)),
				vec3_t(T(7), T(8), T(9)));

			// col(i) returns the i-th column (as Vec4 with w=0)
			PR_EXPECT(All(M.col(0) == vec3_t(T(1), T(2), T(3))));
			PR_EXPECT(All(M.col(1) == vec3_t(T(4), T(5), T(6))));
			PR_EXPECT(All(M.col(2) == vec3_t(T(7), T(8), T(9))));

			// row(i) returns the i-th row (as Vec4 with w=0)
			PR_EXPECT(All(M.row(0) == vec3_t(T(1), T(4), T(7))));
			PR_EXPECT(All(M.row(1) == vec3_t(T(2), T(5), T(8))));
			PR_EXPECT(All(M.row(2) == vec3_t(T(3), T(6), T(9))));

			// Set col/row
			auto M2 = M;
			M2.col(1, vec3_t(T(40), T(50), T(60)));
			PR_EXPECT(All(M2.col(1) == vec3_t(T(40), T(50), T(60))));

			auto M3 = M;
			M3.row(1, vec3_t(T(20), T(50), T(80)));
			PR_EXPECT(M3.x[1] == T(20) && M3.y[1] == T(50) && M3.z[1] == T(80));
		}
		PRUnitTestMethod(Constants, float, double, int32_t, int64_t)
		{
			using vec3_t = Vec3<T>;
			using mat3_t = Mat3x3<T>;

			PR_EXPECT(All(mat3_t::Zero() == mat3_t(T(0))));

			auto I = mat3_t::Identity();
			PR_EXPECT(All(I.x == vec3_t(T(1), T(0), T(0))));
			PR_EXPECT(All(I.y == vec3_t(T(0), T(1), T(0))));
			PR_EXPECT(All(I.z == vec3_t(T(0), T(0), T(1))));

			// Identity * Identity = Identity
			PR_EXPECT(All(I * I == I));
		}
		PRUnitTestMethod(W1Factory, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x3<T>;
			using mat4_t = Mat4x4<T>;

			auto rot = mat3_t::Identity();
			auto pos = vec4_t(T(1), T(2), T(3), T(1));

			// w1() creates a 4x4 matrix from this 3x4
			auto M = rot.w1(pos);
			PR_EXPECT(All(M.x == rot.x4));
			PR_EXPECT(All(M.y == rot.y4));
			PR_EXPECT(All(M.z == rot.z4));
			PR_EXPECT(All(M.w == pos));
		}
		PRUnitTestMethod(TraceScaleUnscaled, float, double)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x3<T>;

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
			using vec3_t = Vec3<T>;
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x3<T>;

			// Mat3x3 is a 3x3 matrix (w=0 invariant). Columns are x,y,z.
			auto m = mat3_t(
				vec3_t(T(1), T(2), T(3)),
				vec3_t(T(1), T(1), T(1)),
				vec3_t(T(4), T(3), T(2)));

			// Mat3x3 * Vec3: 3x3 rotation of a direction
			auto v3 = vec3_t(T(-3), T(4), T(2));
			
			// result[j] = v.x*x[j] + v.y*y[j] + v.z*z[j]
			auto R3 = vec3_t(T(-3*1 + 4*1 + 2*4), T(-3*2 + 4*1 + 2*3), T(-3*3 + 4*1 + 2*2));
			PR_EXPECT(All(m * v3 == R3));

			// Mat3x3 * Vec4: 3x3 rotation, w preserved
			auto v4 = vec4_t(T(-3), T(4), T(2), T(-2));
			auto R4 = vec4_t(R3.x, R3.y, R3.z, T(-2));
			PR_EXPECT(All(m * v4 == R4));
		}
		PRUnitTestMethod(RotationFactories, float, double)
		{
			using vec3_t = Vec3<T>;
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x3<T>;

			// Rotation from axis + angle
			auto axis = vec3_t::Normal(T(0), T(0), T(1));
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
			auto from = vec3_t::XAxis();
			auto to = vec3_t::YAxis();
			auto r2r = mat3_t::Rotation(from, to);
			PR_EXPECT(IsOrthonormal(r2r));
			PR_EXPECT(FEql(r2r * from, to));
		}
		PRUnitTestMethod(ScaleFactory, float, double)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x3<T>;

			// Uniform scale
			auto s1 = mat3_t::Scale(T(3));
			PR_EXPECT(All(s1 * vec4_t(T(1), T(1), T(1), T(0)) == vec4_t(T(3), T(3), T(3), T(0))));

			// Non-uniform scale
			auto s2 = mat3_t::Scale(T(2), T(3), T(4));
			PR_EXPECT(All(s2 * vec4_t(T(1), T(1), T(1), T(0)) == vec4_t(T(2), T(3), T(4), T(0))));
		}
		PRUnitTestMethod(IntrinsicValidation)
		{
			// Verify the optimised (SSE) implementations match the generic implementations.
			// constexpr evaluation forces the 'if consteval' branch (generic scalar path).
			// Runtime evaluation uses the intrinsic path. Comparing the two validates correctness.
			using S = float;
			using vec3_t = Vec3<S>;
			using vec4_t = Vec4<S>;
			using mat3_t = Mat3x3<S>;

			// Construct test matrices from explicit values (constexpr-friendly, no sin/cos)
			constexpr auto A = mat3_t(
				vec3_t(S(0.936), S(0.312), S(-0.159)),
				vec3_t(S(-0.290), S(0.950), S(0.115)),
				vec3_t(S(0.187), S(-0.061), S(0.980)));
			constexpr auto B = mat3_t(
				vec3_t(S(0.866), S(-0.250), S(0.433)),
				vec3_t(S(0.433), S(0.875), S(-0.216)),
				vec3_t(S(-0.250), S(0.413), S(0.875)));
			constexpr auto v3 = vec3_t(S(1), S(-2), S(3));
			constexpr auto v4 = vec4_t(S(1), S(-2), S(3), S(1));
			constexpr auto v4d = vec4_t(S(1), S(-2), S(3), S(0));

			// Reference: constexpr forces the generic (non-intrinsic) path via 'if consteval'
			constexpr auto tA_ref = Transpose(A);
			constexpr auto rv3_ref = A * v3;
			constexpr auto rv4_ref = A * v4;
			constexpr auto rv4d_ref = A * v4d;
			constexpr auto AB_ref = B * A;

			// Runtime: uses the intrinsic (SSE) path
			auto tA = Transpose(A);
			auto rv3 = A * v3;
			auto rv4 = A * v4;
			auto rv4d = A * v4d;
			auto AB = B * A;

			// Compare intrinsic results against generic results
			PR_EXPECT(FEql(tA, tA_ref));
			PR_EXPECT(FEql(rv3, rv3_ref));
			PR_EXPECT(FEql(rv4, rv4_ref));
			PR_EXPECT(FEql(rv4d, rv4d_ref));
			PR_EXPECT(FEql(AB, AB_ref));

			// Verify composition: (B*A)*v == B*(A*v)
			auto composed = AB * v3;
			auto sequential = B * (A * v3);
			PR_EXPECT(FEql(composed, sequential));

			// Double precision
			if constexpr (Vec4<double>::IntrinsicD)
			{
				using Sd = double;
				using vec3d = Vec3<Sd>;
				using mat3d = Mat3x3<Sd>;

				constexpr auto Ad = mat3d(
					vec3d(Sd(0.936), Sd(0.312), Sd(-0.159)),
					vec3d(Sd(-0.290), Sd(0.950), Sd(0.115)),
					vec3d(Sd(0.187), Sd(-0.061), Sd(0.980)));

				constexpr auto tAd_ref = Transpose(Ad);
				auto tAd = Transpose(Ad);
				PR_EXPECT(FEql(tAd, tAd_ref));
			}
		}
	};
}
#endif