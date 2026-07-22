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

		PRUnitTestMethod(Mul_Mat_Vec, float, double)
		{
			// Matrix * Vector convention: Vb = a2b * Va (right-to-left, column-vector)
			// Use a 90° rotation around Z that maps X→Y, Y→-X, plus translation (10,20,30)
			using Mat4x4 = Mat4x4<T>;
			using Vec4 = Vec4<T>;

			// Matrix members 'x, y, z, w' are column vectors stored contigously.
			// Don't get confused into thinking they are row vectors. The mathematical layout is not the same as the in-memory layout.
			// This matrix, in mathematical layout would be this:
			//   <x> <y> <z> <w>     <v>
			//  [ 0  -1   0  10 ]    [1]
			//  [ 1   0   0  20 ]  * [0]
			//  [ 0   0   1  30 ]	 [0]
			//  [ 0   0   0   1 ]	 [1]
			auto a2b = Mat4x4{
				Vec4{ 0,  1 , 0, 0}, // x-column
				Vec4{-1,  0,  0, 0}, // y-column
				Vec4{ 0,  0,  1, 0}, // z-column
				Vec4{10, 20, 30, 1}, // w-column
			};

			// Point (1,0,0,1) in space A should become: rotated X = (0,1,0) + translation = (10,21,30,1)
			{
				auto Vb = a2b * Vec4{ 1, 0, 0, 1 };
				PR_EXPECT(FEql(Vb, Vec4{ 10, 21, 30, 1 }));
			}

			// Direction (1,0,0,0) should just rotate, no translation
			{
				auto Vb = a2b * Vec4{ 1, 0, 0, 0 };
				PR_EXPECT(FEql(Vb, Vec4{ 0, 1, 0, 0 }));
			}

			// Point (0,1,0,1) should become: rotated Y = (-1,0,0) + translation = (9,20,30,1)
			{
				auto Vb = a2b * Vec4{ 0, 1, 0, 1 };
				PR_EXPECT(FEql(Vb, Vec4{ 9, 20, 30, 1 }));
			}
		}
		PRUnitTestMethod(Mul_Mat_Mat, float, double)
		{
			using Mat4x4 = Mat4x4<T>;
			using Vec4 = Vec4<T>;

			// Matrix * Matrix convention: a2c = b2c * a2b (right-to-left)
			// a2b: 90° around Z + translate (10,0,0)
			// b2c: 90° around Z + translate (0,5,0)
			// a2c should be 180° around Z + translate(-5,10,0)
			auto a2b = Mat4x4{
				Vec4{ 0,  1,  0, 0}, // x-column
				Vec4{-1,  0,  0, 0}, // y-column
				Vec4{ 0,  0,  1, 0}, // z-column
				Vec4{10,  0,  0, 1}, // w-column
			};
			auto b2c = Mat4x4{
				Vec4{ 0,  1,  0, 0}, // x-column
				Vec4{-1,  0,  0, 0}, // y-column
				Vec4{ 0,  0,  1, 0}, // z-column
				Vec4{ 0,  5,  0, 1}, // w-column
			};
			auto a2c = b2c * a2b;

			// (1,0,0,0) through a2b gives (0,1,0,0), through b2c gives (-1,0,0,0)
			{
				auto Vc = a2c * Vec4{ 1, 0, 0, 0 };
				PR_EXPECT(FEql(Vc, Vec4{ -1, 0, 0, 0 }));
			}

			// Origin (0,0,0,1) through a2b gives (10,0,0,1), through b2c gives (0,10+5,0,1) = (0,15,0,1)
			{
				auto Vc = a2c * Vec4{ 0, 0, 0, 1 };
				PR_EXPECT(FEql(Vc, Vec4{ 0, 15, 0, 1 }));
			}

			// Same result from chaining: b2c * (a2b * v) == (b2c * a2b) * v
			{
				auto Va = Vec4{ 3, -2, 7, 1 };
				auto Vc_0 = a2c * Va;
				auto Vc_1 = b2c * (a2b * Va);
				PR_EXPECT(FEql(Vc_1, Vc_0));
			}
		}
		PRUnitTestMethod(Construction, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;
			using mat3_t = Mat3x3<T>;
			using mat4_t = Mat4x4<T>;

			// From scalar broadcast
			auto M0 = mat4_t(T(1));
			PR_EXPECT(All(M0.x == vec4_t(T(1))));
			PR_EXPECT(All(M0.y == vec4_t(T(1))));
			PR_EXPECT(All(M0.z == vec4_t(T(1))));
			PR_EXPECT(All(M0.w == vec4_t(T(1))));

			// From Vec4 columns
			auto M1 = mat4_t(
				vec4_t(T(1), T(2), T(3), T(4)),
				vec4_t(T(5), T(6), T(7), T(8)),
				vec4_t(T(9), T(10), T(11), T(12)),
				vec4_t(T(13), T(14), T(15), T(16)));
			PR_EXPECT(All(M1.x == vec4_t(T(1), T(2), T(3), T(4))));
			PR_EXPECT(All(M1.w == vec4_t(T(13), T(14), T(15), T(16))));

			// From Mat3x3 + position
			auto rot = mat3_t::Identity();
			auto pos = vec4_t(T(1), T(2), T(3), T(1));
			auto M2 = mat4_t(rot, pos);
			PR_EXPECT(All(M2.x == rot.x4));
			PR_EXPECT(All(M2.y == rot.y4));
			PR_EXPECT(All(M2.z == rot.z4));
			PR_EXPECT(All(M2.w == pos));

			// Array access
			PR_EXPECT(All(M1[0] == M1.x));
			PR_EXPECT(All(M1[1] == M1.y));
			PR_EXPECT(All(M1[2] == M1.z));
			PR_EXPECT(All(M1[3] == M1.w));
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
			PR_EXPECT(All(M.col(0) == vec4_t(T(1), T(2), T(3), T(4))));
			PR_EXPECT(All(M.col(3) == vec4_t(T(13), T(14), T(15), T(16))));

			// row(i) returns the i-th row
			PR_EXPECT(All(M.row(0) == vec4_t(T(1), T(5), T(9), T(13))));
			PR_EXPECT(All(M.row(3) == vec4_t(T(4), T(8), T(12), T(16))));

			// Set col/row
			auto M2 = M;
			M2.col(2, vec4_t(T(90), T(100), T(110), T(120)));
			PR_EXPECT(All(M2.col(2) == vec4_t(T(90), T(100), T(110), T(120))));

			auto M3 = M;
			M3.row(0, vec4_t(T(10), T(50), T(90), T(130)));
			PR_EXPECT(M3.x[0] == T(10) && M3.y[0] == T(50) && M3.z[0] == T(90) && M3.w[0] == T(130));
		}
		PRUnitTestMethod(Constants, float, double, int32_t, int64_t)
		{
			using vec4_t = Vec4<T>;
			using mat4_t = Mat4x4<T>;

			PR_EXPECT(All(mat4_t::Zero() == mat4_t(T(0))));

			auto I = mat4_t::Identity();
			PR_EXPECT(All(I.x == vec4_t(T(1), T(0), T(0), T(0))));
			PR_EXPECT(All(I.y == vec4_t(T(0), T(1), T(0), T(0))));
			PR_EXPECT(All(I.z == vec4_t(T(0), T(0), T(1), T(0))));
			PR_EXPECT(All(I.w == vec4_t(T(0), T(0), T(0), T(1))));

			// Identity * Identity = Identity
			PR_EXPECT(All(I * I == I));
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
			PR_EXPECT(All(M1 == M2));

			// Translating a position should add the translation
			auto p = vec4_t(T(0), T(0), T(0), T(1));
			auto r = M1 * p;
			PR_EXPECT(All(r == vec4_t(T(1), T(2), T(3), T(1))));

			// Translating a direction should not change it
			auto d = vec4_t(T(1), T(0), T(0), T(0));
			PR_EXPECT(All(M1 * d == d));
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
			using other_t = std::conditional_t<std::is_same_v<T, float>, double, float>;

			// The public overload is same-scalar only, so mixed scalar calls are rejected
			// before the function body is considered.
			static_assert(requires { mat4_t::Transform(quat_t::Identity(), vec4_t::Origin()); });
			static_assert(!requires { mat4_t::Transform(Quat<other_t>::Identity(), vec4_t::Origin()); });

			auto q = quat_t(T(1.0), T(0.5), T(0.7));
			auto pos = vec4_t(T(4), T(-3), T(2), T(1));
			auto m1 = mat4_t::Transform(vec4_t::Normal(T(1), T(0), T(0), T(0)), T(1.0), pos);
			auto m2 = mat4_t::Transform(q, pos);
			PR_EXPECT(IsOrthonormal(m1));
			PR_EXPECT(IsOrthonormal(m2));
			PR_EXPECT(FEql(m2 * vec4_t::Origin(), pos));

			// Random axis-angle round-trip
			std::uniform_real_distribution<T> dist(T(-1), T(1));
			auto ang = dist(rng);
			auto axis = vec4_t::Normal(T(1), T(2), T(3), T(0));
			auto m3 = mat4_t::Transform(axis, ang, pos);
			auto m4 = mat4_t::Transform(quat_t(axis, ang), pos);
			PR_EXPECT(IsOrthonormal(m3));
			PR_EXPECT(IsOrthonormal(m4));
			PR_EXPECT(FEql(m3, m4));
			PR_EXPECT(FEql(m4 * vec4_t::Origin(), pos));
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
			auto M0 = M.w1();
			PR_EXPECT(All(M0.w == vec4_t::Origin()));
			PR_EXPECT(All(M0.x == M.x) && All(M0.y == M.y) && All(M0.z == M.z));
		}
		PRUnitTestMethod(IntrinsicValidation)
		{
			// Verify the optimised (SSE/AVX) implementations match the generic implementations.
			// constexpr evaluation forces the 'if consteval' branch (generic scalar path).
			// Runtime evaluation uses the intrinsic path. Comparing the two validates correctness.
			using S = float;
			using vec4_t = Vec4<S>;
			using mat4_t = Mat4x4<S>;

			// Construct test matrices from explicit values (constexpr-friendly, no sin/cos)
			constexpr auto A = mat4_t(
				vec4_t(S(0.936), S(0.312), S(-0.159), S(0)),
				vec4_t(S(-0.290), S(0.950), S(0.115), S(0)),
				vec4_t(S(0.187), S(-0.061), S(0.980), S(0)),
				vec4_t(S(1.5), S(-0.7), S(3.2), S(1)));
			constexpr auto B = mat4_t(
				vec4_t(S(0.866), S(-0.250), S(0.433), S(0)),
				vec4_t(S(0.433), S(0.875), S(-0.216), S(0)),
				vec4_t(S(-0.250), S(0.413), S(0.875), S(0)),
				vec4_t(S(-2.1), S(0.5), S(1.8), S(1)));
			constexpr auto v = vec4_t(S(1), S(-2), S(3), S(1));
			constexpr auto vd = vec4_t(S(1), S(-2), S(3), S(0));

			// Reference: constexpr forces the generic (non-intrinsic) path via 'if consteval'
			constexpr auto tA_ref = Transpose(A);
			constexpr auto rv_ref = A * v;
			constexpr auto rvd_ref = A * vd;
			constexpr auto AB_ref = B * A;

			// Runtime: uses the intrinsic (SSE) path
			auto tA = Transpose(A);
			auto rv = A * v;
			auto rvd = A * vd;
			auto AB = B * A;

			// Compare intrinsic results against generic results
			PR_EXPECT(FEql(tA, tA_ref));
			PR_EXPECT(FEql(rv, rv_ref));
			PR_EXPECT(FEql(rvd, rvd_ref));
			PR_EXPECT(FEql(AB, AB_ref));

			// Verify composition: (B*A)*v == B*(A*v)
			auto composed = AB * v;
			auto sequential = B * (A * v);
			PR_EXPECT(FEql(composed, sequential));

			// Double precision
			if constexpr (Vec4<double>::IntrinsicD)
			{
				using Sd = double;
				using vec4d = Vec4<Sd>;
				using mat4d = Mat4x4<Sd>;

				constexpr auto Ad = mat4d(
					vec4d(Sd(0.936), Sd(0.312), Sd(-0.159), Sd(0)),
					vec4d(Sd(-0.290), Sd(0.950), Sd(0.115), Sd(0)),
					vec4d(Sd(0.187), Sd(-0.061), Sd(0.980), Sd(0)),
					vec4d(Sd(1.5), Sd(-0.7), Sd(3.2), Sd(1)));
				constexpr auto Bd = mat4d(
					vec4d(Sd(0.866), Sd(-0.250), Sd(0.433), Sd(0)),
					vec4d(Sd(0.433), Sd(0.875), Sd(-0.216), Sd(0)),
					vec4d(Sd(-0.250), Sd(0.413), Sd(0.875), Sd(0)),
					vec4d(Sd(-2.1), Sd(0.5), Sd(1.8), Sd(1)));
				constexpr auto vd_d = vec4d(Sd(1), Sd(-2), Sd(3), Sd(1));

				constexpr auto tAd_ref = Transpose(Ad);
				constexpr auto rvd_d_ref = Ad * vd_d;
				constexpr auto ABd_ref = Bd * Ad;

				auto tAd = Transpose(Ad);
				auto rvd_d = Ad * vd_d;
				auto ABd = Bd * Ad;

				PR_EXPECT(FEql(tAd, tAd_ref));
				PR_EXPECT(FEql(rvd_d, rvd_d_ref));
				PR_EXPECT(FEql(ABd, ABd_ref));
			}
		}
	};
}
#endif