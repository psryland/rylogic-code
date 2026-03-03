//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/maths/forward.h"
#include "pr/maths/maths_core.h"
#include "pr/maths/vector4.h"
#include "pr/maths/vector8.h"
#include "pr/maths/matrix3x4.h"
#include "pr/maths/matrix4x4.h"

namespace pr
{
	// General 6x8 matrix
	template <Scalar S, typename A, typename B>
	struct Mat6x8
	{
		// Careful with memory layout: (Same style of memory layout as m4x4)
		// e.g.
		//  [{x} {y} {z} {w} {u} {v}]
		// is:                                                            memory order
		//  [x.x y.x z.x w.x u.x v.x] = [m00 m00 m00  m01 m01 m01]     [00 05 09  25 29 33]
		//  [x.y y.y z.y w.y u.y v.y] = [m00 m00 m00  m01 m01 m01]     [01 06 10  26 30 34]
		//  [x.z y.z z.z w.z u.z v.z] = [m00 m00 m00  m01 m01 m01]     [02 07 11  27 31 35]
		//  [x.- y.- z.- w.- u.- v.-] = [m00 m00 m00  m01 m01 m01]     [04 08 12  28 32 36]
		//  [x.w y.w z.w w.w u.w v.w] = [m10 m10 m10  m11 m11 m11]     [13 17 21  37 41 45]
		//  [x.u y.u z.u w.u u.u v.u] = [m10 m10 m10  m11 m11 m11]     [14 18 22  38 42 46]
		//  [x.v y.v z.v w.v u.v v.v] = [m10 m10 m10  m11 m11 m11]     [15 19 23  39 43 47]
		//  [x.- y.- z.- w.- u.- v.-] = [m10 m10 m10  m11 m11 m11]     [16 20 24  40 44 48]
		//                                                             
		// Notes:
		//  A,B should be the vector space that the transform operates on.
		//  Transforms within the same vector space should have A == B (e.g. coordinate transforms).
		//  Transforms from one to another vector space have A != B (e.g. inertia transforms).
		Mat3x4<S, void,void> m00, m10, m01, m11;

		// Construct from sub matrices. WARNING: careful with layout.
		Mat6x8() = default;
		constexpr explicit Mat6x8(S x_)
			:m00(x_)
			,m10(x_)
			,m01(x_)
			,m11(x_)
		{}
		constexpr Mat6x8(Mat3x4<S,void,void> const& m00_, Mat3x4<S,void,void> const& m01_, Mat3x4<S,void,void> const& m10_, Mat3x4<S,void,void> const& m11_)
			:m00(m00_)
			,m10(m10_)
			,m01(m01_)
			,m11(m11_)
		{}
		constexpr Mat6x8(Vec8<S,void> x, Vec8<S,void> y, Vec8<S,void> z, Vec8<S,void> u, Vec8<S,void> v, Vec8<S,void> w)
			:m00(x.ang, y.ang, z.ang)
			,m10(x.lin, y.lin, z.lin)
			,m01(u.ang, v.ang, w.ang)
			,m11(u.lin, v.lin, w.lin)
		{}

		// Reinterpret as a different matrix type
		template <typename C, typename D> explicit operator Mat6x8<S,C,D> const&() const
		{
			return reinterpret_cast<Mat6x8<S,C,D> const&>(*this);
		}
		template <typename C, typename D> explicit operator Mat6x8<S,C,D>&()
		{
			return reinterpret_cast<Mat6x8<S,C,D>&>(*this);
		}
		operator Mat6x8<S,void,void> const& () const
		{
			return reinterpret_cast<Mat6x8<S,void,void> const&>(*this);
		}
		operator Mat6x8<S,void,void>& ()
		{
			return reinterpret_cast<Mat6x8<S,void,void>&>(*this);
		}

		// Array of column vectors
		Vec8<S,void> operator [](int i) const
		{
			// Note: Creating a Vec8Proxy doesn't work because by default the compiler selects the
			// mutable overload for non-const instances, so swap-style assignments don't work.
			pr_assert("index out of range" && i >= 0 && i < 6);
			return i < 3
				? Vec8<S,void>{m00[i  ], m10[i  ]}
				: Vec8<S,void>{m01[i-3], m11[i-3]};
		}
		Vec8<S,void> col(int i) const
		{
			pr_assert("index out of range" && i >= 0 && i < 6);
			return (*this)[i];
		}
		void col(int i, Vec8<S,void> rhs)
		{
			pr_assert("index out of range" && i >= 0 && i < 6);
			if (i < 3) { m00[i  ] = rhs.ang; m10[i  ] = rhs.lin; }
			else       { m01[i-3] = rhs.ang; m11[i-3] = rhs.lin; }
		}

		// Basic constants
		static constexpr Mat6x8 Zero()
		{
			return Mat6x8 {
				Mat3x4<S, void, void>::Zero(), Mat3x4<S, void, void>::Zero(),
				Mat3x4<S, void, void>::Zero(), Mat3x4<S, void, void>::Zero()};
		}
		static constexpr Mat6x8 Identity()
		{
			return Mat6x8 {
				Mat3x4<S, void, void>::Identity(), Mat3x4<S, void, void>::Zero(),
				Mat3x4<S, void, void>::Zero(), Mat3x4<S, void, void>::Identity()};
		}

		#pragma region Operators
		friend constexpr Mat6x8 operator + (Mat6x8<S,A,B> const& m)
		{
			return m;
		}
		friend constexpr Mat6x8 operator - (Mat6x8<S,A,B> const& m)
		{
			return Mat6x8{-m.m00, -m.m01, -m.m10, -m.m11};
		}
		friend Mat6x8 operator + (Mat6x8<S,A,B> const& lhs, Mat6x8<S,A,B> const& rhs)
		{
			return Mat6x8{lhs.m00 + rhs.m00, lhs.m01 + rhs.m01, lhs.m10 + rhs.m10, lhs.m11 + rhs.m11};
		}
		friend Mat6x8 operator - (Mat6x8<S,A,B> const& lhs, Mat6x8<S,A,B> const& rhs)
		{
			return Mat6x8{lhs.m00 - rhs.m00, lhs.m01 - rhs.m01, lhs.m10 - rhs.m10, lhs.m11 - rhs.m11};
		}
		friend Mat6x8 operator * (Mat6x8<S,A,B> const& lhs, S rhs)
		{
			return Mat6x8{lhs.m00 * rhs, lhs.m01 * rhs, lhs.m10 * rhs, lhs.m11 * rhs};
		}
		friend Mat6x8 operator * (S lhs, Mat6x8<S,A,B> const& rhs)
		{
			return rhs * lhs;
		}
		friend Vec8<S,B> operator * (Mat6x8<S,A,B> const& lhs, Vec8<S,A> const& rhs)
		{
			// [m00*a + m01*b] = [m00, m01] [a]
			// [m10*a + m11*b]   [m10, m11] [b]
			return Vec8<S,B>{
				lhs.m00 * rhs.ang + lhs.m01 * rhs.lin,
				lhs.m10 * rhs.ang + lhs.m11 * rhs.lin};
		}
		template <typename C> friend Mat6x8<S,A,C> pr_vectorcall operator * (Mat6x8<S,B,C> const& lhs, Mat6x8<S,A,B> const& rhs)
		{
			// [a00, a01] [b00, b01] = [a00*b00 + a01*b10, a00*b01 + a01*b11]
			// [a10, a11] [b10, b11]   [a10*b00 + a11*b10, a10*b01 + a11*b11]
			return Mat6x8<S,A,C>{
				lhs.m00*rhs.m00 + lhs.m01*rhs.m10, lhs.m00*rhs.m01 + lhs.m01*rhs.m11,
				lhs.m10*rhs.m00 + lhs.m11*rhs.m10, lhs.m10*rhs.m01 + lhs.m11*rhs.m11};
		}
		#pragma endregion
	};
	#define PR_MAT6X8_CHECKS(scalar)\
	static_assert(sizeof(Mat6x8<scalar,void,void>) == 6*8*sizeof(scalar), "Mat6<"#scalar"> has the wrong size");\
	static_assert(maths::is_vec<Mat6x8<scalar,void,void>>::value, "Mat6<"#scalar"> is not a Mat6x8");\
	static_assert(std::is_trivially_copyable_v<Mat6x8<scalar,void,void>>, "Mat6x8<"#scalar"> must be a pod type");\
	static_assert(std::alignment_of_v<Mat6x8<scalar, void, void>> == std::alignment_of_v<Vec4<scalar,void>>, "Mat6x8<"#scalar"> is not aligned correctly");
	PR_MAT6X8_CHECKS(float);
	PR_MAT6X8_CHECKS(double);
	PR_MAT6X8_CHECKS(int32_t);
	PR_MAT6X8_CHECKS(int64_t);
	#undef PR_MAT6X8_CHECKS

	// Compare for floating point equality
	template <Scalar S, typename A, typename B> inline bool FEql(Mat6x8<S,A,B> const& lhs, Mat6x8<S,A,B> const& rhs)
	{
		return
			FEql(lhs.m00, rhs.m00) &&
			FEql(lhs.m01, rhs.m01) &&
			FEql(lhs.m10, rhs.m10) &&
			FEql(lhs.m11, rhs.m11);
	}

	// Return the transpose of a spatial matrix
	template <Scalar S, typename A, typename B> inline Mat6x8<S, A, B> Transpose(Mat6x8<S,A,B> const& m)
	{
		return Mat6x8<S, A, B>(
			Transpose(m.m00), Transpose(m.m10),
			Transpose(m.m01), Transpose(m.m11));
	}

	// Invert the 6x6 matrix 'm'
	template <Scalar S, typename A, typename B> inline Mat6x8<S,B,A> Invert(Mat6x8<S,A,B> const& m)
	{
		// 2x2 block matrix inversion
		// R = [A B]  R' = [E F]
		//     [C D]       [G H]
		// For square diagonal partitions of 'R' (i.e. submatrices are square)
		// If 'A' is non-singular then 'R' is invertible iff the Schur complement "D - CA¯B" of A is invertible
		// R'= [A¯ + A¯B(D-CA¯B)¯CA¯ ,  -A¯B(D-CA¯B)¯ ]
		//     [    -(D-CA¯B)¯CA¯    ,    (D-CA¯B)¯   ]
		// or:
		//     [   (A-BD¯C)¯     ,    -(A-BD¯C)¯BD¯   ]
		//     [ -D¯C(A-BD¯C)¯   , D¯+D¯C(A-BD¯C)¯BD¯ ]

		auto& a = m.m00;
		auto& b = m.m01;
		auto& c = m.m10;
		auto& d = m.m11;
		if (IsInvertible(a))
		{
			auto a_inv = Invert(a);
			auto schur = d - c * a_inv * b; // The 'Schur Complement'
			if (IsInvertible(schur))
			{
				auto schur_inv = Invert(schur);
				return Mat6x8f<B,A>{
					a_inv + a_inv * b * schur_inv * c * a_inv , -a_inv * b * schur_inv,
					                   -schur_inv * c * a_inv ,              schur_inv};
			}
		}
		if (IsInvertible(d))
		{
			auto d_inv = Invert(d);
			auto schur = a - b * d_inv * c; // The 'Schur Complement'
			if (IsInvertible(schur))
			{
				auto schur_inv = Invert(schur);
				return Mat6x8f<B,A>{
					             schur_inv ,                    -schur_inv * b * d_inv,
					-d_inv * c * schur_inv , d_inv + d_inv * c * schur_inv * b * d_inv};
			}
		}
		throw std::runtime_error("matrix is singular");
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/maths/matrix.h"
namespace pr::maths
{
	PRUnitTest(Matrix6x8Tests, float)
	{
		using S = T;
		using mat6_t = Mat6x8<S, void, void>;
		using mat3_t = Mat3x4<S, void, void>;
		using vec8_t = Vec8<S, void>;
		using vec4_t = Vec4<S, void>;
		using vec3_t = Vec3<S, void>;

		{// Memory order tests
			auto m1 = Matrix<S>{6, 8,  // = [{1} {2} {3} {4} {5} {6}]
				{
					1, 1, 1, 1, 1, 1, 1, 1,
					2, 2, 2, 2, 2, 2, 2, 2,
					3, 3, 3, 3, 3, 3, 3, 3,
					4, 4, 4, 4, 4, 4, 4, 4,
					5, 5, 5, 5, 5, 5, 5, 5,
					6, 6, 6, 6, 6, 6, 6, 6,
			}};
			auto m2 = Matrix<S>{6, 8,
				{
					1, 1, 1, 1, 2, 2, 2, 2, // = [[1] [3]]
					1, 1, 1, 1, 2, 2, 2, 2, //   [[2] [4]]
					1, 1, 1, 1, 2, 2, 2, 2,
					3, 3, 3, 3, 4, 4, 4, 4,
					3, 3, 3, 3, 4, 4, 4, 4,
					3, 3, 3, 3, 4, 4, 4, 4,
			}};
			auto M1 = mat6_t // = [{1} {2} {3} {4} {5} {6}]
			{
				mat3_t{vec4_t(1), vec4_t(2), vec4_t(3)}, mat3_t{vec4_t(4), vec4_t(5), vec4_t(6)},
				mat3_t{vec4_t(1), vec4_t(2), vec4_t(3)}, mat3_t{vec4_t(4), vec4_t(5), vec4_t(6)}
			};
			auto M2 = mat6_t
			{
				mat3_t(1), mat3_t(3), // = [[1] [3]]
				mat3_t(2), mat3_t(4)  //   [[2] [4]]
			};

			for (int r = 0; r != 6; ++r)
				for (int c = 0; c != 8; ++c)
				{
					PR_EXPECT(FEql(m1(r, c), M1[r][c]));
					PR_EXPECT(FEql(m2(r, c), M2[r][c]));
				}

			auto M3 = mat6_t // = [{1} {2} {3} {4} {5} {6}]
			{
				vec8_t{1, 1, 1, 1, 1, 1, 1, 1},
				vec8_t{2, 2, 2, 2, 2, 2, 2, 2},
				vec8_t{3, 3, 3, 3, 3, 3, 3, 3},
				vec8_t{4, 4, 4, 4, 4, 4, 4, 4},
				vec8_t{5, 5, 5, 5, 5, 5, 5, 5},
				vec8_t{6, 6, 6, 6, 6, 6, 6, 6},
			};
			PR_EXPECT(FEql(M3, M1));
		}
		{// Array access
			auto m1 = mat6_t
			{
				mat3_t{vec4_t(1), vec4_t(2), vec4_t(3)}, mat3_t{vec4_t(4), vec4_t(5), vec4_t(6)}, // = [{1} {2} {3} {4} {5} {6}]
				mat3_t{vec4_t(1), vec4_t(2), vec4_t(3)}, mat3_t{vec4_t(4), vec4_t(5), vec4_t(6)}
			};
			PR_EXPECT(FEql(m1[0], vec8_t{1, 1, 1, 1, 1, 1, 1, 1}));
			PR_EXPECT(FEql(m1[1], vec8_t{2, 2, 2, 2, 2, 2, 2, 2}));
			PR_EXPECT(FEql(m1[2], vec8_t{3, 3, 3, 3, 3, 3, 3, 3}));
			PR_EXPECT(FEql(m1[3], vec8_t{4, 4, 4, 4, 4, 4, 4, 4}));
			PR_EXPECT(FEql(m1[4], vec8_t{5, 5, 5, 5, 5, 5, 5, 5}));
			PR_EXPECT(FEql(m1[5], vec8_t{6, 6, 6, 6, 6, 6, 6, 6}));

			auto tmp = m1.col(0);
			m1.col(0, m1[5]);
			m1.col(5, tmp);
			PR_EXPECT(FEql(m1[0], vec8_t{6, 6, 6, 6, 6, 6, 6, 6}));
			PR_EXPECT(FEql(m1[5], vec8_t{1, 1, 1, 1, 1, 1, 1, 1}));
		}
		{// Multiply vector
			auto m = Matrix<float>{6, 6,
				{
					1, 1, 1, 1, 1, 1, // = [{1} {2} {3} {4} {5} {6}]
					2, 2, 2, 2, 2, 2,
					3, 3, 3, 3, 3, 3,
					4, 4, 4, 4, 4, 4,
					5, 5, 5, 5, 5, 5,
					6, 6, 6, 6, 6, 6,
			}};
			auto v = Matrix<float>{1, 6,
			{
				1, 2, 3, 4, 5, 6,
			}};
			auto e = Matrix<float>{1, 6,
			{
				91, 91, 91, 91, 91, 91,
			}};
			auto r = m * v;
			PR_EXPECT(FEql(r, e));

			auto M = mat6_t
			{
				mat3_t{vec3_t(1), vec3_t(2), vec3_t(3)}, mat3_t{vec3_t(4), vec3_t(5), vec3_t(6)},
				mat3_t{vec3_t(1), vec3_t(2), vec3_t(3)}, mat3_t{vec3_t(4), vec3_t(5), vec3_t(6)}, // = [{1} {2} {3} {4} {5} {6}]
			};
			auto V = vec8_t
			{
				vec4_t{1, 2, 3, 0},
				vec4_t{4, 5, 6, 0},
			};
			auto E = vec8_t
			{
				vec4_t{91, 91, 91, 0},
				vec4_t{91, 91, 91, 0},
			};
			auto R = M * V;
			PR_EXPECT(FEql(R, E));
		}
		{// Multiply matrix
			auto m1 = Matrix<float>{6, 6,
			{
				1, 1, 1, 1, 1, 1, // = [{1} {2} {3} {4} {5} {6}]
				2, 2, 2, 2, 2, 2,
				3, 3, 3, 3, 3, 3,
				4, 4, 4, 4, 4, 4,
				5, 5, 5, 5, 5, 5,
				6, 6, 6, 6, 6, 6,
			}};
			auto m2 = Matrix<float>{6, 6,
			{
				1, 1, 1, 2, 2, 2, // = [[1] [3]]
				1, 1, 1, 2, 2, 2, //   [[2] [4]]
				1, 1, 1, 2, 2, 2,
				3, 3, 3, 4, 4, 4,
				3, 3, 3, 4, 4, 4,
				3, 3, 3, 4, 4, 4,
			}};
			auto m3 = Matrix<float>{6, 6,
			{
				36, 36, 36, 36, 36, 36,
				36, 36, 36, 36, 36, 36,
				36, 36, 36, 36, 36, 36,
				78, 78, 78, 78, 78, 78,
				78, 78, 78, 78, 78, 78,
				78, 78, 78, 78, 78, 78,
			}};
			auto m4 = m1 * m2;
			PR_EXPECT(FEql(m3, m4));

			auto M1 = mat6_t
			{
				mat3_t{vec3_t(1), vec3_t(2), vec3_t(3)}, mat3_t{vec3_t(4), vec3_t(5), vec3_t(6)},
				mat3_t{vec3_t(1), vec3_t(2), vec3_t(3)}, mat3_t{vec3_t(4), vec3_t(5), vec3_t(6)}, // = [{1} {2} {3} {4} {5} {6}]
			};
			auto M2 = mat6_t
			{
				mat3_t{vec3_t(1), vec3_t(1), vec3_t(1)}, mat3_t{vec3_t(3), vec3_t(3), vec3_t(3)}, // = [[1] [3]]
				mat3_t{vec3_t(2), vec3_t(2), vec3_t(2)}, mat3_t{vec3_t(4), vec3_t(4), vec3_t(4)}  //   [[2] [4]]
			};
			auto M3 = mat6_t
			{
				mat3_t{vec3_t(36), vec3_t(36), vec3_t(36)}, mat3_t{vec3_t(78), vec3_t(78), vec3_t(78)},
				mat3_t{vec3_t(36), vec3_t(36), vec3_t(36)}, mat3_t{vec3_t(78), vec3_t(78), vec3_t(78)}
			};
			auto M4 = M1 * M2;
			PR_EXPECT(FEql(M3, M4));
		}
		{// Transpose
			auto m1 = Matrix<float>{6, 6,
			{
				1, 1, 1, 1, 1, 1, // = [{1} {2} {3} {4} {5} {6}]
				2, 2, 2, 2, 2, 2,
				3, 3, 3, 3, 3, 3,
				4, 4, 4, 4, 4, 4,
				5, 5, 5, 5, 5, 5,
				6, 6, 6, 6, 6, 6,
			}};
			auto M1 = mat6_t
			{
				mat3_t{vec3_t(1), vec3_t(2), vec3_t(3)}, mat3_t{vec3_t(4), vec3_t(5), vec3_t(6)}, // = [{1} {2} {3} {4} {5} {6}]
				mat3_t{vec3_t(1), vec3_t(2), vec3_t(3)}, mat3_t{vec3_t(4), vec3_t(5), vec3_t(6)}
			};
			auto m2 = Transpose(m1);
			auto M2 = Transpose(M1);

			for (int i = 0; i != 6; ++i)
			{
				PR_EXPECT(FEql(M2[i], vec8_t{vec3_t{1, 2, 3}, vec3_t{4, 5, 6}}));

				PR_EXPECT(FEql(m2(i, 0), M2[i].ang.x));
				PR_EXPECT(FEql(m2(i, 1), M2[i].ang.y));
				PR_EXPECT(FEql(m2(i, 2), M2[i].ang.z));
				PR_EXPECT(FEql(m2(i, 3), M2[i].lin.x));
				PR_EXPECT(FEql(m2(i, 4), M2[i].lin.y));
				PR_EXPECT(FEql(m2(i, 5), M2[i].lin.z));
			}
		}
		{// Inverse
			auto M = mat6_t
			{
				vec8_t{+1, +1, +2, -1, +6, +2},
				vec8_t{-2, +2, +4, -3, +5, -4},
				vec8_t{+1, +3, -2, -5, +4, +6},
				vec8_t{+1, +4, +3, -7, +3, -5},
				vec8_t{+1, +2, +3, -2, +2, +3},
				vec8_t{+1, -1, -2, -3, +6, -1}
			};
			auto M_ = mat6_t
			{
				vec8_t{+227.0f / 794.0f, -135.0f / 397.0f, -101.0f / 794.0f, +84.0f / 397.0f, -16.0f / 397.0f, -4.0f / 397.0f},
				vec8_t{+219.0f / 397.0f, -75.0f / 794.0f, +382.0f / 1985.f, +179.0f / 794.0f, -2647.f / 3970.f, -976.0f / 1985.f},
				vec8_t{-129.0f / 794.0f, +26.0f / 397.0f, -107.0f / 794.0f, -25.0f / 397.0f, +156.0f / 397.0f, +39.0f / 397.0f},
				vec8_t{+367.0f / 794.0f, -71.0f / 794.0f, +51.0f / 3970.f, +53.0f / 794.0f, -1733.f / 3970.f, -564.0f / 1985.f},
				vec8_t{+159.0f / 794.0f, +19.0f / 794.0f, +87.0f / 3970.f, -3.0f / 794.0f, -621.0f / 3970.f, -28.0f / 1985.f},
				vec8_t{-50.0f / 397.0f, +14.0f / 397.0f, +17.0f / 397.0f, -44.0f / 397.0f, +84.0f / 397.0f, +21.0f / 397.0f}
			};

			auto m_ = Invert(M);
			PR_EXPECT(FEql(m_, M_));

			auto I = M * M_;
			PR_EXPECT(FEql(I, mat6_t::Identity()));

			auto i = M * m_;
			PR_EXPECT(FEql(i, mat6_t::Identity()));
		}
	}
}
#endif
