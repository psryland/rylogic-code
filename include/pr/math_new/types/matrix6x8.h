//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math_new/core/forward.h"
#include "pr/math_new/core/traits.h"
#include "pr/math_new/types/vector4.h"
#include "pr/math_new/types/vector8.h"
#include "pr/math_new/types/matrix3x4.h"
#include "pr/math_new/types/matrix4x4.h"

namespace pr::math
{
	// General 6x8 matrix
	template <ScalarType S, typename A, typename B>
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
		Mat3x4<S> m00, m10, m01, m11;

		// Construct from sub matrices. WARNING: careful with layout.
		Mat6x8() = default;
		constexpr explicit Mat6x8(S x_) noexcept
			:m00(x_)
			, m10(x_)
			, m01(x_)
			, m11(x_)
		{
		}
		constexpr Mat6x8(Mat3x4<S> const& m00_, Mat3x4<S> const& m01_, Mat3x4<S> const& m10_, Mat3x4<S> const& m11_) noexcept
			:m00(m00_)
			, m10(m10_)
			, m01(m01_)
			, m11(m11_)
		{
		}
		constexpr Mat6x8(Vec8<S, void> x, Vec8<S, void> y, Vec8<S, void> z, Vec8<S, void> u, Vec8<S, void> v, Vec8<S, void> w) noexcept
			:m00(x.ang, y.ang, z.ang)
			, m10(x.lin, y.lin, z.lin)
			, m01(u.ang, v.ang, w.ang)
			, m11(u.lin, v.lin, w.lin)
		{
		}

		// Reinterpret as a different matrix type
		template <typename C, typename D> explicit operator Mat6x8<S, C, D> const& () const noexcept
		{
			return reinterpret_cast<Mat6x8<S, C, D> const&>(*this);
		}
		template <typename C, typename D> explicit operator Mat6x8<S, C, D>& () noexcept
		{
			return reinterpret_cast<Mat6x8<S, C, D>&>(*this);
		}
		operator Mat6x8<S, void, void> const& () const noexcept
		{
			return reinterpret_cast<Mat6x8<S, void, void> const&>(*this);
		}
		operator Mat6x8<S, void, void>& () noexcept
		{
			return reinterpret_cast<Mat6x8<S, void, void>&>(*this);
		}

		// Array of column vectors
		Vec8<S, void> operator [](int i) const noexcept
		{
			// Note: Creating a Vec8Proxy doesn't work because by default the compiler selects the
			// mutable overload for non-const instances, so swap-style assignments don't work.
			pr_assert("index out of range" && i >= 0 && i < 6);
			return i < 3
				? Vec8<S, void>{m00[i], m10[i]}
			: Vec8<S, void>{ m01[i - 3], m11[i - 3] };
		}
		Vec8<S, void> col(int i) const noexcept
		{
			pr_assert("index out of range" && i >= 0 && i < 6);
			return (*this)[i];
		}
		void col(int i, Vec8<S, void> rhs) noexcept
		{
			pr_assert("index out of range" && i >= 0 && i < 6);
			if (i < 3) { m00[i] = rhs.ang; m10[i] = rhs.lin; }
			else { m01[i - 3] = rhs.ang; m11[i - 3] = rhs.lin; }
		}

		// Basic constants
		static constexpr Mat6x8 const& Zero() noexcept
		{
			static auto s_zero = Mat6x8{
				math::Zero<Mat3x4<S>>(), math::Zero<Mat3x4<S>>(),
				math::Zero<Mat3x4<S>>(), math::Zero<Mat3x4<S>>()
			};
			return s_zero;
		}
		static constexpr Mat6x8 const& Identity() noexcept
		{
			static auto s_identity = Mat6x8{
				math::Identity<Mat3x4<S>>(), math::Zero<Mat3x4<S>>(),
				math::Zero<Mat3x4<S>>(), math::Identity<Mat3x4<S>>()
			};
			return s_identity;
		}

		#pragma region Operators
		friend constexpr Mat6x8 operator + (Mat6x8<S, A, B> const& m) noexcept
		{
			return m;
		}
		friend constexpr Mat6x8 operator - (Mat6x8<S, A, B> const& m) noexcept
		{
			return Mat6x8{ -m.m00, -m.m01, -m.m10, -m.m11 };
		}
		friend Mat6x8 operator + (Mat6x8<S, A, B> const& lhs, Mat6x8<S, A, B> const& rhs) noexcept
		{
			return Mat6x8{ lhs.m00 + rhs.m00, lhs.m01 + rhs.m01, lhs.m10 + rhs.m10, lhs.m11 + rhs.m11 };
		}
		friend Mat6x8 operator - (Mat6x8<S, A, B> const& lhs, Mat6x8<S, A, B> const& rhs) noexcept
		{
			return Mat6x8{ lhs.m00 - rhs.m00, lhs.m01 - rhs.m01, lhs.m10 - rhs.m10, lhs.m11 - rhs.m11 };
		}
		friend Mat6x8 operator * (Mat6x8<S, A, B> const& lhs, S rhs) noexcept
		{
			return Mat6x8{ lhs.m00 * rhs, lhs.m01 * rhs, lhs.m10 * rhs, lhs.m11 * rhs };
		}
		friend Mat6x8 operator * (S lhs, Mat6x8<S, A, B> const& rhs) noexcept
		{
			return rhs * lhs;
		}
		friend Vec8<S, B> pr_vectorcall operator * (Mat6x8<S, A, B> const& lhs, Vec8<S, A> rhs) noexcept
		{
			// [m00*a + m01*b] = [m00, m01] [a]
			// [m10*a + m11*b]   [m10, m11] [b]
			return Vec8<S, B>{
				lhs.m00* rhs.ang + lhs.m01 * rhs.lin,
					lhs.m10* rhs.ang + lhs.m11 * rhs.lin};
		}
		template <typename C> friend Mat6x8<S, A, C> pr_vectorcall operator * (Mat6x8<S, B, C> const& lhs, Mat6x8<S, A, B> const& rhs) noexcept
		{
			// [a00, a01] [b00, b01] = [a00*b00 + a01*b10, a00*b01 + a01*b11]
			// [a10, a11] [b10, b11]   [a10*b00 + a11*b10, a10*b01 + a11*b11]
			return Mat6x8<S, A, C>{
				lhs.m00* rhs.m00 + lhs.m01 * rhs.m10, lhs.m00* rhs.m01 + lhs.m01 * rhs.m11,
					lhs.m10* rhs.m00 + lhs.m11 * rhs.m10, lhs.m10* rhs.m01 + lhs.m11 * rhs.m11};
		}
		#pragma endregion

		// Compare for floating point equality
		friend constexpr bool FEql(Mat6x8 const& lhs, Mat6x8 const& rhs) noexcept
		{
			return
				FEql(lhs.m00, rhs.m00) &&
				FEql(lhs.m01, rhs.m01) &&
				FEql(lhs.m10, rhs.m10) &&
				FEql(lhs.m11, rhs.m11);
		}

		// Return the transpose of a spatial matrix
		friend constexpr Mat6x8 Transpose(Mat6x8 const& mat) noexcept
		{
			return Mat6x8(
				Transpose(mat.m00), Transpose(mat.m10),
				Transpose(mat.m01), Transpose(mat.m11)
			);
		}

		// Invert the 6x6 matrix 'm'
		friend Mat6x8<S, B, A> Invert(Mat6x8<S, A, B> const& m)
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
					return Mat6x8<S, B, A>{
						a_inv + a_inv * b * schur_inv * c * a_inv,
						-a_inv * b * schur_inv,
						-schur_inv * c * a_inv,
						schur_inv
					};
				}
			}
			if (IsInvertible(d))
			{
				auto d_inv = Invert(d);
				auto schur = a - b * d_inv * c; // The 'Schur Complement'
				if (IsInvertible(schur))
				{
					auto schur_inv = Invert(schur);
					return Mat6x8<S, B, A>{
						schur_inv,
						-schur_inv * b * d_inv,
						-d_inv * c * schur_inv,
						d_inv + d_inv * c * schur_inv * b * d_inv
					};
				}
			}
			throw std::runtime_error("matrix is singular");
		}
	};

	// Note: Vec8 isn't really a VectorType. It can't be used with the generic vector functions. We can do checks though.
	#define PR_MATH_DEFINE_TYPE(element)\
	static_assert(sizeof(Mat6x8<element,void,void>) == 6*8*sizeof(element), "Mat6x8<"#element"> has the wrong size");\
	static_assert(std::is_trivially_copyable_v<Mat6x8<element,void,void>>, "Mat6x8<"#element"> must be a pod type");\
	static_assert(std::alignment_of_v<Mat6x8<element, void, void>> == std::alignment_of_v<Vec4<element>>, "Mat6x8<"#element"> is not aligned correctly");
	PR_MATH_DEFINE_TYPE(float);
	PR_MATH_DEFINE_TYPE(double);
	PR_MATH_DEFINE_TYPE(int32_t);
	PR_MATH_DEFINE_TYPE(int64_t);
	#undef PR_MATH_DEFINE_TYPE
}
