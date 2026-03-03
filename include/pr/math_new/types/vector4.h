//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math_new/core/forward.h"
#include "pr/math_new/core/traits.h"
#include "pr/math_new/core/functions.h"
#include "pr/math_new/types/vector2.h"
#include "pr/math_new/types/vector3.h"

namespace pr::math
{
	template <ScalarType S>
	struct Vec4
	{
		inline static constexpr bool IntrinsicF = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, float>;
		inline static constexpr bool IntrinsicD = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, double>;
		inline static constexpr bool IntrinsicI = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, int32_t>;
		inline static constexpr bool IntrinsicL = PR_MATHS_USE_INTRINSICS && std::is_same_v<S, int64_t>;
		inline static constexpr bool NoIntrinsic = PR_MATHS_USE_INTRINSICS == 0;

		using intrinsic_t =
			std::conditional_t<IntrinsicF, __m128,
			std::conditional_t<IntrinsicD, __m256d,
			std::conditional_t<IntrinsicI, __m128i,
			std::conditional_t<IntrinsicL, __m256i,
			std::byte[4*sizeof(S)]
			>>>>;

		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union alignas(4 * sizeof(S))
		{
			struct { S x, y, z, w; };
			struct { Vec2<S> xy, zw; };
			struct { Vec3<S> xyz; };
			struct { S arr[4]; };
			intrinsic_t vec;
		};
		#pragma warning(pop)

		// Construct
		Vec4() = default;
		constexpr explicit Vec4(S x_) noexcept
			: x(x_)
			, y(x_)
			, z(x_)
			, w(x_)
		{
		}
		constexpr Vec4(S x_, S y_, S z_, S w_) noexcept
			: x(x_)
			, y(y_)
			, z(z_)
			, w(w_)
		{}
		constexpr Vec4(S x_, S y_, S z_) noexcept
			: x(x_)
			, y(y_)
			, z(z_)
			, w(S(0))
		{}
		constexpr explicit Vec4(VectorTypeN<S, 4> auto v) noexcept
			:Vec4(vec(v).x, vec(v).y, vec(v).z, vec(v).w)
		{}
		constexpr Vec4(Vec3<S> v, S w_) noexcept
			:Vec4(v.x, v.y, v.z, w_)
		{}
		constexpr Vec4(Vec2<S> v, S z_, S w_) noexcept
			:Vec4(v.x, v.y, z_, w_)
		{}
		constexpr Vec4(Vec2<S> xy_, Vec2<S> zw_) noexcept
			:Vec4(xy_.x, xy_.y, zw_.x, zw_.y)
		{}
		constexpr Vec4(AxisId axis_id) noexcept
			:Vec4(
				Abs(axis_id) == AxisId::PosX ? static_cast<S>(Sign<int>(axis_id)) : S(0),
				Abs(axis_id) == AxisId::PosY ? static_cast<S>(Sign<int>(axis_id)) : S(0),
				Abs(axis_id) == AxisId::PosZ ? static_cast<S>(Sign<int>(axis_id)) : S(0),
				S(0)
			)
		{}
		constexpr Vec4(intrinsic_t vec_) noexcept requires (!NoIntrinsic)
			:vec(vec_)
		{}
		constexpr explicit Vec4(std::ranges::random_access_range auto&& v) noexcept
			:Vec4(v[0], v[1], v[2], v[3])
		{}

		// Explicit cast to different Scalar type
		template <ScalarType S2> constexpr explicit operator Vec4<S2>() const noexcept
		{
			return Vec4<S2>(
				static_cast<S2>(x),
				static_cast<S2>(y),
				static_cast<S2>(z),
				static_cast<S2>(w)
			);
		}

		// Array access
		constexpr S operator [] (int i) const noexcept
		{
			pr_assert(i >= 0 && i < 4 && "index out of range");
			if consteval { return i == 0 ? x : i == 1 ? y : i == 2 ? z : w; }
			else { return arr[i]; }
		}
		constexpr S& operator [] (int i) noexcept
		{
			pr_assert(i >= 0 && i < 4 && "index out of range");
			if consteval { return i == 0 ? x : i == 1 ? y : i == 2 ? z : w; }
			else { return arr[i]; }
		}

		// Create other vector types
		constexpr Vec4 w0() const noexcept
		{
			Vec4 r(x, y, z, S(0)); // LValue because of alignment
			return r;
		}
		constexpr Vec4 w1() const noexcept
		{
			Vec4 r(x, y, z, S(1)); // LValue because of alignment
			return r;
		}
		constexpr Vec2<S> vec2(int i0, int i1) const noexcept
		{
			return Vec2<S>{arr[i0], arr[i1]};
		}
		constexpr Vec3<S> vec3(int i0, int i1, int i2) const noexcept
		{
			return Vec3<S>{arr[i0], arr[i1], arr[i2]};
		}

		// Constants
		static consteval Vec4 const& Zero() noexcept
		{
			static auto s_zero = math::Zero<Vec4>();
			return s_zero;
		}
		static constexpr Vec4 const& One() noexcept
		{
			static auto s_one = math::One<Vec4>();
			return s_one;
		}
		static constexpr Vec4 const& Tiny() noexcept
		{
			static auto s_tiny = math::Tiny<Vec4>();
			return s_tiny;
		}
		static constexpr Vec4 const& Min() noexcept
		{
			static auto s_min = math::Min<Vec4>();
			return s_min;
		}
		static constexpr Vec4 const& Max() noexcept
		{
			static auto s_max = math::Max<Vec4>();
			return s_max;
		}
		static constexpr Vec4 const& Lowest() noexcept
		{
			static auto s_lowest = math::Lowest<Vec4>();
			return s_lowest;
		}
		static constexpr Vec4 const& Epsilon() noexcept
		{
			static auto s_epsilon = math::Epsilon<Vec4>();
			return s_epsilon;
		}
		static constexpr Vec4 const& Infinity() noexcept
		{
			static auto s_infinity = math::Infinity<Vec4>();
			return s_infinity;
		}
		static constexpr Vec4 const& XAxis() noexcept
		{
			static auto s_x_axis = math::XAxis<Vec4>();
			return s_x_axis;
		}
		static constexpr Vec4 const& YAxis() noexcept
		{
			static auto s_y_axis = math::YAxis<Vec4>();
			return s_y_axis;
		}
		static constexpr Vec4 const& ZAxis() noexcept
		{
			static auto s_z_axis = math::ZAxis<Vec4>();
			return s_z_axis;
		}
		static constexpr Vec4 const& WAxis() noexcept
		{
			static auto s_w_axis = math::WAxis<Vec4>();
			return s_w_axis;
		}
		static constexpr Vec4 const& Origin() noexcept
		{
			static auto s_origin = math::Origin<Vec4>();
			return s_origin;
		}

		// Construct normalised
		static Vec4 Normal(S x, S y, S z, S w) requires std::floating_point<S>
		{
			return Normalise(Vec4(x, y, z, w));
		}

		// Optimised operators
		#pragma region Operators
		friend constexpr Vec4 pr_vectorcall operator * (S lhs, Vec4 rhs) noexcept
		{
			return rhs * lhs;
		}
		friend constexpr Vec4 pr_vectorcall operator * (Vec4 lhs, S rhs) noexcept
		{
			if consteval
			{
				return math::operator*<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					return Vec4{ _mm_mul_ps(lhs.vec, _mm_set_ps1(rhs)) };
				}
				else if constexpr (IntrinsicD)
				{
					return Vec4{ _mm256_mul_pd(lhs.vec, _mm256_set1_pd(rhs)) };
				}
				else
				{
					return math::operator*<Vec4>(lhs, rhs);
				}
			}
		}
		friend constexpr Vec4 pr_vectorcall operator / (Vec4 lhs, S rhs) noexcept
		{
			if consteval
			{
				return math::operator/<Vec4>(lhs, rhs);
			}
			else
			{
				// Don't check for divide by zero by default. For floats +inf/-inf are valid results
				if constexpr (IntrinsicF)
				{
					return Vec4{ _mm_div_ps(lhs.vec, _mm_set_ps1(rhs)) };
				}
				else if constexpr (IntrinsicD)
				{
					return Vec4{ _mm256_div_pd(lhs.vec, _mm256_set1_pd(rhs)) };
				}
				else
				{
					return math::operator/<Vec4>(lhs, rhs);
				}
			}
		}
		friend constexpr Vec4 pr_vectorcall operator / (S lhs, Vec4 rhs) noexcept
		{
			if consteval
			{
				return math::operator/<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					return Vec4{ _mm_div_ps(_mm_set_ps1(lhs), rhs.vec) };
				}
				else if constexpr (IntrinsicD)
				{
					return Vec4{ _mm256_div_pd(_mm256_set1_pd(lhs), rhs.vec) };
				}
				else
				{
					return math::operator/<Vec4>(lhs, rhs);
				}
			}
		}
		friend constexpr Vec4 pr_vectorcall operator + (Vec4 lhs, Vec4 rhs) noexcept
		{
			if consteval
			{
				return math::operator+<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					return Vec4{ _mm_add_ps(lhs.vec, rhs.vec) };
				}
				else if constexpr (IntrinsicD)
				{
					return Vec4{ _mm256_add_pd(lhs.vec, rhs.vec) };
				}
				else
				{
					return math::operator+<Vec4>(lhs, rhs);
				}
			}
		}
		friend constexpr Vec4 pr_vectorcall operator - (Vec4 lhs, Vec4 rhs) noexcept
		{
			if consteval
			{
				return math::operator-<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					return Vec4{ _mm_sub_ps(lhs.vec, rhs.vec) };
				}
				else if constexpr (IntrinsicD)
				{
					return Vec4{ _mm256_sub_pd(lhs.vec, rhs.vec) };
				}
				else
				{
					return math::operator-<Vec4>(lhs, rhs);
				}
			}
		}
		friend constexpr Vec4 pr_vectorcall operator * (Vec4 lhs, Vec4 rhs) noexcept
		{
			if consteval
			{
				return math::operator*<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					return Vec4{ _mm_mul_ps(lhs.vec, rhs.vec) };
				}
				else if constexpr (IntrinsicD)
				{
					return Vec4{ _mm256_mul_pd(lhs.vec, rhs.vec) };
				}
				else
				{
					return math::operator*<Vec4>(lhs, rhs);
				}
			}
		}
		friend constexpr Vec4 pr_vectorcall operator / (Vec4 lhs, Vec4 rhs) noexcept
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			if consteval
			{
				return math::operator/<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					return Vec4{ _mm_div_ps(lhs.vec, rhs.vec) };
				}
				else if constexpr (IntrinsicD)
				{
					return Vec4{ _mm256_div_pd(lhs.vec, rhs.vec) };
				}
				else
				{
				return math::operator/<Vec4>(lhs, rhs);
				}
			}
		}
		friend constexpr Vec4 pr_vectorcall operator % (Vec4 lhs, Vec4 rhs) noexcept
		{
			// Don't check for divide by zero by default. For floats +inf/-inf are valid results
			if consteval
			{
				return math::operator%<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					auto div = _mm_div_ps(lhs.vec, rhs.vec);                                // a / b
					auto trunc = _mm_round_ps(div, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC); // trunc(a / b)
					auto prod = _mm_mul_ps(trunc, rhs.vec);                                 // trunc(a / b) * b
					auto rem = _mm_sub_ps(lhs.vec, prod);                                   // a - b * trunc(a / b) => fmod
					return Vec4{ rem };
				}
				else if constexpr (IntrinsicD)
				{
					auto div = _mm256_div_pd(lhs.vec, rhs.vec);                                // a / b
					auto trunc = _mm256_round_pd(div, _MM_FROUND_TO_ZERO | _MM_FROUND_NO_EXC); // trunc(a / b)
					auto prod = _mm256_mul_pd(trunc, rhs.vec);                                 // trunc(a / b) * b
					auto rem = _mm256_sub_pd(lhs.vec, prod);                                   // a - b * trunc(a / b) => fmod
					return Vec4{ rem };
				}
				else
				{
					return math::operator%<Vec4>(lhs, rhs);
				}
			}
		}
		friend constexpr bool pr_vectorcall operator == (Vec4 lhs, Vec4 rhs) noexcept
		{
			if consteval
			{
				return math::operator==<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					return _mm_movemask_ps(_mm_cmpeq_ps(lhs.vec, rhs.vec)) == 0xF;
				}
				else if constexpr (IntrinsicD)
				{
					return _mm256_movemask_pd(_mm256_cmp_pd(lhs.vec, rhs.vec, _CMP_EQ_OQ)) == 0xF;
				}
				else
				{
					return math::operator==<Vec4>(lhs, rhs);
				}
			}
		}
		#pragma endregion

		// --- SIMD-optimised free functions ---
		// These friend overloads are only active when intrinsics are available.
		// For non-intrinsic types, the generic versions in functions.h are used.

		// Dot product (4-component)
		friend constexpr S pr_vectorcall Dot(Vec4 lhs, Vec4 rhs) noexcept
		{
			if consteval
			{
				return math::Dot<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					return _mm_cvtss_f32(_mm_dp_ps(lhs.vec, rhs.vec, 0xF1));
				}
				else if constexpr (IntrinsicD)
				{
					auto mul = _mm256_mul_pd(lhs.vec, rhs.vec);
					auto hi = _mm256_extractf128_pd(mul, 1);
					auto lo = _mm256_castpd256_pd128(mul);
					auto sum = _mm_add_pd(lo, hi);
					auto shuf = _mm_unpackhi_pd(sum, sum);
					return _mm_cvtsd_f64(_mm_add_sd(sum, shuf));
				}
				else
				{
					return math::Dot<Vec4>(lhs, rhs);
				}
			}
		}

		// Dot product (3-component, w ignored)
		friend constexpr S pr_vectorcall Dot3(Vec4 lhs, Vec4 rhs) noexcept
		{
			if consteval
			{
				return math::Dot3<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					// 0x71 = multiply xyz (bits 6-4), store result in element 0 (bit 0)
					return _mm_cvtss_f32(_mm_dp_ps(lhs.vec, rhs.vec, 0x71));
				}
				else if constexpr (IntrinsicD)
				{
					auto a = _mm256_blend_pd(lhs.vec, _mm256_setzero_pd(), 0x8); // zero w of lhs
					auto mul = _mm256_mul_pd(a, rhs.vec);
					auto hi = _mm256_extractf128_pd(mul, 1);
					auto lo = _mm256_castpd256_pd128(mul);
					auto sum = _mm_add_pd(lo, hi);
					auto shuf = _mm_unpackhi_pd(sum, sum);
					return _mm_cvtsd_f64(_mm_add_sd(sum, shuf));
				}
				else
				{
					return math::Dot3<Vec4>(lhs, rhs);
				}
			}
		}

		// Cross product (3-component, w=0). Only for float — doubles use generic fallback.
		// Note: w is zeroed by cancellation (aw*bw - aw*bw), which produces NaN if w is NaN.
		friend constexpr Vec4 pr_vectorcall Cross(Vec4 lhs, Vec4 rhs) noexcept
		{
			if consteval
			{
				return math::Cross<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					auto a_yzx = _mm_shuffle_ps(lhs.vec, lhs.vec, _MM_SHUFFLE(3, 0, 2, 1));
					auto b_yzx = _mm_shuffle_ps(rhs.vec, rhs.vec, _MM_SHUFFLE(3, 0, 2, 1));
					auto c = _mm_sub_ps(
						_mm_mul_ps(lhs.vec, b_yzx),
						_mm_mul_ps(a_yzx, rhs.vec));
					return Vec4{ _mm_shuffle_ps(c, c, _MM_SHUFFLE(3, 0, 2, 1)) };
				}
				else if constexpr (IntrinsicD)
				{
					// No intrinsic support for cross product, so blend to zero w and use generic implementation
					auto a = _mm256_blend_pd(lhs.vec, _mm256_setzero_pd(), 0x8); // zero w of lhs
					auto b = _mm256_blend_pd(rhs.vec, _mm256_setzero_pd(), 0x8); // zero w of rhs
					Vec4 a_vec{a}, b_vec{b};
					return math::Cross<Vec4>(a_vec, b_vec);
				}
				else
				{
					return math::Cross<Vec4>(lhs, rhs);
				}
			}
		}

		// Length of a vector. Combined dot+sqrt avoids intermediate scalar extraction.
		friend constexpr S pr_vectorcall Length(Vec4 v) noexcept
		{
			if consteval
			{
				return math::Length<Vec4>(v);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					auto dp = _mm_dp_ps(v.vec, v.vec, 0xF1);
					return _mm_cvtss_f32(_mm_sqrt_ss(dp));
				}
				else if constexpr (IntrinsicD)
				{
					auto dp = _mm256_mul_pd(v.vec, v.vec);
					auto hi = _mm256_extractf128_pd(dp, 1);
					auto lo = _mm256_castpd256_pd128(dp);
					auto sum = _mm_add_pd(lo, hi);
					auto shuf = _mm_unpackhi_pd(sum, sum);
					auto length_sq = _mm_add_sd(sum, shuf);
					return _mm_cvtsd_f64(_mm_sqrt_sd(length_sq, length_sq));
				}
				else
				{
					return math::Length<Vec4>(v);
				}
			}
		}

		// Component-wise minimum
		friend constexpr Vec4 pr_vectorcall Min(Vec4 lhs, Vec4 rhs) noexcept
		{
			if consteval
			{
				return math::Min<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					return Vec4{_mm_min_ps(lhs.vec, rhs.vec)};
				}
				else if constexpr (IntrinsicD)
				{
					return Vec4{_mm256_min_pd(lhs.vec, rhs.vec)};
				}
				else
				{
					return math::Min<Vec4>(lhs, rhs);
				}
			}
		}

		// Component-wise maximum
		friend constexpr Vec4 pr_vectorcall Max(Vec4 lhs, Vec4 rhs) noexcept
		{
			if consteval
			{
				return math::Max<Vec4>(lhs, rhs);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					return Vec4{ _mm_max_ps(lhs.vec, rhs.vec) };
				}
				else if constexpr (IntrinsicD)
				{
					return Vec4{ _mm256_max_pd(lhs.vec, rhs.vec) };
				}
				else
				{
					return math::Max<Vec4>(lhs, rhs);
				}
			}
		}

		// Component-wise absolute value
		friend constexpr Vec4 pr_vectorcall Abs(Vec4 v) noexcept
		{
			if consteval
			{
				return math::Abs<Vec4>(v);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					auto sign_mask = _mm_castsi128_ps(_mm_set1_epi32(0x7FFFFFFF));
					return Vec4{_mm_and_ps(v.vec, sign_mask)};
				}
				else if constexpr (IntrinsicD)
				{
					auto sign_mask = _mm256_castsi256_pd(_mm256_set1_epi64x(0x7FFFFFFFFFFFFFFFLL));
					return Vec4{_mm256_and_pd(v.vec, sign_mask)};
				}
				else
				{
					return math::Abs<Vec4>(v);
				}
			}
		}

		// Component-wise clamp
		friend constexpr Vec4 pr_vectorcall Clamp(Vec4 v, Vec4 lo, Vec4 hi) noexcept
		{
			if consteval
			{
				return math::Clamp<Vec4>(v, lo, hi);
			}
			else
			{
				if constexpr (IntrinsicF)
				{
					return Vec4{ _mm_min_ps(_mm_max_ps(v.vec, lo.vec), hi.vec) };
				}
				else if constexpr (IntrinsicD)
				{
					return Vec4{ _mm256_min_pd(_mm256_max_pd(v.vec, lo.vec), hi.vec) };
				}
				else
				{
					return math::Clamp<Vec4>(v, lo, hi);
				}
			}
		}
	};

	#define PR_MATH_DEFINE_TYPE(element)\
	template <> struct vector_traits<Vec4<element>>\
		: vector_traits_base<element, element, 4>\
		, vector_access_member<Vec4<element>, element, 4>\
	{};\
	\
	static_assert(VectorType<Vec4<element>>, "Vec4<"#element"> is not a valid vector type");\
	static_assert(IsRank1<Vec4<element>>, "Vec4<"#element"> is not rank 1");\
	static_assert(sizeof(Vec4<element>) == 4*sizeof(element), "Vec4<"#element"> has the wrong size");\
	static_assert(std::is_trivially_copyable_v<Vec4<element>>, "Vec4<"#element"> is not trivially copyable");

	PR_MATH_DEFINE_TYPE(float);
	PR_MATH_DEFINE_TYPE(double);
	PR_MATH_DEFINE_TYPE(int32_t);
	PR_MATH_DEFINE_TYPE(int64_t);
	#undef PR_MATH_DEFINE_TYPE

	// Deferred definitions for Vec3::w0() and Vec3::w1() (Vec4 must be complete)
	template <ScalarType S>
	constexpr Vec4<S> Vec3<S>::w0() const noexcept
	{
		return Vec4<S>(x, y, z, S(0));
	}
	template <ScalarType S>
	constexpr Vec4<S> Vec3<S>::w1() const noexcept
	{
		return Vec4<S>(x, y, z, S(1));
	}
}
