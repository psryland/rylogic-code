//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/core/traits.h"
#include "pr/math/core/constants.h"
#include "pr/math/core/axis_id.h"
#include "pr/math/core/functions.h"

namespace pr::math
{
	template <ScalarType S>
	struct Vec2
	{
		// Notes:
		//  - Can't use __m64 because it has an alignment of 8.
		//    v2 is a member of the union in v3 which needs alignment of 4, or the size of v3 becomes 16.

		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union
		{
			struct { S x, y; };
			struct { S arr[2]; };
		};
		#pragma warning(pop)

		// Construct
		Vec2() = default;
		constexpr explicit Vec2(S x_) noexcept
			: x(x_)
			, y(x_)
		{}
		constexpr Vec2(S x_, S y_) noexcept
			: x(x_)
			, y(y_)
		{}
		// Build from a random-access range. Caller guarantees at least 2 readable elements;
		// for sized ranges the precondition is checked with pr_assert.
		template <std::ranges::random_access_range R>
			requires (!VectorType<std::remove_cvref_t<R>>)
			      && std::convertible_to<std::ranges::range_reference_t<R>, S>
		constexpr Vec2(R&& v) noexcept
		{
			if constexpr (std::ranges::sized_range<R>)
				pr_assert(std::ranges::size(v) >= 2 && "range must have at least 2 elements");
			auto it = std::ranges::begin(v);
			x = static_cast<S>(*it);
			y = static_cast<S>(*(it + 1));
		}

		// Build from a same-scalar external rank-1 vector of dimension 2.
		constexpr explicit Vec2(Rank1VecSN<S, 2> auto v) noexcept
			:Vec2(static_cast<S>(vec(v).x), static_cast<S>(vec(v).y))
		{}
		constexpr Vec2(AxisId axis_id) noexcept
			:Vec2(
				Abs(axis_id) == AxisId::PosX ? static_cast<S>(Sign<int>(axis_id)) : S(0),
				Abs(axis_id) == AxisId::PosY ? static_cast<S>(Sign<int>(axis_id)) : S(0)
			)
		{}

		// Explicit cast to different Scalar type
		template <ScalarType S2> constexpr explicit operator Vec2<S2>() const noexcept
		{
			return Vec2<S2>(
				static_cast<S2>(x),
				static_cast<S2>(y)
			);
		}

		// Array access
		constexpr S operator [] (int i) const noexcept
		{
			pr_assert(i >= 0 && i < 2 && "index out of range");
			if consteval { return i == 0 ? x : y; } else { return arr[i]; }
		}
		constexpr S& operator [] (int i) noexcept
		{
			pr_assert(i >= 0 && i < 2 && "index out of range");
			if consteval { return i == 0 ? x : y; } else { return arr[i]; }
		}

		// Constants
		static constexpr Vec2 const& Zero() noexcept
		{
			static auto s_zero = math::Zero<Vec2>();
			return s_zero;
		}
		static constexpr Vec2 const& One() noexcept
		{
			static auto s_one = math::One<Vec2>();
			return s_one;
		}
		static constexpr Vec2 const& Tiny() noexcept
		{
			static auto s_tiny = math::Tiny<Vec2>();
			return s_tiny;
		}
		static constexpr Vec2 const& Min() noexcept
		{
			static auto s_min = math::Min<Vec2>();
			return s_min;
		}
		static constexpr Vec2 const& Max() noexcept
		{
			static auto s_max = math::Max<Vec2>();
			return s_max;
		}
		static constexpr Vec2 const& Lowest() noexcept
		{
			static auto s_lowest = math::Lowest<Vec2>();
			return s_lowest;
		}
		static constexpr Vec2 const& Epsilon() noexcept
		{
			static auto s_epsilon = math::Epsilon<Vec2>();
			return s_epsilon;
		}
		static constexpr Vec2 const& Infinity() noexcept
		{
			static auto s_infinity = math::Infinity<Vec2>();
			return s_infinity;
		}
		static constexpr Vec2 const& XAxis() noexcept
		{
			static auto s_x_axis = math::XAxis<Vec2>();
			return s_x_axis;
		}
		static constexpr Vec2 const& YAxis() noexcept
		{
			static auto s_y_axis = math::YAxis<Vec2>();
			return s_y_axis;
		}
		static constexpr Vec2 const& Origin() noexcept
		{
			static auto s_origin = math::Origin<Vec2>();
			return s_origin;
		}

		// Construct normalised
		static constexpr Vec2 Normal(S x, S y) requires std::floating_point<S>
		{
			return Normalise(Vec2(x, y));
		}
	};

	#define PR_MATH_DEFINE_TYPE(element)\
	template <> struct vector_traits<Vec2<element>>\
		: vector_traits_base<element, element, 2>\
		, vector_access_member<Vec2<element>, element, 2>\
	{\
		template <ScalarType S> using rebind = Vec2<S>;\
	};\
	\
	static_assert(VectorType<Vec2<element>>, "Vec2<"#element"> is not a valid vector type");\
	static_assert(IsRank1<Vec2<element>>, "Vec2<"#element"> is not rank 1");\
	static_assert(sizeof(Vec2<element>) == 2*sizeof(element), "Vec2<"#element"> has the wrong size");\
	static_assert(std::is_trivially_copyable_v<Vec2<element>>, "Vec2<"#element"> is not trivially copyable");

	PR_MATH_DEFINE_TYPE(float);
	PR_MATH_DEFINE_TYPE(double);
	PR_MATH_DEFINE_TYPE(int32_t);
	PR_MATH_DEFINE_TYPE(int64_t);
	PR_MATH_DEFINE_TYPE(uint32_t);
	PR_MATH_DEFINE_TYPE(uint64_t);
	PR_MATH_DEFINE_TYPE(bool);
	#undef PR_MATH_DEFINE_TYPE
}
