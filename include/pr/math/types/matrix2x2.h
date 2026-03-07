//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/core/traits.h"
#include "pr/math/core/constants.h"
#include "pr/math/core/functions.h"
#include "pr/math/types/vector2.h"

namespace pr::math
{
	template <ScalarType S>
	struct Mat2x2
	{
		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union
		{
			struct { Vec2<S> x, y; };
			struct { Vec2<S> arr[2]; };
		};
		#pragma warning(pop)

		// Construct
		Mat2x2() = default;
		constexpr explicit Mat2x2(S x_) noexcept
			:x(x_)
			,y(x_)
		{}
		constexpr Mat2x2(S xx, S xy, S yx, S yy) noexcept
			:x(xx, xy)
			,y(yx, yy)
		{}
		constexpr Mat2x2(Vec2<S> x_, Vec2<S> y_) noexcept
			:x(x_)
			,y(y_)
		{}
		constexpr Mat2x2(std::ranges::random_access_range auto&& v) noexcept
			:Mat2x2(
				Vec2<S>(v[0], v[1]),
				Vec2<S>(v[2], v[3]))
		{}

		// Explicit cast to different Scalar type
		template <ScalarType S2> constexpr explicit operator Mat2x2<S2>() const noexcept
		{
			return Mat2x2<S2>(
				static_cast<Vec2<S2>>(x),
				static_cast<Vec2<S2>>(y)
			);
		}

		// Array access
		constexpr Vec2<S> const& operator [](int i) const noexcept
		{
			pr_assert(i >= 0 && i < 2 && "index out of range");
			if consteval { return i == 0 ? x : y; }
			else { return arr[i]; }
		}
		constexpr Vec2<S>& operator [](int i) noexcept
		{
			pr_assert(i >= 0 && i < 2 && "index out of range");
			if consteval { return i == 0 ? x : y; }
			else { return arr[i]; }
		}

		// Constants
		static constexpr Mat2x2 const& Zero() noexcept
		{
			static auto s_zero = math::Zero<Mat2x2>();
			return s_zero;
		}
		static constexpr Mat2x2 const& Identity() noexcept
		{
			static auto s_identity = math::Identity<Mat2x2>();
			return s_identity;
		}

		// Create a rotation matrix
		static Mat2x2 Rotation(S angle) noexcept
		{
			return math::Rotation<Mat2x2>(angle);
		}

		// Create a scale matrix
		static constexpr Mat2x2 Scale(S scale) noexcept
		{
			return math::Scale<Mat2x2>(scale);
		}
		static constexpr Mat2x2 Scale(S sx, S sy) noexcept
		{
			return math::Scale<Mat2x2>(Vec2<S>(sx, sy));
		}

		// Create a shear matrix
		static Mat2x2 Shear(S sxy, S syx) noexcept
		{
			return math::Shear<Mat2x2>(sxy, syx);
		}
	};

	#define PR_MATH_DEFINE_TYPE(component, element)\
	template <> struct vector_traits<Mat2x2<element>>\
		: vector_traits_base<element, component, 2>\
		, vector_access_member<Mat2x2<element>, component, 2>\
	{};\
	\
	static_assert(VectorType<Mat2x2<element>>, "Mat2x2<"#element"> is not a valid vector type");\
	static_assert(IsRank2<Mat2x2<element>>, "Mat2x2<"#element"> is not rank 2");\
	static_assert(sizeof(Mat2x2<element>) == 2*2*sizeof(element), "Mat2x2<"#element"> has the wrong size");\
	static_assert(std::is_trivially_copyable_v<Mat2x2<element>>, "Mat2x2<"#element"> is not trivially copyable");

	PR_MATH_DEFINE_TYPE(Vec2<float>, float);
	PR_MATH_DEFINE_TYPE(Vec2<double>, double);
	PR_MATH_DEFINE_TYPE(Vec2<int32_t>, int32_t);
	PR_MATH_DEFINE_TYPE(Vec2<int64_t>, int64_t);
	#undef PR_MATH_DEFINE_TYPE
}


		//#pragma region Operators
		//friend constexpr Mat2x2 operator + (Mat2x2<S,A,B> const& mat)
		//{
		//	return mat;
		//}
		//friend constexpr Mat2x2 operator - (Mat2x2<S,A,B> const& mat)
		//{
		//	return Mat2x2{-mat.x, -mat.y};
		//}
		//friend Mat2x2 operator * (S lhs, Mat2x2<S,A,B> const& rhs)
		//{
		//	return rhs * lhs;
		//}
		//friend Mat2x2 operator * (Mat2x2<S,A,B> const& lhs, S rhs)
		//{
		//	return Mat2x2{lhs.x * rhs, lhs.y * rhs};
		//}
		//friend Mat2x2 operator / (Mat2x2<S,A,B> const& lhs, S rhs)
		//{
		//	// Don't check for divide by zero by default. For floats +inf/-inf are valid results
		//	//pr_assert("divide by zero" && rhs != 0);
		//	return Mat2x2{lhs.x / rhs, lhs.y / rhs};
		//}
		//friend Mat2x2 operator % (Mat2x2<S,A,B> const& lhs, S rhs)
		//{
		//	// Don't check for divide by zero by default. For floats +inf/-inf are valid results
		//	//pr_assert("divide by zero" && rhs != 0);
		//	return Mat2x2{lhs.x % rhs, lhs.y % rhs};
		//}
		//friend Mat2x2 operator + (Mat2x2<S,A,B> const& lhs, Mat2x2<S,A,B> const& rhs)
		//{
		//	return Mat2x2{lhs.x + rhs.x, lhs.y + rhs.y};
		//}
		//friend Mat2x2 operator - (Mat2x2<S,A,B> const& lhs, Mat2x2<S,A,B> const& rhs)
		//{
		//	return Mat2x2{lhs.x - rhs.x, lhs.y - rhs.y};
		//}
		//#pragma endregions
