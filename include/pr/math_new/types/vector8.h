//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math_new/core/forward.h"
#include "pr/math_new/core/traits.h"
#include "pr/math_new/core/functions.h"
#include "pr/math_new/types/vector3.h"
#include "pr/math_new/types/vector4.h"

namespace pr::math
{
	template <ScalarType S, typename T>
	struct Vec8
	{
		// Notes:
		//  - Spatial vectors describe a vector at a point plus the field of vectors around that point.
		//  - Spatial vectors live in two separate vector spaces. The T parameter protects against mixing spaces.
		using Vec3 = Vec3<S>;
		using Vec4 = Vec4<S>;

		#pragma warning(push)
		#pragma warning(disable:4201) // nameless struct
		union
		{
			struct { Vec4 ang, lin; };
			struct { S rx, ry, rz, rw, tx, ty, tz, tw; };
			struct { S arr[8]; };
		};
		#pragma warning(pop)

		Vec8() = default;
		constexpr explicit Vec8(S x) noexcept
			:ang(x)
			, lin(x)
		{
		}
		constexpr Vec8(Vec3 ang_, Vec3 lin_) noexcept
			: ang(ang_, S(0))
			, lin(lin_, S(0))
		{
		}
		constexpr Vec8(Vec4 ang_, Vec4 lin_) noexcept
			: ang(ang_)
			, lin(lin_)
		{
		}
		constexpr Vec8(S wx, S wy, S wz, S vx, S vy, S vz) noexcept
			: ang(wx, wy, wz, S(0))
			, lin(vx, vy, vz, S(0))
		{
		}
		constexpr Vec8(S wx, S wy, S wz, S ww, S vx, S vy, S vz, S vw) noexcept
			: ang(wx, wy, wz, ww)
			, lin(vx, vy, vz, vw)
		{
		}

		// Explicit cast to different Scalar type
		template <ScalarType S2> constexpr explicit operator Vec8<S2, T>() const noexcept
		{
			return Vec8<S2, T>(
				static_cast<math::Vec4<S2>>(ang),
				static_cast<math::Vec4<S2>>(lin)
			);
		}

		// Reinterpret as a different vector type
		template <typename U> explicit operator Vec8<S, U> const& () const noexcept
		{
			return reinterpret_cast<Vec8<S, U> const&>(*this);
		}
		template <typename U> explicit operator Vec8<S, U>& () noexcept
		{
			return reinterpret_cast<Vec8<S, U>&>(*this);
		}
		operator Vec8<S, void> const& () const noexcept
		{
			return reinterpret_cast<Vec8<S, void> const&>(*this);
		}
		operator Vec8<S, void>& () noexcept
		{
			return reinterpret_cast<Vec8<S, void>&>(*this);
		}

		// Array access
		constexpr S const& operator [] (int i) const noexcept
		{
			pr_assert(i >= 0 && i < _countof(arr) && "index out of range");
			return arr[i];
		}
		constexpr S& operator [] (int i) noexcept
		{
			pr_assert(i >= 0 && i < _countof(arr) && "index out of range");
			return arr[i];
		}

		// Constants
		static constexpr Vec8 const& Zero() noexcept
		{
			static auto s_zero = Vec8{ Vec4::Zero(), Vec4::Zero() };
			return s_zero;
		}

		// Sample the vector field at 'ofs'.
		// Returns the direction and magnitude of the vector field at 'ofs'
		Vec4 LinAt(Vec4 ofs) const noexcept
		{
			return Vec4{ lin + Cross(ang, ofs) };
		}

		// Sample the vector field at 'ofs'
		// Not really sure what the physical interpretation of this is.
		// Returns 'ang - ofs x lin', which is the angular required at 'ofs' to ensure that the angular
		// is constant over the whole vector field, given that 'ofs x lin' contributes to the angular.
		Vec4 AngAt(Vec4 ofs) const noexcept
		{
			return Vec4{ ang - Cross(ofs, lin) };
		}

		// Operators
		#pragma region Operators
		friend constexpr Vec8 pr_vectorcall operator + (Vec8 lhs) noexcept
		{
			return lhs;
		}
		friend constexpr Vec8 pr_vectorcall operator - (Vec8 lhs) noexcept
		{
			return Vec8{ -lhs.ang, -lhs.lin };
		}
		friend Vec8 pr_vectorcall operator * (S lhs, Vec8 rhs) noexcept
		{
			return rhs * lhs;
		}
		friend Vec8 pr_vectorcall operator * (Vec8 lhs, S rhs) noexcept
		{
			return Vec8{ lhs.ang * rhs, lhs.lin * rhs };
		}
		friend Vec8 pr_vectorcall operator / (Vec8 lhs, S rhs) noexcept
		{
			return Vec8{ lhs.ang / rhs, lhs.lin / rhs };
		}
		friend Vec8 pr_vectorcall operator % (Vec8 lhs, S rhs) noexcept
		{
			return Vec8{ lhs.ang % rhs, lhs.lin % rhs };
		}
		friend Vec8 pr_vectorcall operator + (Vec8 lhs, Vec8 rhs) noexcept
		{
			return Vec8{ lhs.ang + rhs.ang, lhs.lin + rhs.lin };
		}
		friend Vec8 pr_vectorcall operator - (Vec8 lhs, Vec8 rhs) noexcept
		{
			return Vec8{ lhs.ang - rhs.ang, lhs.lin - rhs.lin };
		}
		friend Vec8 pr_vectorcall operator * (Vec8 lhs, Vec8 rhs) noexcept
		{
			return Vec8{ lhs.ang * rhs.ang, lhs.lin * rhs.lin };
		}
		friend Vec8 pr_vectorcall operator / (Vec8 lhs, Vec8 rhs) noexcept
		{
			return Vec8{ lhs.ang / rhs.ang, lhs.lin / rhs.lin };
		}
		friend Vec8 pr_vectorcall operator % (Vec8 lhs, Vec8 rhs) noexcept
		{
			return Vec8{ lhs.ang % rhs.ang, lhs.lin % rhs.lin };
		}
		friend bool pr_vectorcall operator == (Vec8 lhs, Vec8 rhs) noexcept
		{
			return lhs.ang == rhs.ang && lhs.lin == rhs.lin;
		}
		friend bool pr_vectorcall operator != (Vec8 lhs, Vec8 rhs) noexcept
		{
			return !(lhs == rhs);
		}
		#pragma endregion
	};

	// Note: Vec8 isn't really a VectorType. It can't be used with the generic vector functions. We can do checks though.
	#define PR_MATH_DEFINE_TYPE(element)\
	static_assert(sizeof(Vec8<element, void>) == 8*sizeof(element), "Vec8<"#element",T> has the wrong size");\
	static_assert(std::is_trivially_copyable_v<Vec8<element, void>>, "Vec8<"#element",T> is not trivially copyable");
	PR_MATH_DEFINE_TYPE(float);
	PR_MATH_DEFINE_TYPE(double);
	PR_MATH_DEFINE_TYPE(int32_t);
	PR_MATH_DEFINE_TYPE(int64_t);
	#undef PR_MATH_DEFINE_TYPE

	// Compare for floating point equality
	template <ScalarType S, typename T>
	constexpr bool FEql(Vec8<S, T> lhs, Vec8<S, T> rhs) noexcept
	{
		return
			FEql(lhs.ang, rhs.ang) &&
			FEql(lhs.lin, rhs.lin);
	}

	// Project a vector onto an axis. Loosely "dot(vec,axis)*axis"
	template <ScalarType S, typename T>
	constexpr inline Vec8<S, T> pr_vectorcall Proj(Vec8<S, T> vec, Vec4<S> axis) noexcept
	{
		return Vec8<S, T>{
			Dot(vec.ang, axis)* axis,
			Dot(vec.lin, axis)* axis
		};
	}

	// Reflect a vector. Reverses the components of 'vec' in the direction of 'normal'
	template <ScalarType S, typename T>
	constexpr inline Vec8<S, T> pr_vectorcall Reflect(Vec8<S, T> vec, Vec4<S> normal) noexcept
	{
		return vec - S(2) * Proj(vec, normal);
	}
}
