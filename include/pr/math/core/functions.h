//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/core/traits.h"
#include "pr/math/core/constants.h"
#include "pr/math/core/axis_id.h"

namespace pr::math
{
	// Notes:
	//  - This file contains operators for generic vector types.
	//  - Overloads for specific types can be created (e.g. Vec4)

	// Operators
	template <VectorType Vec> constexpr Vec pr_vectorcall operator + (Vec lhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = +vec(lhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = +vec(lhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = +vec(lhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = +vec(lhs).w;
		return res;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator - (Vec lhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = -vec(lhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = -vec(lhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = -vec(lhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = -vec(lhs).w;
		return res;
	}
	template <VectorType Vec> constexpr Vec& pr_vectorcall operator += (Vec& lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		if constexpr (vt::dimension > 0) vec(lhs).x += vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(lhs).y += vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(lhs).z += vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(lhs).w += vec(rhs).w;
		return lhs;
	}
	template <VectorType Vec> constexpr Vec& pr_vectorcall operator -= (Vec& lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		if constexpr (vt::dimension > 0) vec(lhs).x -= vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(lhs).y -= vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(lhs).z -= vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(lhs).w -= vec(rhs).w;
		return lhs;
	}
	template <VectorType Vec> constexpr Vec& pr_vectorcall operator *= (Vec& lhs, Vec rhs) noexcept requires (IsRank1<Vec>)
	{
		using vt = vector_traits<Vec>;
		if constexpr (vt::dimension > 0) vec(lhs).x *= vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(lhs).y *= vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(lhs).z *= vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(lhs).w *= vec(rhs).w;
		return lhs;
	}
	template <VectorType Vec> constexpr Vec& pr_vectorcall operator *= (Vec& lhs, typename vector_traits<Vec>::element_t rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		if constexpr (vt::dimension > 0) vec(lhs).x *= rhs;
		if constexpr (vt::dimension > 1) vec(lhs).y *= rhs;
		if constexpr (vt::dimension > 2) vec(lhs).z *= rhs;
		if constexpr (vt::dimension > 3) vec(lhs).w *= rhs;
		return lhs;
	}
	template <VectorType Vec> constexpr Vec& pr_vectorcall operator /= (Vec& lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		if constexpr (vt::dimension > 0) vec(lhs).x /= vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(lhs).y /= vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(lhs).z /= vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(lhs).w /= vec(rhs).w;
		return lhs;
	}
	template <VectorType Vec> constexpr Vec& pr_vectorcall operator /= (Vec& lhs, typename vector_traits<Vec>::element_t rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		if constexpr (vt::dimension > 0) vec(lhs).x /= rhs;
		if constexpr (vt::dimension > 1) vec(lhs).y /= rhs;
		if constexpr (vt::dimension > 2) vec(lhs).z /= rhs;
		if constexpr (vt::dimension > 3) vec(lhs).w /= rhs;
		return lhs;
	}
	template <VectorType Vec> constexpr Vec& pr_vectorcall operator %= (Vec& lhs, Vec rhs) noexcept requires (IsRank1<Vec>)
	{
		using vt = vector_traits<Vec>;
		if constexpr (std::is_integral_v<typename vt::element_t>)
		{
			if constexpr (vt::dimension > 0) vec(lhs).x %= vec(rhs).x;
			if constexpr (vt::dimension > 1) vec(lhs).y %= vec(rhs).y;
			if constexpr (vt::dimension > 2) vec(lhs).z %= vec(rhs).z;
			if constexpr (vt::dimension > 3) vec(lhs).w %= vec(rhs).w;
		}
		if constexpr (std::is_floating_point_v<typename vt::element_t>)
		{
			if constexpr (vt::dimension > 0) vec(lhs).x = std::fmod(vec(lhs).x, vec(rhs).x);
			if constexpr (vt::dimension > 1) vec(lhs).y = std::fmod(vec(lhs).y, vec(rhs).y);
			if constexpr (vt::dimension > 2) vec(lhs).z = std::fmod(vec(lhs).z, vec(rhs).z);
			if constexpr (vt::dimension > 3) vec(lhs).w = std::fmod(vec(lhs).w, vec(rhs).w);
		}
		return lhs;
	}
	template <VectorType Vec> constexpr Vec& pr_vectorcall operator %= (Vec& lhs, typename vector_traits<Vec>::element_t rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		if constexpr (std::is_integral_v<typename vt::element_t>)
		{
			if constexpr (vt::dimension > 0) vec(lhs).x %= rhs;
			if constexpr (vt::dimension > 1) vec(lhs).y %= rhs;
			if constexpr (vt::dimension > 2) vec(lhs).z %= rhs;
			if constexpr (vt::dimension > 3) vec(lhs).w %= rhs;
		}
		if constexpr (std::is_floating_point_v<typename vt::element_t>)
		{
			if constexpr (vt::dimension > 0) vec(lhs).x = std::fmod(vec(lhs).x, rhs);
			if constexpr (vt::dimension > 1) vec(lhs).y = std::fmod(vec(lhs).y, rhs);
			if constexpr (vt::dimension > 2) vec(lhs).z = std::fmod(vec(lhs).z, rhs);
			if constexpr (vt::dimension > 3) vec(lhs).w = std::fmod(vec(lhs).w, rhs);
		}
		return lhs;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator + (Vec lhs, Vec rhs) noexcept
	{
		Vec res = lhs;
		return res += rhs;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator - (Vec lhs, Vec rhs) noexcept
	{
		Vec res = lhs;
		return res -= rhs;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator * (Vec lhs, Vec rhs) noexcept requires (IsRank1<Vec>)
	{
		Vec res = lhs;
		return res *= rhs;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator * (Vec lhs, typename vector_traits<Vec>::element_t rhs) noexcept
	{
		Vec res = lhs;
		return res *= rhs;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator * (typename vector_traits<Vec>::element_t lhs, Vec rhs) noexcept
	{
		Vec res = rhs;
		return res *= lhs;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator / (Vec lhs, Vec rhs) noexcept
	{
		Vec res = lhs;
		return res /= rhs;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator / (Vec lhs, typename vector_traits<Vec>::element_t rhs) noexcept
	{
		Vec res = lhs;
		return res /= rhs;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator / (typename vector_traits<Vec>::element_t lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = lhs / vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = lhs / vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = lhs / vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = lhs / vec(rhs).w;
		return res;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator % (Vec lhs, Vec rhs) noexcept
	{
		Vec res = lhs;
		return res %= rhs;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator % (Vec lhs, typename vector_traits<Vec>::element_t rhs) noexcept
	{
		Vec res = lhs;
		return res %= rhs;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall operator % (typename vector_traits<Vec>::element_t lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = lhs % vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = lhs % vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = lhs % vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = lhs % vec(rhs).w;
		return res;
	}
	template <TensorType Vec> constexpr auto pr_vectorcall operator <=> (Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		if constexpr (vt::dimension > 0) if (auto cmp = std::partial_order(vec(lhs).x, vec(rhs).x); cmp != 0) return cmp;
		if constexpr (vt::dimension > 1) if (auto cmp = std::partial_order(vec(lhs).y, vec(rhs).y); cmp != 0) return cmp;
		if constexpr (vt::dimension > 2) if (auto cmp = std::partial_order(vec(lhs).z, vec(rhs).z); cmp != 0) return cmp;
		if constexpr (vt::dimension > 3) if (auto cmp = std::partial_order(vec(lhs).w, vec(rhs).w); cmp != 0) return cmp;
		return std::partial_ordering::equivalent;
	}
	template <TensorType Vec> constexpr bool pr_vectorcall operator == (Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		if constexpr (vt::dimension > 0) if (!(vec(lhs).x == vec(rhs).x)) return false;
		if constexpr (vt::dimension > 1) if (!(vec(lhs).y == vec(rhs).y)) return false;
		if constexpr (vt::dimension > 2) if (!(vec(lhs).z == vec(rhs).z)) return false;
		if constexpr (vt::dimension > 3) if (!(vec(lhs).w == vec(rhs).w)) return false;
		return true;
	}
	template <TensorType Vec> constexpr bool pr_vectorcall operator != (Vec lhs, Vec rhs) noexcept
	{
		return !(lhs == rhs);
	}

	// Bitwise operators(only for integral vectors)
	template <VectorType Vec> requires (std::integral<typename vector_traits<Vec>::element_t>) constexpr Vec pr_vectorcall operator ~ (Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = ~vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = ~vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = ~vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = ~vec(rhs).w;
		return res;
	}
	template <VectorType Vec> requires (std::integral<typename vector_traits<Vec>::element_t>) constexpr Vec pr_vectorcall operator ! (Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = !vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = !vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = !vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = !vec(rhs).w;
		return res;
	}
	template <VectorType Vec> requires (std::integral<typename vector_traits<Vec>::element_t>) constexpr Vec pr_vectorcall operator | (Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = vec(lhs).x | vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = vec(lhs).y | vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = vec(lhs).z | vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = vec(lhs).w | vec(rhs).w;
		return res;
	}
	template <VectorType Vec> requires (std::integral<typename vector_traits<Vec>::element_t>) constexpr Vec pr_vectorcall operator & (Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = vec(lhs).x & vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = vec(lhs).y & vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = vec(lhs).z & vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = vec(lhs).w & vec(rhs).w;
		return res;
	}
	template <VectorType Vec> requires (std::integral<typename vector_traits<Vec>::element_t>) constexpr Vec pr_vectorcall operator ^ (Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = vec(lhs).x ^ vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = vec(lhs).y ^ vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = vec(lhs).z ^ vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = vec(lhs).w ^ vec(rhs).w;
		return res;
	}
	template <VectorType Vec> requires (std::integral<typename vector_traits<Vec>::element_t>) constexpr Vec pr_vectorcall operator << (Vec lhs, typename vector_traits<Vec>::element_t rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = vec(lhs).x << rhs;
		if constexpr (vt::dimension > 1) vec(res).y = vec(lhs).y << rhs;
		if constexpr (vt::dimension > 2) vec(res).z = vec(lhs).z << rhs;
		if constexpr (vt::dimension > 3) vec(res).w = vec(lhs).w << rhs;
		return res;
	}
	template <VectorType Vec> requires (std::integral<typename vector_traits<Vec>::element_t>) constexpr Vec pr_vectorcall operator << (Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = vec(lhs).x << vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = vec(lhs).y << vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = vec(lhs).z << vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = vec(lhs).w << vec(rhs).w;
		return res;
	}
	template <VectorType Vec> requires (std::integral<typename vector_traits<Vec>::element_t>) constexpr Vec pr_vectorcall operator >> (Vec lhs, typename vector_traits<Vec>::element_t rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = vec(lhs).x >> rhs;
		if constexpr (vt::dimension > 1) vec(res).y = vec(lhs).y >> rhs;
		if constexpr (vt::dimension > 2) vec(res).z = vec(lhs).z >> rhs;
		if constexpr (vt::dimension > 3) vec(res).w = vec(lhs).w >> rhs;
		return res;
	}
	template <VectorType Vec> requires (std::integral<typename vector_traits<Vec>::element_t>) constexpr Vec pr_vectorcall operator >> (Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = vec(lhs).x >> vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = vec(lhs).y >> vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = vec(lhs).z >> vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = vec(lhs).w >> vec(rhs).w;
		return res;
	}
	template <VectorType Vec> requires (std::integral<typename vector_traits<Vec>::element_t>) constexpr Vec pr_vectorcall operator || (Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = vec(lhs).x || vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = vec(lhs).y || vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = vec(lhs).z || vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = vec(lhs).w || vec(rhs).w;
		return res;
	}
	template <VectorType Vec> requires (std::integral<typename vector_traits<Vec>::element_t>) constexpr Vec pr_vectorcall operator && (Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = vec(lhs).x && vec(rhs).x;
		if constexpr (vt::dimension > 1) vec(res).y = vec(lhs).y && vec(rhs).y;
		if constexpr (vt::dimension > 2) vec(res).z = vec(lhs).z && vec(rhs).z;
		if constexpr (vt::dimension > 3) vec(res).w = vec(lhs).w && vec(rhs).w;
		return res;
	}

	// Quaternion Operators
	template <QuaternionType Quat> constexpr Quat pr_vectorcall operator + (Quat const& lhs) noexcept
	{
		return lhs;
	}
	template <QuaternionType Quat> constexpr Quat pr_vectorcall operator - (Quat const& lhs) noexcept // Note: Not conjugate
	{
		return {
			-vec(lhs).x,
			-vec(lhs).y,
			-vec(lhs).z,
			-vec(lhs).w,
		};
	}
	template <QuaternionType Quat> constexpr Quat pr_vectorcall operator ~ (Quat const& lhs) noexcept // This is conjugate
	{
		return {
			-vec(lhs).x,
			-vec(lhs).y,
			-vec(lhs).z,
			 vec(lhs).w,
		};
	}
	template <QuaternionType Quat> constexpr Quat& pr_vectorcall operator *= (Quat& lhs, typename vector_traits<Quat>::element_t rhs) noexcept
	{
		vec(lhs).x *= rhs;
		vec(lhs).y *= rhs;
		vec(lhs).z *= rhs;
		vec(lhs).w *= rhs;
		return lhs;
	}
	template <QuaternionType Quat> constexpr Quat pr_vectorcall operator * (Quat const& lhs, typename vector_traits<Quat>::element_t rhs) noexcept
	{
		Quat res = lhs;
		return res *= rhs;
	}
	template <QuaternionType Quat> constexpr Quat pr_vectorcall operator * (typename vector_traits<Quat>::element_t lhs, Quat const& rhs) noexcept
	{
		Quat res = rhs;
		return res *= lhs;
	}
	template <QuaternionType Quat> constexpr Quat pr_vectorcall operator * (Quat const& lhs, Quat const& rhs) noexcept
	{
		// Quaternion multiply
		// Note about 'quat multiply' vs. 'r = q*v*conj(q)':
		// To rotate a vector or another quaternion, use the "sandwich product"
		// However, combining rotations is done using q1 * q2.
		// This is because:
		//'  r1 = a * v * conj(a)  - first rotation 
		//'  r2 = b * r1 * conj(b) - second rotation
		//'  r2 = b * a * v * conj(a) * conj(b)     
		//'  r2 = (b*a) * v * conj(b*a)             
		Quat res = {};
		vec(res).x = vec(lhs).w * vec(rhs).x + vec(lhs).x * vec(rhs).w + vec(lhs).y * vec(rhs).z - vec(lhs).z * vec(rhs).y;
		vec(res).y = vec(lhs).w * vec(rhs).y - vec(lhs).x * vec(rhs).z + vec(lhs).y * vec(rhs).w + vec(lhs).z * vec(rhs).x;
		vec(res).z = vec(lhs).w * vec(rhs).z + vec(lhs).x * vec(rhs).y - vec(lhs).y * vec(rhs).x + vec(lhs).z * vec(rhs).w;
		vec(res).w = vec(lhs).w * vec(rhs).w - vec(lhs).x * vec(rhs).x - vec(lhs).y * vec(rhs).y - vec(lhs).z * vec(rhs).z;
		return res;
	}
	template <QuaternionType Quat> constexpr Quat pr_vectorcall operator /= (Quat& lhs, typename vector_traits<Quat>::element_t rhs) noexcept
	{
		vec(lhs).x /= rhs;
		vec(lhs).y /= rhs;
		vec(lhs).z /= rhs;
		vec(lhs).w /= rhs;
		return lhs;
	}
	template <QuaternionType Quat> constexpr Quat pr_vectorcall operator / (Quat const& lhs, typename vector_traits<Quat>::element_t rhs) noexcept
	{
		Quat res = lhs;
		return res /= rhs;
	}

	// Matrix Multiply forward
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr typename vector_traits<Mat>::component_t pr_vectorcall operator * (Mat const& a2b, typename vector_traits<Mat>::component_t v) noexcept;
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr Mat pr_vectorcall operator * (Mat const& b2c, Mat const& a2b) noexcept;

	// Constants
	template <typename S> constexpr S Zero() noexcept
	{
		return S(0);
	}
	template <typename S> constexpr S One() noexcept
	{
		return S(1);
	}
	template <typename S> constexpr S Min() noexcept
	{
		return std::numeric_limits<S>::lowest();
	}
	template <typename S> constexpr S Max() noexcept
	{
		return std::numeric_limits<S>::max();
	}
	template <typename S> constexpr S Infinity() noexcept
	{
		return std::numeric_limits<S>::infinity();
	}
	template <typename S> constexpr S Epsilon() noexcept
	{
		return std::numeric_limits<S>::epsilon();
	}
	template <typename S> constexpr S Tiny() noexcept
	{
		return constants<S>::tiny;
	}
	template <typename S> constexpr S Lowest() noexcept
	{
		return std::numeric_limits<S>::lowest();
	}
	template <VectorType Vec> constexpr Vec Zero() noexcept
	{
		using vt = vector_traits<Vec>;
		using C = typename vt::component_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Zero<C>();
		if constexpr (vt::dimension > 1) vec(res).y = Zero<C>();
		if constexpr (vt::dimension > 2) vec(res).z = Zero<C>();
		if constexpr (vt::dimension > 3) vec(res).w = Zero<C>();
		return res;
	}
	template <VectorType Vec> constexpr Vec One() noexcept
	{
		using vt = vector_traits<Vec>;
		using C = typename vt::component_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = One<C>();
		if constexpr (vt::dimension > 1) vec(res).y = One<C>();
		if constexpr (vt::dimension > 2) vec(res).z = One<C>();
		if constexpr (vt::dimension > 3) vec(res).w = One<C>();
		return res;
	}
	template <VectorType Vec> constexpr Vec Min() noexcept
	{
		using vt = vector_traits<Vec>;
		using C = typename vt::component_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Min<C>();
		if constexpr (vt::dimension > 1) vec(res).y = Min<C>();
		if constexpr (vt::dimension > 2) vec(res).z = Min<C>();
		if constexpr (vt::dimension > 3) vec(res).w = Min<C>();
		return res;
	}
	template <VectorType Vec> constexpr Vec Max() noexcept
	{
		using vt = vector_traits<Vec>;
		using C = typename vt::component_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Max<C>();
		if constexpr (vt::dimension > 1) vec(res).y = Max<C>();
		if constexpr (vt::dimension > 2) vec(res).z = Max<C>();
		if constexpr (vt::dimension > 3) vec(res).w = Max<C>();
		return res;
	}
	template <VectorType Vec> constexpr Vec Tiny() noexcept
	{
		using vt = vector_traits<Vec>;
		using C = typename vt::component_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Tiny<C>();
		if constexpr (vt::dimension > 1) vec(res).y = Tiny<C>();
		if constexpr (vt::dimension > 2) vec(res).z = Tiny<C>();
		if constexpr (vt::dimension > 3) vec(res).w = Tiny<C>();
		return res;
	}
	template <VectorType Vec> constexpr Vec Lowest() noexcept
	{
		using vt = vector_traits<Vec>;
		using C = typename vt::component_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Lowest<C>();
		if constexpr (vt::dimension > 1) vec(res).y = Lowest<C>();
		if constexpr (vt::dimension > 2) vec(res).z = Lowest<C>();
		if constexpr (vt::dimension > 3) vec(res).w = Lowest<C>();
		return res;
	}
	template <VectorType Vec> constexpr Vec Infinity() noexcept
	{
		using vt = vector_traits<Vec>;
		using C = typename vt::component_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Infinity<C>();
		if constexpr (vt::dimension > 1) vec(res).y = Infinity<C>();
		if constexpr (vt::dimension > 2) vec(res).z = Infinity<C>();
		if constexpr (vt::dimension > 3) vec(res).w = Infinity<C>();
		return res;
	}
	template <VectorType Vec> constexpr Vec Epsilon() noexcept
	{
		using vt = vector_traits<Vec>;
		using C = typename vt::component_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Epsilon<C>();
		if constexpr (vt::dimension > 1) vec(res).y = Epsilon<C>();
		if constexpr (vt::dimension > 2) vec(res).z = Epsilon<C>();
		if constexpr (vt::dimension > 3) vec(res).w = Epsilon<C>();
		return res;
	}
	template <VectorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension >= 1) constexpr Vec XAxis() noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = S(1);
		if constexpr (vt::dimension > 1) vec(res).y = S(0);
		if constexpr (vt::dimension > 2) vec(res).z = S(0);
		if constexpr (vt::dimension > 3) vec(res).w = S(0);
		return res;
	}
	template <VectorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension >= 2) constexpr Vec YAxis() noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = S(0);
		if constexpr (vt::dimension > 1) vec(res).y = S(1);
		if constexpr (vt::dimension > 2) vec(res).z = S(0);
		if constexpr (vt::dimension > 3) vec(res).w = S(0);
		return res;
	}
	template <VectorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension >= 3) constexpr Vec ZAxis() noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = S(0);
		if constexpr (vt::dimension > 1) vec(res).y = S(0);
		if constexpr (vt::dimension > 2) vec(res).z = S(1);
		if constexpr (vt::dimension > 3) vec(res).w = S(0);
		return res;
	}
	template <VectorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension >= 4) constexpr Vec WAxis() noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = S(0);
		if constexpr (vt::dimension > 1) vec(res).y = S(0);
		if constexpr (vt::dimension > 2) vec(res).z = S(0);
		if constexpr (vt::dimension > 3) vec(res).w = S(1);
		return res;
	}
	template <VectorType Vec> requires (IsRank1<Vec>) constexpr Vec Origin() noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = S(0);
		if constexpr (vt::dimension > 1) vec(res).y = S(0);
		if constexpr (vt::dimension > 2) vec(res).z = S(0);
		if constexpr (vt::dimension > 3) vec(res).w = S(1);
		return res;
	}
	template <VectorType Mat> requires (IsRank2<Mat>) constexpr Mat Identity() noexcept
	{
		using vt = vector_traits<Mat>;

		Mat res = {};
		if constexpr (vt::dimension > 0) vec(res).x = XAxis<typename vt::component_t>();
		if constexpr (vt::dimension > 1) vec(res).y = YAxis<typename vt::component_t>();
		if constexpr (vt::dimension > 2) vec(res).z = ZAxis<typename vt::component_t>();
		if constexpr (vt::dimension > 3) vec(res).w = Origin<typename vt::component_t>();
		return res;
	}
	template <QuaternionType Quat> constexpr Quat Identity() noexcept
	{
		using S = typename vector_traits<Quat>::element_t;
		return { S(0), S(0), S(0), S(1) };
	}

	// NaN test
	template <ScalarType S> constexpr bool IsNaN(S value) noexcept
	{
		return value != value; // NaN is the only value that is not equal to itself
	}
	template <TensorType Vec> constexpr bool pr_vectorcall IsNaN(Vec v, bool any = true) noexcept // false = all
	{
		using vt = vector_traits<Vec>;
		bool yes;
		if (any)
		{
			yes = false;
			if constexpr (vt::dimension > 0) yes = yes || IsNaN(vec(v).x);
			if constexpr (vt::dimension > 1) yes = yes || IsNaN(vec(v).y);
			if constexpr (vt::dimension > 2) yes = yes || IsNaN(vec(v).z);
			if constexpr (vt::dimension > 3) yes = yes || IsNaN(vec(v).w);
		}
		else
		{
			yes = true;
			if constexpr (vt::dimension > 0) yes = yes && IsNaN(vec(v).x);
			if constexpr (vt::dimension > 1) yes = yes && IsNaN(vec(v).y);
			if constexpr (vt::dimension > 2) yes = yes && IsNaN(vec(v).z);
			if constexpr (vt::dimension > 3) yes = yes && IsNaN(vec(v).w);
		}
		return yes;
	}

	// Finite test
	template <ScalarType S> constexpr bool IsFinite(S value) noexcept
	{
		if consteval
		{
			// When float operations are performed at compile time, the compiler warnings about 'inf' and 'nan' are annoying and unhelpful, so handle them manually.
			return value != std::numeric_limits<S>::infinity() && value != -std::numeric_limits<S>::infinity() && value == value;
		}
		else
		{
			if constexpr (std::floating_point<S>)
				return std::isfinite(value);
			else
				return true;
		}
	}
	template <ScalarType S> constexpr bool IsFinite(S value, S max_value) noexcept
	{
		return IsFinite(value) && value < max_value && value > -max_value;
	}
	template <TensorType Vec> constexpr bool pr_vectorcall IsFinite(Vec v, bool any = false) noexcept
	{
		using vt = vector_traits<Vec>;
		bool yes;
		if (any)
		{
			yes = false;
			if constexpr (vt::dimension > 0) yes = yes || IsFinite(vec(v).x);
			if constexpr (vt::dimension > 1) yes = yes || IsFinite(vec(v).y);
			if constexpr (vt::dimension > 2) yes = yes || IsFinite(vec(v).z);
			if constexpr (vt::dimension > 3) yes = yes || IsFinite(vec(v).w);
		}
		else
		{
			yes = true;
			if constexpr (vt::dimension > 0) yes = yes && IsFinite(vec(v).x);
			if constexpr (vt::dimension > 1) yes = yes && IsFinite(vec(v).y);
			if constexpr (vt::dimension > 2) yes = yes && IsFinite(vec(v).z);
			if constexpr (vt::dimension > 3) yes = yes && IsFinite(vec(v).w);
		}
		return yes;
	}

	// Return true if any element satisfies 'Pred'
	template <TensorType Vec, typename Pred> constexpr bool pr_vectorcall Any(Vec v, Pred pred) noexcept
	{
		using vt = vector_traits<Vec>;
		bool yes = false;
		if constexpr (vt::dimension > 0) yes = yes || pred(vec(v).x);
		if constexpr (vt::dimension > 1) yes = yes || pred(vec(v).y);
		if constexpr (vt::dimension > 2) yes = yes || pred(vec(v).z);
		if constexpr (vt::dimension > 3) yes = yes || pred(vec(v).w);
		return yes;
	}

	// Return true if all elements satisfy 'Pred'
	template <TensorType Vec, typename Pred> constexpr bool pr_vectorcall All(Vec v, Pred pred) noexcept
	{
		using vt = vector_traits<Vec>;
		bool yes = true;
		if constexpr (vt::dimension > 0) yes = yes && pred(vec(v).x);
		if constexpr (vt::dimension > 1) yes = yes && pred(vec(v).y);
		if constexpr (vt::dimension > 2) yes = yes && pred(vec(v).z);
		if constexpr (vt::dimension > 3) yes = yes && pred(vec(v).w);
		return yes;
	}
	template <std::ranges::input_range Range, typename Pred> constexpr bool Any(Range&& range, Pred pred) noexcept requires (!TensorType<std::decay_t<Range>>)
	{
		for (auto&& element : range)
			if (pred(element)) return true;
		return false;
	}
	template <std::ranges::input_range Range, typename Pred> constexpr bool All(Range&& range, Pred pred) noexcept requires (!TensorType<std::decay_t<Range>>)
	{
		for (auto&& element : range)
			if (!pred(element)) return false;
		return true;
	}

	// Compile time function for applying 'op' to each component of a vector
	template <ScalarType S, typename Op> constexpr S CompOp(S a, Op op)
	{
	}
	template <ScalarType S, typename Op> constexpr S CompOp(S a, S b, Op op)
	{
	}
	template <ScalarType S, typename Op> constexpr S CompOp(S a, S b, S c, Op op)
	{
	}
	template <ScalarType S, typename Op> constexpr S CompOp(S a, S b, S c, S d, Op op)
	{
	}
	template <VectorType Vec, typename Op> constexpr Vec CompOp(Vec a, Op op)
	{
		Vec res = {};
		if constexpr (vector_traits<Vec>::dimension > 0) vec(res).x = op(vec(a).x);
		if constexpr (vector_traits<Vec>::dimension > 1) vec(res).y = op(vec(a).y);
		if constexpr (vector_traits<Vec>::dimension > 2) vec(res).z = op(vec(a).z);
		if constexpr (vector_traits<Vec>::dimension > 3) vec(res).w = op(vec(a).w);
		return res;
	}
	template <VectorType Vec, typename Op> constexpr Vec CompOp(Vec a, Vec b, Op op)
	{
		Vec res = {};
		if constexpr (vector_traits<Vec>::dimension > 0) vec(res).x = op(vec(a).x, vec(b).x);
		if constexpr (vector_traits<Vec>::dimension > 1) vec(res).y = op(vec(a).y, vec(b).y);
		if constexpr (vector_traits<Vec>::dimension > 2) vec(res).z = op(vec(a).z, vec(b).z);
		if constexpr (vector_traits<Vec>::dimension > 3) vec(res).w = op(vec(a).w, vec(b).w);
		return res;
	}
	template <VectorType Vec, typename Op> constexpr Vec CompOp(Vec a, Vec b, Vec c, Op op)
	{
		Vec res = {};
		if constexpr (vector_traits<Vec>::dimension > 0) vec(res).x = op(vec(a).x, vec(b).x, vec(c).x);
		if constexpr (vector_traits<Vec>::dimension > 1) vec(res).y = op(vec(a).y, vec(b).y, vec(c).y);
		if constexpr (vector_traits<Vec>::dimension > 2) vec(res).z = op(vec(a).z, vec(b).z, vec(c).z);
		if constexpr (vector_traits<Vec>::dimension > 3) vec(res).w = op(vec(a).w, vec(b).w, vec(c).w);
		return res;
	}
	template <VectorType Vec, typename Op> constexpr Vec CompOp(Vec a, Vec b, Vec c, Vec d, Op op)
	{
		Vec res = {};
		if constexpr (vector_traits<Vec>::dimension > 0) vec(res).x = op(vec(a).x, vec(b).x, vec(c).x, vec(d).x);
		if constexpr (vector_traits<Vec>::dimension > 1) vec(res).y = op(vec(a).y, vec(b).y, vec(c).y, vec(d).y);
		if constexpr (vector_traits<Vec>::dimension > 2) vec(res).z = op(vec(a).z, vec(b).z, vec(c).z, vec(d).z);
		if constexpr (vector_traits<Vec>::dimension > 3) vec(res).w = op(vec(a).w, vec(b).w, vec(c).w, vec(d).w);
		return res;
	}

	// Absolute value (component-wise)
	template <std::integral S> constexpr S Abs(S v) noexcept
	{
		if constexpr (std::is_unsigned_v<S>)
			return v;
		else
			return v >= S(0) ? v : -v;
	}
	template <std::floating_point S> constexpr S Abs(S v) noexcept
	{
		return v >= S(0) ? v : -v;
	}
	template <TensorType Vec> constexpr Vec pr_vectorcall Abs(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Abs(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = Abs(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = Abs(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = Abs(vec(v).w);
		return res;
	}

	// Min/Max
	template <ScalarType S> constexpr S Min(S x, S y) noexcept
	{
		return (x < y) ? x : y;
	}
	template <ScalarType S> constexpr S Max(S x, S y) noexcept
	{
		return (x < y) ? y : x;
	}
	template <TensorType Vec> constexpr Vec pr_vectorcall Min(Vec x, Vec y) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Min(vec(x).x, vec(y).x);
		if constexpr (vt::dimension > 1) vec(res).y = Min(vec(x).y, vec(y).y);
		if constexpr (vt::dimension > 2) vec(res).z = Min(vec(x).z, vec(y).z);
		if constexpr (vt::dimension > 3) vec(res).w = Min(vec(x).w, vec(y).w);
		return res;
	}
	template <TensorType Vec> constexpr Vec pr_vectorcall Max(Vec x, Vec y) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Max(vec(x).x, vec(y).x);
		if constexpr (vt::dimension > 1) vec(res).y = Max(vec(x).y, vec(y).y);
		if constexpr (vt::dimension > 2) vec(res).z = Max(vec(x).z, vec(y).z);
		if constexpr (vt::dimension > 3) vec(res).w = Max(vec(x).w, vec(y).w);
		return res;
	}
	template <typename T, typename... A> constexpr T Min(T const& x, T const& y, A&&... a) noexcept
	{
		return Min(Min(x, y), std::forward<A>(a)...);
	}
	template <typename T, typename... A> constexpr T Max(T const& x, T const& y, A&&... a) noexcept
	{
		return Max(Max(x, y), std::forward<A>(a)...);
	}

	// Clamp
	template <ScalarType S> constexpr S Clamp(S x, S mn, S mx) noexcept
	{
		pr_assert(!(mx < mn) && "[min,max] must be a positive range");
		return (mx < x) ? mx : (x < mn) ? mn : x;
	}
	template <TensorType Vec> constexpr Vec pr_vectorcall Clamp(Vec x, Vec mn, Vec mx) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Clamp(vec(x).x, vec(mn).x, vec(mx).x);
		if constexpr (vt::dimension > 1) vec(res).y = Clamp(vec(x).y, vec(mn).y, vec(mx).y);
		if constexpr (vt::dimension > 2) vec(res).z = Clamp(vec(x).z, vec(mn).z, vec(mx).z);
		if constexpr (vt::dimension > 3) vec(res).w = Clamp(vec(x).w, vec(mn).w, vec(mx).w);
		return res;
	}

	// Square/Signed Square
	template <ScalarType S> constexpr S Sqr(S x) noexcept
	{
		if constexpr (std::is_same_v<S, int8_t>)
			pr_assert("Overflow" && Abs(x) <= 0xB);
		if constexpr (std::is_same_v<S, uint8_t>)
			pr_assert("Overflow" && Abs(x) <= 0xF);
		if constexpr (std::is_same_v<S, int16_t>)
			pr_assert("Overflow" && Abs(x) <= 0xB5);
		if constexpr (std::is_same_v<S, uint16_t>)
			pr_assert("Overflow" && Abs(x) <= 0xFF);
		if constexpr (std::is_same_v<S, int32_t> || std::is_same_v<S, long>)
			pr_assert("Overflow" && Abs(x) <= 0xB504);
		if constexpr (std::is_same_v<S, uint32_t> || std::is_same_v<S, unsigned long>)
			pr_assert("Overflow" && Abs(x) <= 0xFFFFU);
		if constexpr (std::is_same_v<S, int64_t>)
			pr_assert("Overflow" && Abs(x) <= 0xB504F333LL);
		if constexpr (std::is_same_v<S, uint64_t>)
			pr_assert("Overflow" && Abs(x) <= 0xFFFFFFFFULL);

		return x * x;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall Sqr(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Sqr(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = Sqr(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = Sqr(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = Sqr(vec(v).w);
		return res;
	}

	// Cube a value
	template <ScalarType S> constexpr S Cube(S x) noexcept
	{
		if constexpr (std::is_same_v<S, int8_t>)
			pr_assert("Overflow" && Abs(x) <= 0x5);
		if constexpr (std::is_same_v<S, uint8_t>)
			pr_assert("Overflow" && Abs(x) <= 0x6);
		if constexpr (std::is_same_v<S, int16_t>)
			pr_assert("Overflow" && Abs(x) <= 0x1F);
		if constexpr (std::is_same_v<S, uint16_t>)
			pr_assert("Overflow" && Abs(x) <= 0x28);
		if constexpr (std::is_same_v<S, int32_t> || std::is_same_v<S, long>)
			pr_assert("Overflow" && Abs(x) <= 0x50A);
		if constexpr (std::is_same_v<S, uint32_t> || std::is_same_v<S, unsigned long>)
			pr_assert("Overflow" && Abs(x) <= 0x659U);
		if constexpr (std::is_same_v<S, int64_t>)
			pr_assert("Overflow" && Abs(x) <= 0x1FFFFFLL);
		if constexpr (std::is_same_v<S, uint64_t>)
			pr_assert("Overflow" && Abs(x) <= 0x285145ULL);

		return x * x * x;
	}
	template <VectorType Vec> constexpr Vec pr_vectorcall Cube(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Cube(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = Cube(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = Cube(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = Cube(vec(v).w);
		return res;
	}

	// Signed square
	template <ScalarType S> constexpr S SignedSqr(S x) noexcept
	{
		return x >= S() ? +Sqr(x) : -Sqr(x);
	}
	template <TensorType Vec> constexpr Vec pr_vectorcall SignedSqr(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = SignedSqr(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = SignedSqr(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = SignedSqr(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = SignedSqr(vec(v).w);
		return res;
	}

	// Square root/Signed Square root
	constexpr double SqrtCT(double x) noexcept
	{
		// For a finite and non-negative value of "x", returns an approximation for the square root of "x", otherwise returns NaN.
		struct L
		{
			constexpr static double NewtonRaphson(double x, double curr, double prev) noexcept
			{
				return curr == prev ? curr : NewtonRaphson(x, 0.5 * (curr + x / curr), curr);
			}
		};
		return x >= 0 && x < std::numeric_limits<double>::infinity() ? L::NewtonRaphson(x, x, 0) : std::numeric_limits<double>::quiet_NaN();
	}
	template <ScalarType S> constexpr S Sqrt(S x) noexcept
	{
		if constexpr (std::floating_point<S>)
			pr_assert("Sqrt of undefined value" && IsFinite(x));
		if constexpr (std::is_signed_v<S>)
			pr_assert("Sqrt of negative value" && x >= S(0));

		if consteval
		{
			return static_cast<S>(SqrtCT(static_cast<double>(x)));
		}
		else
		{
			return static_cast<S>(std::sqrt(x));
		}
	}
	template <ScalarType S> constexpr S SignedSqrt(S x) noexcept
	{
		return x >= S(0) ? +Sqrt(x) : -Sqrt(-x);
	}
	template <ScalarType S> constexpr S CompSqrt(S x) noexcept
	{
		return Sqrt(x);
	}
	template <ScalarType S> constexpr S CompSignedSqrt(S x) noexcept
	{
		return SignedSqrt(x);
	}
	template <TensorType Vec> constexpr Vec pr_vectorcall Sqrt(Vec) noexcept requires (IsRank1<Vec>)
	{
		// Sqrt is ill-defined for non-square matrices.
		// Matrices have an overload that finds the matrix whose product is 'x'.
		static_assert(std::is_same_v<Vec, void>, "Sqrt is not defined for general vector types");
	}
	template <TensorTypeFP Vec> constexpr Vec pr_vectorcall CompSqrt(Vec v) noexcept // Component Sqrt
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = CompSqrt(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = CompSqrt(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = CompSqrt(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = CompSqrt(vec(v).w);
		return res;
	}
	template <TensorTypeFP Vec> constexpr Vec pr_vectorcall CompSignedSqrt(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = CompSignedSqrt(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = CompSignedSqrt(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = CompSignedSqrt(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = CompSignedSqrt(vec(v).w);
		return res;
	}

	// Integer square root
	template <std::integral T> constexpr T ISqrt(T x) noexcept
	{
		// Compile time version of the square root.
		//  - For a finite and non-negative value of "x", returns an approximation for the square root of "x"
		//  - This method always converges or oscillates about the answer with a difference of 1.
		//  - Otherwise, returns 0
		if (x < 0)
			return std::numeric_limits<T>::quiet_NaN();

		T curr = x, prev = 0, pprev = 0;
		for (;curr != prev && curr != pprev;)
		{
			pprev = prev;
			prev = curr;
			curr = (curr + x / curr) >> 1;
		}
		return Abs(x - curr * curr) < Abs(x - prev * prev) ? curr : prev;
	}
	template <std::integral T> constexpr T CompISqrt(T x) noexcept
	{
		return ISqrt(x);
	}
	template <TensorType Vec> constexpr Vec pr_vectorcall CompISqrt(Vec v) noexcept requires (std::integral<typename vector_traits<Vec>::element_t>)
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = CompISqrt(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = CompISqrt(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = CompISqrt(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = CompISqrt(vec(v).w);
		return res;
	}

	// Return the component sum
	template <VectorType Vec> constexpr typename vector_traits<Vec>::element_t pr_vectorcall CompSum(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		S sum = {};
		if constexpr (IsRank1<Vec>)
		{
			if constexpr (vt::dimension > 0) sum += vec(v).x;
			if constexpr (vt::dimension > 1) sum += vec(v).y;
			if constexpr (vt::dimension > 2) sum += vec(v).z;
			if constexpr (vt::dimension > 3) sum += vec(v).w;
		}
		else
		{
			if constexpr (vt::dimension > 0) sum += CompSum(vec(v).x);
			if constexpr (vt::dimension > 1) sum += CompSum(vec(v).y);
			if constexpr (vt::dimension > 2) sum += CompSum(vec(v).z);
			if constexpr (vt::dimension > 3) sum += CompSum(vec(v).w);
		}
		return sum;
	}

	// Scale each component of 'mat' by the values in 'scale'
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr Mat pr_vectorcall CompMul(Mat const& mat, typename vector_traits<Mat>::component_t scale) noexcept
	{
		Mat m = mat;
		if constexpr (vector_traits<Mat>::dimension > 0) vec(m).x *= scale;
		if constexpr (vector_traits<Mat>::dimension > 1) vec(m).y *= scale;
		if constexpr (vector_traits<Mat>::dimension > 2) vec(m).z *= scale;
		if constexpr (vector_traits<Mat>::dimension > 3) vec(m).w *= scale;
		return m;
	}

	// Min/Max element (i.e. nearest to -inf/+inf)
	template <ScalarType S> constexpr S MinElement(S v) noexcept
	{
		return v;
	}
	template <ScalarType S> constexpr S MaxElement(S v) noexcept
	{
		return v;
	}
	template <TensorType Vec> constexpr typename vector_traits<Vec>::element_t pr_vectorcall MinElement(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		auto minimum = Max<typename vt::element_t>();
		if constexpr (vt::dimension > 0) minimum = std::min(minimum, MinElement(vec(v).x));
		if constexpr (vt::dimension > 1) minimum = std::min(minimum, MinElement(vec(v).y));
		if constexpr (vt::dimension > 2) minimum = std::min(minimum, MinElement(vec(v).z));
		if constexpr (vt::dimension > 3) minimum = std::min(minimum, MinElement(vec(v).w));
		return minimum;
	}
	template <TensorType Vec> constexpr typename vector_traits<Vec>::element_t pr_vectorcall MaxElement(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		auto maximum = Min<typename vt::element_t>();
		if constexpr (vt::dimension > 0) maximum = std::max(maximum, MaxElement(vec(v).x));
		if constexpr (vt::dimension > 1) maximum = std::max(maximum, MaxElement(vec(v).y));
		if constexpr (vt::dimension > 2) maximum = std::max(maximum, MaxElement(vec(v).z));
		if constexpr (vt::dimension > 3) maximum = std::max(maximum, MaxElement(vec(v).w));
		return maximum;
	}
	template <std::ranges::input_range Range> constexpr auto MinElement(Range&& range) noexcept
	{
		// Note: if 'range' is a Vec4[], this will find the minimum element among all components of all vectors, not the minimum vector by component comparison.
		using value_t = std::ranges::range_value_t<Range>;
		using element_t = decltype(MinElement(std::declval<value_t>()));

		element_t minimum = Max<element_t>();
		for (auto&& element : range)
			minimum = std::min(minimum, MinElement(element));
		return minimum;
	}
	template <std::ranges::input_range Range> constexpr auto MaxElement(Range&& range) noexcept
	{
		// Note: if 'range' is a Vec4[], this will find the maximum element among all components of all vectors, not the maximum vector by component comparison.
		using value_t = std::ranges::range_value_t<Range>;
		using element_t = decltype(MaxElement(std::declval<value_t>()));

		element_t maximum = Min<element_t>();
		for (auto&& element : range)
			maximum = std::max(maximum, MaxElement(element));
		return maximum;
	}

	// Min/Max absolute element (i.e. nearest to 0/+inf)
	template <ScalarType S> constexpr S MinElementAbs(S v) noexcept
	{
		return Abs(v);
	}
	template <ScalarType S> constexpr S MaxElementAbs(S v) noexcept
	{
		return Abs(v);
	}
	template <TensorType Vec> constexpr typename vector_traits<Vec>::element_t pr_vectorcall MinElementAbs(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;
		auto res = Max<S>();
		if constexpr (vt::dimension > 0) res = Min(res, MinElementAbs(vec(v).x));
		if constexpr (vt::dimension > 1) res = Min(res, MinElementAbs(vec(v).y));
		if constexpr (vt::dimension > 2) res = Min(res, MinElementAbs(vec(v).z));
		if constexpr (vt::dimension > 3) res = Min(res, MinElementAbs(vec(v).w));
		return res;
	}
	template <TensorType Vec> constexpr typename vector_traits<Vec>::element_t pr_vectorcall MaxElementAbs(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;
		auto res = Min<S>();
		if constexpr (vt::dimension > 0) res = Max(res, MaxElementAbs(vec(v).x));
		if constexpr (vt::dimension > 1) res = Max(res, MaxElementAbs(vec(v).y));
		if constexpr (vt::dimension > 2) res = Max(res, MaxElementAbs(vec(v).z));
		if constexpr (vt::dimension > 3) res = Max(res, MaxElementAbs(vec(v).w));
		return res;
	}

	// Smallest/Largest element index. Returns the index of the first min/max element if elements are equal.
	template <TensorType Vec> constexpr int pr_vectorcall MinElementIndex(Vec v) noexcept requires (IsRank1<Vec>)
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		auto idx = 0;
		auto val = S{};
		if constexpr (vt::dimension > 0) idx = (val = vec(v).x, 0);
		if constexpr (vt::dimension > 1) idx = vec(v).y < val ? (val = vec(v).y, 1) : idx;
		if constexpr (vt::dimension > 2) idx = vec(v).z < val ? (val = vec(v).z, 2) : idx;
		if constexpr (vt::dimension > 3) idx = vec(v).w < val ? (val = vec(v).w, 3) : idx;
		return idx;
	}
	template <TensorType Vec> constexpr int pr_vectorcall MaxElementIndex(Vec v) noexcept requires (IsRank1<Vec>)
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		auto idx = 0;
		auto val = S{};
		if constexpr (vt::dimension > 0) idx = (val = vec(v).x, 0);
		if constexpr (vt::dimension > 1) idx = vec(v).y > val ? (val = vec(v).y, 1) : idx;
		if constexpr (vt::dimension > 2) idx = vec(v).z > val ? (val = vec(v).z, 2) : idx;
		if constexpr (vt::dimension > 3) idx = vec(v).w > val ? (val = vec(v).w, 3) : idx;
		return idx;
	}
	template <std::ranges::input_range Range> constexpr int MinElementIndex(Range&& range) noexcept
	{
		using value_t = std::ranges::range_value_t<Range>;
		using element_t = decltype(MinElement(std::declval<value_t>()));

		int idx = 0, min_idx = 0;
		element_t min_val = Max<element_t>();
		for (auto&& element : range)
		{
			auto val = MinElement(element);
			if (val < min_val)
			{
				min_val = val;
				min_idx = idx;
			}
			idx++;
		}
		return min_idx;
	}
	template <std::ranges::input_range Range> constexpr int MaxElementIndex(Range&& range) noexcept
	{
		using value_t = std::ranges::range_value_t<Range>;
		using element_t = decltype(MaxElement(std::declval<value_t>()));

		int idx = 0, max_idx = 0;
		element_t max_val = Min<element_t>();
		for (auto&& element : range)
		{
			auto val = MaxElement(element);
			if (val > max_val)
			{
				max_val = val;
				max_idx = idx;
			}
			idx++;
		}
		return max_idx;
	}

	// Floating point comparisons. *WARNING* 'tol' is an absolute tolerance. Returns true if 'a' is in the range (b-tol,b+tol)
	template <std::floating_point T> constexpr bool FEqlAbsolute(T a, T b, T tol) noexcept
	{
		// When float operations are performed at compile time, the compiler warnings about 'inf'
		pr_assert(tol >= 0 || !(tol == tol)); // NaN is not an error, comparisons with NaN are defined to always be false
		return Abs(a - b) < tol;
	}
	template <TensorTypeFP Vec> constexpr bool pr_vectorcall FEqlAbsolute(Vec lhs, Vec rhs, auto tol) noexcept
	{
		using vt = vector_traits<Vec>;
		bool eql = true;
		if constexpr (vt::dimension > 0) eql &= FEqlAbsolute(vec(lhs).x, vec(rhs).x, tol);
		if constexpr (vt::dimension > 1) eql &= FEqlAbsolute(vec(lhs).y, vec(rhs).y, tol);
		if constexpr (vt::dimension > 2) eql &= FEqlAbsolute(vec(lhs).z, vec(rhs).z, tol);
		if constexpr (vt::dimension > 3) eql &= FEqlAbsolute(vec(lhs).w, vec(rhs).w, tol);
		return eql;
	}

	// Floating point comparisons. *WARNING* 'tol' is a relative tolerance, relative to the largest of 'a' or 'b'
	template <std::floating_point T> constexpr bool FEqlRelative(T a, T b, T tol) noexcept
	{
		// Floating point compare is dangerous and subtle.
		// See: https://randomascii.wordpress.com/2012/02/25/comparing-floating-point-numbers-2012-edition/
		// and: http://floating-point-gui.de/errors/NearlyEqualsTest.java
		// Tests against zero treat 'tol' as an absolute difference threshold.
		// Tests between two non-zero values use 'tol' as a relative difference threshold.
		// i.e.
		//    FEql(2e-30, 1e-30) == false
		//    FEql(2e-30 - 1e-30, 0) == true

		// Handles tests against zero where relative error is meaningless
		// Tests with 'b == 0' are the most common so do them first
		if (b == 0) return Abs(a) < tol;
		if (a == 0) return Abs(b) < tol;

		// Handle infinities and exact values
		if (a == b) return true;

		// Test relative error as a fraction of the largest value
		auto abs_max_element = std::max(Abs(a), Abs(b));
		return FEqlAbsolute(a, b, tol * abs_max_element);
	}
	template <TensorTypeFP Vec> constexpr bool pr_vectorcall FEqlRelative(Vec lhs, Vec rhs, auto tol) noexcept
	{
		auto max_a = MaxElement(Abs(lhs));
		auto max_b = MaxElement(Abs(rhs));
		if (max_b == 0) return max_a < tol;
		if (max_a == 0) return max_b < tol;
		auto abs_max_element = std::max(max_a, max_b);
		return FEqlAbsolute(lhs, rhs, tol * abs_max_element);
	}

	// FEqlRelative using 'Tiny'. Returns true if a in the range (b - max(a,b)*tiny, b + max(a,b)*tiny)
	template <std::floating_point S> constexpr bool FEql(S a, S b) noexcept
	{
		// Don't add a 'tol' parameter because it looks like the function should perform a == b +- tol, which isn't what it does.
		return FEqlRelative(a, b, constants<S>::tiny);
	}
	template <TensorTypeFP Vec> constexpr bool pr_vectorcall FEql(Vec lhs, Vec rhs) noexcept
	{
		using S = typename vector_traits<Vec>::element_t;
		return FEqlRelative(lhs, rhs, constants<S>::tiny);
	}

	// Ceil/Floor/Round/Modulus
	template <ScalarTypeFP S> constexpr S Ceil(S x) noexcept
	{
		if consteval
		{
			return
				(x == +std::numeric_limits<S>::infinity()) ? x :
				(x == -std::numeric_limits<S>::infinity()) ? x :
				(x == x) ? static_cast<S>(std::ceil(x)) :
				std::numeric_limits<S>::quiet_NaN();
		}
		else
		{
			return static_cast<S>(std::ceil(x));
		}
	}
	template <ScalarTypeFP S> constexpr S Floor(S x) noexcept
	{
		if consteval
		{
			return
				(x == +std::numeric_limits<S>::infinity()) ? x :
				(x == -std::numeric_limits<S>::infinity()) ? x :
				(x == x) ? static_cast<S>(std::floor(x)) :
				std::numeric_limits<S>::quiet_NaN();
		}
		else
		{
			return static_cast<S>(std::floor(x));
		}
	}
	template <ScalarTypeFP S> constexpr S Round(S x) noexcept
	{
		if consteval
		{
			return
				(x == +std::numeric_limits<S>::infinity()) ? x :
				(x == -std::numeric_limits<S>::infinity()) ? x :
				(x == x) ? static_cast<S>(std::round(x)) :
				std::numeric_limits<S>::quiet_NaN();
		}
		else
		{
			return static_cast<S>(std::round(x));
		}
	}
	template <ScalarTypeFP S> constexpr S RoundSD(S d, int significant_digits) noexcept
	{
		pr_assert(significant_digits >= 0 && "'significant_digits' value must be >= 0");

		// No significant digits is always zero
		if (d == 0 || significant_digits == 0)
			return 0;

		if constexpr (std::is_same_v<S, long long>) // int64_t is 19 digits
		{
			if (significant_digits > 19)
				return d;
		}
		if constexpr (std::is_same_v<S, float>) // float's mantissa is 7 digits
		{
			if (significant_digits > 7)
				return d;
		}
		if constexpr (std::is_same_v<S, double>) // double's mantissa is 17 digits
		{
			if (significant_digits > 17)
				return d;
		}

		auto pow = static_cast<int>(std::floor(std::log10(std::abs(d))));
		auto scale = std::pow(S(10), significant_digits - pow - 1);
		auto result = scale != 0 ? static_cast<S>(Round<double>(d * scale) / scale) : S{};
		return result;
	}
	template <ScalarType S> constexpr S Modulus(S x, S y) noexcept
	{
		if constexpr (std::floating_point<S>)
			return std::fmod(x, y);
		else
			return x % y;
	}
	template <TensorTypeFP Vec> constexpr Vec pr_vectorcall Ceil(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Ceil(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = Ceil(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = Ceil(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = Ceil(vec(v).w);
		return res;
	}
	template <TensorTypeFP Vec> constexpr Vec pr_vectorcall Floor(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Floor(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = Floor(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = Floor(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = Floor(vec(v).w);
		return res;
	}
	template <TensorTypeFP Vec> constexpr Vec pr_vectorcall Round(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Round(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = Round(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = Round(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = Round(vec(v).w);
		return res;
	}
	template <TensorTypeFP Vec> constexpr Vec pr_vectorcall RoundSD(Vec v, int significant_digits) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = RoundSD(vec(v).x, significant_digits);
		if constexpr (vt::dimension > 1) vec(res).y = RoundSD(vec(v).y, significant_digits);
		if constexpr (vt::dimension > 2) vec(res).z = RoundSD(vec(v).z, significant_digits);
		if constexpr (vt::dimension > 3) vec(res).w = RoundSD(vec(v).w, significant_digits);
		return res;
	}
	template <TensorType Vec> constexpr Vec Modulus(Vec x, Vec y) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Modulus(vec(x).x, vec(y).x);
		if constexpr (vt::dimension > 1) vec(res).y = Modulus(vec(x).y, vec(y).y);
		if constexpr (vt::dimension > 2) vec(res).z = Modulus(vec(x).z, vec(y).z);
		if constexpr (vt::dimension > 3) vec(res).w = Modulus(vec(x).w, vec(y).w);
		return res;
	}

	// Wrap 'x' to range [mn, mx)
	template <ScalarType S> constexpr S Wrap(S x, S mn, S mx) noexcept
	{
		// Given the range ['mn', 'mx') and 'x' somewhere on the number line.
		// Return 'x' wrapped into the range, allowing for 'x' < 'mn'.
		auto range = mx - mn;
		return mn + Modulus((Modulus(x - mn, range) + range), range);
	}

	// Converts bool to +1,-1 (note: no 0 value)
	constexpr int Bool2SignI(bool positive) noexcept
	{
		return positive ? +1 : -1;
	}
	constexpr float Bool2SignF(bool positive) noexcept
	{
		return positive ? +1.0f : -1.0f;
	}

	// Sign, returns +1 if x >= 0 otherwise -1. If 'zero_is_positive' is false, then 0 in gives 0 out.
	template <ScalarType S> constexpr S Sign(S x, bool zero_is_positive = true) noexcept
	{
		if constexpr (std::is_unsigned_v<S>)
			return x > 0 ? +S(1) : static_cast<S>(zero_is_positive);
		else
			return x > 0 ? +S(1) : x < 0 ? -S(1) : static_cast<S>(zero_is_positive);
	}
	template <TensorType Vec> constexpr Vec pr_vectorcall Sign(Vec v, bool zero_is_positive = true) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Sign(vec(v).x, zero_is_positive);
		if constexpr (vt::dimension > 1) vec(res).y = Sign(vec(v).y, zero_is_positive);
		if constexpr (vt::dimension > 2) vec(res).z = Sign(vec(v).z, zero_is_positive);
		if constexpr (vt::dimension > 3) vec(res).w = Sign(vec(v).w, zero_is_positive);
		return res;
	}

	// Divide 'a' by 'b' if 'b' is not equal to zero, otherwise return 'def'
	template <typename T> constexpr T Div(T a, T b, T def = {}) noexcept requires (requires (T x) { x / x; x != x; })
	{
		return b != T{} ? a / b : def;
	}

	// Truncate value
	template <ScalarType S> constexpr S Trunc(S x, ETruncate trunc = ETruncate::TowardZero) noexcept requires (std::floating_point<S>)
	{
		switch (trunc)
		{
			case ETruncate::ToNearest:  return static_cast<S>(static_cast<long long>(x + Sign(x) * S(0.5)));
			case ETruncate::TowardZero: return static_cast<S>(static_cast<long long>(x));
			default: pr_assert("Unknown truncation type" && false); return x;
		}
	}
	template <VectorTypeFP Vec> constexpr Vec pr_vectorcall Trunc(Vec v, ETruncate trunc = ETruncate::TowardZero) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Trunc(vec(v).x, trunc);
		if constexpr (vt::dimension > 1) vec(res).y = Trunc(vec(v).y, trunc);
		if constexpr (vt::dimension > 2) vec(res).z = Trunc(vec(v).z, trunc);
		if constexpr (vt::dimension > 3) vec(res).w = Trunc(vec(v).w, trunc);
		return res;
	}

	// Fractional part
	template <ScalarType S> constexpr S Frac(S x) noexcept requires (std::floating_point<S>)
	{
		if consteval
		{
			if (x == +std::numeric_limits<S>::infinity() || x == -std::numeric_limits<S>::infinity() || !(x == x))
				return std::numeric_limits<S>::quiet_NaN();
			else
				return x - Floor(x);
		}
		else
		{
			S n;
			return std::modf(x, &n);
		}
	}
	template <VectorTypeFP Vec> constexpr Vec pr_vectorcall Frac(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Frac(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = Frac(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = Frac(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = Frac(vec(v).w);
		return res;
	}
	
	// Trigonometric/Hyperbolic functions and their inverses
	template <ScalarTypeFP S> inline S Sin(S x) noexcept
	{
		return static_cast<S>(std::sin(x));
	}
	template <ScalarTypeFP S> inline S Cos(S x) noexcept
	{
		return static_cast<S>(std::cos(x));
	}
	template <ScalarTypeFP S> inline S Tan(S x) noexcept
	{
		return static_cast<S>(std::tan(x));
	}
	template <ScalarTypeFP S> inline S Asin(S x) noexcept
	{
		return static_cast<S>(std::asin(x));
	}
	template <ScalarTypeFP S> inline S Acos(S x) noexcept
	{
		return static_cast<S>(std::acos(x));
	}
	template <ScalarTypeFP S> inline S Atan(S x) noexcept
	{
		return static_cast<S>(std::atan(x));
	}
	template <ScalarTypeFP S> inline S Atan2(S y, S x) noexcept
	{
		return static_cast<S>(std::atan2(y, x));
	}
	template <ScalarTypeFP S> inline S Sinh(S x) noexcept
	{
		return static_cast<S>(std::sinh(x));
	}
	template <ScalarTypeFP S> inline S Cosh(S x) noexcept
	{
		return static_cast<S>(std::cosh(x));
	}
	template <ScalarTypeFP S> inline S Tanh(S x) noexcept
	{
		return static_cast<S>(std::tanh(x));
	}
	template <ScalarTypeFP S> inline S Exp(S x) noexcept
	{
		return static_cast<S>(std::exp(x));
	}
	template <ScalarTypeFP S> inline S Log(S x) noexcept
	{
		return static_cast<S>(std::log(x));
	}
	template <ScalarTypeFP S> inline S Log10(S x) noexcept
	{
		return static_cast<S>(std::log10(x));
	}
	template <ScalarTypeFP S> inline S Pow(S x, S y) noexcept
	{
		return static_cast<S>(std::pow(x, y));
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Sin(Vec x) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Sin(vec(x).x);
		if constexpr (vt::dimension > 1) vec(res).y = Sin(vec(x).y);
		if constexpr (vt::dimension > 2) vec(res).z = Sin(vec(x).z);
		if constexpr (vt::dimension > 3) vec(res).w = Sin(vec(x).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Cos(Vec x) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Cos(vec(x).x);
		if constexpr (vt::dimension > 1) vec(res).y = Cos(vec(x).y);
		if constexpr (vt::dimension > 2) vec(res).z = Cos(vec(x).z);
		if constexpr (vt::dimension > 3) vec(res).w = Cos(vec(x).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Tan(Vec x) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Tan(vec(x).x);
		if constexpr (vt::dimension > 1) vec(res).y = Tan(vec(x).y);
		if constexpr (vt::dimension > 2) vec(res).z = Tan(vec(x).z);
		if constexpr (vt::dimension > 3) vec(res).w = Tan(vec(x).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Asin(Vec x) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Asin(vec(x).x);
		if constexpr (vt::dimension > 1) vec(res).y = Asin(vec(x).y);
		if constexpr (vt::dimension > 2) vec(res).z = Asin(vec(x).z);
		if constexpr (vt::dimension > 3) vec(res).w = Asin(vec(x).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Acos(Vec x) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Acos(vec(x).x);
		if constexpr (vt::dimension > 1) vec(res).y = Acos(vec(x).y);
		if constexpr (vt::dimension > 2) vec(res).z = Acos(vec(x).z);
		if constexpr (vt::dimension > 3) vec(res).w = Acos(vec(x).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Atan(Vec x) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Atan(vec(x).x);
		if constexpr (vt::dimension > 1) vec(res).y = Atan(vec(x).y);
		if constexpr (vt::dimension > 2) vec(res).z = Atan(vec(x).z);
		if constexpr (vt::dimension > 3) vec(res).w = Atan(vec(x).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Atan2(Vec y, Vec x) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Atan2(vec(y).x, vec(x).x);
		if constexpr (vt::dimension > 1) vec(res).y = Atan2(vec(y).y, vec(x).y);
		if constexpr (vt::dimension > 2) vec(res).z = Atan2(vec(y).z, vec(x).z);
		if constexpr (vt::dimension > 3) vec(res).w = Atan2(vec(y).w, vec(x).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Sinh(Vec x) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Sinh(vec(x).x);
		if constexpr (vt::dimension > 1) vec(res).y = Sinh(vec(x).y);
		if constexpr (vt::dimension > 2) vec(res).z = Sinh(vec(x).z);
		if constexpr (vt::dimension > 3) vec(res).w = Sinh(vec(x).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Cosh(Vec x) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Cosh(vec(x).x);
		if constexpr (vt::dimension > 1) vec(res).y = Cosh(vec(x).y);
		if constexpr (vt::dimension > 2) vec(res).z = Cosh(vec(x).z);
		if constexpr (vt::dimension > 3) vec(res).w = Cosh(vec(x).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Tanh(Vec x) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Tanh(vec(x).x);
		if constexpr (vt::dimension > 1) vec(res).y = Tanh(vec(x).y);
		if constexpr (vt::dimension > 2) vec(res).z = Tanh(vec(x).z);
		if constexpr (vt::dimension > 3) vec(res).w = Tanh(vec(x).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Exp(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Exp(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = Exp(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = Exp(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = Exp(vec(v).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Log(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Log(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = Log(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = Log(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = Log(vec(v).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Log10(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Log10(vec(v).x);
		if constexpr (vt::dimension > 1) vec(res).y = Log10(vec(v).y);
		if constexpr (vt::dimension > 2) vec(res).z = Log10(vec(v).z);
		if constexpr (vt::dimension > 3) vec(res).w = Log10(vec(v).w);
		return res;
	}
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Pow(Vec x, Vec y) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Pow(vec(x).x, vec(y).x);
		if constexpr (vt::dimension > 1) vec(res).y = Pow(vec(x).y, vec(y).y);
		if constexpr (vt::dimension > 2) vec(res).z = Pow(vec(x).z, vec(y).z);
		if constexpr (vt::dimension > 3) vec(res).w = Pow(vec(x).w, vec(y).w);
		return res;
	}

	// Raise 'x' to an integer power
	constexpr int Pow2(int n) noexcept
	{
		return 1 << n;
	}
	template <ScalarType S> constexpr S Pow(S x, int y) noexcept
	{
		return y == 0 ? 1 : x * Pow(x, y - 1);
	}
	template <TensorType Vec> constexpr Vec pr_vectorcall Pow(Vec v, int y) noexcept
	{
		using vt = vector_traits<Vec>;
		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Pow(vec(v).x, y);
		if constexpr (vt::dimension > 1) vec(res).y = Pow(vec(v).y, y);
		if constexpr (vt::dimension > 2) vec(res).z = Pow(vec(v).z, y);
		if constexpr (vt::dimension > 3) vec(res).w = Pow(vec(v).w, y);
		return res;
	}

	// atan2 that returns a positive angle in the range [0, tau)
	template <ScalarType S> inline S Atan2Positive(S y, S x) noexcept
	{
		auto a = std::atan2(y, x);
		if (a < 0) a += constants<S>::tau;
		return a;
	}

	// Convert degrees/radians
	template <ScalarType S> constexpr S DegreesToRadians(S degrees) noexcept
	{
		return static_cast<S>(degrees * constants<S>::tau / S(360));
	}
	template <ScalarType S> constexpr S RadiansToDegrees(S radians) noexcept
	{
		return static_cast<S>(radians * S(360) / constants<S>::tau);
	}
	
	// Return the normalised fraction that 'x' is, in the range ['min', 'max']
	template <ScalarTypeFP S> constexpr S Frac(S min, S x, S max) noexcept
	{
		pr_assert("Positive definite interval required for 'Frac'" && Abs(max - min) > 0);
		return (x - min) / (max - min);
	}
	template <VectorTypeFP Vec> constexpr Vec pr_vectorcall Frac(Vec min, Vec x, Vec max) noexcept
	{
		auto n = x - min;
		auto d = max - min;
		return n / d;
	}

	// Linearly interpolate from 'lhs' to 'rhs'
	template <ScalarTypeFP S> constexpr S Lerp(S lhs, S rhs, std::floating_point auto frac) noexcept
	{
		return static_cast<S>(lhs + frac * (rhs - lhs));
	}
	template <VectorTypeFP Vec> constexpr Vec pr_vectorcall Lerp(Vec lhs, Vec rhs, typename vector_traits<Vec>::element_t frac) noexcept
	{
		// Don't implement this for integral vector types, callers can just cast from FP to intg.
		return lhs + frac * (rhs - lhs);
	}

	// Spherical linear interpolation from 'a' to 'b' for t=[0,1]
	template <VectorTypeFP Vec> inline Vec pr_vectorcall Slerp(Vec a, Vec b, typename vector_traits<Vec>::element_t frac) noexcept
	{
		pr_assert("Cannot spherically interpolate to/from the zero vector" && a != Zero<Vec>() && b != Zero<Vec>());

		auto a_len = Length(a);
		auto b_len = Length(b);
		auto len = Lerp(a_len, b_len, frac);
		auto vec = Normalise(((1 - frac) / a_len) * a + (frac / b_len) * b);
		return len * vec;
	}

	// Quantise a value to a power of two. 'scale' should be a power of 2, i.e. 256, 1024, 2048, etc
	template <ScalarTypeFP S, std::integral I> constexpr S Quantise(S x, I scale) noexcept
	{
		// The purpose of 'Quantise' is to round 'x' to the nearest representable floating number using 'N' mantissa bits where '1 << N' == 'scale.
		return static_cast<I>(x * scale) / static_cast<S>(scale);
	}
	template <VectorTypeFP Vec, std::integral I> constexpr Vec pr_vectorcall Quantise(Vec v, I scale) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Quantise(vec(v).x, scale);
		if constexpr (vt::dimension > 1) vec(res).y = Quantise(vec(v).y, scale);
		if constexpr (vt::dimension > 2) vec(res).z = Quantise(vec(v).z, scale);
		if constexpr (vt::dimension > 3) vec(res).w = Quantise(vec(v).w, scale);
		return res;
	}

	// Return the cosine of the angle of the triangle apex opposite 'opp'
	template <ScalarTypeFP S> constexpr S CosAngle(S adj0, S adj1, S opp) noexcept
	{
		pr_assert("Angle undefined an when adjacent length is zero" && !FEql(adj0, S{}) && !FEql(adj1, S{}));
		return Clamp<S>((adj0*adj0 + adj1*adj1 - opp*opp) / (S(2) * adj0 * adj1), -S(1), +S(1));
	}

	// Return the cosine of the angle between two vectors
	template <VectorTypeFP Vec> requires (IsRank1<Vec>) inline typename vector_traits<Vec>::element_t pr_vectorcall CosAngle(Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;
		pr_assert("CosAngle undefined for zero vectors" && lhs != Vec{} && rhs != Vec{});
		return Clamp(Dot(lhs, rhs) / Sqrt(LengthSq(lhs) * LengthSq(rhs)), -S(1), +S(1));
	}

	// Return the angle (in radians) of the triangle apex opposite 'opp'
	template <ScalarTypeFP S> inline S Angle(S adj0, S adj1, S opp) noexcept
	{
		return std::acos(CosAngle(adj0, adj1, opp));
	}

	// Return the angle between two vectors
	template <VectorTypeFP Vec> requires (IsRank1<Vec>) inline typename vector_traits<Vec>::element_t pr_vectorcall Angle(Vec lhs, Vec rhs) noexcept
	{
		return std::acos(CosAngle(lhs, rhs));
	}

	// Return the length of a triangle side given by two adjacent side lengths and an angle between them
	template <ScalarTypeFP S> inline S Length(S adj0, S adj1, S angle) noexcept
	{
		auto len_sq = adj0*adj0 + adj1*adj1 - 2 * adj0 * adj1 * std::cos(angle);
		return len_sq > 0 ? Sqrt(len_sq) : 0;
	}

	// Returns 1 if 'hi' is > 'lo' otherwise 0
	template <ScalarType S> constexpr S Step(S lo, S hi) noexcept
	{
		return lo <= hi ? S(0) : S(1);
	}

	// Returns the 'Hermite' interpolation (3t^2 - 2t^3) between 'lo' and 'hi' for t=[0,1]
	template <ScalarTypeFP S> constexpr S SmoothStep(S lo, S hi, S t) noexcept
	{
		if (lo == hi) return lo;
		t = Clamp((t - lo) / (hi - lo), S(0), S(1));
		return t * t * (S(3) - S(2) * t);
	}

	// Returns a fifth-order 'Perlin' interpolation (6t^5 - 15t^4 + 10t^3) between 'lo' and 'hi' for t=[0,1]
	template <ScalarTypeFP S> constexpr S SmoothStep2(S lo, S hi, S t) noexcept
	{
		if (lo == hi) return lo;
		t = Clamp((t - lo) / (hi - lo), S(0), S(1));
		return static_cast<S>(t * t * t * (t * (t * S(6) - S(15)) + S(10)));
	}

	// Scale a value on the range [-inf,+inf] to within the range [-1,+1].
	template <ScalarTypeFP S> inline S Sigmoid(S x, S n = S(1)) noexcept
	{
		// 'n' is a horizontal scaling factor.
		// If n = 1, [-1,+1] maps to [-0.5, +0.5]
		// If n = 10, [-10,+10] maps to [-0.5, +0.5], etc
		return static_cast<S>(std::atan(x/n) / constants<S>::tau_by_4);
	}

	// Scale a value on the range [0,1] such that:' f(0) = 0, f(1) = 1, and df(0.5) = 0'
	template <ScalarTypeFP S> constexpr S UnitCubic(S x) noexcept
	{
		// This is used to weight values so that values near 0.5 are favoured
		return S(4) * Cube(x - S(0.5)) + S(0.5);
	}

	// Low precision reciprocal square root
	template <ScalarTypeFP S> inline S Rsqrt0(S x) noexcept
	{
		S r;
		if constexpr (PR_MATHS_USE_INTRINSICS && std::is_same_v<S, double>)
		{
			// todo
			//__m128d r0;
			//r0 = _mm_load_sd(&x);
			//r0 = _mm_rsqrt_sd(r0);
			//_mm_store_sd(&r, r0);
			r = S(1) / Sqrt(x);
		}
		else if constexpr (PR_MATHS_USE_INTRINSICS && std::is_same_v<S, float>)
		{
			__m128 r0;
			r0 = _mm_load_ss(&x);
			r0 = _mm_rsqrt_ss(r0);
			_mm_store_ss(&r, r0);
		}
		else
		{
			r = S(1) / Sqrt(x);
		}
		return r;
	}

	// High(er) precision reciprocal square root
	template <ScalarTypeFP S> inline S Rsqrt1(S x) noexcept
	{
		constexpr S c0 = +3.0;
		constexpr S c1 = -0.5;

		S r;
		if constexpr (PR_MATHS_USE_INTRINSICS && std::is_same_v<S, double>)
		{
			//todo
			//__m128d r0, r1;
			//r0 = _mm_load_sd(&x);
			//r1 = _mm_rsqrt_sd(r0);
			//r0 = _mm_mul_sd(r0, r1); // The general 'Newton-Raphson' reciprocal square root recurrence:
			//r0 = _mm_mul_sd(r0, r1); // (3 - b * X * X) * (X / 2)
			//r0 = _mm_sub_sd(r0, _mm_load_sd(&c0));
			//r1 = _mm_mul_sd(r1, _mm_load_sd(&c1));
			//r0 = _mm_mul_sd(r0, r1);
			//_mm_store_sd(&r, r0);
			r = S(1) / Sqrt(x);
		}
		else if constexpr (PR_MATHS_USE_INTRINSICS && std::is_same_v<S, float>)
		{
			__m128 r0, r1;
			r0 = _mm_load_ss(&x);
			r1 = _mm_rsqrt_ss(r0);
			r0 = _mm_mul_ss(r0, r1); // The general 'Newton-Raphson' reciprocal square root recurrence:
			r0 = _mm_mul_ss(r0, r1); // (3 - b * X * X) * (X / 2)
			r0 = _mm_sub_ss(r0, _mm_load_ss(&c0));
			r1 = _mm_mul_ss(r1, _mm_load_ss(&c1));
			r0 = _mm_mul_ss(r0, r1);
			_mm_store_ss(&r, r0);
		}
		else
		{
			r = S(1) / Sqrt(x);
		}
		return r;
	}

	// Cube root
	template <ScalarTypeFP S> inline S Cubert(S x) noexcept
	{
		// This works because the integer interpretation of an IEEE 754 float
		// is approximately the log2(x) scaled by 2^23. The basic idea is to
		// use the log2(x) value as the initial guess then do some 'Newton-Raphson'
		// iterations to find the actual root.
		
		if (x == 0)
			return x;

		auto flip_sign = x < 0;
		if (flip_sign) x = -x;
		
		if constexpr (std::is_same_v<S, float>)
		{
			union { float f; unsigned long i; } as;
			as.f = x;
			as.i = (as.i + 2U * 0x3f800000) / 3U;
			auto guess = as.f;

			x *= 1.0f / 3.0f;
			guess = (x / (guess * guess) + guess * (2.0f / 3.0f));
			guess = (x / (guess * guess) + guess * (2.0f / 3.0f));
			guess = (x / (guess * guess) + guess * (2.0f / 3.0f));
			return (flip_sign ? -guess : guess);
		}
		else
		{
			union { double f; unsigned long long i; } as;
			as.f = x;
			as.i = (as.i + 2ULL * 0x3FF0000000000000ULL) / 3ULL;
			auto guess = as.f;

			x *= 1.0 / 3.0;
			guess = (x / (guess * guess) + guess * (2.0 / 3.0));
			guess = (x / (guess * guess) + guess * (2.0 / 3.0));
			guess = (x / (guess * guess) + guess * (2.0 / 3.0));
			guess = (x / (guess * guess) + guess * (2.0 / 3.0));
			guess = (x / (guess * guess) + guess * (2.0 / 3.0));
			return (flip_sign ? -guess : guess);
		}
	}

	// Fast hash
	inline uint32_t Hash(float value, uint32_t max_value) noexcept
	{
		constexpr uint32_t h = 0x8da6b343; // Arbitrary prime
		int n = static_cast<int>(h * value);
		n = n % max_value;
		if (n < 0) n += max_value;
		return static_cast<uint32_t>(n);
	}
	template <VectorType Vec> inline uint32_t Hash(Vec v, uint32_t max_value) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		constexpr uint32_t h[] = {0x8da6b343, 0xd8163841, 0xcb1ab31f}; // Arbitrary Primes

		int n = 0;
		if constexpr (vt::dimension > 0) n += static_cast<int>(h[0 % _countof(h)] * vec(v).x);
		if constexpr (vt::dimension > 1) n += static_cast<int>(h[1 % _countof(h)] * vec(v).y);
		if constexpr (vt::dimension > 2) n += static_cast<int>(h[2 % _countof(h)] * vec(v).z);
		if constexpr (vt::dimension > 3) n += static_cast<int>(h[3 % _countof(h)] * vec(v).w);
		n = n % max_value;
		if (n < 0) n += max_value;
		return static_cast<uint32_t>(n);
	}

	// Return the greatest common factor between 'a' and 'b'
	template <std::integral S> constexpr S GreatestCommonFactor(S a, S b) noexcept
	{
		// Uses the Euclidean algorithm. If the greatest common factor is 1, then 'a' and 'b' are co-prime
		while (b) { auto t = b; b = a % b; a = t; }
		return a;
	}

	// Return the least common multiple between 'a' and 'b'
	template <std::integral S> constexpr S LeastCommonMultiple(S a, S b) noexcept
	{
		return (a * b) / GreatestCommonFactor(a, b);
	}

	// Convert a decimal back to a rational. Returns [numerator, denominator]
	template <ScalarTypeFP S> std::tuple<int, int> DecimalToRational(double num) noexcept
	{
		(void)num;
		// Algorithm:
		//  c = the number of digits that form the repeating part of 'num'
		//  l = the offset from the decimal point to the start of the repeating part of 'num'
		//  let d = 9[c times]0[l times]
		//     e.g. 0.1435282828... has c = 2 (28), and l = 4 (0.1435), so d = 990000
		//          0.125           has c = 0, and l = 3, so d = 1000
		//  let n = the digits right of the decimal point with the repeating part removed
		//     e.g. 0.1435282828... so n = 1435
		//          0.125           so n = 125
		//  Find the GCF between n and d to get the rational: n/d
		//
		// Explanation:
		//   0.123(45)... (45 repeating) equals 0.123 + (45/10000 + 45/1000000 + 45/100000000 + ...)
		//   which is 123/1000 + the sum of the geometric series with ratio = 1/100
		//     = 123/1000 + (45/10000).(1/(1-(1/100)))
		//     = 123/1000 + (45/10000).(1/99)
		//     = 123/1000 + (45/990000)
		//     = (99*123 + 45)/990000
		//     = 12222/990000

		// todo: implement this
		return { 0, 1 };
	}

	// Returns the number to add to pad 'size' up to 'alignment'
	template <std::integral S> constexpr S Pad(S size, int alignment) noexcept
	{
		pr_assert(((alignment - 1) & alignment) == 0 && "alignment should be a power of two");
		return static_cast<S>(~(size - 1) & (alignment - 1));
	}

	// Returns 'size' increased to a multiple of 'alignment'
	template <std::integral S> constexpr S PadTo(S size, int alignment) noexcept
	{
		return size + Pad<S>(size, alignment);
	}

	// An infinite range arithmetic sequence
	template <typename Type> auto ArithmeticSequence(Type initial_value, Type step) noexcept
	{
		// A sequence defined by:
		//  an = a0 + n * step
		//  Sn = (n + 1) * (a0 + an) / 2
		struct I
		{
			Type a0, step, n;
			I(Type initial_value, Type step) noexcept
				:a0(initial_value)
				,step(step)
				,n(0)
			{}
			Type operator*() const noexcept
			{
				return static_cast<Type>(a0 + n * step);
			}
			I& operator++() noexcept
			{
				++n;
				return *this;
			}
			bool operator!=(I const&) const noexcept
			{
				return true; // Infinite range
			}
		};
		struct R
		{
			Type a0, step;
			auto begin() const { return I{ a0, step}; }
			auto end() const { return I{0, 0}; }
		};
		return R{ initial_value, step };
	}
	
	// The sum of the first 'n' terms of an arithmetic sequence
	template <typename Type> constexpr Type ArithmeticSum(Type a0, Type step, int n) noexcept
	{
		auto an = a0 + n * step;
		return static_cast<Type>((n + 1) * (a0 + an) / 2);
	}

	// An infinite range geometric sequence
	template <typename Type> auto GeometricSequence(Type initial_value, Type ratio) noexcept
	{
		// A sequence defined by:
		//  an = am * r = a0 * r^n, (where m = n - 1)
		//  Sn = a0.(1 - r^(n+1)) / (1 - r)
		struct I
		{
			Type a0, ratio, n;
			I(Type initial_value, Type ratio) noexcept
				:a0(initial_value)
				,ratio(ratio)
				,n(0)
			{}
			Type operator*() const noexcept
			{
				return static_cast<Type>(a0 * Pow<double>(ratio, n));
			}
			I& operator++() noexcept
			{
				++n;
				return *this;
			}
			bool operator!=(I const&) const noexcept
			{
				return true; // Infinite range
			}
		};
		struct R
		{
			Type a0, ratio;
			auto begin() const { return I{ a0, ratio}; }
			auto end() const { return I{0, 0}; }
		};
		return R{ initial_value, ratio };
	};
	
	// The sum of the first 'n' terms of a geometric sequence
	template <typename Type> constexpr Type GeometricSum(Type a0, Type ratio, int n) noexcept
	{
		auto rn = Pow<double>(ratio, n + 1);
		return static_cast<Type>(a0 * (1 - rn) / (1 - ratio));
	}

	// Permutes the elements in 'arr' with each iteration. The first permutation is the ordered array; [0,N). Number of permutations == n!
	template <std::integral T> auto PermutationsOf(std::span<T> arr) noexcept
	{
		// Algorithm:
		// - find the last pair of values that has increasing order.
		// - swap the first of the pair with the next greater value from the values in [i+1,n)
		// - sort the values in [i+1,n)
		// e.g.
		//   Given '524761', the last pair with increasing order is '47'.
		//   Swap '4' with '6' because its the next greater value to the right of '4' => '526741'
		//   Sort the values right of '6' => '526147'

		struct I
		{
			std::span<T> m_arr;
			bool m_done;

			I(std::span<T> arr, bool done) noexcept
				:m_arr(arr)
				,m_done(done)
			{}
			std::span<T> operator*() const noexcept
			{
				return m_arr;
			}
			I& operator++() noexcept
			{
				int n = static_cast<int>(m_arr.size());

				// Find the last pair of values that has increasing order.
				int i = n - 1;
				for (; i-- > 0 && m_arr[i] > m_arr[i+1];) {}
				if (i == -1)
				{
					m_done = true;
					return *this;
				}

				// Swap 'arr[i]' with the nearest value greater than 'arr[i]'
				// to the right of 'i' then sort the values in the range: [i+1, n)
				int j = i + 1;
				for (int k = j + 1; k < n; ++k)
				{
					if (m_arr[k] < m_arr[i]) continue;
					if (m_arr[k] > m_arr[j]) continue;
					j = k;
				}
				std::swap(m_arr[i], m_arr[j]);
				std::sort(m_arr.data() + i + 1, m_arr.data() + n);
				return *this;
			}
			bool operator!=(I const& rhs) const noexcept
			{
				return m_done != rhs.m_done;
			}
		};
		struct R
		{
			std::span<T> m_arr;
			R(std::span<T> arr) :m_arr(arr) { std::sort(m_arr.begin(), m_arr.end()); }
			auto begin() const { return I{ m_arr, m_arr.empty() || m_arr.front() == m_arr.back() }; }
			auto end() const { return I{ m_arr, true }; }
		};
		return R{ arr };
	}

	// Vector dot product
	template <TensorType Vec> requires (IsRank1<Vec>)
	constexpr typename vector_traits<Vec>::element_t Dot(Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;
		auto product = S(0);
		if constexpr (vt::dimension > 0) product += vec(lhs).x * vec(rhs).x;
		if constexpr (vt::dimension > 1) product += vec(lhs).y * vec(rhs).y;
		if constexpr (vt::dimension > 2) product += vec(lhs).z * vec(rhs).z;
		if constexpr (vt::dimension > 3) product += vec(lhs).w * vec(rhs).w;
		return product;
	}
	template <TensorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension >= 3)
	constexpr typename vector_traits<Vec>::element_t Dot3(Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;
		auto product = S(0);
		product += vec(lhs).x * vec(rhs).x;
		product += vec(lhs).y * vec(rhs).y;
		product += vec(lhs).z * vec(rhs).z;
		return product;
	}

	// Vector cross product
	template <VectorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension == 2)
	constexpr typename vector_traits<Vec>::component_t Cross(Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		// Note: Sign flipped compared to the old library (pr::maths). Now uses the standard convention: a.x*b.y - a.y*b.x.
		return vec(lhs).x * vec(rhs).y - vec(lhs).y * vec(rhs).x; // 2D Cross product == Dot(Rotate90CW(lhs), rhs)
	}
	template <VectorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension == 3)
	constexpr Vec Cross(Vec lhs, Vec rhs) noexcept
	{
		using vt = vector_traits<Vec>;
		return Vec {
			vec(lhs).y * vec(rhs).z - vec(lhs).z * vec(rhs).y,
			vec(lhs).z * vec(rhs).x - vec(lhs).x * vec(rhs).z,
			vec(lhs).x * vec(rhs).y - vec(lhs).y * vec(rhs).x,
		};
	}
	template <VectorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension == 4)
	constexpr Vec Cross(Vec lhs, Vec rhs) noexcept
	{
		// Although this operates on 4D vectors, there is no such thing as a 4D cross product.
		// This function is provided for convenience when working with 4D homogeneous vectors
		// and since 4D cross is meaningless, there shouldn't be any confusion.
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		return Vec {
			vec(lhs).y * vec(rhs).z - vec(lhs).z * vec(rhs).y,
			vec(lhs).z * vec(rhs).x - vec(lhs).x * vec(rhs).z,
			vec(lhs).x * vec(rhs).y - vec(lhs).y * vec(rhs).x,
			S(0)
		};
	}

	// Vector triple product: a . b x c
	template <VectorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension >= 3)
	constexpr typename vector_traits<Vec>::element_t Triple(Vec a, Vec b, Vec c) noexcept
	{
		// Cross product is only defined for 3D vectors so Triple4 doesn't make sense.
		// Therefore, we don't need an overload for 4D vectors, Triple3 would be the same as Triple.
		return Dot(a, Cross(b, c));
	}

	// Squared Length of a vector
	template <TensorType Vec> requires (IsRank1<Vec>)
	constexpr typename vector_traits<Vec>::element_t LengthSq(Vec v) noexcept
	{
		return Dot(v, v);
	}

	// Length of a vector
	template <std::integral S> constexpr S Length(S x) noexcept
	{
		// Defined for use in recursive vector functions
		return Abs(x);
	}
	template <std::floating_point S> constexpr S Length(S x) noexcept
	{
		// Defined for use in recursive vector functions
		return Abs(x);
	}
	template <TensorType Vec> requires (IsRank1<Vec>)
	constexpr typename vector_traits<Vec>::element_t Length(Vec v) noexcept
	{
		using S = typename vector_traits<Vec>::element_t;
		return Sqrt<S>(LengthSq(v));
	}

	// Length of a list of parameters
	template <typename T, typename... A> constexpr T Len(T x, A&&... a) noexcept
	{
		if constexpr (sizeof...(A) == 0)
		{
			return Abs(x);
		}
		else
		{
			auto rest = Len(std::forward<A>(a)...);
			return Sqrt(Sqr(x) + Sqr(rest));
		}
	}

	// Return the trace of this matrix, 
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr typename vector_traits<Mat>::element_t pr_vectorcall Trace(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		S res = {};
		if constexpr (vt::dimension > 0) res += vec(vec(mat).x).x;
		if constexpr (vt::dimension > 1) res += vec(vec(mat).y).y;
		if constexpr (vt::dimension > 2) res += vec(vec(mat).z).z;
		if constexpr (vt::dimension > 3) res += vec(vec(mat).w).w;
		return res;
	}

	// Return the determinant of 'mat'
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr typename vector_traits<Mat>::element_t pr_vectorcall Determinant(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;

		// The Determinant function uses cofactor expansion along column 0.
		if constexpr (vt::dimension == 1)
		{
			return vec(mat).x;
		}
		else if constexpr (vt::dimension == 2)
		{
			return Cross(vec(mat).x, vec(mat).y);
		}
		else if constexpr (vt::dimension == 3)
		{
			return Triple(vec(mat).x, vec(mat).y, vec(mat).z);
		}
		else if constexpr (vt::dimension == 4)
		{
			auto c1 = (mat.z.z * mat.w.w) - (mat.z.w * mat.w.z);
			auto c2 = (mat.z.y * mat.w.w) - (mat.z.w * mat.w.y);
			auto c3 = (mat.z.y * mat.w.z) - (mat.z.z * mat.w.y);
			auto c4 = (mat.z.x * mat.w.w) - (mat.z.w * mat.w.x);
			auto c5 = (mat.z.x * mat.w.z) - (mat.z.z * mat.w.x);
			auto c6 = (mat.z.x * mat.w.y) - (mat.z.y * mat.w.x);
			return
				mat.x.x * (mat.y.y*c1 - mat.y.z*c2 + mat.y.w*c3) -
				mat.x.y * (mat.y.x*c1 - mat.y.z*c4 + mat.y.w*c5) +
				mat.x.z * (mat.y.x*c2 - mat.y.y*c4 + mat.y.w*c6) -
				mat.x.w * (mat.y.x*c3 - mat.y.y*c5 + mat.y.z*c6);
		}
	}

	// Return the 4x4 determinant of the affine transform 'mat'
	template <VectorType Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 4)
	constexpr typename vector_traits<Mat>::element_t pr_vectorcall DeterminantAffine(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;

		// Theres probably versions of this for 2x2 and 3x3 as well
		if constexpr (vt::dimension == 4)
		{
			pr_assert(IsAffine(mat) && "'mat' must be an affine transform to use this function");
			return
				(mat.x.x * mat.y.y * mat.z.z) +
				(mat.x.y * mat.y.z * mat.z.x) +
				(mat.x.z * mat.y.x * mat.z.y) -
				(mat.x.z * mat.y.y * mat.z.x) -
				(mat.x.y * mat.y.x * mat.z.z) -
				(mat.x.x * mat.y.z * mat.z.y);
		}
	}

	// Return a vector containing the diagonal elements of this matrix, 
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr typename vector_traits<Mat>::component_t pr_vectorcall Diagonal(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		using Vec = typename vt::component_t;

		Vec res = {};
		if constexpr (vt::dimension > 0) vec(res).x = vec(vec(mat).x).x;
		if constexpr (vt::dimension > 1) vec(res).y = vec(vec(mat).y).y;
		if constexpr (vt::dimension > 2) vec(res).z = vec(vec(mat).z).z;
		if constexpr (vt::dimension > 3) vec(res).w = vec(vec(mat).w).w;
		return res;
	}

	// Return the kernel of 'mat'
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr typename vector_traits<Mat>::component_t pr_vectorcall Kernel(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		using Vec = typename vt::component_t;

		if constexpr (vt::dimension == 2)
		{
			auto yx = vec(vec(mat).y).x, yy = vec(vec(mat).y).y;
			return Vec{
				+yy,
				-yx,
			};
		}
		else if constexpr (vt::dimension == 3)
		{
			auto yx = vec(vec(mat).y).x, yy = vec(vec(mat).y).y, yz = vec(vec(mat).y).z;
			auto zx = vec(vec(mat).z).x, zy = vec(vec(mat).z).y, zz = vec(vec(mat).z).z;
			return Vec{
				+yy * zz - yz * zy,
				-yx * zz + yz * zx,
				+yx * zy - yy * zx,
			};
		}
		else if constexpr (vt::dimension == 4)
		{
			auto yx = vec(vec(mat).y).x, yy = vec(vec(mat).y).y, yz = vec(vec(mat).y).z, yw = vec(vec(mat).y).w;
			auto zx = vec(vec(mat).z).x, zy = vec(vec(mat).z).y, zz = vec(vec(mat).z).z, zw = vec(vec(mat).z).w;
			auto wx = vec(vec(mat).w).x, wy = vec(vec(mat).w).y, wz = vec(vec(mat).w).z, ww = vec(vec(mat).w).w;
			return Vec{
				+yy * (zz * ww - zw * wz) - yz * (zy * ww - zw * wy) + yw * (zy * wz - zz * wy),
				-yx * (zz * ww - zw * wz) + yz * (zx * ww - zw * wx) - yw * (zx * wz - zz * wx),
				+yx * (zy * ww - zw * wy) - yy * (zx * ww - zw * wx) + yw * (zx * wy - zy * wx),
				-yx * (zy * wz - zz * wy) + yy * (zx * wz - zz * wx) - yz * (zx * wy - zy * wx),
			};
		}
	}

	// Normalise a vector
	template <TensorTypeFP Vec> requires (IsRank1<Vec>)
	constexpr Vec pr_vectorcall Normalise(Vec v) noexcept
	{
		return v / Length(v);
	}
	template <TensorTypeFP Vec> requires (IsRank1<Vec>)
	constexpr Vec pr_vectorcall Normalise(Vec v, Vec value_if_zero_length) noexcept
	{
		using S = typename vector_traits<Vec>::element_t;
		auto len = Length(v);
		return len > tiny<S> ? v / len : value_if_zero_length;
	}
	template <TensorTypeFP Vec, typename IfZeroFactory> requires (IsRank1<Vec> && requires (IfZeroFactory f) { { f() } -> std::convertible_to<Vec>; })
	constexpr Vec pr_vectorcall Normalise(Vec v, IfZeroFactory value_if_zero_length) noexcept
	{
		using S = typename vector_traits<Vec>::element_t;
		auto len = Length(v);
		return len > Tiny<S>() ? v / len : value_if_zero_length();
	}

	// Normalise the columns of a matrix returning the lengths prior to renormalising
	template <VectorTypeFP Mat> requires (IsRank2<Mat>)
	inline std::tuple<Mat, typename vector_traits<Mat>::component_t> Normalise(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;

		auto m = mat;
		auto scale = One<Vec>();
		if constexpr (vt::dimension > 0) { vec(scale).x = Length(vec(m).x); vec(m).x /= vec(scale).x; }
		if constexpr (vt::dimension > 1) { vec(scale).y = Length(vec(m).y); vec(m).y /= vec(scale).y; }
		if constexpr (vt::dimension > 2) { vec(scale).z = Length(vec(m).z); vec(m).z /= vec(scale).z; }
		if constexpr (vt::dimension > 3) { vec(scale).w = Length(vec(m).w); vec(m).w /= vec(scale).w; }
		return { m, scale };
	}

	// Return true if 'mat' is an orthonormal matrix
	template <TensorTypeFP Vec> requires (IsRank1<Vec>)
	constexpr bool pr_vectorcall IsNormalised(Vec v, typename vector_traits<Vec>::element_t tol = tiny<typename vector_traits<Vec>::element_t>) noexcept
	{
		using S = typename vector_traits<Vec>::element_t;
		return Abs(LengthSq(v) - S(1)) < tol;
	}

	// Return true if 'mat' is an orthogonal matrix
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr bool pr_vectorcall IsOrthogonal(Mat const& mat, typename vector_traits<Mat>::element_t tol = tiny<typename vector_traits<Mat>::element_t>) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		if constexpr (vt::dimension == 2)
		{
			return
				Abs(Dot(vec(mat).x, vec(mat).y)) <= tol;
		}
		else if constexpr (vt::dimension == 3)
		{
			return
				Abs(Dot(vec(mat).x, vec(mat).y)) <= tol &&
				Abs(Dot(vec(mat).x, vec(mat).z)) <= tol &&
				Abs(Dot(vec(mat).y, vec(mat).z)) <= tol;
		}
		else if constexpr (vt::dimension == 4)
		{
			return
				Abs(Dot(vec(mat).x, vec(mat).y)) <= tol &&
				Abs(Dot(vec(mat).x, vec(mat).z)) <= tol &&
				Abs(Dot(vec(mat).x, vec(mat).w)) <= tol &&
				Abs(Dot(vec(mat).y, vec(mat).z)) <= tol &&
				Abs(Dot(vec(mat).y, vec(mat).w)) <= tol &&
				Abs(Dot(vec(mat).z, vec(mat).w)) <= tol;
		}
	}

	// Return true if 'mat' is an orthonormal matrix
	template <VectorTypeFP Mat> requires (IsRank2<Mat>)
	constexpr bool pr_vectorcall IsOrthonormal(Mat const& mat, typename vector_traits<Mat>::element_t tol = tiny<typename vector_traits<Mat>::element_t>) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		if constexpr (vt::dimension >= 1) if (Abs(LengthSq(vec(mat).x) - S(1)) > tol) return false;
		if constexpr (vt::dimension >= 2) if (Abs(LengthSq(vec(mat).y) - S(1)) > tol) return false;
		if constexpr (vt::dimension >= 3) if (Abs(LengthSq(vec(mat).z) - S(1)) > tol) return false;
		if constexpr (vt::dimension == 2) if (Abs(Determinant(mat) - S(1)) > tol) return false;
		if constexpr (vt::dimension >= 3) if (Abs(Triple(vec(mat).x, vec(mat).y, vec(mat).z) - S(1)) > tol) return false;
		return true;
	}

	// Return true if 'mat' is an affine transform
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension >= 3)
	constexpr bool pr_vectorcall IsAffine(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		if constexpr (vt::dimension >= 3)
		{
			if (vec(vec(mat).x).w != S(0)) return false;
			if (vec(vec(mat).y).w != S(0)) return false;
			if (vec(vec(mat).z).w != S(0)) return false;
		}
		if constexpr (vt::dimension >= 4)
		{
			if (vec(vec(mat).w).w != S(1)) return false;
		}
		return true;
	}

	// True if 'mat' can be inverted
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr bool pr_vectorcall IsInvertible(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		return Determinant(mat) != S(0);
	}

	// True if 'mat' is symmetric
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr bool pr_vectorcall IsSymmetric(Mat const& mat, typename vector_traits<Mat>::element_t tol = tiny<typename vector_traits<Mat>::element_t>) noexcept
	{
		using vt = vector_traits<Mat>;

		if constexpr (vt::dimension >= 2)
		{
			if (Abs(vec(vec(mat).x).y - vec(vec(mat).y).x) > tol)
				return false;
		}
		if constexpr (vt::dimension >= 3)
		{
			if (Abs(vec(vec(mat).x).z - vec(vec(mat).z).x) > tol ||
				Abs(vec(vec(mat).y).z - vec(vec(mat).z).y) > tol)
				return false;
		}
		if constexpr (vt::dimension >= 4)
		{
			if (Abs(vec(vec(mat).x).w - vec(vec(mat).w).x) > tol ||
				Abs(vec(vec(mat).y).w - vec(vec(mat).w).y) > tol ||
				Abs(vec(vec(mat).z).w - vec(vec(mat).w).z) > tol)
				return false;
		}
		return true;
	}

	// True if 'mat' is anti-symmetric
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr bool pr_vectorcall IsAntiSymmetric(Mat const& mat, typename vector_traits<Mat>::element_t tol = tiny<typename vector_traits<Mat>::element_t>) noexcept
	{
		using vt = vector_traits<Mat>;

		if constexpr (vt::dimension >= 2)
		{
			if (Abs(vec(vec(mat).x).y + vec(vec(mat).y).x) > tol)
				return false;
		}
		if constexpr (vt::dimension >= 3)
		{
			if (Abs(vec(vec(mat).x).z + vec(vec(mat).z).x) > tol ||
				Abs(vec(vec(mat).y).z + vec(vec(mat).z).y) > tol)
				return false;
		}
		if constexpr (vt::dimension >= 4)
		{
			if (Abs(vec(vec(mat).x).w + vec(vec(mat).w).x) > tol ||
				Abs(vec(vec(mat).y).w + vec(vec(mat).w).y) > tol ||
				Abs(vec(vec(mat).z).w + vec(vec(mat).w).z) > tol)
				return false;
		}
		return true;
	}

	// Returns true if 'a' and 'b' are parallel
	template <VectorType Vec> requires (IsRank1<Vec>)
	constexpr bool pr_vectorcall IsParallel(Vec v0, Vec v1, typename vector_traits<Vec>::element_t tol = tiny<typename vector_traits<Vec>::element_t>) noexcept
	{
		using vt = vector_traits<Vec>;

		// 2D cross product is a scalar
		if constexpr (vt::dimension == 2)
			return Sqr(Cross(v0, v1)) <= Sqr(tol);

		// 3D cross product returns a vector
		else if constexpr (vt::dimension == 3)
			return LengthSq(Cross(v0, v1)) <= Sqr(tol);

		// 4D uses Cross (ignores w)
		else
			return LengthSq(Cross(v0, v1)) <= Sqr(tol);
	}

	// Returns a vector guaranteed not parallel to 'v'
	template <VectorType Vec> requires (IsRank1<Vec>)
	constexpr Vec pr_vectorcall CreateNotParallelTo(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		if constexpr (vt::dimension < 3)
		{
			return Vec{ -vec(v).y, vec(v).x };
		}
		else
		{
			bool x_aligned = Abs(vec(v).x) > Abs(vec(v).y) && Abs(vec(v).x) > Abs(vec(v).z);
			return Vec{ static_cast<S>(!x_aligned), S(0), static_cast<S>(x_aligned) };
		}
	}

	// Rotate a 2D vector by 90deg CW (when at the origin, looking in the direction of +Z)
	// CW rotates +X toward +Y. Cross2D(a,b) == Dot(Rotate90CW(a), b)
	template <VectorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension == 2)
	constexpr Vec pr_vectorcall Rotate90CW(Vec v) noexcept
	{
		return Vec{ -vec(v).y, +vec(v).x };
	}

	// Rotate a 2D vector by 90deg CCW (when at the origin, looking in the direction of +Z)
	// CCW rotates +X toward -Y.
	template <VectorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension == 2)
	constexpr Vec pr_vectorcall Rotate90CCW(Vec v) noexcept
	{
		return Vec{ +vec(v).y, -vec(v).x };
	}

	// Returns a vector perpendicular to 'v' with the same length of 'v'
	template <VectorType Vec> requires (IsRank1<Vec>)
	constexpr Vec pr_vectorcall Perpendicular(Vec vec) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;
		pr_assert(vec != Zero<Vec>() && "Cannot make a perpendicular to a zero vector");

		Vec v;
		if constexpr (vt::dimension == 2)
		{
			v = Rotate90CCW(vec);
		}
		if constexpr (vt::dimension >= 3)
		{
			v = Cross(vec, CreateNotParallelTo(vec));
			v *= Sqrt(LengthSq(vec) / LengthSq(v));
		}
		return v;
	}

	// Returns a vector perpendicular to 'vec' favouring 'previous' as the preferred perpendicular.
	template <VectorType Vec> requires (IsRank1<Vec>)
	constexpr Vec pr_vectorcall Perpendicular(Vec vec, Vec previous) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		// The length of the returned vector will be 'Length(vec)' or 'Length(previous)' (typically they'd be the same)
		// Either 'vec' or 'previous' can be zero, but not both.
		if (vec == Zero<Vec>())
		{
			// Both 'vec' and 'previous' cannot be zero
			pr_assert(previous != Zero<Vec>() && "Cannot make a perpendicular to a zero vector");
			return previous;
		}
		if (IsParallel(vec, previous)) // includes 'previous' == zero
		{
			// If 'previous' is parallel to 'vec', choose a new perpendicular
			return Perpendicular(vec);
		}

		// If 'previous' is still perpendicular, reuse it
		S dot_vp = Dot(vec, previous);
		if (Abs(dot_vp) < tiny<S>)
			return previous;

		// Otherwise, make a perpendicular that is close to 'previous'
		Vec v = {};
		if constexpr (vt::dimension == 2)
		{
			// Cross2D > 0 means 'previous' is on the CW side of 'vec'
			v = Cross(vec, previous) >= S(0) ? Rotate90CW(vec) : Rotate90CCW(vec);
		}
		else
		{
			v = Cross(Cross(vec, previous), vec);
			if constexpr (std::floating_point<S>) v = Normalise(v);
		}
		return v;
	}

	// Permute the values in a vector. Rolls <--. e.g. 0:xyzw, 1:yzwx, 2:zwxy, etc
	template <VectorType Vec> requires (IsRank1<Vec>)
	constexpr Vec pr_vectorcall Permute(Vec v, int n) noexcept
	{
		using vt = vector_traits<Vec>;

		// Negative 'n' rolls in the opposite direction. e.g. -1: wxyz, -2: zwxy, etc
		if (n < 0)
			n = vt::dimension + (n % vt::dimension);

		if constexpr (vt::dimension == 2)
		{
			switch (n % 2)
			{
				case 0:  return v;
				case 1:  return Vec{ vec(v).y, vec(v).x };
			}
		}
		if constexpr (vt::dimension == 3)
		{
			switch (n % 3)
			{
				case 0:  return v;
				case 1:  return Vec{ vec(v).y, vec(v).z, vec(v).x };
				case 2:  return Vec{ vec(v).z, vec(v).x, vec(v).y };
			}
		}
		if constexpr (vt::dimension == 4)
		{
			switch (n % 4)
			{
				case 0:  return v;
				case 1:  return Vec{ vec(v).y, vec(v).z, vec(v).w, vec(v).x };
				case 2:  return Vec{ vec(v).z, vec(v).w, vec(v).x, vec(v).y };
				case 3:  return Vec{ vec(v).w, vec(v).x, vec(v).y, vec(v).z };
			}
		}
		pr_assert(false && "Invalid vector dimension");
	}
	
	// Permute the vectors in a matrix by 'n'. Rolls <--. e.g. 0:xyzw, 1:yzwx, 2:zwxy, etc
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr Mat pr_vectorcall Permute(Mat mat, int n) noexcept
	{
		// Rolls the vectors around, last becomes first
		using vt = vector_traits<Mat>;

		// Negative 'n' rolls in the opposite direction. e.g. -1: wxyz, -2: zwxy, etc
		if (n < 0)
			n = vt::dimension + (n % vt::dimension);

		if constexpr (vt::dimension == 2)
		{
			switch (n % 2)
			{
				case 0: return mat;
				case 1: return Mat{ vec(mat).y, vec(mat).x };
				default: return mat; // unreachable
			};
		}
		if constexpr (vt::dimension == 3)
		{
			switch (n % 3)
			{
				case 0: return mat;
				case 1: return Mat{ vec(mat).y, vec(mat).z, vec(mat).x };
				case 2: return Mat{ vec(mat).z, vec(mat).x, vec(mat).y };
				default: return mat; // unreachable
			};
		}
		if constexpr (vt::dimension == 4)
		{
			switch (n % 4)
			{
				case 0: return mat;
				case 1: return Mat{ vec(mat).y, vec(mat).z, vec(mat).w, vec(mat).x };
				case 2: return Mat{ vec(mat).z, vec(mat).w, vec(mat).x, vec(mat).y };
				case 3: return Mat{ vec(mat).w, vec(mat).x, vec(mat).y, vec(mat).z };
				default: return mat; // unreachable
			};
		}
	}

	// Returns a n-bit bitmask of the orthant that the vector is in. 0=(-x,-y,-z), 1=(+x,-y,-z), 2=(-x,+y,-z), 3=(+x,+y,-z), 4=(-x,-y+z), 5=(+x,-y,+z), 6=(-x,+y,+z), 7=(+x,+y,+z)
	template <VectorType Vec> requires (IsRank1<Vec>)
	constexpr uint32_t Orthant(Vec v) noexcept // Octant, Quadrant, etc depending on the dimension of 'Vec'
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;

		uint32_t mask = 0;
		if constexpr (vt::dimension > 0) mask |= (vec(v).x >= S(0)) << 0;
		if constexpr (vt::dimension > 1) mask |= (vec(v).y >= S(0)) << 1;
		if constexpr (vt::dimension > 2) mask |= (vec(v).z >= S(0)) << 2;
		if constexpr (vt::dimension > 3) mask |= (vec(v).w >= S(0)) << 3;
		return mask;
	}

	// Return the transpose of 'mat'
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr Mat pr_vectorcall Transpose(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;

		Mat m = mat;
		if constexpr (vector_traits<Mat>::dimension >= 2)
		{
			std::swap(vec(vec(m).x).y, vec(vec(m).y).x);
		}
		if constexpr (vector_traits<Mat>::dimension >= 3)
		{
			std::swap(vec(vec(m).x).z, vec(vec(m).z).x);
			std::swap(vec(vec(m).y).z, vec(vec(m).z).y);
		}
		if constexpr (vector_traits<Mat>::dimension >= 4)
		{
			std::swap(vec(vec(m).x).w, vec(vec(m).w).x);
			std::swap(vec(vec(m).y).w, vec(vec(m).w).y);
			std::swap(vec(vec(m).z).w, vec(vec(m).w).z);
		}
		return m;
	}
	template <VectorType Mat> requires (IsRank2<Mat>&& vector_traits<Mat>::dimension >= 3)
	constexpr Mat pr_vectorcall Transpose3x3(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;

		Mat m = mat;
		std::swap(vec(vec(m).x).y, vec(vec(m).y).x);
		std::swap(vec(vec(m).x).z, vec(vec(m).z).x);
		std::swap(vec(vec(m).y).z, vec(vec(m).z).y);
		return m;
	}

	// Return the inverse of 'mat' (assuming an orthonormal matrix)
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 4)
	constexpr Mat pr_vectorcall InvertOrthonormal(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		pr_assert(IsOrthonormal(mat) && "Matrix is not orthonormal");

		Mat m = mat;
		if constexpr (vector_traits<Mat>::dimension >= 2)
		{
			std::swap(vec(vec(m).x).y, vec(vec(m).y).x);
		}
		if constexpr (vector_traits<Mat>::dimension >= 3)
		{
			std::swap(vec(vec(m).x).z, vec(vec(m).z).x);
			std::swap(vec(vec(m).y).z, vec(vec(m).z).y);
		}
		if constexpr (vector_traits<Mat>::dimension >= 4)
		{
			vec(vec(m).w).x = -Dot(vec(mat).x, vec(mat).w);
			vec(vec(m).w).y = -Dot(vec(mat).y, vec(mat).w);
			vec(vec(m).w).z = -Dot(vec(mat).z, vec(mat).w);
		}
		return m;
	}

	// Invert the orthonormal matrix 'mat'
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension >= 3)
	constexpr Mat pr_vectorcall InvertAffine(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;
		pr_assert(IsAffine(mat) && "Matrix is not affine");

		Mat m = mat;

		auto scale = Vec{ LengthSq(vec(mat).x), LengthSq(vec(mat).y), LengthSq(vec(mat).z) };
		if (!FEql(scale, One<Vec>()))
		{
			pr_assert(vec(scale).x != 0 && vec(scale).y != 0 && vec(scale).z != 0 && "Cannot invert a degenerate matrix");
			scale = CompSqrt(scale);
		}

		// Remove scale
		vec(m).x /= vec(scale).x;
		vec(m).y /= vec(scale).y;
		vec(m).z /= vec(scale).z;

		// Invert rotation
		m = Transpose3x3(m);

		// Invert scale
		vec(m).x /= vec(scale).x;
		vec(m).y /= vec(scale).y;
		vec(m).z /= vec(scale).z;

		// Invert translation
		if constexpr (vt::dimension == 4)
		{
			vec(m).w = Vec{
				-(vec(vec(mat).w).x * vec(vec(m).x).x + vec(vec(mat).w).y * vec(vec(m).y).x + vec(vec(mat).w).z * vec(vec(m).z).x),
				-(vec(vec(mat).w).x * vec(vec(m).x).y + vec(vec(mat).w).y * vec(vec(m).y).y + vec(vec(mat).w).z * vec(vec(m).z).y),
				-(vec(vec(mat).w).x * vec(vec(m).x).z + vec(vec(mat).w).y * vec(vec(m).y).z + vec(vec(mat).w).z * vec(vec(m).z).z),
				S(1)
			};
		}

		return m;
	}

	// Return the inverse of 'mat'
	template <VectorTypeFP Mat> requires (IsRank2<Mat>)
	constexpr Mat pr_vectorcall Invert(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;

		if constexpr (vt::dimension == 2)
		{
			Mat inv = {};
			vec(inv).x = Vec{ +vec(vec(mat).y).y, -vec(vec(mat).y).x };
			vec(inv).y = Vec{ -vec(vec(mat).x).y, +vec(vec(mat).x).x };

			auto det = Determinant(mat);
			pr_assert(det != S(0) && "Matrix has no inverse");
			auto scale = S(1) / det;
			vec(inv).x *= scale;
			vec(inv).y *= scale;
			return inv;
		}
		else if constexpr (vt::dimension == 3)
		{
			Mat inv = {};
			vec(inv).x = Cross(vec(mat).y, vec(mat).z);
			vec(inv).y = Cross(vec(mat).z, vec(mat).x);
			vec(inv).z = Cross(vec(mat).x, vec(mat).y);
			inv = Transpose(inv);
			
			auto det = Determinant(mat);
			pr_assert(det != S(0) && "Matrix has no inverse");
			auto scale = S(1) / det;
			vec(inv).x *= scale;
			vec(inv).y *= scale;
			vec(inv).z *= scale;
			return inv;
		}
		else if constexpr (vt::dimension == 4)
		{
			S xx = vec(vec(mat).x).x, xy = vec(vec(mat).x).y, xz = vec(vec(mat).x).z, xw = vec(vec(mat).x).w;
			S yx = vec(vec(mat).y).x, yy = vec(vec(mat).y).y, yz = vec(vec(mat).y).z, yw = vec(vec(mat).y).w;
			S zx = vec(vec(mat).z).x, zy = vec(vec(mat).z).y, zz = vec(vec(mat).z).z, zw = vec(vec(mat).z).w;
			S wx = vec(vec(mat).w).x, wy = vec(vec(mat).w).y, wz = vec(vec(mat).w).z, ww = vec(vec(mat).w).w;

			Mat inv = {};
			vec(inv).x = Vec{
				+yy * zz * ww - yy * zw * wz - zy * yz * ww + zy * yw * wz + wy * yz * zw - wy * yw * zz,
				-xy * zz * ww + xy * zw * wz + zy * xz * ww - zy * xw * wz - wy * xz * zw + wy * xw * zz,
				+xy * yz * ww - xy * yw * wz - yy * xz * ww + yy * xw * wz + wy * xz * yw - wy * xw * yz,
				-xy * yz * zw + xy * yw * zz + yy * xz * zw - yy * xw * zz - zy * xz * yw + zy * xw * yz };
			vec(inv).y = Vec{
				-yx * zz * ww + yx * zw * wz + zx * yz * ww - zx * yw * wz - wx * yz * zw + wx * yw * zz,
				+xx * zz * ww - xx * zw * wz - zx * xz * ww + zx * xw * wz + wx * xz * zw - wx * xw * zz,
				-xx * yz * ww + xx * yw * wz + yx * xz * ww - yx * xw * wz - wx * xz * yw + wx * xw * yz,
				+xx * yz * zw - xx * yw * zz - yx * xz * zw + yx * xw * zz + zx * xz * yw - zx * xw * yz };
			vec(inv).z = Vec{
				+yx * zy * ww - yx * zw * wy - zx * yy * ww + zx * yw * wy + wx * yy * zw - wx * yw * zy,
				-xx * zy * ww + xx * zw * wy + zx * xy * ww - zx * xw * wy - wx * xy * zw + wx * xw * zy,
				+xx * yy * ww - xx * yw * wy - yx * xy * ww + yx * xw * wy + wx * xy * yw - wx * xw * yy,
				-xx * yy * zw + xx * yw * zy + yx * xy * zw - yx * xw * zy - zx * xy * yw + zx * xw * yy };
			vec(inv).w = Vec{
				-yx * zy * wz + yx * zz * wy + zx * yy * wz - zx * yz * wy - wx * yy * zz + wx * yz * zy,
				+xx * zy * wz - xx * zz * wy - zx * xy * wz + zx * xz * wy + wx * xy * zz - wx * xz * zy,
				-xx * yy * wz + xx * yz * wy + yx * xy * wz - yx * xz * wy - wx * xy * yz + wx * xz * yy,
				+xx * yy * zz - xx * yz * zy - yx * xy * zz + yx * xz * zy + zx * xy * yz - zx * xz * yy };

			auto det = xx * vec(vec(inv).x).x + xy * vec(vec(inv).y).x + xz * vec(vec(inv).z).x + xw * vec(vec(inv).w).x;
			pr_assert(det != S(0) && "matrix has no inverse");
			auto scale = S(1) / det;
			vec(inv).x *= scale;
			vec(inv).y *= scale;
			vec(inv).z *= scale;
			vec(inv).w *= scale;
			return inv;

			// Reference invert 4x4 implementation
			if constexpr (false)
			{
				auto mA = Transpose(mat); // Take the transpose so that row operations are faster
				auto mB = Identity<Mat>();

				// Loop through columns of 'A'
				for (int j = 0; j != 4; ++j)
				{
					// Select the pivot element: maximum magnitude in this row
					auto pivot = 0; auto val = S(0);
					if (j <= 0 && val < Abs(mA.x[j])) { pivot = 0; val = Abs(mA.x[j]); }
					if (j <= 1 && val < Abs(mA.y[j])) { pivot = 1; val = Abs(mA.y[j]); }
					if (j <= 2 && val < Abs(mA.z[j])) { pivot = 2; val = Abs(mA.z[j]); }
					if (j <= 3 && val < Abs(mA.w[j])) { pivot = 3; val = Abs(mA.w[j]); }
					if (val < tiny<S>)
					{
						pr_assert(false && "Matrix has no inverse");
						return mat;
					}

					// Interchange rows to put pivot element on the diagonal
					if (pivot != j) // skip if already on diagonal
					{
						std::swap(mA[j], mA[pivot]);
						std::swap(mB[j], mB[pivot]);
					}

					// Divide row by pivot element. Pivot element becomes 1
					auto scale = mA[j][j];
					mA[j] /= scale;
					mB[j] /= scale;

					// Subtract this row from others to make the rest of column j zero
					if (j != 0) { scale = mA.x[j]; mA.x -= scale * mA[j]; mB.x -= scale * mB[j]; }
					if (j != 1) { scale = mA.y[j]; mA.y -= scale * mA[j]; mB.y -= scale * mB[j]; }
					if (j != 2) { scale = mA.z[j]; mA.z -= scale * mA[j]; mB.z -= scale * mB[j]; }
					if (j != 3) { scale = mA.w[j]; mA.w -= scale * mA[j]; mB.w -= scale * mB[j]; }
				}

				// When these operations have been completed, A should have been transformed to the identity matrix
				// and B should have been transformed into the inverse of the original A
				mB = Transpose(mB);
				return mB;
			}
		}
	}

	// Return the inverse of 'mat' using double precision floats
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 4)
	constexpr Mat pr_vectorcall InvertPrecise(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;

		auto xx = vec(vec(mat).x).x, xy = vec(vec(mat).x).y, xz = vec(vec(mat).x).z, xw = vec(vec(mat).x).w;
		auto yx = vec(vec(mat).y).x, yy = vec(vec(mat).y).y, yz = vec(vec(mat).y).z, yw = vec(vec(mat).y).w;
		auto zx = vec(vec(mat).z).x, zy = vec(vec(mat).z).y, zz = vec(vec(mat).z).z, zw = vec(vec(mat).z).w;
		auto wx = vec(vec(mat).w).x, wy = vec(vec(mat).w).y, wz = vec(vec(mat).w).z, ww = vec(vec(mat).w).w;

		double inv[4][4];
		inv[0][0] = 0.0 + yy * zz * ww - yy * zw * wz - zy * yz * ww + zy * yw * wz + wy * yz * zw - wy * yw * zz;
		inv[0][1] = 0.0 - xy * zz * ww + xy * zw * wz + zy * xz * ww - zy * xw * wz - wy * xz * zw + wy * xw * zz;
		inv[0][2] = 0.0 + xy * yz * ww - xy * yw * wz - yy * xz * ww + yy * xw * wz + wy * xz * yw - wy * xw * yz;
		inv[0][3] = 0.0 - xy * yz * zw + xy * yw * zz + yy * xz * zw - yy * xw * zz - zy * xz * yw + zy * xw * yz;
		inv[1][0] = 0.0 - yx * zz * ww + yx * zw * wz + zx * yz * ww - zx * yw * wz - wx * yz * zw + wx * yw * zz;
		inv[1][1] = 0.0 + xx * zz * ww - xx * zw * wz - zx * xz * ww + zx * xw * wz + wx * xz * zw - wx * xw * zz;
		inv[1][2] = 0.0 - xx * yz * ww + xx * yw * wz + yx * xz * ww - yx * xw * wz - wx * xz * yw + wx * xw * yz;
		inv[1][3] = 0.0 + xx * yz * zw - xx * yw * zz - yx * xz * zw + yx * xw * zz + zx * xz * yw - zx * xw * yz;
		inv[2][0] = 0.0 + yx * zy * ww - yx * zw * wy - zx * yy * ww + zx * yw * wy + wx * yy * zw - wx * yw * zy;
		inv[2][1] = 0.0 - xx * zy * ww + xx * zw * wy + zx * xy * ww - zx * xw * wy - wx * xy * zw + wx * xw * zy;
		inv[2][2] = 0.0 + xx * yy * ww - xx * yw * wy - yx * xy * ww + yx * xw * wy + wx * xy * yw - wx * xw * yy;
		inv[2][3] = 0.0 - xx * yy * zw + xx * yw * zy + yx * xy * zw - yx * xw * zy - zx * xy * yw + zx * xw * yy;
		inv[3][0] = 0.0 - yx * zy * wz + yx * zz * wy + zx * yy * wz - zx * yz * wy - wx * yy * zz + wx * yz * zy;
		inv[3][1] = 0.0 + xx * zy * wz - xx * zz * wy - zx * xy * wz + zx * xz * wy + wx * xy * zz - wx * xz * zy;
		inv[3][2] = 0.0 - xx * yy * wz + xx * yz * wy + yx * xy * wz - yx * xz * wy - wx * xy * yz + wx * xz * yy;
		inv[3][3] = 0.0 + xx * yy * zz - xx * yz * zy - yx * xy * zz + yx * xz * zy + zx * xy * yz - zx * xz * yy;

		auto det = xx * inv[0][0] + xy * inv[1][0] + xz * inv[2][0] + xw * inv[3][0];
		pr_assert(det != 0.0 && "matrix has no inverse");

		Mat m;
		vec(m).x = Vec(S(inv[0][0]), S(inv[0][1]), S(inv[0][2]), S(inv[0][3]));
		vec(m).y = Vec(S(inv[1][0]), S(inv[1][1]), S(inv[1][2]), S(inv[1][3]));
		vec(m).z = Vec(S(inv[2][0]), S(inv[2][1]), S(inv[2][2]), S(inv[2][3]));
		vec(m).w = Vec(S(inv[3][0]), S(inv[3][1]), S(inv[3][2]), S(inv[3][3]));
		m /= S(det);
		return m;
	}

	// Return the square root of a matrix. The square root is the matrix B where B.B = mat.
	template <VectorTypeFP Mat> requires (IsRank2<Mat>)
	constexpr Mat pr_vectorcall Sqrt(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		// Using 'Denman-Beavers' square root iteration. Should converge quadratically
		auto a = mat;             // Converges to mat^0.5
		auto b = Identity<Mat>(); // Converges to mat^-0.5
		for (int i = 0; i != 10; ++i)
		{
			auto a_next = S(0.5) * (a + Invert(b));
			auto b_next = S(0.5) * (b + Invert(a));
			a = a_next;
			b = b_next;
		}
		return a;
	}

	// Orthonormalise the rotation component of 'mat'
	template <VectorTypeFP Mat> requires (IsRank2<Mat>)
	inline Mat pr_vectorcall Orthonorm(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;

		auto m = mat;
		if constexpr (vt::dimension == 2)
		{
			vec(m).x = Normalise(vec(m).x);
			vec(m).y = Rotate90CW(vec(m).x);
		}
		if constexpr (vt::dimension >= 3)
		{
			vec(m).x = Normalise(vec(m).x);
			vec(m).y = Normalise(Cross(vec(m).z, vec(m).x));
			vec(m).z = Cross(vec(m).x, vec(m).y);
		}
		pr_assert(IsOrthonormal(m));
		return m;
	}

	// Matrix Multiply
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr typename vector_traits<Mat>::component_t pr_vectorcall operator * (Mat const& a2b, typename vector_traits<Mat>::component_t v) noexcept
	{
		using vt = vector_traits<Mat>;
		using Vec = typename vt::component_t;

		Vec res = {};
		auto a2bT = Transpose(a2b);
		if constexpr (vt::dimension > 0) vec(res).x = Dot(vec(a2bT).x, v);
		if constexpr (vt::dimension > 1) vec(res).y = Dot(vec(a2bT).y, v);
		if constexpr (vt::dimension > 2) vec(res).z = Dot(vec(a2bT).z, v);
		if constexpr (vt::dimension > 3) vec(res).w = Dot(vec(a2bT).w, v);
		return res;
	}
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr Mat pr_vectorcall operator * (Mat const& b2c, Mat const& a2b) noexcept
	{
		// Note:
		//  - The reason for this order is because matrices are applied from right to left
		//    e.g.
		//       auto Va =             V = vector in space 'a'
		//       auto Vb =       a2b * V = vector in space 'b'
		//       auto Vc = b2c * a2b * V = vector in space 'c'
		//  - The shape of the result is:
		//       [a2c] = [b2c] * [a2b]
		//       [1x3]   [2x3]   [1x2]
		using vt = vector_traits<Mat>;
		using Vec = typename vt::component_t;

		Mat ans = {};
		auto b2cT = Transpose(b2c);
		if constexpr (vt::dimension == 2)
		{
			vec(ans).x = Vec{ Dot(vec(b2cT).x, vec(a2b).x), Dot(vec(b2cT).y, vec(a2b).x) };
			vec(ans).y = Vec{ Dot(vec(b2cT).x, vec(a2b).y), Dot(vec(b2cT).y, vec(a2b).y) };
		}
		if constexpr (vt::dimension == 3)
		{
			vec(ans).x = Vec{ Dot(vec(b2cT).x, vec(a2b).x), Dot(vec(b2cT).y, vec(a2b).x), Dot(vec(b2cT).z, vec(a2b).x) };
			vec(ans).y = Vec{ Dot(vec(b2cT).x, vec(a2b).y), Dot(vec(b2cT).y, vec(a2b).y), Dot(vec(b2cT).z, vec(a2b).y) };
			vec(ans).z = Vec{ Dot(vec(b2cT).x, vec(a2b).z), Dot(vec(b2cT).y, vec(a2b).z), Dot(vec(b2cT).z, vec(a2b).z) };
		}
		if constexpr (vt::dimension == 4)
		{
			vec(ans).x = Vec(Dot(vec(b2cT).x, vec(a2b).x), Dot(vec(b2cT).y, vec(a2b).x), Dot(vec(b2cT).z, vec(a2b).x), Dot(vec(b2cT).w, vec(a2b).x));
			vec(ans).y = Vec(Dot(vec(b2cT).x, vec(a2b).y), Dot(vec(b2cT).y, vec(a2b).y), Dot(vec(b2cT).z, vec(a2b).y), Dot(vec(b2cT).w, vec(a2b).y));
			vec(ans).z = Vec(Dot(vec(b2cT).x, vec(a2b).z), Dot(vec(b2cT).y, vec(a2b).z), Dot(vec(b2cT).z, vec(a2b).z), Dot(vec(b2cT).w, vec(a2b).z));
			vec(ans).w = Vec(Dot(vec(b2cT).x, vec(a2b).w), Dot(vec(b2cT).y, vec(a2b).w), Dot(vec(b2cT).z, vec(a2b).w), Dot(vec(b2cT).w, vec(a2b).w));
		}
		return ans;
	}
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr Mat& operator *= (Mat& lhs, Mat const& rhs) noexcept
	{
		return lhs = lhs * rhs;
	}

	// Matrix creation -----

	// Create a translation matrix
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 4)
	constexpr Mat Translation(typename vector_traits<Mat>::component_t xyz) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		pr_assert((xyz.w == S(0) || xyz.w == S(1)) && "translation should be an affine vector");
		Mat m = Identity<Mat>();
		vec(m).w = xyz;         // 'xyz' can be a position or an offset
		vec(vec(m).w).w = S(1); // Ensure affine
		return m;
	}
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 4)
	constexpr Mat Translation(typename vector_traits<Mat>::element_t x, typename vector_traits<Mat>::element_t y, typename vector_traits<Mat>::element_t z) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;

		return Translation<Mat>(Vec{ x, y, z, S(1) });
	}

	// Create a 2D rotation matrix
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 2)
	inline Mat pr_vectorcall Rotation(typename vector_traits<Mat>::element_t angle) noexcept
	{
		using vt = vector_traits<Mat>;
		using Vec = typename vt::component_t;
		return Mat{
			Vec{+std::cos(angle), +std::sin(angle)},
			Vec{-std::sin(angle), +std::cos(angle)},
		};
	}

	// Create a 3D rotation matrix. Order is: roll, pitch, yaw (to match DirectX)
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall RotationRad(typename vector_traits<Mat>::element_t pitch, typename vector_traits<Mat>::element_t yaw, typename vector_traits<Mat>::element_t roll) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;
		
		S cos_p = std::cos(pitch), sin_p = std::sin(pitch);
		S cos_y = std::cos(yaw  ), sin_y = std::sin(yaw  );
		S cos_r = std::cos(roll ), sin_r = std::sin(roll );
		return Mat{
			Vec{+cos_y * cos_r + sin_y * sin_p * sin_r , cos_p * sin_r , -sin_y * cos_r + cos_y * sin_p * sin_r},
			Vec{-cos_y * sin_r + sin_y * sin_p * cos_r , cos_p * cos_r , +sin_y * sin_r + cos_y * sin_p * cos_r},
			Vec{+sin_y * cos_p                         ,        -sin_p ,                          cos_y * cos_p}
		};
	}
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall RotationDeg(typename vector_traits<Mat>::element_t pitch, typename vector_traits<Mat>::element_t yaw, typename vector_traits<Mat>::element_t roll) noexcept
	{
		return RotationRad<Mat>(
			DegreesToRadians(pitch),
			DegreesToRadians(yaw),
			DegreesToRadians(roll)
		);
	}

	// Create a 3D rotation matrix from an axis and angle.
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	constexpr Mat pr_vectorcall Rotation(typename vector_traits<Mat>::component_t axis_norm, typename vector_traits<Mat>::component_t axis_sine_angle, typename vector_traits<Mat>::element_t cos_angle) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		pr_assert(IsNormalised(axis_norm) && "'axis_norm' should be normalised");

		Mat m = {};
		auto trace_vec = axis_norm * (S(1) - cos_angle);

		vec(vec(m).x).x = vec(trace_vec).x * vec(axis_norm).x + cos_angle;
		vec(vec(m).y).y = vec(trace_vec).y * vec(axis_norm).y + cos_angle;
		vec(vec(m).z).z = vec(trace_vec).z * vec(axis_norm).z + cos_angle;

		vec(trace_vec).x *= vec(axis_norm).y;
		vec(trace_vec).z *= vec(axis_norm).x;
		vec(trace_vec).y *= vec(axis_norm).z;

		vec(vec(m).x).y = vec(trace_vec).x + vec(axis_sine_angle).z;
		vec(vec(m).x).z = vec(trace_vec).z - vec(axis_sine_angle).y;
		vec(vec(m).y).x = vec(trace_vec).x - vec(axis_sine_angle).z;
		vec(vec(m).y).z = vec(trace_vec).y + vec(axis_sine_angle).x;
		vec(vec(m).z).x = vec(trace_vec).z + vec(axis_sine_angle).y;
		vec(vec(m).z).y = vec(trace_vec).y - vec(axis_sine_angle).x;

		return m;
	}

	// Create a 3D rotation from an axis and angle. 'axis' should be normalised
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall Rotation(typename vector_traits<Mat>::component_t axis_norm, typename vector_traits<Mat>::element_t angle) noexcept
	{
		return Rotation<Mat>(axis_norm, axis_norm * std::sin(angle), std::cos(angle));
	}

	// Create a 3D rotation from an angular displacement vector. length = angle(rad), direction = axis
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall Rotation(typename vector_traits<Mat>::component_t angular_displacement) noexcept // This is ExpMap3x3.
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		pr_assert(angular_displacement.w == S(0) && "'angular_displacement' should be a scaled direction vector");

		// Rodrigues' formula:  exp(omega) = I + (sin(theta)/theta) * omega + ((1 - cos(theta)/theta²) * omega²
		auto len = Length(angular_displacement);
		return len > constants<S>::tiny
			? Rotation<Mat>(angular_displacement / len, len)
			: Identity<Mat>();
	}

	// Create a 3D rotation representing the rotation from one vector to another. (Vectors do not need to be normalised)
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall Rotation(typename vector_traits<Mat>::component_t from, typename vector_traits<Mat>::component_t to) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;
		pr_assert(!FEql(from, Vec{}));
		pr_assert(!FEql(to  , Vec{}));

		auto len = Length(from) * Length(to);
		auto cos_angle = Dot3(from, to) / len;
		if (cos_angle >= 1.0f - tiny<S>) return Identity<Mat>();
		if (cos_angle <= tiny<S> - S(1)) return Rotation<Mat>(Normalise(Perpendicular(from - to)), constants<S>::tau_by_2);

		auto axis_size_angle = Cross(from, to) / len;
		auto axis_norm = Normalise(axis_size_angle);
		return Rotation<Mat>(axis_norm, axis_size_angle, cos_angle);
	}

	// Create a 3D rotation transform from one basis axis to another. Remember AxisId can be cast to Vec4
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall Rotation(AxisId from_axis, AxisId to_axis) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		// 'o2f' = the rotation from Z to 'from_axis'
		// 'o2t' = the rotation from Z to 'to_axis'
		// 'f2t' = o2t * Invert(o2f)
		Mat o2f = {}, o2t = {};
		switch (from_axis)
		{
			case -1: o2f = RotationRad<Mat>(S(0), -constants<S>::tau_by_4, S(0)); break;
			case +1: o2f = RotationRad<Mat>(S(0), +constants<S>::tau_by_4, S(0)); break;
			case -2: o2f = RotationRad<Mat>(+constants<S>::tau_by_4, S(0), S(0)); break;
			case +2: o2f = RotationRad<Mat>(-constants<S>::tau_by_4, S(0), S(0)); break;
			case -3: o2f = RotationRad<Mat>(S(0), +constants<S>::tau_by_2, S(0)); break;
			case +3: o2f = Identity<Mat>(); break;
			default: pr_assert(false && "axis_id must one of +/-1, +/-2, +/-3"); o2f = Identity<Mat>(); break;
		}
		switch (to_axis)
		{
			case -1: o2t = RotationRad<Mat>(S(0), -constants<S>::tau_by_4, S(0)); break;
			case +1: o2t = RotationRad<Mat>(S(0), +constants<S>::tau_by_4, S(0)); break;
			case -2: o2t = RotationRad<Mat>(+constants<S>::tau_by_4, S(0), S(0)); break;
			case +2: o2t = RotationRad<Mat>(-constants<S>::tau_by_4, S(0), S(0)); break;
			case -3: o2t = RotationRad<Mat>(S(0), +constants<S>::tau_by_2, S(0)); break;
			case +3: o2t = Identity<Mat>(); break;
			default: pr_assert(false && "axis_id must one of +/-1, +/-2, +/-3"); o2t = Identity<Mat>(); break;
		}
		return o2t * InvertAffine(o2f);
	}

	// Create a scale matrix
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr Mat pr_vectorcall Scale(typename vector_traits<Mat>::element_t scale) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		Mat mat = {};
		if constexpr (vt::dimension > 0) vec(vec(mat).x).x = scale;
		if constexpr (vt::dimension > 1) vec(vec(mat).y).y = scale;
		if constexpr (vt::dimension > 2) vec(vec(mat).z).z = scale;
		if constexpr (vt::dimension > 3) vec(vec(mat).w).w = scale;
		return mat;
	}
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr Mat pr_vectorcall Scale(typename vector_traits<Mat>::component_t scale) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		Mat mat = {};
		if constexpr (vt::dimension > 0) vec(vec(mat).x).x = vec(scale).x;
		if constexpr (vt::dimension > 1) vec(vec(mat).y).y = vec(scale).y;
		if constexpr (vt::dimension > 2) vec(vec(mat).z).z = vec(scale).z;
		if constexpr (vt::dimension > 3) vec(vec(mat).w).w = vec(scale).w;
		return mat;
	}

	// Create a 2D shear matrix
	template <VectorType Mat, ScalarType S = typename vector_traits<Mat>::element_t> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 2)
	constexpr Mat Shear(S sxy, S syx) noexcept
	{
		using vt = vector_traits<Mat>;
		using Vec = typename vt::component_t;

		return Mat {
			Vec{ S(1), sxy },
			Vec{ syx, S(1) },
		};
	}

	// Create a 3D shear matrix
	template <VectorType Mat, ScalarType S = typename vector_traits<Mat>::element_t> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	constexpr Mat Shear(S sxy, S sxz, S syx, S syz, S szx, S szy) noexcept
	{
		using vt = vector_traits<Mat>;
		using Vec = typename vt::component_t;

		return Mat {
			Vec{ S(1), sxy, sxz },
			Vec{ syx, S(1), syz },
			Vec{ szx, szy, S(1) },
		};
	}

	// Create a Look-At transform
	template <VectorType Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 4)
	inline Mat LookAt(typename vector_traits<Mat>::component_t eye, typename vector_traits<Mat>::component_t at, typename vector_traits<Mat>::component_t up) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;
		pr_assert(vec(eye).w == S(1) && vec(at).w == S(1) && vec(up).w == S(0) && "Invalid position/direction vectors passed to LookAt");
		pr_assert(eye - at != Zero<Vec>() && "LookAt 'eye' and 'at' positions are coincident");
		pr_assert(!IsParallel(eye - at, up, S(0)) && "LookAt 'forward' and 'up' axes are aligned");

		Mat mat = {};
		vec(mat).z = Normalise(eye - at);
		vec(mat).x = Normalise(Cross(up, vec(mat).z));
		vec(mat).y = Cross(vec(mat).z, vec(mat).x);
		vec(mat).w = eye;
		return mat;
	}

	// Construct an orthographic projection matrix
	template <VectorType Mat, ScalarType S = typename vector_traits<Mat>::element_t> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 4)
	constexpr Mat ProjectionOrthographic(S w, S h, S zn, S zf, bool righthanded) noexcept
	{
		using vt = vector_traits<Mat>;
		pr_assert(IsFinite(w) && IsFinite(h) && w > S(0) && h > S(0) && "invalid view rect");
		pr_assert(IsFinite(zn) && IsFinite(zf) && (zn - zf) != 0 && "invalid near/far planes");

		auto rh = Bool2SignF(righthanded);

		Mat mat = {};
		vec(vec(mat).x).x = S(2) / w;
		vec(vec(mat).y).y = S(2) / h;
		vec(vec(mat).z).z = rh / (zn - zf);
		vec(vec(mat).w).w = S(1);
		vec(vec(mat).w).z = rh * zn / (zn - zf);
		return mat;
	}

	// Construct a perspective projection matrix. 'w' and 'h' are measured at 'zn'
	template <VectorType Mat, ScalarType S = typename vector_traits<Mat>::element_t> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 4)
	constexpr Mat ProjectionPerspective(S w, S h, S zn, S zf, bool righthanded) noexcept
	{
		using vt = vector_traits<Mat>;
		pr_assert(IsFinite(w) && IsFinite(h) && w > S(0) && h > S(0) && "invalid view rect");
		pr_assert(IsFinite(zn) && IsFinite(zf) && zn > S(0) && zf > S(0) && (zn - zf) != S(0) && "invalid near/far planes");

		// Getting your head around perspective transforms:
		//   p0 = c2s * Vec4(0,0,-zn,1); p0/p0.w = (0,0,0,1)
		//   p1 = c2s * Vec4(0,0,-zf,1); p1/p1.w = (0,0,1,1)

		auto rh = Bool2SignF(righthanded);

		Mat mat = {};
		vec(vec(mat).x).x = S(2) * zn / w;
		vec(vec(mat).y).y = S(2) * zn / h;
		vec(vec(mat).z).w = -rh;
		vec(vec(mat).z).z = rh * zf / (zn - zf);
		vec(vec(mat).w).z = zn * zf / (zn - zf);
		return mat;
	}

	// Construct a perspective projection matrix offset from the centre
	template <VectorType Mat, ScalarType S = typename vector_traits<Mat>::element_t> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 4)
	constexpr Mat ProjectionPerspective(S l, S r, S t, S b, S zn, S zf, bool righthanded) noexcept
	{
		using vt = vector_traits<Mat>;
		pr_assert(IsFinite(l)  && IsFinite(r) && IsFinite(t) && IsFinite(b) && (r - l) > S(0) && (t - b) > S(0) && "invalid view rect");
		pr_assert(IsFinite(zn) && IsFinite(zf) && zn > S(0) && zf > S(0) && (zn - zf) != S(0) && "invalid near/far planes");
		
		auto rh = Bool2SignF(righthanded);
		
		Mat mat = {};
		vec(vec(mat).x).x = S(2) * zn / (r - l);
		vec(vec(mat).y).y = S(2) * zn / (t - b);
		vec(vec(mat).z).x = rh * (r + l) / (r - l);
		vec(vec(mat).z).y = rh * (t + b) / (t - b);
		vec(vec(mat).z).w = -rh;
		vec(vec(mat).z).z = rh * zf / (zn - zf);
		vec(vec(mat).w).z = zn * zf / (zn - zf);
		return mat;
	}

	// Construct a perspective projection matrix using field of view
	template <VectorType Mat, ScalarType S = typename vector_traits<Mat>::element_t> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 4)
	inline Mat ProjectionPerspectiveFOV(S fovY, S aspect, S zn, S zf, bool righthanded) noexcept
	{
		using vt = vector_traits<Mat>;
		pr_assert(IsFinite(aspect) && aspect > S(0) && "invalid aspect ratio");
		pr_assert(IsFinite(zn) && IsFinite(zf) && zn > S(0) && zf > S(0) && (zn - zf) != S(0) && "invalid near/far planes");

		auto rh = Bool2SignF(righthanded);

		Mat mat = {};
		vec(vec(mat).y).y = S(1) / std::tan(fovY / S(2));
		vec(vec(mat).x).x = vec(vec(mat).y).y / aspect;
		vec(vec(mat).z).w = -rh;
		vec(vec(mat).z).z = rh * zf / (zn - zf);
		vec(vec(mat).w).z = zn * zf / (zn - zf);
		return mat;
	}

	// General operations -----

	// Diagonalise a 3x3 matrix. From numerical recipes
	template <VectorType Mat> requires (IsRank2<Mat> && ArrayAccess<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall Diagonalise(Mat const& mat_, Mat& eigen_vectors, typename vector_traits<Mat>::component_t& eigen_values) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;

		struct L
		{
			static void Rotate(Mat& mat, int i, int j, int k, int l, S s, S tau) noexcept
			{
				auto temp = mat[j][i];
				auto h = mat[l][k];
				mat[j][i] = temp - s * (h + temp * tau);
				mat[l][k] = h + s * (temp - h * tau);
			}
		};

		// Initialise the Eigen values and b to be the diagonal elements of 'mat'
		eigen_values = Diagonal(mat_);
		eigen_vectors = Identity<Mat>();
		Vec b = eigen_values;

		auto mat = mat_;
		S sum = {};
		S const diagonal_eps = S(1.0e-4);
		do
		{
			auto z = Vec{};

			// Sweep through all elements above the diagonal
			for (int i = 0; i != 3; ++i) //ip
			{
				for (int j = i + 1; j != 3; ++j) //iq
				{
					if (Abs(mat[j][i]) > diagonal_eps/S(3))
					{
						auto h     = eigen_values[j] - eigen_values[i];
						auto theta = S(0.5) * h / mat[j][i];
						auto t     = Sign(theta) / (Abs(theta) + Sqrt(S(1) + Sqr(theta)));
						auto c     = S(1) / Sqrt(S(1) + Sqr(t));
						auto s     = t * c;
						auto tau   = s / (S(1) + c);
						h          = t * mat[j][i];

						z[i] -= h;
						z[j] += h;
						eigen_values[i] -= h;
						eigen_values[j] += h;
						mat[j][i] = S(0);

						for (int k = 0; k != i; ++k)
							L::Rotate(mat, k, i, k, j, s, tau); //changes mat( 0:i-1 ,i) and mat( 0:i-1 ,j)

						for (int k = i + 1; k != j; ++k)
							L::Rotate(mat, i, k, k, j, s, tau); //changes mat(i, i+1:j-1 ) and mat( i+1:j-1 ,j)

						for (int k = j + 1; k != 3; ++k)
							L::Rotate(mat, i, k, j, k, s, tau); //changes mat(i, j+1:2 ) and mat(j, j+1:2 )

						for (int k = 0; k != 3; ++k)
							L::Rotate(eigen_vectors, k, i, k, j, s, tau); //changes EigenVec( 0:2 ,i) and evec( 0:2 ,j)
					}
				}
			}

			b = b + z;
			eigen_values = b;

			// Calculate sum of abs. values of off diagonal elements, to see if we've finished
			sum  = Abs(vec(vec(mat).y).x);
			sum += Abs(vec(vec(mat).z).x);
			sum += Abs(vec(vec(mat).z).y);
		}
		while (sum > diagonal_eps);
		return mat;
	}

	// Return the axis and angle of a rotation matrix
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline std::tuple<typename vector_traits<Mat>::component_t, typename vector_traits<Mat>::element_t> pr_vectorcall AxisAngle(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;
		pr_assert(IsOrthonormal(mat) && "Matrix is not a pure rotation matrix");

		auto angle = std::acos(S(0.5) * (Trace(mat) - S(1)));
		auto axis = S(1000) * Kernel(Identity<Mat>() - mat);
		if (axis == Vec{})
			return { Vec{1, 0, 0, 0}, S(0) };
		
		axis = Normalise(axis);

		// Determine the correct sign of the angle
		auto vec = CreateNotParallelTo(axis);
		auto X = vec - Dot(axis, vec) * axis;
		auto Xprim = mat * X;
		auto XcXp = Cross(X, Xprim);
		if (Dot3(XcXp, axis) < S(0))
			angle = -angle;

		return { axis, angle };
	}

	// Create a scale matrix from 'mat'. Use 'Diagonal' to get the scale components as a vector
	template <VectorType Mat> requires (IsRank2<Mat>)
	constexpr Mat pr_vectorcall ScaleFrom(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;

		Mat res = {};
		if constexpr (vt::dimension > 0) vec(vec(res).x).x = Length(vec(mat).x);
		if constexpr (vt::dimension > 1) vec(vec(res).y).y = Length(vec(mat).y);
		if constexpr (vt::dimension > 2) vec(vec(res).z).z = Length(vec(mat).z);
		if constexpr (vt::dimension > 3) vec(vec(res).w).w = Length(vec(mat).w);
		return res;
	}

	// Return a copy of 'mat' with the component vectors normalised
	template <VectorTypeFP Mat> requires (IsRank2<Mat>)
	inline Mat pr_vectorcall Unscaled(Mat const& mat) noexcept
	{
		using vt = vector_traits<Mat>;

		Mat res = {};
		if constexpr (vt::dimension > 0) vec(res).x = Normalise(vec(mat).x);
		if constexpr (vt::dimension > 1) vec(res).y = Normalise(vec(mat).y);
		if constexpr (vt::dimension > 2) vec(res).z = Normalise(vec(mat).z);
		if constexpr (vt::dimension > 3) vec(res).w = Normalise(vec(mat).w);
		return res;
	}

	// Construct a rotation matrix that transforms 'from' onto the z axis
	// Other points can then be projected onto the XY plane by rotating by this
	// matrix and then setting the z value to zero
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat RotationToZAxis(typename vector_traits<Mat>::component_t from) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;

		auto x = vec(from).x;
		auto y = vec(from).y;
		auto z = vec(from).z;
		auto r = Sqr(x) + Sqr(y);
		auto d = Sqrt(r);

		if (Abs(d) < tiny<S>)
		{
			auto mat = Identity<Mat>();
			vec(vec(mat).x).x = z; // Create an identity transform or a 180 degree rotation
			vec(vec(mat).z).z = z; // about Y depending on the sign of 'from.z'
			return mat;
		}
		else
		{
			return Mat{
				Vec{x * z / d, -y / d, x},
				Vec{y * z / d, +x / d, y},
				Vec{   -r / d,   S(0), z}
			};
		}
	}

	// Make an orientation matrix from a direction vector
	// 'dir' is the direction to align the axis 'axis_id' to. (Doesn't need to be normalised)
	// 'up' is the preferred up direction, however if up is parallel to 'dir'
	// then a vector perpendicular to 'dir' will be chosen.
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall OriFromDir(typename vector_traits<Mat>::component_t dir, AxisId axis_id, typename vector_traits<Mat>::component_t up_ = {}) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;
		pr_assert(dir != Zero<Vec>() && "'dir' cannot be a zero vector");

		// Get the preferred up direction (handling parallel cases)
		auto up = Perpendicular(dir, up_);

		Mat ori = {};
		vec(ori).z = Normalise(Sign(S(axis_id)) * dir);
		vec(ori).x = Normalise(Cross(up, vec(ori).z));
		vec(ori).y = Cross(vec(ori).z, vec(ori).x);

		// Permute the column vectors so +Z becomes 'axis'
		return Permute(ori, -Abs(axis_id));
	}

	// Make a scaled orientation matrix from a direction vector
	// Returns a transform for scaling and rotating the 'axis'th axis to 'dir'
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall ScaledOriFromDir(typename vector_traits<Mat>::component_t dir, AxisId axis, typename vector_traits<Mat>::component_t up = {}) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		auto len = Length(dir);
		return len > tiny<S>
			? OriFromDir<Mat>(dir, axis, up) * Scale<Mat>(len)
			: Zero<Mat>();
	}

	// Return a vector representing the approximate rotation between two orthonormal transforms
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	constexpr typename vector_traits<Mat>::component_t pr_vectorcall RotationVectorApprox(Mat const& from, Mat const& to) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;
		pr_assert(IsOrthonormal(from) && IsOrthonormal(to) && "This only works for orthonormal matrices");
		
		auto cpm_x_i2wR = to - from;
		auto w2iR = Transpose(from);
		auto cpm = cpm_x_i2wR * w2iR;
		return Vec{
			vec(vec(cpm).y).z,
			vec(vec(cpm).z).x,
			vec(vec(cpm).x).y
		};
	}
	
	// Create a cross product matrix for 'vec'.
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	constexpr Mat pr_vectorcall CPM(typename vector_traits<Mat>::component_t v) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;

		// This matrix can be used to calculate the cross product with
		// another vector: e.g. Cross(v1, v2) == CPM(v1) * v2
		return Mat{
			Vec{     S(0),  vec(v).z, -vec(v).y},
			Vec{-vec(v).z,      S(0),  vec(v).x},
			Vec{ vec(v).y, -vec(v).x,      S(0)}
		};
	}

	// Return 'exp(omega)' (Rodriges' formula)
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall ExpMap3x3(typename vector_traits<Mat>::component_t omega) noexcept
	{
		// Converts an angular velocity into a finite rotation that stays within SO(3).
		// If you have an angular velocity, w, that is constant over a time step,
		// then:
		//   R(t + dt) = R(t) * ExpMap(w * dt)
		//   (no need to orthonormalise)
		//
		// Rodrigues' formula:  exp(omega) = I + (sin(theta)/theta) * omega + ((1 - cos(theta)/theta²) * omega²
		// If you want the shortest rotation from R0 to R1:
		//   R(t) = R0 * ExpMap(t * LogMap(Transpose(R0) * ​R1​))
		return Rotation<Mat>(omega);
	}

	// Returns the Axis*Angle vector representation of a rotation matrix (Inverse of ExpMap)
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline typename vector_traits<Mat>::component_t pr_vectorcall LogMap3x3(Mat const& rot) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;

		auto cos_angle = Clamp<S>((Trace(rot) - S(1)) / S(2), -S(1), +S(1));
		auto theta = std::acos(cos_angle);
		if (Abs(theta) < tiny<S>)
			return Zero<Vec>();

		auto s = S(1) / (S(2) * std::sin(theta));
		auto axis = s * Vec{
			vec(vec(rot).y).z - vec(vec(rot).z).y,
			vec(vec(rot).z).x - vec(vec(rot).x).z,
			vec(vec(rot).x).y - vec(vec(rot).y).x
		};
		return theta * axis;
	}

	// Evaluates 'ori' after 'time' for a constant angular velocity and angular acceleration
	template <VectorTypeFP Mat> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall RotationAt(float time, Mat const& ori, typename vector_traits<Mat>::component_t avel, typename vector_traits<Mat>::component_t aacc) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using Vec = typename vt::component_t;

		// Orientation can be computed analytically if angular velocity
		// and angular acceleration are parallel or angular acceleration is zero.
		if (LengthSq(Cross(avel, aacc)) < tiny<S>)
		{
			auto w = avel + aacc * time;
			return ExpMap3x3<Mat>(w * time) * ori;
		}
		else
		{
			// Otherwise, use the SPIRAL(6) algorithm. 6th order accurate for moderate 'time_s'

			// 3-point Gauss-Legendre nodes for 6th order accuracy
			constexpr S root15 = S(3.87298334620741688518);
			constexpr S c1 = S(0.5) - root15 / S(10);
			constexpr S c2 = S(0.5);
			constexpr S c3 = S(0.5) + root15 / S(10);

			// Evaluate instantaneous angular velocity at nodes
			auto w0 = avel + aacc * c1 * time;
			auto w1 = avel + aacc * c2 * time;
			auto w2 = avel + aacc * c3 * time;

			auto u0 = ExpMap3x3<Mat>(w0 * time / S(3));
			auto u1 = ExpMap3x3<Mat>(w1 * time / S(3));
			auto u2 = ExpMap3x3<Mat>(w2 * time / S(3));

			return u2 * u1 * u0 * ori;
		}
	}

	// Divide a circle into N sectors and return an index for the sector that 'v' is in
	template <VectorType Vec> requires (IsRank1<Vec> && vector_traits<Vec>::dimension == 2)
	inline int pr_vectorcall Sector(Vec v, int sectors) noexcept
	{
		using S = typename vector_traits<Vec>::element_t;
		return static_cast<int>(Atan2Positive(vec(v).y, vec(v).x) * sectors / constants<S>::tau);
	}

	// Random -----

	// Create a random value on interval ['vmin', 'vmax']
	template <ScalarType S, typename Rng = std::default_random_engine>
	constexpr S Random(Rng& rng, S vmin, S vmax) noexcept
	{
		if constexpr (std::integral<S>)
		{
			std::uniform_int_distribution<S> dist(vmin, vmax);
			return dist(rng);
		}
		else
		{
			std::uniform_real_distribution<S> dist(vmin, vmax);
			return dist(rng);
		}
	}

	// Create a random value centred on 'centre' with radius 'radius'
	template <ScalarType S, typename Rng = std::default_random_engine>
	constexpr S RandomC(Rng& rng, S centre, S radius) noexcept
	{
		return Random<S, Rng>(rng, centre - radius, centre + radius);
	}

	// Create a random vector with unit length
	template <VectorType Vec, typename Rng = std::default_random_engine>
	inline Vec pr_vectorcall RandomN(Rng& rng) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;
		using C = typename vt::component_t;

		if constexpr (IsRank1<Vec>)
		{
			if constexpr (std::floating_point<S>)
			{
				std::uniform_real_distribution<S> dist(S(-1), S(1));
				for (;;)
				{
					Vec res = {};
					if constexpr (vt::dimension > 0) vec(res).x = dist(rng);
					if constexpr (vt::dimension > 1) vec(res).y = dist(rng);
					if constexpr (vt::dimension > 2) vec(res).z = dist(rng);
					if constexpr (vt::dimension > 3) vec(res).w = dist(rng);
					if (auto len = LengthSq(res); len > S(0.01) && len <= S(1))
						return res / Sqrt(len);
				}
			}
			else
			{
				// For integer types, pick a random axis-aligned unit vector
				std::uniform_int_distribution<int> axis_dist(0, vt::dimension - 1);
				std::uniform_int_distribution<int> sign_dist(0, 1);
				Vec res = {};
				res[axis_dist(rng)] = S(sign_dist(rng) * 2 - 1);
				return res;
			}
		}
		else
		{
			// Each column is independently a random unit vector
			Vec res = {};
			if constexpr (vt::dimension > 0) vec(res).x = RandomN<C>(rng);
			if constexpr (vt::dimension > 1) vec(res).y = RandomN<C>(rng);
			if constexpr (vt::dimension > 2) vec(res).z = RandomN<C>(rng);
			if constexpr (vt::dimension > 3) vec(res).w = RandomN<C>(rng);
			return res;
		}
	}

	// Create a random vector with components on interval [vmin, vmax]
	template <VectorType Vec, typename Rng = std::default_random_engine>
	constexpr Vec pr_vectorcall Random(Rng& rng, Vec vmin, Vec vmax) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;
		using C = typename vt::component_t;

		if constexpr (IsRank1<Vec>)
		{
			Vec res = {};
			if constexpr (std::floating_point<S>)
			{
				if constexpr (vt::dimension > 0) { std::uniform_real_distribution<S> dist_x(vec(vmin).x, vec(vmax).x); vec(res).x = dist_x(rng); }
				if constexpr (vt::dimension > 1) { std::uniform_real_distribution<S> dist_y(vec(vmin).y, vec(vmax).y); vec(res).y = dist_y(rng); }
				if constexpr (vt::dimension > 2) { std::uniform_real_distribution<S> dist_z(vec(vmin).z, vec(vmax).z); vec(res).z = dist_z(rng); }
				if constexpr (vt::dimension > 3) { std::uniform_real_distribution<S> dist_w(vec(vmin).w, vec(vmax).w); vec(res).w = dist_w(rng); }
			}
			else
			{
				if constexpr (vt::dimension > 0) { std::uniform_int_distribution<S> dist_x(vec(vmin).x, vec(vmax).x); vec(res).x = dist_x(rng); }
				if constexpr (vt::dimension > 1) { std::uniform_int_distribution<S> dist_y(vec(vmin).y, vec(vmax).y); vec(res).y = dist_y(rng); }
				if constexpr (vt::dimension > 2) { std::uniform_int_distribution<S> dist_z(vec(vmin).z, vec(vmax).z); vec(res).z = dist_z(rng); }
				if constexpr (vt::dimension > 3) { std::uniform_int_distribution<S> dist_w(vec(vmin).w, vec(vmax).w); vec(res).w = dist_w(rng); }
			}
			return res;
		}
		else
		{
			Vec res = {};
			if constexpr (vt::dimension > 0) vec(res).x = Random<C>(rng, vec(vmin).x, vec(vmax).x);
			if constexpr (vt::dimension > 1) vec(res).y = Random<C>(rng, vec(vmin).y, vec(vmax).y);
			if constexpr (vt::dimension > 2) vec(res).z = Random<C>(rng, vec(vmin).z, vec(vmax).z);
			if constexpr (vt::dimension > 3) vec(res).w = Random<C>(rng, vec(vmin).w, vec(vmax).w);
			return res;
		}
	}

	// Create a random vector with length on interval [min_length, max_length]
	template <VectorType Vec, typename Rng = std::default_random_engine>
	constexpr Vec pr_vectorcall Random(Rng& rng, typename vector_traits<Vec>::component_t min_length, typename vector_traits<Vec>::component_t max_length) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;
		using C = typename vt::component_t;

		if constexpr (IsRank1<Vec>)
		{
			if constexpr (std::floating_point<S>)
			{
				std::uniform_real_distribution<S> dist(min_length, max_length);
				return dist(rng) * RandomN<Vec>(rng);
			}
			else
			{
				std::uniform_int_distribution<S> dist(min_length, max_length);
				return dist(rng) * RandomN<Vec>(rng);
			}
		}
		else
		{
			Vec res = {};
			if constexpr (vt::dimension > 0) vec(res).x = Random<C>(rng, vec(min_length).x, vec(max_length).x);
			if constexpr (vt::dimension > 1) vec(res).y = Random<C>(rng, vec(min_length).y, vec(max_length).y);
			if constexpr (vt::dimension > 2) vec(res).z = Random<C>(rng, vec(min_length).z, vec(max_length).z);
			if constexpr (vt::dimension > 3) vec(res).w = Random<C>(rng, vec(min_length).w, vec(max_length).w);
			return res;
		}
	}

	// Create a random vector centred on 'centre' with radius 'radius'
	template <VectorType Vec, typename Rng = std::default_random_engine>
	constexpr Vec pr_vectorcall Random(Rng& rng, Vec centre, typename vector_traits<Vec>::component_t radius) noexcept
	{
		using vt = vector_traits<Vec>;
		using S = typename vt::element_t;
		using C = typename vt::component_t;

		if constexpr (IsRank1<Vec>)
		{
			return Random<Vec>(rng, S(0), radius) + centre;
		}
		else
		{
			Vec res = {};
			if constexpr (vt::dimension > 0) vec(res).x = Random<C>(rng, vec(centre).x, vec(radius).x);
			if constexpr (vt::dimension > 1) vec(res).y = Random<C>(rng, vec(centre).y, vec(radius).y);
			if constexpr (vt::dimension > 2) vec(res).z = Random<C>(rng, vec(centre).z, vec(radius).z);
			if constexpr (vt::dimension > 3) vec(res).w = Random<C>(rng, vec(centre).w, vec(radius).w);
			return res;
		}
	}

	// Create a 3D matrix containing random rotation about 'axis' and between angles [min_angle, max_angle)
	template <VectorTypeFP Mat, typename Rng = std::default_random_engine> requires (IsRank2<Mat> && vector_traits<Mat>::dimension >= 3)
	inline Mat pr_vectorcall Random(Rng& rng, typename vector_traits<Mat>::component_t axis_norm, typename vector_traits<Mat>::element_t min_angle, typename vector_traits<Mat>::element_t max_angle) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		std::uniform_real_distribution<S> dist(min_angle, max_angle);
		return Rotation<Mat>(axis_norm, dist(rng));
	}

	// Create a random 3D rotation matrix
	template <VectorTypeFP Mat, typename Rng = std::default_random_engine> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 3)
	inline Mat pr_vectorcall Random(Rng& rng) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;
		using C = typename vt::component_t;

		std::uniform_real_distribution<S> dist(S(0), constants<S>::tau);
		auto axis = RandomN<Vec3<S>>(rng).w0();
		return Rotation<Mat>(axis, dist(rng));
	}

	// Create a 2D matrix containing random rotation between angles [min_angle, max_angle)
	template <VectorType Mat, typename Rng = std::default_random_engine> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 2)
	inline Mat pr_vectorcall Random(Rng& rng, typename vector_traits<Mat>::element_t min_angle, typename vector_traits<Mat>::element_t max_angle) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		std::uniform_real_distribution<S> dist(min_angle, max_angle);
		return Rotation<Mat>(dist(rng));
	}

	// Create a random 2D rotation matrix
	template <VectorType Mat, typename Rng = std::default_random_engine> requires (IsRank2<Mat> && vector_traits<Mat>::dimension == 2)
	inline Mat pr_vectorcall Random(Rng& rng) noexcept
	{
		using vt = vector_traits<Mat>;
		using S = typename vt::element_t;

		return Random<Mat>(rng, S(0), constants<S>::tau);
	}
}
