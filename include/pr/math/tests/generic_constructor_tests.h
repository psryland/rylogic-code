//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
// Tests for the generic (templated) constructors on Vec2, Vec3, Vec4, and Quat.
#pragma once
#include "pr/math/math.h"

#if PR_UNITTESTS
#include "pr/common/unittests.h"

// External vector types registered via vector_traits outside pr::math.
// .x/.y/.z/.w members make vector_access_member work without extra glue.
namespace
{
	template <typename T> struct ExtVec2 { T x, y; };
	template <typename T> struct ExtVec3 { T x, y, z; };
	template <typename T> struct ExtVec4 { T x, y, z, w; };
}

namespace pr::math
{
	template <typename T> struct vector_traits<ExtVec2<T>>
		: vector_traits_base<T, T, 2>
		, vector_access_member<ExtVec2<T>, T, 2>
	{
		template <ScalarType S> using rebind = Vec2<S>;
	};
	template <typename T> struct vector_traits<ExtVec3<T>>
		: vector_traits_base<T, T, 3>
		, vector_access_member<ExtVec3<T>, T, 3>
	{
		template <ScalarType S> using rebind = Vec3<S>;
	};
	template <typename T> struct vector_traits<ExtVec4<T>>
		: vector_traits_base<T, T, 4>
		, vector_access_member<ExtVec4<T>, T, 4>
	{
		template <ScalarType S> using rebind = Vec4<S>;
	};
}

namespace
{
	// A random-access range backed by a pointer pair. Has begin()/end() returning
	// raw pointers (random-access iterators) but no range-level operator[].
	// This exercises the iterator-based element-access path in the constructors.
	template <typename T>
	struct PtrRange
	{
		T const* m_first;
		T const* m_last;

		T const* begin() const { return m_first; }
		T const* end()   const { return m_last; }
	};
}

namespace pr::math::tests
{
	PRUnitTestClass(GenericConstructors)
	{
		PRUnitTestMethod(Vec2, float, double, int32_t, int64_t)
		{
			// Rank1VecSN constructor: external vector, same scalar, dimension 2
			auto ext = ExtVec2<T>{T(3), T(7)};
			Vec2<T> from_ext(ext);
			PR_EXPECT(from_ext.x == T(3) && from_ext.y == T(7));

			// Constexpr path
			constexpr auto ext_cx = ExtVec2<T>{T(11), T(22)};
			constexpr Vec2<T> from_ext_cx(ext_cx);
			static_assert(from_ext_cx.x == T(11) && from_ext_cx.y == T(22));

			// Range constructor: PtrRange has no range-level operator[]
			T const data[] = { T(5), T(6), T(99) };
			PtrRange<T> pr{data, data + 3};
			Vec2<T> from_range(pr);
			PR_EXPECT(from_range.x == T(5) && from_range.y == T(6));

			// std::array: sized_range — precondition is checked
			constexpr std::array<T, 2> arr{T(8), T(9)};
			constexpr Vec2<T> from_arr(arr);
			static_assert(from_arr.x == T(8) && from_arr.y == T(9));

			// Unbounded iota: not a sized_range — precondition assert is skipped
			auto iota = std::views::iota(0);
			Vec2<T> from_iota(iota);
			PR_EXPECT(from_iota.x == T(0) && from_iota.y == T(1));

			// Rank-1 guard: Mat2x2 has dimension 2 but is rank-2 — must be rejected
			static_assert(!std::is_constructible_v<Vec2<T>, Mat2x2<T>>,
				"Vec2 must not construct from a rank-2 matrix (rank-1 guard)");
		}

		PRUnitTestMethod(Vec3, float, double, int32_t, int64_t)
		{
			// Rank1VecSN constructor
			auto ext = ExtVec3<T>{T(1), T(2), T(3)};
			Vec3<T> from_ext(ext);
			PR_EXPECT(from_ext.x == T(1) && from_ext.y == T(2) && from_ext.z == T(3));

			constexpr auto ext_cx = ExtVec3<T>{T(10), T(20), T(30)};
			constexpr Vec3<T> from_ext_cx(ext_cx);
			static_assert(from_ext_cx.x == T(10) && from_ext_cx.y == T(20) && from_ext_cx.z == T(30));

			// Range via PtrRange
			T const data[] = { T(4), T(5), T(6), T(99) };
			PtrRange<T> pr{data, data + 4};
			Vec3<T> from_range(pr);
			PR_EXPECT(from_range.x == T(4) && from_range.y == T(5) && from_range.z == T(6));

			constexpr std::array<T, 3> arr{T(7), T(8), T(9)};
			constexpr Vec3<T> from_arr(arr);
			static_assert(from_arr.x == T(7) && from_arr.y == T(8) && from_arr.z == T(9));

			// Unbounded iota: not a sized_range
			auto iota = std::views::iota(0);
			Vec3<T> from_iota(iota);
			PR_EXPECT(from_iota.x == T(0) && from_iota.y == T(1) && from_iota.z == T(2));

			// Rank-1 guard: Mat3x3 has dimension 3 but is rank-2 — must be rejected
			static_assert(!std::is_constructible_v<Vec3<T>, Mat3x3<T>>,
				"Vec3 must not construct from a rank-2 matrix (rank-1 guard)");
		}

		PRUnitTestMethod(Vec4, float, double, int32_t, int64_t)
		{
			// Rank1VecSN constructor
			auto ext = ExtVec4<T>{T(1), T(2), T(3), T(4)};
			Vec4<T> from_ext(ext);
			PR_EXPECT(from_ext.x == T(1) && from_ext.y == T(2) && from_ext.z == T(3) && from_ext.w == T(4));

			constexpr auto ext_cx = ExtVec4<T>{T(10), T(20), T(30), T(40)};
			constexpr Vec4<T> from_ext_cx(ext_cx);
			static_assert(from_ext_cx.x == T(10) && from_ext_cx.y == T(20) && from_ext_cx.z == T(30) && from_ext_cx.w == T(40));

			// Range via PtrRange
			T const data[] = { T(5), T(6), T(7), T(8), T(99) };
			PtrRange<T> pr{data, data + 5};
			Vec4<T> from_range(pr);
			PR_EXPECT(from_range.x == T(5) && from_range.y == T(6) && from_range.z == T(7) && from_range.w == T(8));

			constexpr std::array<T, 4> arr{T(9), T(10), T(11), T(12)};
			constexpr Vec4<T> from_arr(arr);
			static_assert(from_arr.x == T(9) && from_arr.y == T(10) && from_arr.z == T(11) && from_arr.w == T(12));

			// Unbounded iota: not a sized_range
			auto iota = std::views::iota(0);
			Vec4<T> from_iota(iota);
			PR_EXPECT(from_iota.x == T(0) && from_iota.y == T(1) && from_iota.z == T(2) && from_iota.w == T(3));

			// Rank-1 guard: Mat4x4 has dimension 4 but is rank-2 — must be rejected
			static_assert(!std::is_constructible_v<Vec4<T>, Mat4x4<T>>,
				"Vec4 must not construct from a rank-2 matrix (rank-1 guard)");
		}

		PRUnitTestMethod(Quat, float, double)
		{
			// Rank1VecSN constructor
			auto ext = ExtVec4<T>{T(0), T(0), T(0), T(1)};
			Quat<T> from_ext(ext);
			PR_EXPECT(from_ext.x == T(0) && from_ext.y == T(0) && from_ext.z == T(0) && from_ext.w == T(1));

			// Range via PtrRange
			T const data[] = { T(0), T(0), T(1), T(0) };
			PtrRange<T> pr{data, data + 4};
			Quat<T> from_range(pr);
			PR_EXPECT(from_range.x == T(0) && from_range.y == T(0) && from_range.z == T(1) && from_range.w == T(0));

			// Unbounded iota: not a sized_range
			auto iota = std::views::iota(0);
			Quat<T> from_iota(iota);
			PR_EXPECT(from_iota.x == T(0) && from_iota.y == T(1) && from_iota.z == T(2) && from_iota.w == T(3));

			// Rank-1 guard: Mat4x4 has dimension 4 but is rank-2 — must be rejected
			static_assert(!std::is_constructible_v<Quat<T>, Mat4x4<T>>,
				"Quat must not construct from a rank-2 matrix (rank-1 guard)");
		}

		// Rank1VecSN requires same_as<element_t, S>, so cross-scalar external types
		// must not construct (no silent narrowing through this path).
		PRUnitTestMethod(ScalarConversionPolicy, float)
		{
			static_assert(std::is_constructible_v<Vec2<float>, ExtVec2<float>>);
			static_assert(std::is_constructible_v<Vec3<float>, ExtVec3<float>>);
			static_assert(std::is_constructible_v<Vec4<float>, ExtVec4<float>>);
			static_assert(std::is_constructible_v<Quat<float>, ExtVec4<float>>);

			static_assert(!std::is_constructible_v<Vec2<double>, ExtVec2<float>>);
			static_assert(!std::is_constructible_v<Vec3<double>, ExtVec3<float>>);
			static_assert(!std::is_constructible_v<Vec4<double>, ExtVec4<float>>);
		}
	};
}
#endif
