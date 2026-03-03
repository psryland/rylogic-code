//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include <concepts>
#include <type_traits>
#include <span>
#include <ranges>
#include <limits>
#include <memory>
#include <array>
#include <vector>
#include <random>
#include <algorithm>
#include <numeric>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cassert>
#include <intrin.h>
#include <immintrin.h>
#include <emmintrin.h>
#include <iterator>
// #include <thread>
// #include <complex>
// No non-standard dependencies

// Use intrinsics by default
#ifndef PR_MATHS_USE_INTRINSICS
#define PR_MATHS_USE_INTRINSICS 1
#endif

// Use 'vectorcall' if intrinsics are enabled
#if PR_MATHS_USE_INTRINSICS
#  define pr_vectorcall __vectorcall
#  pragma intrinsic(sin, cos, tan, asin, acos, atan, atan2, sinh, cosh, tanh, pow, fmod, sqrt, exp, log10, log, abs, fabs, labs, memcmp, memcpy, memset)
#else
#  define pr_vectorcall __fastcall
#endif

// C++11's thread_local
#ifndef thread_local
#define thread_local __declspec(thread)
#endif

// Allow assert handler replacement
#ifndef pr_assert
#define pr_assert(x) do { if consteval {} else { assert(x); } } while (0)
#endif

// C++11's 'alignas'
#if _MSC_VER < 1900
#  ifndef alignas
#    define alignas(alignment) __declspec(align(alignment))
#  endif
#endif
#if _MSC_VER < 1900
#  ifndef constexpr
#    define constexpr
#  endif
#endif

namespace pr::math
{
	// Concept for scalar types
	template <typename T>
	concept ScalarType = std::floating_point<T> || std::integral<T>;
	template <typename T>
	concept ScalarTypeFP = std::floating_point<T>;

	// Concept for a vector-like container template
	template <template <typename...> class C, typename T>
	concept VectorLike = requires(C<T>& c, T const& val)
	{
		{ c.push_back(val) };
		{ c.size() } -> std::convertible_to<std::size_t>;
		{ c[0] } -> std::convertible_to<T const&>;
		c.begin();
		c.end();
	};
	static_assert(VectorLike<std::vector, int>, "std::vector should satisfy VectorLike");

	// Forward declarations
	template <ScalarType S> struct Vec2;
	template <ScalarType S> struct Vec3;
	template <ScalarType S> struct Vec4;
	template <ScalarType S, typename T> struct Vec8;
	template <ScalarTypeFP S> struct Quat;
	template <ScalarType S> struct Mat2x2;
	template <ScalarType S> struct Mat3x4;
	template <ScalarType S> struct Mat4x4;
	template <ScalarType S, typename A, typename B> struct Mat6x8;
	template <ScalarType S> struct Xform;

	enum class ETruncate { TowardZero, ToNearest };
}
