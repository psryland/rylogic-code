//*********************************************
// Maths Library
//  Copyright (c) Rylogic Ltd 2006
//*********************************************
#pragma once
#include <concepts>
#include <type_traits>
#include <iterator>
#include <algorithm>
#include <memory>
#include <thread>
#include <span>
#include <array>
#include <numeric>
#include <limits>
#include <stdexcept>
#include <intrin.h>
#include <immintrin.h>
#include <emmintrin.h>
#include <cmath>
#include <bit>
#include <cstdlib>
#include <cstdint>
#include <complex>
#include <cassert>
#include <random>

// Libraries built to use DirectXMath should be fine when linked in projects
// that don't use DirectXMath because all of the maths types have the same
// size/alignment requirements regardless.

static_assert(_MSC_VER >= 1900, "VS v140 is required due to a value initialisation bug in v120");

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
#define pr_assert(x) assert(x)
//#define pr_assert(condition, message)\
//	do {\
//		if constexpr (std::is_constant_evaluated()) { \
//			if (!(condition)) throw std::logic_error(message); \
//		} else\
//			if (!(condition)) pr_assert_fail(#condition, message, __FILE__, __LINE__); \
//		} \
//	} while(0)
#endif
//#ifndef pr_assert_fail
//#define pr_assert_fail(condition, message) assert((message) && (condition))
//#endif

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

namespace pr
{
	// Support component types for vectors
	template <typename T> concept Scalar =
		std::is_floating_point_v<T> ||
		std::is_integral_v<T>;

	template <Scalar S, typename T> struct Vec2;
	template <typename T> using Vec2f = Vec2<float, T>;
	template <typename T> using Vec2d = Vec2<double, T>;
	template <typename T> using Vec2i = Vec2<int32_t, T>;
	template <typename T> using Vec2l = Vec2<int64_t, T>;

	template <Scalar S, typename T> struct Vec3;
	template <typename T> using Vec3f = Vec3<float, T>;
	template <typename T> using Vec3d = Vec3<double, T>;
	template <typename T> using Vec3i = Vec3<int32_t, T>;
	template <typename T> using Vec3l = Vec3<int64_t, T>;

	template <Scalar S, typename T> struct Vec4;
	template <typename T> using Vec4f = Vec4<float, T>;
	template <typename T> using Vec4d = Vec4<double, T>;
	template <typename T> using Vec4i = Vec4<int32_t, T>;
	template <typename T> using Vec4l = Vec4<int64_t, T>;

	template <Scalar S, typename T> struct Vec8;
	template <typename T> using Vec8f = Vec8<float, T>;
	template <typename T> using Vec8d = Vec8<double, T>;
	template <typename T> using Vec8i = Vec8<int32_t, T>;
	template <typename T> using Vec8l = Vec8<int64_t, T>;

	template <Scalar S, typename A, typename B> struct Mat2x2;
	template <typename A, typename B> using Mat2x2f = Mat2x2<float, A, B>;
	template <typename A, typename B> using Mat2x2d = Mat2x2<double, A, B>;
	template <typename A, typename B> using Mat2x2i = Mat2x2<int32_t, A, B>;
	template <typename A, typename B> using Mat2x2l = Mat2x2<int64_t, A, B>;

	template <Scalar S, typename A, typename B> struct Mat3x4;
	template <typename A, typename B> using Mat3x4f = Mat3x4<float, A, B>;
	template <typename A, typename B> using Mat3x4d = Mat3x4<double, A, B>;
	template <typename A, typename B> using Mat3x4i = Mat3x4<int32_t, A, B>;
	template <typename A, typename B> using Mat3x4l = Mat3x4<int64_t, A, B>;

	template <Scalar S, typename A, typename B> struct Mat4x4;
	template <typename A, typename B> using Mat4x4f = Mat4x4<float, A, B>;
	template <typename A, typename B> using Mat4x4d = Mat4x4<double, A, B>;
	template <typename A, typename B> using Mat4x4i = Mat4x4<int32_t, A, B>;
	template <typename A, typename B> using Mat4x4l = Mat4x4<int64_t, A, B>;

	template <Scalar S, typename A, typename B> struct Mat6x8;
	template <typename A, typename B> using Mat6x8f = Mat6x8<float, A, B>;
	template <typename A, typename B> using Mat6x8d = Mat6x8<double, A, B>;
	template <typename A, typename B> using Mat6x8i = Mat6x8<int32_t, A, B>;
	template <typename A, typename B> using Mat6x8l = Mat6x8<int64_t, A, B>;

	template <Scalar S, typename A, typename B> struct Quat;
	template <typename A, typename B> using Quatf = Quat<float, A, B>;
	template <typename A, typename B> using Quatd = Quat<double, A, B>;

	template <Scalar S, typename A, typename B> struct Xform;
	template <typename A, typename B> using Xformf = Xform<float, A, B>;
	template <typename A, typename B> using Xformd = Xform<double, A, B>;

	struct BBox;
	struct BSphere;
	struct OBox;
	template <typename V> struct Rectangle;
	struct Line3;
	struct ISize;
	struct Frustum;

	namespace maths
	{
		// Dependent false
		template <typename T>
		concept always_false = false;

		// The 'is_vec' trait means, "Can be converted to a N component vector"
		// Don't specialise this for scalars because that could lead to accidental use of vectors in scalar functions.
		template <typename T> struct is_vec :std::false_type
		{
			// The type of the x,y,etc elements of the vector (can be vectors for matrix types)
			using elem_type = void;

			// The type of the components (typically int, float, double, etc)
			using comp_type = void;

			// The dimension of the vector
			static int const dim = 0;
		};
		template <typename T> using vec_elem_t = typename is_vec<T>::elem_type;
		template <typename T> using vec_comp_t = typename is_vec<T>::comp_type;

		// Concepts of vector types
		template <typename V> concept VectorX = is_vec<V>::dim >= 1;
		template <typename V> concept Vector2 = is_vec<V>::dim >= 2;
		template <typename V> concept Vector3 = is_vec<V>::dim >= 3;
		template <typename V> concept Vector4 = is_vec<V>::dim >= 4;
		template <typename V> concept VectorFP = VectorX<V> && std::floating_point<vec_comp_t<V>>;
		template <typename V> concept VectorIg = VectorX<V> && std::integral<vec_comp_t<V>>;

		// Concepts of matrix types
		template <typename M> concept MatrixX = is_vec<M>::dim >= 1 && VectorX<vec_elem_t<M>>;
		template <typename M> concept Matrix2 = is_vec<M>::dim >= 2 && VectorX<vec_elem_t<M>>;
		template <typename M> concept Matrix3 = is_vec<M>::dim >= 3 && VectorX<vec_elem_t<M>>;
		template <typename M> concept Matrix4 = is_vec<M>::dim >= 4 && VectorX<vec_elem_t<M>>;
		template <typename M> concept MatrixFP = MatrixX<M> && std::floating_point<vec_comp_t<M>>;
		template <typename M> concept MatrixIg = MatrixX<M> && std::integral<vec_comp_t<M>>;

		// Concepts for all math types
		template <typename T> concept VecOrMatType = VectorX<T> || MatrixX<T>;

		#pragma region Traits
		template <Scalar S, int N> struct is_vec<S[N]>
		{
			using elem_type = S;
			using comp_type = S;
			static int const dim = N;
		};
		template <Scalar S, int N> struct is_vec<std::array<S,N>>
		{
			using elem_type = S;
			using comp_type = S;
			static int const dim = N;
		};
		template <Scalar S, typename T> struct is_vec<Vec2<S,T>> :std::true_type
		{
			using elem_type = S;
			using comp_type = S;
			static int const dim = 2;
		};
		template <Scalar S, typename T> struct is_vec<Vec3<S,T>> :std::true_type
		{
			using elem_type = S;
			using comp_type = S;
			static int const dim = 3;
		};
		template <Scalar S, typename T> struct is_vec<Vec4<S,T>> :std::true_type
		{
			using elem_type = S;
			using comp_type = S;
			static int const dim = 4;
		};
		template <Scalar S, typename T> struct is_vec<Vec8<S,T>> :std::true_type
		{
			using elem_type = S;
			using comp_type = S;
			static int const dim = 8;
		};
		template <Scalar S, typename A, typename B> struct is_vec<Mat2x2<S,A,B>> :std::true_type
		{
			using elem_type = Vec2<S, void>;
			using comp_type = S;
			static int const dim = 2;
		};
		template <Scalar S, typename A, typename B> struct is_vec<Mat3x4<S,A,B>> :std::true_type
		{
			using elem_type = Vec4<S, void>;
			using comp_type = S;
			static int const dim = 3;
		};
		template <Scalar S, typename A, typename B> struct is_vec<Mat4x4<S,A,B>> :std::true_type
		{
			using elem_type = Vec4<S, void>;
			using comp_type = S;
			static int const dim = 4;
		};
		template <Scalar S, typename A, typename B> struct is_vec<Mat6x8<S,A,B>> :std::true_type
		{
			using elem_type = Vec8<S, void>;
			using comp_type = S;
			static int const dim = 6;
		};
		template <Scalar S, typename A, typename B> struct is_vec<Quat<S,A,B>> :std::true_type
		{
			using elem_type = S;
			using comp_type = S;
			static int const dim = 4;
		};

		// Checks
		static_assert(!VectorX<char*>);
		static_assert(!VectorX<wchar_t*>);
		static_assert(!VectorX<float*>);
		static_assert(!VectorX<int*>);
		static_assert(Vector2<float[2]>);
		static_assert(Vector2<int[2]>);
		#pragma endregion

		// Component accessor with default for out-of-bounds
		template <int idx, maths::VectorX V> constexpr maths::vec_elem_t<V> comp(V const& v)
		{
			if constexpr (maths::is_vec<V>::dim > idx)
				return v[idx];
			else
				return maths::vec_elem_t<V>{};
		}
		template <int idx, typename E, maths::VectorX V> constexpr E comp(V const& v)
		{
			if constexpr (maths::is_vec<V>::dim > idx)
				return static_cast<E>(v[idx]);
			else
				return E{};
		}
	}

	// Old names
	using v2 = Vec2<float, void>;
	using v3 = Vec3<float, void>;
	using v4 = Vec4<float, void>;
	using v8 = Vec8<float, void>;
	using quat = Quat<float, void, void>;
	using xform = Xform<float, void, void>;
	using m2x2 = Mat2x2<float, void, void>;
	using m3x4 = Mat3x4<float, void, void>;
	using m4x4 = Mat4x4f<void,void>;
	using m6x8 = Mat6x8f<void,void>;
	using iv2 = Vec2<int, void>;
	using iv3 = Vec3i<void>;
	using iv4 = Vec4i<void>;
	using v4 = Vec4<float, void>;

	// Helper trait for 'underlying_type' that works for non-enums as well
	template <typename T, bool = std::is_enum_v<T>> struct underlying_type : std::underlying_type<T> {};
	template <typename T> struct underlying_type<T, false> { using type = T; };
	template <typename T> using underlying_type_t = typename underlying_type<T>::type;

	// Maths library build options
	struct MathsBuildOptions
	{
		int PrMathsUseIntrinsics;

		MathsBuildOptions()
			:PrMathsUseIntrinsics(PR_MATHS_USE_INTRINSICS)
		{}
	};
}
