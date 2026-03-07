//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"

namespace pr::math
{
	// Specialise these traits as needed for your type.
	template <typename Vec> struct vector_traits
	{
		using element_t = void;
		using component_t = void;
		inline static constexpr int dimension = 0;
		inline static constexpr bool is_vector_v = false;
		inline static constexpr bool is_quaternion_v = false;
	};

	// Concept for vector-like types. Note, can be rank-1 (e.g. Vec3) or rank-2 (e.g. Mat4x4) vectors
	template <typename T>
	concept VectorType = vector_traits<std::remove_cv_t<T>>::is_vector_v;

	// Concept for vector-like types with dimension N
	template <typename T, int N>
	concept VectorTypeN = VectorType<T> && vector_traits<std::remove_cv_t<T>>::dimension == N;

	// Concept for floating point vectors
	template <typename T>
	concept VectorTypeFP = VectorType<T> && std::floating_point<typename vector_traits<std::remove_cv_t<T>>::element_t>;

	// Concept for quaternion-like types
	template <typename T>
	concept QuaternionType = vector_traits<std::remove_cv_t<T>>::is_quaternion_v;

	// Concept for either a vector or quaternion-like type
	template <typename T>
	concept TensorType = VectorType<T> || QuaternionType<T>;

	// Concept for either a vector or quaternion-like type with a floating point element type
	template <typename T>
	concept TensorTypeFP = VectorTypeFP<T> || QuaternionType<T>;

	// Concept for rank-1 vectors
	template <typename T>
	concept IsRank1 = VectorType<T> && !VectorType<typename vector_traits<T>::component_t>;

	// Concept for rank-2 vectors
	template <typename T>
	concept IsRank2 = VectorType<T> && VectorType<typename vector_traits<T>::component_t> && !VectorType<typename vector_traits<typename vector_traits<T>::component_t>::component_t>;

	// Concept to ensure two vector types have the same element type
	template <typename T, typename U>
	concept SameS = TensorType<T> && TensorType<U> && std::is_same_v<typename vector_traits<std::remove_cv_t<T>>::element_t, typename vector_traits<std::remove_cv_t<U>>::element_t>;

	// Concept for vector types that support array access (e.g. m[i])
	template <typename T>
	concept ArrayAccess = VectorType<T> && (
		(IsRank1<T> && requires (T t, int i) { t[i]; }) ||
		(IsRank2<T> && requires (T t, int i) { t[i][i]; })
	);

	// Use these types for standard attributes
	template <typename ElementType, typename ComponentType, int Dim> struct vector_traits_base
	{
		using element_t = ElementType;
		using component_t = ComponentType;
		inline static constexpr int dimension = Dim;
		inline static constexpr bool is_vector_v = true;
		inline static constexpr bool is_quaternion_v = false;
	};
	template <typename ElementType> struct quaternion_traits_base
	{
		using element_t = ElementType;
		using component_t = ElementType;
		inline static constexpr int dimension = 4;
		inline static constexpr bool is_vector_v = false;
		inline static constexpr bool is_quaternion_v = true;
	};

	// Adapters for accessing the members of typical vector types
	template <typename Vec, typename ElementType, int Dim> struct vector_access_member
	{
		static constexpr ElementType x(Vec const& v) requires (Dim > 0) { return v.x; }
		static constexpr ElementType y(Vec const& v) requires (Dim > 1) { return v.y; }
		static constexpr ElementType z(Vec const& v) requires (Dim > 2) { return v.z; }
		static constexpr ElementType w(Vec const& v) requires (Dim > 3) { return v.w; }

		static constexpr ElementType& x(Vec& v) requires (Dim > 0) { return v.x; }
		static constexpr ElementType& y(Vec& v) requires (Dim > 1) { return v.y; }
		static constexpr ElementType& z(Vec& v) requires (Dim > 2) { return v.z; }
		static constexpr ElementType& w(Vec& v) requires (Dim > 3) { return v.w; }
	};
	template <typename Vec, typename ElementType, int Dim> struct vector_access_MEMBER
	{
		static constexpr ElementType x(Vec const& v) requires (Dim > 0) { return v.X; }
		static constexpr ElementType y(Vec const& v) requires (Dim > 1) { return v.Y; }
		static constexpr ElementType z(Vec const& v) requires (Dim > 2) { return v.Z; }
		static constexpr ElementType w(Vec const& v) requires (Dim > 3) { return v.W; }

		static constexpr ElementType& x(Vec& v) requires (Dim > 0) { return v.X; }
		static constexpr ElementType& y(Vec& v) requires (Dim > 1) { return v.Y; }
		static constexpr ElementType& z(Vec& v) requires (Dim > 2) { return v.Z; }
		static constexpr ElementType& w(Vec& v) requires (Dim > 3) { return v.W; }
	};
	template <typename Vec, typename ElementType, int Dim> struct vector_access_array
	{
		static constexpr ElementType x(Vec const& v) requires (Dim > 0) { return v[0]; }
		static constexpr ElementType y(Vec const& v) requires (Dim > 1) { return v[1]; }
		static constexpr ElementType z(Vec const& v) requires (Dim > 2) { return v[2]; }
		static constexpr ElementType w(Vec const& v) requires (Dim > 3) { return v[3]; }

		static constexpr ElementType& x(Vec& v) requires (Dim > 0) { return v[0]; }
		static constexpr ElementType& y(Vec& v) requires (Dim > 1) { return v[1]; }
		static constexpr ElementType& z(Vec& v) requires (Dim > 2) { return v[2]; }
		static constexpr ElementType& w(Vec& v) requires (Dim > 3) { return v[3]; }
	};

	// Vector component access
	template <TensorType Vec> [[msvc::forceinline]] constexpr auto vec(Vec& v) noexcept
	{
		using vt = vector_traits<std::remove_cv_t<Vec>>;
		using S = std::conditional_t<std::is_const_v<Vec>, typename vt::component_t, typename vt::component_t&>;
		struct Proxy1 { S x; };
		struct Proxy2 { S x; S y; };
		struct Proxy3 { S x; S y; S z; };
		struct Proxy4 { S x; S y; S z; S w; };

		if constexpr (vt::dimension == 0) return;
		else if constexpr (vt::dimension == 1) return Proxy1{vt::x(v)};
		else if constexpr (vt::dimension == 2) return Proxy2{vt::x(v), vt::y(v)};
		else if constexpr (vt::dimension == 3) return Proxy3{vt::x(v), vt::y(v), vt::z(v)};
		else if constexpr (vt::dimension == 4) return Proxy4{vt::x(v), vt::y(v), vt::z(v), vt::w(v)};
		else static_assert(vt::dimension <= 4);
	}
	template <TensorType Vec> [[msvc::forceinline]] constexpr auto vec(Vec&& v) noexcept
	{
		using vt = vector_traits<std::remove_cv_t<Vec>>;
		using S = typename vt::component_t;
		struct Proxy1 { S x; };
		struct Proxy2 { S x; S y; };
		struct Proxy3 { S x; S y; S z; };
		struct Proxy4 { S x; S y; S z; S w; };

		if constexpr (vt::dimension == 0) return;
		else if constexpr (vt::dimension == 1) return Proxy1{vt::x(v)};
		else if constexpr (vt::dimension == 2) return Proxy2{vt::x(v), vt::y(v)};
		else if constexpr (vt::dimension == 3) return Proxy3{vt::x(v), vt::y(v), vt::z(v)};
		else if constexpr (vt::dimension == 4) return Proxy4{vt::x(v), vt::y(v), vt::z(v), vt::w(v)};
		else static_assert(vt::dimension <= 4);
	}
}
