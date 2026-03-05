//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once

#include "pr/maths/forward.h"
#include "pr/maths/vector2.h"
#include "pr/maths/vector3.h"
#include "pr/maths/vector4.h"
#include "pr/maths/vector8.h"
#include "pr/maths/matrix2x2.h"
#include "pr/maths/matrix3x4.h"
#include "pr/maths/matrix4x4.h"
#include "pr/maths/constants_vector.h"

namespace std
{
	template <typename Scalar, typename T>
	class numeric_limits<pr::Vec2<Scalar, T>>
	{
	public:
		static pr::Vec2<Scalar, T> min() throw()     { return pr::Vec2<Scalar, T>::Min(); }
		static pr::Vec2<Scalar, T> max() throw()     { return pr::Vec2<Scalar, T>::Max(); }
		static pr::Vec2<Scalar, T> lowest() throw()  { return pr::Vec2<Scalar, T>::Lowest(); }
		static pr::Vec2<Scalar, T> epsilon() throw() { return pr::Vec2<Scalar, T>::Epsilon(); }

		static const bool is_specialized = true;
		static const bool is_signed = true;
		static const bool is_integer = false;
		static const bool is_exact = false;
		static const bool has_infinity = false;
		static const bool has_quiet_NaN = false;
		static const bool has_signaling_NaN = false;
		static const bool has_denorm_loss = true;
		static const int radix = 10;
	};

	template <typename Scalar, typename T>
	class numeric_limits<pr::Vec3<Scalar, T>>
	{
	public:
		static pr::Vec3<Scalar, T> min() throw()     { return pr::Vec3<Scalar, T>:: Min(); }
		static pr::Vec3<Scalar, T> max() throw()     { return pr::Vec3<Scalar, T>:: Max(); }
		static pr::Vec3<Scalar, T> lowest() throw()  { return pr::Vec3<Scalar, T>:: Lowest(); }
		static pr::Vec3<Scalar, T> epsilon() throw() { return pr::Vec3<Scalar, T>:: Epsilon(); }

		static const bool is_specialized = true;
		static const bool is_signed = true;
		static const bool is_integer = false;
		static const bool is_exact = false;
		static const bool has_infinity = false;
		static const bool has_quiet_NaN = false;
		static const bool has_signaling_NaN = false;
		static const bool has_denorm_loss = true;
		static const int radix = 10;
	};

	template <typename T>
	class numeric_limits<pr::Vec4f<T>>
	{
	public:
		static pr::Vec4f<T> min() throw()     { return pr::v4Min; }
		static pr::Vec4f<T> max() throw()     { return pr::v4::Max(); }
		static pr::Vec4f<T> lowest() throw()  { return pr::v4Lowest; }
		static pr::Vec4f<T> epsilon() throw() { return pr::v4Epsilon; }

		static const bool is_specialized = true;
		static const bool is_signed = true;
		static const bool is_integer = false;
		static const bool is_exact = false;
		static const bool has_infinity = false;
		static const bool has_quiet_NaN = false;
		static const bool has_signaling_NaN = false;
		static const bool has_denorm_loss = true;
		static const int radix = 10;
	};

	template <typename A, typename B> 
	class std::numeric_limits<pr::Mat2x2f<A,B>>
	{
	public:
		static pr::Mat2x2f<A,B> min() throw()     { return pr::m2x2Min; }
		static pr::Mat2x2f<A,B> max() throw()     { return pr::m2x2Max; }
		static pr::Mat2x2f<A,B> lowest() throw()  { return pr::m2x2Lowest; }
		static pr::Mat2x2f<A,B> epsilon() throw() { return pr::m2x2Epsilon; }

		static const bool is_specialized = true;
		static const bool is_signed = true;
		static const bool is_integer = false;
		static const bool is_exact = false;
		static const bool has_infinity = false;
		static const bool has_quiet_NaN = false;
		static const bool has_signaling_NaN = false;
		static const bool has_denorm_loss = true;
		static const int radix = 10;
	};


	template <typename A, typename B> 
	class std::numeric_limits<pr::Mat3x4f<A,B>>
	{
	public:
		static pr::Mat3x4f<A,B> min() throw()     { return pr::m3x4Min; }
		static pr::Mat3x4f<A,B> max() throw()     { return pr::m3x4Max; }
		static pr::Mat3x4f<A,B> lowest() throw()  { return pr::m3x4Lowest; }
		static pr::Mat3x4f<A,B> epsilon() throw() { return pr::m3x4Epsilon; }

		static const bool is_specialized = true;
		static const bool is_signed = true;
		static const bool is_integer = false;
		static const bool is_exact = false;
		static const bool has_infinity = false;
		static const bool has_quiet_NaN = false;
		static const bool has_signaling_NaN = false;
		static const bool has_denorm_loss = true;
		static const int radix = 10;
	};

	template <typename A, typename B> 
	class std::numeric_limits<pr::Mat4x4f<A,B>>
	{
	public:
		static pr::Mat4x4f<A,B> min() throw()     { return pr::m4x4Min; }
		static pr::Mat4x4f<A,B> max() throw()     { return pr::m4x4Max; }
		static pr::Mat4x4f<A,B> lowest() throw()  { return pr::m4x4Lowest; }
		static pr::Mat4x4f<A,B> epsilon() throw() { return pr::m4x4Epsilon; }

		static const bool is_specialized = true;
		static const bool is_signed = true;
		static const bool is_integer = false;
		static const bool is_exact = false;
		static const bool has_infinity = false;
		static const bool has_quiet_NaN = false;
		static const bool has_signaling_NaN = false;
		static const bool has_denorm_loss = true;
		static const int radix = 10;
	};

	template <typename T>
	class numeric_limits<pr::Vec2i<T>>
	{
	public:
		static pr::Vec2i<T> min() throw()     { return pr::iv2Min; }
		static pr::Vec2i<T> max() throw()     { return pr::iv2Max; }
		static pr::Vec2i<T> lowest() throw()  { return pr::iv2Lowest; }

		static const bool is_specialized = true;
		static const bool is_signed = true;
		static const bool is_integer = true;
		static const bool is_exact = true;
		static const bool has_infinity = false;
		static const bool has_quiet_NaN = false;
		static const bool has_signaling_NaN = false;
		static const bool has_denorm_loss = false;
		static const int radix = 10;
	};

	template <typename T>
	class numeric_limits<pr::Vec3i<T>>
	{
	public:
		static pr::Vec3i<T> min() throw()     { return pr::iv3Min; }
		static pr::Vec3i<T> max() throw()     { return pr::iv3Max; }
		static pr::Vec3i<T> lowest() throw()  { return pr::iv3Lowest; }

		static const bool is_specialized = true;
		static const bool is_signed = true;
		static const bool is_integer = true;
		static const bool is_exact = true;
		static const bool has_infinity = false;
		static const bool has_quiet_NaN = false;
		static const bool has_signaling_NaN = false;
		static const bool has_denorm_loss = false;
		static const int radix = 10;
	};

	template <typename T>
	class numeric_limits<pr::Vec4i<T>>
	{
	public:
		static pr::Vec4i<T> min() throw()     { return pr::iv4Min; }
		static pr::Vec4i<T> max() throw()     { return pr::iv4Max; }
		static pr::Vec4i<T> lowest() throw()  { return pr::iv4Lowest; }

		static const bool is_specialized = true;
		static const bool is_signed = true;
		static const bool is_integer = true;
		static const bool is_exact = true;
		static const bool has_infinity = false;
		static const bool has_quiet_NaN = false;
		static const bool has_signaling_NaN = false;
		static const bool has_denorm_loss = false;
		static const int radix = 10;
	};
}
