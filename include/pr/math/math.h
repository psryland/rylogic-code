//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once

// Define switches:
// PR_MATHS_USE_DIRECTMATH
// PR_MATHS_USE_INTRINSICS
// Also remember NOMINMAX

// Notes:
//  - Always pass vectors by value, not by const&. Even if intrinsics are not used
//    the optimiser will pack the float[] into a vector register to pass it to a function call.
//  - Pass matrix types by const&. Basically if the type is larger than 2 registers, pass by const&.
//  - Functions are implemented for vector concepts, not specific types. So that they can be used on
//    any type that conforms to the vector concept
//  - Type constants (e.g. Vec3::Zero()) return a const reference so that the constant value has an address.
//    These can't be compiled-time constants. The constant functions (e.g. Zero<Vec3>()) return by value so
//    that they can be used in compile-time contexts.

// Design Goals:
//  - Vector types and functions should be independent. Functions should work for any type that meets
//    the vector concept.
//  - Vector types do not contain functions because they would be limited to just that type. They can,
//    however, contain methods that forward to global functions (e.g. Vec3::Length() forwards to Length(Vec3)).

// Checklist:
//  - Check each function for mathematical correctness and numerical stability.
//  - Look for any patterns that appear to be broken, or inconsistent style.
//  - Look for any missing constexpr opportunities, or other qualifiers that could be added to improve the code/performance (e.g. noexcept, [[nodiscard]], etc).
//  - Check that pr_assert is used rather than assert
//  - Check for consistency with the 'Design Goals'
//  - Check that 'pr_vectorcall' is used correctly for all vector functions.
//  - Check that vector types (i.e. Rank1) are passed by value, and that matrix types (i.e. Rank2) are passed by reference
//  - Check that constants and static factory functions within the vector and matrix types are all thin wrappers around functions from 'functions.h' and constants from 'constants.h'.
//  - Check for any duplication that should be removed.
//  - Pure functions shouldn't throw exceptions, but should use assert instead to allow noexcept to be used.

#include "pr/math/core/forward.h"
#include "pr/math/core/traits.h"
#include "pr/math/core/constants.h"
#include "pr/math/core/axis_id.h"
#include "pr/math/core/functions.h"
#include "pr/math/types/vector2.h"
#include "pr/math/types/vector3.h"
#include "pr/math/types/vector4.h"
#include "pr/math/types/vector8.h"
#include "pr/math/types/quaternion.h"
#include "pr/math/types/transform.h"
#include "pr/math/types/matrix2x2.h"
#include "pr/math/types/matrix3x4.h"
#include "pr/math/types/matrix4x4.h"
#include "pr/math/types/matrix6x8.h"
#include "pr/math/types/matrix.h"
#include "pr/math/types/half.h"
#include "pr/math/primitives/bsphere.h"
#include "pr/math/primitives/bbox.h"
#include "pr/math/primitives/oriented_box.h"
#include "pr/math/primitives/rectangle.h"
#include "pr/math/primitives/plane.h"
#include "pr/math/primitives/spline.h"
#include "pr/math/primitives/frustum.h"
#include "pr/math/spatial/spatial.h"
#include "pr/math/stats/stat.h"
#include "pr/math/utils/primes.h"
#include "pr/math/utils/interpolate.h"
#include "pr/math/utils/dynamics.h"
#include "pr/math/utils/polynomial.h"
// No non-standard dependencies outside of './'

// Depends on pr/common/to.h and pr/str/ — not part of math_new core
//#include "pr/math/conversion.h"

namespace pr
{
	using namespace math;

	using v2 = math::Vec2<float>;
	using v3 = math::Vec3<float>;
	using v4 = math::Vec4<float>;
	using v8 = math::Vec8<float, void>;
	using iv2 = math::Vec2<int32_t>;
	using iv3 = math::Vec3<int32_t>;
	using iv4 = math::Vec4<int32_t>;
	using quat = math::Quat<float>;
	using xform = math::Xform<float>;
	using m2x2 = math::Mat2x2<float>;
	using m3x4 = math::Mat3x4<float>;
	using m4x4 = math::Mat4x4<float>;
	using m6x8 = math::Mat6x8<float, void, void>;
	using BBox = math::BoundingBox<float>;
	using BSphere = math::BoundingSphere<float>;
	using OBox = math::OrientedBox<float>;
	using Plane = math::Plane3<float>;
	using Frustum = math::Frustum3<float>;

	using v8motion = math::Vec8<float, math::spatial::Motion>;
	using v8force = math::Vec8<float, math::spatial::Force>;
}