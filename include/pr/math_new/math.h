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

#include "pr/math_new/core/forward.h"
#include "pr/math_new/core/traits.h"
#include "pr/math_new/core/constants.h"
#include "pr/math_new/core/axis_id.h"
#include "pr/math_new/core/functions.h"
#include "pr/math_new/types/vector2.h"
#include "pr/math_new/types/vector3.h"
#include "pr/math_new/types/vector4.h"
#include "pr/math_new/types/vector8.h"
#include "pr/math_new/types/quaternion.h"
#include "pr/math_new/types/transform.h"
#include "pr/math_new/types/matrix2x2.h"
#include "pr/math_new/types/matrix3x4.h"
#include "pr/math_new/types/matrix4x4.h"
#include "pr/math_new/types/matrix6x8.h"
#include "pr/math_new/types/matrix.h"
#include "pr/math_new/types/half.h"
#include "pr/math_new/primitives/bsphere.h"
#include "pr/math_new/primitives/bbox.h"
#include "pr/math_new/primitives/oriented_box.h"
#include "pr/math_new/primitives/rectangle.h"
#include "pr/math_new/primitives/plane.h"
#include "pr/math_new/primitives/spline.h"
#include "pr/math_new/primitives/frustum.h"
#include "pr/math_new/spatial/spatial.h"
#include "pr/math_new/stats/stat.h"
#include "pr/math_new/utils/primes.h"
#include "pr/math_new/utils/interpolate.h"
#include "pr/math_new/utils/dynamics.h"
#include "pr/math_new/utils/polynomial.h"
// No non-standard dependencies outside of './'

// Depends on pr/common/to.h and pr/str/ — not part of math_new core
//#include "pr/math_new/utils/conversion.h"
