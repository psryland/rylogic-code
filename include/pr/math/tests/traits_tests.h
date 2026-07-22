//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/math.h"

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	// Compile-time concept tests for cv/ref-qualified vector types.
	//
	// Before the fix, nested vector_traits<T> lookups inside IsRank1/IsRank2 did not
	// strip cv/ref from T. For a qualified type such as Mat3x3<float> const, there is
	// no specialization of vector_traits for that exact type, so the base template fires
	// and returns component_t = void. That caused:
	//   IsRank1<Mat3x3<float> const>  = true   (wrong: it is rank-2)
	//   IsRank2<Mat3x3<float> const>  = false  (wrong: it is rank-2)
	// Similarly, VectorType and related concepts used remove_cv_t but not remove_reference_t,
	// so lvalue/rvalue reference-qualified types also failed top-level classification.
	//
	// All assertions here are compile-time (static_assert). Any regression immediately
	// produces a build error rather than a runtime failure.

	namespace
	{
		// Aliases used throughout the assertions
		using v3f = Vec3<float>;
		using m3f = Mat3x3<float>;
		using qf  = Quat<float>;

		// --- VectorType ---
		// Unqualified
		static_assert(VectorType<v3f>);
		static_assert(VectorType<m3f>);
		// const-qualified
		static_assert(VectorType<v3f const>);
		static_assert(VectorType<m3f const>);
		// lvalue-reference-qualified
		static_assert(VectorType<v3f&>);
		static_assert(VectorType<m3f&>);
		// const lvalue-reference-qualified
		static_assert(VectorType<v3f const&>);
		static_assert(VectorType<m3f const&>);
		// rvalue-reference-qualified
		static_assert(VectorType<v3f&&>);
		// Negative: plain scalars are not vectors
		static_assert(!VectorType<float>);
		static_assert(!VectorType<int>);

		// --- IsRank1 (rank-1 = scalar elements, e.g. Vec3) ---
		// Unqualified: these always worked
		static_assert(IsRank1<v3f>);
		static_assert(!IsRank1<m3f>);
		// const-qualified: key regression cases
		static_assert(IsRank1<v3f const>);
		static_assert(!IsRank1<m3f const>);   // was wrongly true before the fix
		// reference-qualified
		static_assert(IsRank1<v3f&>);
		static_assert(IsRank1<v3f const&>);
		// Negative: rank-2 types must not satisfy IsRank1 under any qualifier
		static_assert(!IsRank1<m3f&>);
		static_assert(!IsRank1<m3f const&>);

		// --- IsRank2 (rank-2 = vector elements, e.g. Mat3x3) ---
		// Unqualified: these always worked
		static_assert(IsRank2<m3f>);
		static_assert(!IsRank2<v3f>);
		// const-qualified: key regression cases
		static_assert(IsRank2<m3f const>);    // was wrongly false before the fix
		static_assert(!IsRank2<v3f const>);
		// reference-qualified
		static_assert(IsRank2<m3f&>);
		static_assert(IsRank2<m3f const&>);
		// Negative: rank-1 types must not satisfy IsRank2 under any qualifier
		static_assert(!IsRank2<v3f&>);
		static_assert(!IsRank2<v3f const&>);

		// --- QuaternionType ---
		static_assert(QuaternionType<qf>);
		static_assert(QuaternionType<qf const>);
		static_assert(QuaternionType<qf&>);
		static_assert(QuaternionType<qf const&>);
		// Negative: vectors are not quaternions
		static_assert(!QuaternionType<v3f>);
		static_assert(!QuaternionType<m3f>);

		// --- SameS (same scalar element type) ---
		// Qualifiers must not affect the element comparison
		static_assert(SameS<v3f, v3f>);
		static_assert(SameS<v3f const, v3f>);
		static_assert(SameS<v3f&, v3f>);
		static_assert(SameS<v3f const&, v3f>);
		static_assert(SameS<m3f, v3f>);          // both float
		static_assert(SameS<m3f const, v3f>);
		// Negative: different scalar types
		static_assert(!SameS<v3f, Vec3<double>>);
		static_assert(!SameS<v3f const, Vec3<double>>);

		// --- ArrayAccess ---
		// rank-1: supports t[i]
		static_assert(ArrayAccess<v3f>);
		static_assert(ArrayAccess<v3f const>);
		// rank-2: supports t[i][i]. Requires IsRank2<T> to be correct for cv/ref qualifiers.
		static_assert(ArrayAccess<m3f>);
		static_assert(ArrayAccess<m3f const>);    // needed IsRank2<m3f const> = true
	}

	// Trivial runtime test class so this file appears in the test runner output.
	PRUnitTestClass(VectorTraits)
	{
		// All concept assertions are static_assert at namespace scope above.
		// Any failure is a compile error rather than a runtime failure.
		PRUnitTestMethod(CvRefClassification)
		{
		}
	};
}
#endif
