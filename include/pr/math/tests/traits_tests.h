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
	// Compile-time concept tests for cv/ref/volatile-qualified vector types.
	// All assertions are static_assert; any regression is a build error, not a runtime failure.
	namespace
	{
		using v3f = Vec3<float>;
		using m3f = Mat3x3<float>;
		using qf  = Quat<float>;

		// --- VectorType ---
		static_assert(VectorType<v3f>);
		static_assert(VectorType<v3f const>);
		static_assert(VectorType<v3f volatile>);
		static_assert(VectorType<v3f&>);
		static_assert(VectorType<v3f const&>);
		static_assert(VectorType<v3f&&>);
		static_assert(VectorType<m3f>);
		static_assert(VectorType<m3f const>);
		static_assert(VectorType<m3f volatile>);
		static_assert(!VectorType<float>);
		static_assert(!VectorType<int>);

		// --- IsRank1 (scalar elements, e.g. Vec3) ---
		static_assert(IsRank1<v3f>);
		static_assert(IsRank1<v3f const>);
		static_assert(IsRank1<v3f volatile>);
		static_assert(IsRank1<v3f&>);
		static_assert(IsRank1<v3f const&>);
		// Negative: rank-2 types must not satisfy IsRank1 under any qualifier
		static_assert(!IsRank1<m3f>);
		static_assert(!IsRank1<m3f const>);
		static_assert(!IsRank1<m3f volatile>);
		static_assert(!IsRank1<m3f&>);
		static_assert(!IsRank1<m3f const&>);

		// --- IsRank2 (vector elements, e.g. Mat3x3) ---
		static_assert(IsRank2<m3f>);
		static_assert(IsRank2<m3f const>);
		static_assert(IsRank2<m3f volatile>);
		static_assert(IsRank2<m3f&>);
		static_assert(IsRank2<m3f const&>);
		// Negative: rank-1 types must not satisfy IsRank2 under any qualifier
		static_assert(!IsRank2<v3f>);
		static_assert(!IsRank2<v3f const>);
		static_assert(!IsRank2<v3f volatile>);
		static_assert(!IsRank2<v3f&>);
		static_assert(!IsRank2<v3f const&>);

		// --- QuaternionType ---
		static_assert(QuaternionType<qf>);
		static_assert(QuaternionType<qf const>);
		static_assert(QuaternionType<qf volatile>);
		static_assert(QuaternionType<qf&>);
		static_assert(QuaternionType<qf const&>);
		static_assert(!QuaternionType<v3f>);
		static_assert(!QuaternionType<m3f>);

		// --- SameS (same scalar element type; both operands must be vector or quaternion types) ---
		static_assert(SameS<v3f, v3f>);
		static_assert(SameS<v3f const, v3f>);
		static_assert(SameS<v3f volatile, v3f>);
		static_assert(SameS<v3f&, v3f>);
		static_assert(SameS<v3f const&, v3f>);
		static_assert(SameS<m3f, v3f>);        // both float
		static_assert(SameS<m3f const, v3f>);
		// Negative: different scalar types
		static_assert(!SameS<v3f, Vec3<double>>);
		static_assert(!SameS<v3f const, Vec3<double>>);
		// Negative: plain scalars satisfy neither VectorType nor QuaternionType
		static_assert(!SameS<float, int>);
		static_assert(!SameS<float, v3f>);
		static_assert(!SameS<v3f, int>);

		// --- ArrayAccess ---
		// rank-1: operator[] has const and non-const overloads only; volatile is not supported
		static_assert(ArrayAccess<v3f>);
		static_assert(ArrayAccess<v3f const>);
		static_assert(ArrayAccess<v3f&>);
		static_assert(ArrayAccess<v3f const&>);
		static_assert(!ArrayAccess<v3f volatile>);
		// rank-2: same pattern
		static_assert(ArrayAccess<m3f>);
		static_assert(ArrayAccess<m3f const>);
		static_assert(ArrayAccess<m3f&>);
		static_assert(ArrayAccess<m3f const&>);
		static_assert(!ArrayAccess<m3f volatile>);
	}
}
#endif
