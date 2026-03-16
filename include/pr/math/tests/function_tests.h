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
	PRUnitTestClass(FunctionTests)
	{
		// Notes:
		//  - Tests for functions in functions.h
		//  - Tests are ordered to match the function order in functions.h

		template <VectorType Vec> static constexpr bool Eql(Vec lhs, Vec rhs)
		{
			using vt = vector_traits<Vec>;

			if constexpr (std::floating_point<typename vt::element_t>)
				return FEql(lhs, rhs);
			else
				return lhs == rhs;
		};

		PRUnitTestMethod(ExplicitCasts
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = typename vt::element_t;

			if constexpr (IsRank1<vec_t>)
			{
				constexpr auto dim = vt::dimension;

				// Cast to float
				if constexpr (!std::is_same_v<S, float>)
				{
					auto v = vec_t(S(1));
					if constexpr (dim == 2) { auto c = static_cast<Vec2<float>>(v); PR_EXPECT(Eql(c, Vec2<float>(1.0f))); }
					if constexpr (dim == 3) { auto c = static_cast<Vec3<float>>(v); PR_EXPECT(Eql(c, Vec3<float>(1.0f))); }
					if constexpr (dim == 4) { auto c = static_cast<Vec4<float>>(v); PR_EXPECT(Eql(c, Vec4<float>(1.0f))); }
				}

				// Cast to double
				if constexpr (!std::is_same_v<S, double>)
				{
					auto v = vec_t(S(1));
					if constexpr (dim == 2) { auto c = static_cast<Vec2<double>>(v); PR_EXPECT(Eql(c, Vec2<double>(1.0))); }
					if constexpr (dim == 3) { auto c = static_cast<Vec3<double>>(v); PR_EXPECT(Eql(c, Vec3<double>(1.0))); }
					if constexpr (dim == 4) { auto c = static_cast<Vec4<double>>(v); PR_EXPECT(Eql(c, Vec4<double>(1.0))); }
				}

				// Cast to int32_t
				if constexpr (!std::is_same_v<S, int32_t>)
				{
					auto v = vec_t(S(2));
					if constexpr (dim == 2) { auto c = static_cast<Vec2<int32_t>>(v); PR_EXPECT(Eql(c, Vec2<int32_t>(2))); }
					if constexpr (dim == 3) { auto c = static_cast<Vec3<int32_t>>(v); PR_EXPECT(Eql(c, Vec3<int32_t>(2))); }
					if constexpr (dim == 4) { auto c = static_cast<Vec4<int32_t>>(v); PR_EXPECT(Eql(c, Vec4<int32_t>(2))); }
				}

				// Cast to int64_t
				if constexpr (!std::is_same_v<S, int64_t>)
				{
					auto v = vec_t(S(2));
					if constexpr (dim == 2) { auto c = static_cast<Vec2<int64_t>>(v); PR_EXPECT(Eql(c, Vec2<int64_t>(2))); }
					if constexpr (dim == 3) { auto c = static_cast<Vec3<int64_t>>(v); PR_EXPECT(Eql(c, Vec3<int64_t>(2))); }
					if constexpr (dim == 4) { auto c = static_cast<Vec4<int64_t>>(v); PR_EXPECT(Eql(c, Vec4<int64_t>(2))); }
				}

				// Verify round-trip cast preserves values
				{
					auto v = vec_t(S(3));
					auto round_trip = v;
					if constexpr (dim == 2) { round_trip = static_cast<vec_t>(static_cast<Vec2<double>>(v)); }
					if constexpr (dim == 3) { round_trip = static_cast<vec_t>(static_cast<Vec3<double>>(v)); }
					if constexpr (dim == 4) { round_trip = static_cast<vec_t>(static_cast<Vec4<double>>(v)); }
					PR_EXPECT(Eql(round_trip, v));
				}
			}
			else if constexpr (IsRank2<vec_t>)
			{
				constexpr auto dim = vt::dimension;

				// Cast to float
				if constexpr (!std::is_same_v<S, float>)
				{
					constexpr auto m = Identity<vec_t>();
					if constexpr (dim == 2) { auto c = static_cast<Mat2x2<float>>(m); PR_EXPECT(c == Mat2x2<float>::Identity()); }
					if constexpr (dim == 3) { auto c = static_cast<Mat3x4<float>>(m); PR_EXPECT(c == Mat3x4<float>::Identity()); }
					if constexpr (dim == 4) { auto c = static_cast<Mat4x4<float>>(m); PR_EXPECT(c == Mat4x4<float>::Identity()); }
				}

				// Cast to double
				if constexpr (!std::is_same_v<S, double>)
				{
					constexpr auto m = Identity<vec_t>();
					if constexpr (dim == 2) { auto c = static_cast<Mat2x2<double>>(m); PR_EXPECT(c == Mat2x2<double>::Identity()); }
					if constexpr (dim == 3) { auto c = static_cast<Mat3x4<double>>(m); PR_EXPECT(c == Mat3x4<double>::Identity()); }
					if constexpr (dim == 4) { auto c = static_cast<Mat4x4<double>>(m); PR_EXPECT(c == Mat4x4<double>::Identity()); }
				}

				// Cast to int32_t
				if constexpr (!std::is_same_v<S, int32_t>)
				{
					constexpr auto m = Identity<vec_t>();
					if constexpr (dim == 2) { auto c = static_cast<Mat2x2<int32_t>>(m); PR_EXPECT(c == Mat2x2<int32_t>::Identity()); }
					if constexpr (dim == 3) { auto c = static_cast<Mat3x4<int32_t>>(m); PR_EXPECT(c == Mat3x4<int32_t>::Identity()); }
					if constexpr (dim == 4) { auto c = static_cast<Mat4x4<int32_t>>(m); PR_EXPECT(c == Mat4x4<int32_t>::Identity()); }
				}

				// Cast to int64_t
				if constexpr (!std::is_same_v<S, int64_t>)
				{
					constexpr auto m = Identity<vec_t>();
					if constexpr (dim == 2) { auto c = static_cast<Mat2x2<int64_t>>(m); PR_EXPECT(c == Mat2x2<int64_t>::Identity()); }
					if constexpr (dim == 3) { auto c = static_cast<Mat3x4<int64_t>>(m); PR_EXPECT(c == Mat3x4<int64_t>::Identity()); }
					if constexpr (dim == 4) { auto c = static_cast<Mat4x4<int64_t>>(m); PR_EXPECT(c == Mat4x4<int64_t>::Identity()); }
				}
			}
		}

		// ---- Operators (functions.h line ~18) ----
		PRUnitTestMethod(Operators
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;

			constexpr auto V0 = vec_t(T(2));
			constexpr auto V1 = vec_t(T(3));

			static_assert(Eql(+V0, vec_t(T(+2))));
			static_assert(Eql(-V0, vec_t(T(-2))));

			static_assert(Eql(V0 + V1, vec_t(T(+5))));
			static_assert(Eql(V0 - V1, vec_t(T(-1))));

			if constexpr (IsRank1<vec_t>)
			{
				static_assert(Eql(V0 * V1, T(6)));
				static_assert(Eql(V0 / V1, T(2) / T(3)));
				PR_EXPECT(Eql(V0 % V1, T(2)));

				static_assert(Eql(V0 * T(3), vec_t(T(6))));
				static_assert(Eql(V0 / T(2), vec_t(T(1))));
				PR_EXPECT(Eql(V0 % T(2), vec_t(T(0))));

				static_assert(Eql(T(3) * V0, vec_t(T(6))));
				static_assert(Eql(T(8) / V0, vec_t(T(4))));
			}

			if constexpr (IsRank2<vec_t>)
			{
				using S = typename vector_traits<vec_t>::element_t;
				PR_EXPECT(Eql(V0 * S(3), vec_t(T(6))));
				PR_EXPECT(Eql(V0 / S(2), vec_t(T(1))));

				PR_EXPECT(Eql(S(3) * V0, vec_t(T(6))));
				// Note: S(8) / mat is not defined for matrices
			}

			static_assert(V0 == vec_t(T(2)));
			static_assert(V0 != vec_t(T(3)));

			static_assert(V0 < V1);
			static_assert(V1 > V0);
			static_assert(!(V0 >= V1));
			static_assert(!(V1 <= V0));

			// Bitwise, shift, and logical operators (integer types only)
			if constexpr (std::integral<typename vector_traits<vec_t>::element_t>)
			{
				using E = typename vector_traits<vec_t>::element_t;
				constexpr auto VZ = vec_t(E(0));

				static_assert(Eql(~V0, vec_t(static_cast<E>(~E(2)))));
				static_assert(Eql(!VZ, vec_t(E(1))));
				static_assert(Eql(!V0, vec_t(E(0))));
				static_assert(Eql(V0 | V1, vec_t(static_cast<E>(E(2) | E(3)))));
				static_assert(Eql(V0 & V1, vec_t(static_cast<E>(E(2) & E(3)))));
				static_assert(Eql(V0 ^ V1, vec_t(static_cast<E>(E(2) ^ E(3)))));
				static_assert(Eql(V0 << E(1), vec_t(static_cast<E>(E(2) << E(1)))));
				static_assert(Eql(V0 << vec_t(E(1)), vec_t(static_cast<E>(E(2) << E(1)))));
				static_assert(Eql(V0 >> E(1), vec_t(static_cast<E>(E(2) >> E(1)))));
				static_assert(Eql(V0 >> vec_t(E(1)), vec_t(static_cast<E>(E(2) >> E(1)))));
				static_assert(Eql(V0 || VZ, vec_t(E(1))));
				static_assert(Eql(VZ || VZ, vec_t(E(0))));
				static_assert(Eql(V0 && V1, vec_t(E(1))));
				static_assert(Eql(V0 && VZ, vec_t(E(0))));
			}
		}

		// ---- Constants (functions.h line ~399) ----
		PRUnitTestMethod(Constants
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = typename vt::element_t;
			constexpr auto dim = vt::dimension;

			static_assert(Zero<vec_t>() == vec_t(S(0)));

			if constexpr (IsRank1<vec_t>)
			{
				static_assert(One<vec_t>() == vec_t(S(1)));
				static_assert(Tiny<vec_t>() == vec_t(tiny<S>));
				if constexpr (std::floating_point<S>)
				{
					static_assert(Epsilon<vec_t>() == vec_t(limits<S>::epsilon()));
					static_assert(Infinity<vec_t>() == vec_t(limits<S>::infinity()));
				}

				if constexpr (dim >= 2) static_assert(XAxis<vec_t>()[0] == S(1) && XAxis<vec_t>()[1] == S(0));
				if constexpr (dim >= 2) static_assert(YAxis<vec_t>()[0] == S(0) && YAxis<vec_t>()[1] == S(1));
				if constexpr (dim >= 3) static_assert(ZAxis<vec_t>()[2] == S(1) && ZAxis<vec_t>()[0] == S(0));
				if constexpr (dim >= 4) static_assert(WAxis<vec_t>()[3] == S(1) && WAxis<vec_t>()[0] == S(0));

				if constexpr (dim == 2) static_assert(Origin<vec_t>() == vec_t(S(0)));
				if constexpr (dim == 3) static_assert(Origin<vec_t>() == vec_t(S(0)));
				if constexpr (dim == 4) static_assert(Origin<vec_t>()[3] == S(1));
			}

			if constexpr (IsRank2<vec_t>)
			{
				constexpr auto I = Identity<vec_t>();
				static_assert(vec(I).x[0] == S(1));
				static_assert(vec(I).y[1] == S(1));
				if constexpr (vt::dimension >= 3) static_assert(vec(I).z[2] == S(1));
				if constexpr (vt::dimension >= 4) static_assert(vec(I).w[3] == S(1));
				static_assert(vec(I).x[1] == S(0));
				static_assert(vec(I).y[0] == S(0));
			}
		}

		// ---- IsNaN (functions.h line ~573) ----
		PRUnitTestMethod(IsNaNTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			if constexpr (std::floating_point<S>)
			{
				PR_EXPECT(!IsNaN(vec_t(S(1))));
				PR_EXPECT(IsNaN(vec_t(limits<S>::quiet_NaN())));
				PR_EXPECT(!IsNaN(vec_t(limits<S>::infinity())));
			}
			if constexpr (std::integral<S>)
			{
				PR_EXPECT(!IsNaN(vec_t(S(1))));
				PR_EXPECT(!IsNaN(vec_t(limits<S>::max())));
				PR_EXPECT(!IsNaN(vec_t(limits<S>::min())));
			}
		}

		// ---- IsFinite (functions.h line ~601) ----
		PRUnitTestMethod(IsFiniteTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			if constexpr (std::floating_point<S>)
			{
				PR_EXPECT(IsFinite(vec_t(S(1))));
				PR_EXPECT(!IsFinite(vec_t(limits<S>::infinity())));
				PR_EXPECT(!IsFinite(vec_t(limits<S>::quiet_NaN())));
			}
			if constexpr (std::integral<S>)
			{
				PR_EXPECT(IsFinite(vec_t(S(1))));
				PR_EXPECT(IsFinite(vec_t(limits<S>::max())));
				PR_EXPECT(IsFinite(vec_t(limits<S>::min())));
			}
		}

		// ---- Any, All (functions.h line ~651) ----
		PRUnitTestMethod(AnyAllTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = typename vt::element_t;
			using C = typename vt::component_t;

			constexpr auto V_pos = vec_t(S(5));
			constexpr auto V_zero = vec_t(S(0));

			// For matrices, All/Any iterates over components (vectors), not scalars
			if constexpr (IsRank2<vec_t>)
			{
				PR_EXPECT(All(V_pos, [](C x) { return x > C(S(0)); }));
				PR_EXPECT(Any(V_pos, [](C x) { return x > C(S(0)); }));
				PR_EXPECT(!Any(V_zero, [](C x) { return x > C(S(0)); }));
				PR_EXPECT(!All(V_zero, [](C x) { return x > C(S(0)); }));
			}
			else
			{
				static_assert(All(V_pos, [](S x) { return x > S(0); }));
				static_assert(Any(V_pos, [](S x) { return x > S(0); }));
				static_assert(!Any(V_zero, [](S x) { return x > S(0); }));
				static_assert(!All(V_zero, [](S x) { return x > S(0); }));
			}
		}

		// ---- Abs (functions.h line ~675) ----
		PRUnitTestMethod(Abs
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto V0 = vec_t(S(-3));
			static_assert(Abs(V0) == vec_t(S(3)));
			static_assert(Abs(vec_t(S(3))) == vec_t(S(3)));
			static_assert(Abs(vec_t(S(0))) == vec_t(S(0)));
		}

		// ---- Min, Max, Clamp (functions.h line ~695) ----
		PRUnitTestMethod(MinMaxClamp
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto V0 = vec_t(S(3));
			constexpr auto V1 = vec_t(S(1));
			constexpr auto V2 = vec_t(S(5));

			static_assert(Min(V0, V1, V2) == vec_t(S(1)));
			static_assert(Max(V0, V1, V2) == vec_t(S(5)));
			static_assert(Clamp(V0, V1, V2) == vec_t(S(3)));
			static_assert(Clamp(V2, V1, V0) == vec_t(S(3)));
			static_assert(Clamp(V1, V0, V2) == vec_t(S(3)));
		}

		// ---- Square, SignedSqr, Sqrt, CompSqrt (functions.h line ~750) ----
		PRUnitTestMethod(SquareAndSqrt
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			static_assert(Sqr(S(3)) == S(9));
			if constexpr (IsRank2<vec_t>)
			{
				PR_EXPECT(SignedSqr(vec_t(S(-3))) == vec_t(S(-9)));
				PR_EXPECT(SignedSqr(vec_t(S(3))) == vec_t(S(9)));
			}
			else
			{
				static_assert(SignedSqr(vec_t(S(-3))) == vec_t(S(-9)));
				static_assert(SignedSqr(vec_t(S(3))) == vec_t(S(9)));
			}

			// CompSqrt (floating point, rank-1 only - CompSqrt calls Sqrt(component) which static_asserts for vectors)
			if constexpr (std::floating_point<S> && IsRank1<vec_t>)
			{
				static_assert(Eql(CompSqrt(vec_t(S(9))), vec_t(S(3))));
			}
		}

		// ---- SqrtCT (functions.h line ~770) ----
		PRUnitTestMethod(SqrtCTTests
		, float, double
		) {
			using S = T;

			static_assert(SqrtCT(S(0)) == S(0));
			static_assert(SqrtCT(S(1)) == S(1));
			static_assert(SqrtCT(S(4)) == S(2));
			static_assert(SqrtCT(S(9)) == S(3));

			// Negative input gives NaN (std::isnan is not constexpr in MSVC)
			PR_EXPECT(std::isnan(SqrtCT(S(-1))));
		}

		// ---- SignedSqrt, CompSignedSqrt (functions.h line ~814) ----
		PRUnitTestMethod(CompSignedSqrtTests
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			if constexpr (IsRank2<vec_t>)
			{
				PR_EXPECT(Eql(CompSignedSqrt(vec_t(S(9))), vec_t(S(3))));
				PR_EXPECT(Eql(CompSignedSqrt(vec_t(S(-9))), vec_t(S(-3))));
				PR_EXPECT(Eql(CompSignedSqrt(vec_t(S(0))), vec_t(S(0))));
			}
			else
			{
				static_assert(Eql(CompSignedSqrt(vec_t(S(9))), vec_t(S(3))));
				static_assert(Eql(CompSignedSqrt(vec_t(S(-9))), vec_t(S(-3))));
				static_assert(Eql(CompSignedSqrt(vec_t(S(0))), vec_t(S(0))));
			}
		}

		// ---- ISqrt (functions.h line ~830) ----
		PRUnitTestMethod(ISqrtAndCompISqrtTests
		, int32_t, int64_t
		, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using S = T;

			if constexpr (VectorType<T>)
			{
				static_assert(CompISqrt(S(0)) == S(0));
				static_assert(CompISqrt(S(1)) == S(1));
				static_assert(CompISqrt(S(4)) == S(2));
				static_assert(CompISqrt(S(9)) == S(3));
				static_assert(CompISqrt(S(10)) == S(3));
				static_assert(CompISqrt(S(100)) == S(10));
			}
			else
			{
				static_assert(ISqrt(S(0)) == S(0));
				static_assert(ISqrt(S(1)) == S(1));
				static_assert(ISqrt(S(4)) == S(2));
				static_assert(ISqrt(S(9)) == S(3));
				static_assert(ISqrt(S(10)) == S(3));
				static_assert(ISqrt(S(100)) == S(10));
			}
		}

		// ---- CompSum (functions.h line ~850) ----
		PRUnitTestMethod(CompSumTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = typename vt::element_t;
			constexpr auto dim = vt::dimension;

			constexpr auto V0 = vec_t(S(3));
			if constexpr (IsRank2<vec_t>)
			{
				constexpr auto comp_dim = vector_traits<typename vt::component_t>::dimension;
				PR_EXPECT(CompSum(V0) == S(dim * comp_dim * 3));
				PR_EXPECT(CompSum(vec_t(S(0))) == S(0));
			}
			else
			{
				static_assert(CompSum(V0) == S(dim * 3));
				static_assert(CompSum(vec_t(S(0))) == S(0));
			}
		}

		// ---- CompMul (functions.h line ~865) ----
		PRUnitTestMethod(CompMulTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			auto I = Identity<mat_t>();
			auto scale = Vec(S(2));
			auto result = CompMul(I, scale);

			PR_EXPECT(vec(vec(result).x).x == S(2));
			PR_EXPECT(vec(vec(result).y).y == S(2));
		}

		// ---- MinElement, MaxElement, MinElementAbs, MaxElementAbs (functions.h line ~876) ----
		PRUnitTestMethod(MinMaxElement
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto V0 = vec_t(S(5));
			constexpr auto V1 = vec_t(S(-3));
			static_assert(MinElement(V0) == S(5));
			static_assert(MaxElement(V0) == S(5));
			static_assert(MinElementAbs(V0) == S(5));
			static_assert(MaxElementAbs(V1) == S(3));
		}

		// ---- MinElementIndex, MaxElementIndex (functions.h line ~930) ----
		PRUnitTestMethod(MinMaxElementIndex
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto V0 = XAxis<vec_t>();
			static_assert(MaxElementIndex(V0) == 0);
			static_assert(MinElementIndex(V0) == 1);
		}

		// ---- FEqlAbsolute (functions.h line ~954) ----
		PRUnitTestMethod(FEqlAbsoluteTests
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto V0 = vec_t(S(1));
			constexpr auto V1 = vec_t(S(1) + S(0.0001));
			constexpr auto V2 = vec_t(S(2));

			static_assert(FEqlAbsolute(V0, V0, S(0.001)));
			static_assert(FEqlAbsolute(V0, V1, S(0.001)));
			static_assert(!FEqlAbsolute(V0, V2, S(0.5)));
		}

		// ---- FEqlRelative (functions.h line ~976) ----
		PRUnitTestMethod(FEqlRelativeTests
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto V0 = vec_t(S(1));
			static_assert(FEqlRelative(V0, V0, S(0.001)));
			static_assert(!FEqlRelative(vec_t(S(1)), vec_t(S(2)), S(0.001)));
		}

		// ---- FEql (functions.h line ~1014) ----
		PRUnitTestMethod(FEqlTests
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto V0 = vec_t(S(1));
			static_assert(FEql(V0, V0));
			static_assert(!FEql(V0, vec_t(S(2))));
			static_assert(FEql(vec_t(S(0)), vec_t(S(0))));
		}

		// ---- Ceil, Floor, Round (functions.h line ~1030) ----
		PRUnitTestMethod(CeilFloorRound
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			auto V0 = vec_t(S(3.25));
			PR_EXPECT(Ceil(V0) == vec_t(S(4)));
			PR_EXPECT(Floor(V0) == vec_t(S(3)));
			PR_EXPECT(Round(V0) == vec_t(S(3)));

			auto V1 = vec_t(S(-3.25));
			PR_EXPECT(Ceil(V1) == vec_t(S(-3)));
			PR_EXPECT(Floor(V1) == vec_t(S(-4)));
			PR_EXPECT(Round(V1) == vec_t(S(-3)));

			auto V2 = vec_t(S(3.75));
			PR_EXPECT(Round(V2) == vec_t(S(4)));
		}

		// ---- RoundSD (functions.h line ~1075) ----
		PRUnitTestMethod(RoundSDTests
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using S = T;
			using E = typename vector_traits<S>::element_t;

			PR_EXPECT(FEql(RoundSD(S(E(12345.6789)), 3), S(E(12300.0))));
			PR_EXPECT(FEql(RoundSD(S(E(0.001234)), 2), S(E(0.0012))));
		}

		// ---- Modulus (functions.h line ~1104) ----
		PRUnitTestMethod(ModulusTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			PR_EXPECT(Modulus(vec_t(S(7)), vec_t(S(3))) == vec_t(S(1)));
			PR_EXPECT(Modulus(vec_t(S(6)), vec_t(S(3))) == vec_t(S(0)));
		}

		// ---- Wrap (functions.h line ~1163) ----
		PRUnitTestMethod(WrapTests
		, float, double, int32_t, int64_t
		) {
			using S = T;

			PR_EXPECT(Wrap(S(5), S(0), S(10)) == S(5));
			PR_EXPECT(Wrap(S(12), S(0), S(10)) == S(2));

			if constexpr (std::is_signed_v<S>)
			{
				PR_EXPECT(Wrap(S(-1), S(0), S(10)) == S(9));
			}
		}

		// ---- Bool2Sign, Sign (functions.h line ~1172) ----
		PRUnitTestMethod(SignTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			static_assert(Bool2SignI(true) == +1);
			static_assert(Bool2SignI(false) == -1);
			static_assert(Bool2SignF(true) == +1.0f);
			static_assert(Bool2SignF(false) == -1.0f);

			static_assert(Sign(vec_t(S(5))) == vec_t(S(1)));
			static_assert(Sign(vec_t(S(0))) == vec_t(S(1)));
			static_assert(Sign(vec_t(S(0)), false) == vec_t(S(0)));

			if constexpr (std::is_signed_v<S>)
			{
				static_assert(Sign(vec_t(S(-5))) == vec_t(S(-1)));
			}
		}

		// ---- Div (functions.h line ~1201) ----
		PRUnitTestMethod(DivTests
		, float, double, int32_t, int64_t
		) {
			using S = T;

			static_assert(Div(S(10), S(2)) == S(5));
			static_assert(Div(S(10), S(0)) == S(0));
			static_assert(Div(S(10), S(0), S(99)) == S(99));
		}

		// ---- Trunc (functions.h line ~1207) ----
		PRUnitTestMethod(TruncTests
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			PR_EXPECT(Trunc(vec_t(S(3.75))) == vec_t(S(3)));
			PR_EXPECT(Trunc(vec_t(S(-3.75))) == vec_t(S(-3)));
		}

		// ---- Frac (functions.h line ~1228) ----
		PRUnitTestMethod(Frac
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			PR_EXPECT(FEql(Frac(vec_t(S(3.25))), vec_t(S(0.25))));
			PR_EXPECT(FEql(Frac(vec_t(S(3.0))), vec_t(S(0.0))));
		}

		// ---- Sqr (functions.h line ~1255) ----
		PRUnitTestMethod(SqrTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			static_assert(Sqr(vec_t(S(3))) == vec_t(S(9)));
			static_assert(Sqr(vec_t(S(-3))) == vec_t(S(9)));
			static_assert(Sqr(vec_t(S(0))) == vec_t(S(0)));
		}

		// ---- Cube (functions.h line ~1288) ----
		PRUnitTestMethod(CubeTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			static_assert(Cube(vec_t(S(2))) == vec_t(S(8)));
			static_assert(Cube(vec_t(S(-2))) == vec_t(S(-8)));
		}

		// ---- Pow (functions.h line ~1321) ----
		PRUnitTestMethod(PowTests
		, float, double, int32_t, int64_t
		) {
			using S = T;

			static_assert(Pow(S(2), 0) == S(1));
			static_assert(Pow(S(2), 1) == S(2));
			static_assert(Pow(S(2), 3) == S(8));
			static_assert(Pow(S(3), 2) == S(9));
		}

		// ---- DegreesToRadians, RadiansToDegrees (functions.h line ~1327) ----
		PRUnitTestMethod(DegreesRadians
		, float, double
		) {
			using S = T;

			static_assert(FEql(DegreesToRadians(S(180)), constants<S>::tau / S(2)));
			static_assert(FEql(DegreesToRadians(S(360)), constants<S>::tau));
			static_assert(FEql(RadiansToDegrees(constants<S>::tau / S(2)), S(180)));
			static_assert(FEql(RadiansToDegrees(constants<S>::tau), S(360)));
		}

		// ---- Dot (functions.h line ~1338) ----
		PRUnitTestMethod(DotProduct
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = typename vt::element_t;
			constexpr auto dim = vt::dimension;

			constexpr auto V0 = vec_t(S(2));
			constexpr auto V1 = vec_t(S(3));

			static_assert(Dot(V0, V1) == S(dim * 6));
			static_assert(Dot(V0, V0) == LengthSq(V0));
			static_assert(Dot(V0, V1) == Dot(V1, V0));
			static_assert(Dot(V0, vec_t(S(0))) == S(0));
		}

		// ---- Dot3 (functions.h line ~1350) ----
		PRUnitTestMethod(Dot3Tests
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto V0 = vec_t(S(2));
			constexpr auto V1 = vec_t(S(3));

			// Dot3 only uses x,y,z (3 components)
			static_assert(Dot3(V0, V1) == S(18));
			static_assert(Dot3(V0, V1) == Dot3(V1, V0));
		}

		// ---- Cross 2D (functions.h line ~1363) ----
		PRUnitTestMethod(CrossProduct2D
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto V0 = vec_t(S(2), S(3));
			static_assert(Cross(V0, V0) == S(0));

			constexpr auto V1 = vec_t(S(1), S(4));
			static_assert(Cross(V0, V1) == -Cross(V1, V0));
			static_assert(Cross(XAxis<vec_t>(), YAxis<vec_t>()) == S(1));
		}

		// ---- Cross 3D (functions.h line ~1369) ----
		PRUnitTestMethod(CrossProduct3D
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto X = XAxis<vec_t>();
			constexpr auto Y = YAxis<vec_t>();
			constexpr auto Z = ZAxis<vec_t>();

			static_assert(Cross(X, Y) == Z);
			static_assert(Cross(Y, Z) == X);
			static_assert(Cross(Z, X) == Y);
			static_assert(Cross(X, Y) == -Cross(Y, X));
			static_assert(Cross(X, X) == Zero<vec_t>());
		}

		// ---- Cross 4D (functions.h line ~1379) ----
		PRUnitTestMethod(CrossProduct4D
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto X = XAxis<vec_t>();
			constexpr auto Y = YAxis<vec_t>();
			constexpr auto Z = ZAxis<vec_t>();

			static_assert(Cross(X, Y) == Z);
			static_assert(Cross(Y, Z) == X);
			static_assert(Cross(Z, X) == Y);
			static_assert(Cross(X, Y) == -Cross(Y, X));
		}

		// ---- Triple, Triple3 (functions.h line ~1392) ----
		PRUnitTestMethod(TripleProduct
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto X = XAxis<vec_t>();
			constexpr auto Y = YAxis<vec_t>();
			constexpr auto Z = ZAxis<vec_t>();

			static_assert(Triple(X, Y, Z) == S(1));
			static_assert(Triple(Y, Z, X) == S(1));
			static_assert(Triple(X, Y, Z) == -Triple(X, Z, Y));
		}

		// ---- LengthSq (functions.h line ~1404) ----
		PRUnitTestMethod(LengthSqTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = typename vt::element_t;
			constexpr auto dim = vt::dimension;

			constexpr auto V0 = vec_t(S(2));
			static_assert(LengthSq(V0) == S(dim * 4));
		}

		// ---- Length (functions.h line ~1410) ----
		PRUnitTestMethod(LengthTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			static_assert(Length(XAxis<vec_t>()) == S(1));
			static_assert(Length(Zero<vec_t>()) == S(0));

			constexpr auto V0 = vec_t(S(2));
			constexpr auto len = Length(V0);
			if constexpr (std::floating_point<S>)
				static_assert(FEql(len * len, LengthSq(V0)));
			else
				// Note: len for integer types will be truncated, so we can't do an exact assert.
				static_assert(len * len <= LengthSq(V0));
		}

		// ---- Length (functions.h line ~2096) ----
		PRUnitTestMethod(LenTests)
		{
			PR_EXPECT(Len(3.0) == 3.0);
			PR_EXPECT(Len(0.0) == 0.0);
			PR_EXPECT(Len(-2.0) == 2.0);
			PR_EXPECT(Len(1.0f) == 1.0f);
			PR_EXPECT(Len(5) == 5);
			PR_EXPECT(Len(-0.5) == 0.5);

			PR_EXPECT(FEql(Len(3.0, 4.0), 5.0));               // 3-4-5
			PR_EXPECT(FEql(Len(1.0, 2.0, 2.0), 3.0));          // 1² + 2² + 2² = 9
			PR_EXPECT(FEql(Len(0.0, 0.0, 0.0), 0.0));          // zero vector
			PR_EXPECT(FEql(Len(1.0, 1.0, 1.0, 1.0), 2.0));     // 4 * 1² = 4, sqrt = 2
		}

		// ---- Trace (functions.h line ~1429) ----
		PRUnitTestMethod(TraceTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;

			static_assert(Trace(Identity<mat_t>()) == S(vt::dimension));
			static_assert(Trace(Zero<mat_t>()) == S(0));
		}

		// ---- Determinant (functions.h line ~1444) ----
		PRUnitTestMethod(DeterminantTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			PR_EXPECT(Determinant(Identity<mat_t>()) == S(1));
			PR_EXPECT(Determinant(Zero<mat_t>()) == S(0));

			// Det(k*I) = k^dim
			if constexpr (std::floating_point<S>)
			{
				constexpr auto dim = vector_traits<mat_t>::dimension;
				constexpr auto I2 = Identity<mat_t>() * S(2);
				PR_EXPECT(FEql(Determinant(I2), Pow(S(2), dim)));
			}
		}

		// ---- DeterminantAffine (functions.h line ~1492) ----
		PRUnitTestMethod(DeterminantAffineTests
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			PR_EXPECT(DeterminantAffine(Identity<mat_t>()) == S(1));

			auto I2 = Identity<mat_t>() * S(2);
			vec(vec(I2).w).w = S(1); // Restore affine w.w
			PR_EXPECT(FEql(DeterminantAffine(I2), S(8)));
		}

		// ---- Diagonal (functions.h line ~1502) ----
		PRUnitTestMethod(DiagonalTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			// Diagonal only extracts 'dimension' elements, remaining components stay 0
			Vec expected_diag = {};
			if constexpr (vt::dimension > 0) vec(expected_diag).x = S(1);
			if constexpr (vt::dimension > 1) vec(expected_diag).y = S(1);
			if constexpr (vt::dimension > 2) vec(expected_diag).z = S(1);
			if constexpr (vt::dimension > 3) vec(expected_diag).w = S(1);

			PR_EXPECT(Diagonal(Identity<mat_t>()) == expected_diag);
			PR_EXPECT(Diagonal(Zero<mat_t>()) == Vec(S(0)));
		}

		// ---- Kernel (functions.h line ~1527) ----
		PRUnitTestMethod(KernelTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			// For the identity matrix, (I - I) = Zero, so Kernel should work on singular matrices.
			// For a matrix M with det=0, Kernel(M) gives a vector in the null space.
			// Test: Kernel of a rank-deficient matrix is in its null space.
			auto M = Identity<mat_t>();
			vec(vec(M).x).x = S(0); // Make first column zero → singular

			// k should be non-zero (there IS a kernel)
			auto k = Kernel(M);
			PR_EXPECT(k != Zero<Vec>());

			// Kernel vector should satisfy M^T * k ≈ 0 (it's in the null space of M^T)
			auto Mt = Transpose(M);
			Vec Mt_k = Mt * k;
			if constexpr (std::floating_point<S>)
				PR_EXPECT(FEql(Mt_k, Zero<Vec>()));
			else
				PR_EXPECT(Mt_k == Zero<Vec>());
		}

		// ---- Normalise, IsNormalised (functions.h line ~1563) ----
		PRUnitTestMethod(Normalise
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			constexpr auto V0 = vec_t(S(3));
			PR_EXPECT(IsNormalised(Normalise(V0)));

			auto N0 = Normalise(V0);
			PR_EXPECT(FEql(N0[0], N0[1]));

			PR_EXPECT(FEql(Normalise(Zero<vec_t>(), V0), V0));

			static_assert(IsNormalised(XAxis<vec_t>()));
		}

		// ---- Normalise(Mat) (functions.h line ~1588) ----
		PRUnitTestMethod(NormaliseMatTests
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;

			// Normalise a scaled identity, verify columns are unit-length and scales returned
			constexpr auto scaled = Identity<mat_t>() * S(3);
			auto [norm_mat, scales] = Normalise(scaled);
			PR_EXPECT(IsOrthonormal(norm_mat));
			if constexpr (vt::dimension > 0) PR_EXPECT(FEql(vec(scales).x, S(3)));
			if constexpr (vt::dimension > 1) PR_EXPECT(FEql(vec(scales).y, S(3)));
			if constexpr (vt::dimension > 2) PR_EXPECT(FEql(vec(scales).z, S(3)));
			if constexpr (vt::dimension > 4) PR_EXPECT(FEql(vec(scales).w, S(3)));
		}

		// ---- IsOrthogonal (functions.h line ~1608) ----
		PRUnitTestMethod(IsOrthogonalTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			PR_EXPECT(IsOrthogonal(Identity<mat_t>()));

			// Zero matrix has zero dot products between columns, so it IS "orthogonal" by the function's definition.
			// Use a matrix with non-orthogonal columns instead.
			auto M = Identity<mat_t>();
			vec(vec(M).x).y = S(1); // col x = (1,1,...), col y = (0,1,...), Dot != 0
			PR_EXPECT(!IsOrthogonal(M));
		}

		// ---- IsOrthonormal (functions.h line ~1639) ----
		PRUnitTestMethod(IsOrthonormalTests
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			PR_EXPECT(IsOrthonormal(Identity<mat_t>()));
			PR_EXPECT(!IsOrthonormal(Zero<mat_t>()));
			PR_EXPECT(!IsOrthonormal(Identity<mat_t>() * S(2)));
		}

		// ---- IsAffine (functions.h line ~1654) ----
		PRUnitTestMethod(IsAffineTests
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;

			PR_EXPECT(IsAffine(Identity<mat_t>()));

			// For dim==4, Zero fails because w.w != 1. For dim==3, Zero is structurally affine
			// (all column .w == 0, and there's no 4th column to check w.w == 1).
			if constexpr (vt::dimension >= 4)
				PR_EXPECT(!IsAffine(Zero<mat_t>()));
			else
				PR_EXPECT(IsAffine(Zero<mat_t>()));
		}

		// ---- IsInvertible (functions.h line ~1674) ----
		PRUnitTestMethod(IsInvertibleTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;

			PR_EXPECT(IsInvertible(Identity<mat_t>()));
			PR_EXPECT(!IsInvertible(Zero<mat_t>()));
		}

		// ---- IsSymmetric (functions.h line ~1684) ----
		PRUnitTestMethod(IsSymmetricTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;

			PR_EXPECT(IsSymmetric(Identity<mat_t>()));
			PR_EXPECT(IsSymmetric(Zero<mat_t>()));
		}

		// ---- IsAntiSymmetric (functions.h line ~1711) ----
		PRUnitTestMethod(IsAntiSymmetricTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;

			PR_EXPECT(IsAntiSymmetric(Zero<mat_t>()));
		}

		// ---- IsParallel (functions.h line ~1738) ----
		PRUnitTestMethod(IsParallelTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			auto V0 = vec_t(S(1));
			auto V1 = vec_t(S(2));

			PR_EXPECT(IsParallel(V0, V1));
			PR_EXPECT(IsParallel(V0, -V1));
			PR_EXPECT(!IsParallel(XAxis<vec_t>(), YAxis<vec_t>()));
		}

		// ---- CreateNotParallelTo (functions.h line ~1757) ----
		PRUnitTestMethod(CreateNotParallelToTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;

			constexpr auto V0 = XAxis<vec_t>();
			PR_EXPECT(!IsParallel(V0, CreateNotParallelTo(V0)));

			constexpr auto V1 = YAxis<vec_t>();
			PR_EXPECT(!IsParallel(V1, CreateNotParallelTo(V1)));
		}

		// ---- Rotate90CW and Rotate90CCW (functions.h line ~2432) ----
		PRUnitTestMethod(Rotate90CWCCWTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			// CW rotates +X toward +Y
			{
				auto v = vec_t{ S(1), S(0) };
				auto cw = Rotate90CW(v);
				PR_EXPECT(cw.x == S(0));
				PR_EXPECT(cw.y == S(1));
			}

			// CW rotates +Y toward -X
			{
				auto v = vec_t{ S(0), S(1) };
				auto cw = Rotate90CW(v);
				PR_EXPECT(cw.x == S(-1));
				PR_EXPECT(cw.y == S(0));
			}

			// CCW rotates +X toward -Y
			{
				auto v = vec_t{ S(1), S(0) };
				auto ccw = Rotate90CCW(v);
				PR_EXPECT(ccw.x == S(0));
				PR_EXPECT(ccw.y == S(-1));
			}

			// CCW rotates +Y toward +X
			{
				auto v = vec_t{ S(0), S(1) };
				auto ccw = Rotate90CCW(v);
				PR_EXPECT(ccw.x == S(1));
				PR_EXPECT(ccw.y == S(0));
			}

			// CW then CCW is identity
			{
				auto v = vec_t{ S(3), S(7) };
				PR_EXPECT(Rotate90CCW(Rotate90CW(v)) == v);
				PR_EXPECT(Rotate90CW(Rotate90CCW(v)) == v);
			}

			// CW preserves length
			{
				auto v = vec_t{ S(3), S(4) };
				PR_EXPECT(LengthSq(Rotate90CW(v)) == LengthSq(v));
				PR_EXPECT(LengthSq(Rotate90CCW(v)) == LengthSq(v));
			}

			// CW is perpendicular
			{
				auto v = vec_t{ S(5), S(2) };
				PR_EXPECT(Dot(v, Rotate90CW(v)) == S(0));
				PR_EXPECT(Dot(v, Rotate90CCW(v)) == S(0));
			}

			// Cross2D(a,b) == Dot(Rotate90CW(a), b)
			if constexpr (std::is_floating_point_v<S>)
			{
				auto a = vec_t{ S(3), S(5) };
				auto b = vec_t{ S(7), S(2) };
				PR_EXPECT(FEql(Cross(a, b), Dot(Rotate90CW(a), b)));
			}
		}

		// ---- Perpendicular (functions.h line ~1775) ----
		PRUnitTestMethod(PerpendicularTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = typename vt::element_t;

			constexpr auto V0 = XAxis<vec_t>();
			auto perp = Perpendicular(V0);

			auto T = static_cast<double>(Dot(V0, perp));
			auto len = static_cast<double>(Length(perp));
			auto len0 = static_cast<double>(Length(V0));

			PR_EXPECT(FEql(T, 0.0));
			PR_EXPECT(FEql(len, len0));
		}

		// ---- Perpendicular with previous (functions.h line ~1792) ----
		PRUnitTestMethod(PerpendicularWithPreviousTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = typename vt::element_t;

			constexpr auto V0 = XAxis<vec_t>();
			constexpr auto prev = YAxis<vec_t>();
			auto perp = Perpendicular(V0, prev);

			// Result should be perpendicular to V0
			PR_EXPECT(Dot(V0, perp) == S(0));

			// When previous is already perpendicular, result should favour it
			if constexpr (std::floating_point<S>)
				PR_EXPECT(FEql(Length(perp), Length(V0)));
			else
				PR_EXPECT(LengthSq(perp) > S(0));

			// When previous is zero, should still produce a valid perpendicular
			auto perp2 = Perpendicular(V0, vec_t(S(0)));
			PR_EXPECT(Dot(V0, perp2) == S(0));
		}

		// ---- Permute rank-1 (functions.h line ~1817) ----
		PRUnitTestMethod(Permute
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = typename vt::element_t;
			constexpr auto dim = vt::dimension;

			constexpr auto X = XAxis<vec_t>();
			static_assert(Permute(X, 0) == X);
			static_assert(Permute(X, 1)[0] == S(0));
			static_assert(Permute(X, 1)[dim - 1] == S(1));
			static_assert(Permute(X, dim) == X);
		}

		// ---- Permute rank-2 (functions.h line ~1857) ----
		PRUnitTestMethod(PermuteMatrix
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			constexpr auto dim = vector_traits<mat_t>::dimension;

			constexpr auto I = Identity<mat_t>();
			static_assert(Permute(I, 0) == I);
			static_assert(Permute(I, dim) == I);
		}

		// ---- Orthant (functions.h line ~1894) ----
		PRUnitTestMethod(OrthantTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = typename vt::element_t;
			constexpr auto dim = vt::dimension;

			constexpr uint32_t all_positive = (1u << dim) - 1u;
			static_assert(Orthant(vec_t(S(1))) == all_positive);
			static_assert(Orthant(vec_t(S(-1))) == 0u);
		}

		// ---- Transpose (functions.h line ~1909) ----
		PRUnitTestMethod(TransposeTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			constexpr auto I = Identity<mat_t>();
			PR_EXPECT(Transpose(I) == I);

			auto M = mat_t(S(1)) + I;
			PR_EXPECT(Transpose(Transpose(M)) == M);
		}

		// ---- Transpose3x3 (functions.h line ~1932) ----
		PRUnitTestMethod(Transpose3x3Tests
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			// @Copilot, Can you make this test more thorough? Maybe test a non-identity matrix as well?
			PR_EXPECT(Transpose3x3(Identity<mat_t>()) == Identity<mat_t>());
		}

		// ---- InvertOrthonormal (functions.h line ~1945) ----
		PRUnitTestMethod(InvertOrthonormalTests
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			// @Copilot, Can you make this test more thorough? Maybe test a non-identity matrix as well?
			PR_EXPECT(FEql(InvertOrthonormal(Identity<mat_t>()), Identity<mat_t>()));
		}

		// ---- InvertAffine (functions.h line ~1971) ----
		PRUnitTestMethod(InvertAffineTests
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			// @Copilot, Can you make this test more thorough? Maybe test a non-identity matrix as well?
			PR_EXPECT(FEql(InvertAffine(Identity<mat_t>()), Identity<mat_t>()));
		}

		// ---- Invert (functions.h line ~2016) ----
		PRUnitTestMethod(InvertTests
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			auto I = Identity<mat_t>();
			PR_EXPECT(FEql(Invert(I), I));

			// M * Invert(M) == I
			auto M = I * S(2);
			PR_EXPECT(FEql(M * Invert(M), I));
		}

		// ---- InvertPrecise (functions.h line ~2117) ----
		PRUnitTestMethod(InvertPreciseTests
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			// @Copilot, Can you make this test more thorough? Maybe test a non-identity matrix as well?
			PR_EXPECT(FEql(InvertPrecise(Identity<mat_t>()), Identity<mat_t>()));
		}

		// ---- Sqrt matrix (functions.h line ~2159) ----
		PRUnitTestMethod(MatrixSqrtTests
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;

			auto sqrtI = Sqrt(Identity<mat_t>());
			PR_EXPECT(FEql(sqrtI * sqrtI, Identity<mat_t>()));
		}

		// ---- Orthonorm (functions.h line ~2179) ----
		PRUnitTestMethod(OrthonormTests
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			// @Copilot, Can you make this test more thorough? Maybe test a non-identity matrix as well?
			PR_EXPECT(FEql(Orthonorm(Identity<mat_t>()), Identity<mat_t>()));
			PR_EXPECT(IsOrthonormal(Orthonorm(Identity<mat_t>())));
		}

		// ---- Matrix multiply (functions.h line ~2190) ----
		PRUnitTestMethod(MatrixMultiply
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			mat_t I = Identity<mat_t>();

			mat_t M = {};
			if constexpr (vt::dimension > 0) vec(M).x = Vec(S(vt::dimension));
			if constexpr (vt::dimension > 1) vec(M).y = Vec(S(vt::dimension));
			if constexpr (vt::dimension > 2) vec(M).z = Vec(S(vt::dimension));
			if constexpr (vt::dimension > 3) vec(M).w = Vec(S(vt::dimension));

			// Mat*Vec only produces 'dimension' components. Build a vector compatible with the matrix dimension.
			Vec V = {};
			if constexpr (vt::dimension > 0) vec(V).x = S(1);
			if constexpr (vt::dimension > 1) vec(V).y = S(1);
			if constexpr (vt::dimension > 2) vec(V).z = S(1);
			if constexpr (vt::dimension > 3) vec(V).w = S(1);

			PR_EXPECT(I * V == V);
			PR_EXPECT(I * I == I);
			
			Vec R = M * V;
			auto expected = S(vt::dimension * vt::dimension);
			if constexpr (vt::dimension > 0) PR_EXPECT(vec(R).x == expected);
			if constexpr (vt::dimension > 1) PR_EXPECT(vec(R).y == expected);
			if constexpr (vt::dimension > 2) PR_EXPECT(vec(R).z == expected);
			if constexpr (vt::dimension > 3) PR_EXPECT(vec(R).w == expected);

			// Mat * scalar and scalar * Mat
			if constexpr (std::floating_point<S>)
			{
				PR_EXPECT((I * S(2)) * V == V * S(2));
				PR_EXPECT(S(2) * I == I * S(2));
			}
		}

		// ---- Translation (functions.h line ~2246) ----
		PRUnitTestMethod(TranslationTests
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			auto pos = Vec(S(1), S(2), S(3), S(1));
			auto T0 = Translation<mat_t>(pos);

			auto origin = Vec(S(0), S(0), S(0), S(1));
			PR_EXPECT(FEql(T0 * origin, pos));

			auto T1 = Translation<mat_t>(S(1), S(2), S(3));
			PR_EXPECT(FEql(T0, T1));
		}

		// ---- Rotation (functions.h line ~2269) ----
		PRUnitTestMethod(Rotation2DTests
		, Mat2x2<float>, Mat2x2<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			auto R0 = Rotation<mat_t>(S(0));
			PR_EXPECT(FEql(R0, Identity<mat_t>()));

			auto R90 = Rotation<mat_t>(constants<S>::tau / S(4));
			PR_EXPECT(IsOrthonormal(R90));
		}
		PRUnitTestMethod(RotationTests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			auto R0 = Rotation<mat_t>(Vec::ZAxis(), S(0));
			PR_EXPECT(IsOrthonormal(R0));

			auto R_full = Rotation<mat_t>(Vec::ZAxis(), constants<S>::tau);
			PR_EXPECT(FEql(R_full, Identity<mat_t>()));

			PR_EXPECT(FEql(RotationRad<mat_t>(S(0), S(0), S(0)), Identity<mat_t>()));
			PR_EXPECT(FEql(RotationDeg<mat_t>(S(0), S(0), S(0)), Identity<mat_t>()));
		}

		// ---- Rotation overloads (functions.h line ~2340) ----
		PRUnitTestMethod(RotationAxisAngleOverloads
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			auto axis = Vec::ZAxis();

			// Rotation(axis, angle)
			auto R0 = Rotation<mat_t>(axis, S(0));
			PR_EXPECT(FEql(R0, Identity<mat_t>()));

			// Rotation(axis, sin*axis, cos) — low-level overload
			auto R1 = Rotation<mat_t>(axis, axis * std::sin(S(0)), std::cos(S(0)));
			PR_EXPECT(FEql(R1, Identity<mat_t>()));

			// Rotation(angular_displacement) — zero displacement gives identity
			auto R2 = Rotation<mat_t>(Vec(S(0)));
			PR_EXPECT(FEql(R2, Identity<mat_t>()));

			// Rotation(from, to) — same vector gives identity
			auto R3 = Rotation<mat_t>(Vec::XAxis(), Vec::XAxis());
			PR_EXPECT(FEql(R3, Identity<mat_t>()));

			// Rotation(from, to) — X to Y is 90° about Z
			auto R4 = Rotation<mat_t>(Vec::XAxis(), Vec::YAxis());
			PR_EXPECT(IsOrthonormal(R4));
			PR_EXPECT(FEql(R4 * Vec::XAxis(), Vec::YAxis()));

			// Rotation(AxisId, AxisId) — identity when same axis
			auto R5 = Rotation<mat_t>(AxisId::PosZ, AxisId::PosZ);
			PR_EXPECT(FEql(R5, Identity<mat_t>()));

			// Rotation(AxisId, AxisId) — X to Y
			auto R6 = Rotation<mat_t>(AxisId::PosX, AxisId::PosY);
			PR_EXPECT(IsOrthonormal(R6));
			PR_EXPECT(FEql(R6 * Vec::XAxis(), Vec::YAxis()));
		}

		// ---- Scale (functions.h line ~2413) ----
		PRUnitTestMethod(ScaleTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			PR_EXPECT(Scale<mat_t>(S(1)) == Identity<mat_t>());

			auto S2 = Scale<mat_t>(S(2));
			PR_EXPECT(vec(vec(S2).x).x == S(2));
			PR_EXPECT(vec(vec(S2).y).y == S(2));
		}

		// ---- Shear (functions.h line ~2441) ----
		PRUnitTestMethod(ShearTests
		, Mat2x2<float>, Mat2x2<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			static_assert(Shear<mat_t>(S(0), S(0)) == Identity<mat_t>());
		}
		PRUnitTestMethod(Shear3DTests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			static_assert(Shear<mat_t>(S(0), S(0), S(0), S(0), S(0), S(0)) == Identity<mat_t>());
		}

		// ---- LookAt (functions.h line ~2493) ----
		PRUnitTestMethod(LookAtTests
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			auto eye = Vec(S(0), S(0), S(0), S(1));
			auto at = Vec(S(0), S(0), S(-1), S(1));
			auto up = Vec(S(0), S(1), S(0), S(0));
			auto L = LookAt<mat_t>(eye, at, up);
			PR_EXPECT(IsOrthonormal(L));
		}

		// ---- ProjectionOrthographic (functions.h line ~2512) ----
		PRUnitTestMethod(ProjectionOrthographicTests
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			auto P = ProjectionOrthographic<mat_t>(S(2), S(2), S(0.1), S(100), true);
			PR_EXPECT(vec(vec(P).x).x != S(0));
		}

		// ---- ProjectionPerspective (functions.h line ~2532) ----
		PRUnitTestMethod(ProjectionPerspectiveTests
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			auto P = ProjectionPerspective<mat_t>(S(2), S(2), S(0.1), S(100), true);
			PR_EXPECT(vec(vec(P).x).x != S(0));
		}

		// ---- ProjectionPerspectiveFOV (functions.h line ~2578) ----
		PRUnitTestMethod(ProjectionPerspectiveFOVTests
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			auto P = ProjectionPerspectiveFOV<mat_t>(constants<S>::tau / S(4), S(1), S(0.1), S(100), true);
			PR_EXPECT(vec(vec(P).x).x != S(0));
		}

		// ---- Diagonalise (functions.h line ~2675) ----
		PRUnitTestMethod(DiagonaliseTests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			// Diagonalise a symmetric matrix (diagonal matrix is trivially symmetric)
			auto M = Scale<mat_t>(Vec(S(3), S(2), S(1)));
			mat_t eigen_vectors;
			Vec eigen_values;
			auto D = Diagonalise(M, eigen_vectors, eigen_values);

			// Eigen values should be the diagonal elements (in some order)
			PR_EXPECT(FEql(CompSum(eigen_values), S(6)));

			// D should be diagonal (off-diagonals ≈ 0)
			PR_EXPECT(FEql(vec(vec(D).x).y, S(0)));
			PR_EXPECT(FEql(vec(vec(D).x).z, S(0)));
		}

		// ---- AxisAngle (functions.h line ~2679) ----
		PRUnitTestMethod(AxisAngleTests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			auto [axis, angle] = AxisAngle(Identity<mat_t>());
			PR_EXPECT(FEql(angle, S(0)));
		}

		// ---- ScaleFrom (functions.h line ~2706) ----
		PRUnitTestMethod(ScaleFromTests
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			auto S0 = ScaleFrom(Identity<mat_t>());
			PR_EXPECT(vec(vec(S0).x).x == S(1));
			PR_EXPECT(vec(vec(S0).y).y == S(1));
		}

		// ---- Unscaled (functions.h line ~2721) ----
		PRUnitTestMethod(UnscaledTests
		, Mat2x2<float>, Mat2x2<double>
		, Mat3x4<float>, Mat3x4<double>
		, Mat4x4<float>, Mat4x4<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			// Unscaled(Identity) should give Identity (columns already unit-length)
			PR_EXPECT(FEql(Unscaled(Identity<mat_t>()), Identity<mat_t>()));

			// Unscaled of a scaled matrix should be orthonormal
			auto M = Identity<mat_t>() * S(3);
			PR_EXPECT(IsOrthonormal(Unscaled(M)));
		}

		// ---- RotationToZAxis (functions.h line ~2739) ----
		PRUnitTestMethod(RotationToZAxisTests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			// Rotating Z axis to Z should give identity
			auto R = RotationToZAxis<mat_t>(Vec::ZAxis());
			PR_EXPECT(FEql(R, Identity<mat_t>()));

			// Rotating any unit vector to Z should produce a valid rotation
			auto R2 = RotationToZAxis<mat_t>(Vec::XAxis());
			PR_EXPECT(FEql(R2 * Vec::XAxis(), Vec::ZAxis()));
		}

		// ---- OriFromDir (functions.h line ~2773) ----
		PRUnitTestMethod(OriFromDirTests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			// Direction along +Z with axis +Z should give identity
			auto ori = OriFromDir<mat_t>(Vec::ZAxis(), AxisId::PosZ);
			PR_EXPECT(IsOrthonormal(ori));
			PR_EXPECT(FEql(vec(ori).z, Vec::ZAxis()));

			// Direction along +X with axis +X should align X axis
			auto ori2 = OriFromDir<mat_t>(Vec::XAxis(), AxisId::PosX);
			PR_EXPECT(IsOrthonormal(ori2));
			PR_EXPECT(FEql(vec(ori2).x, Vec::XAxis()));
		}

		// ---- ScaledOriFromDir (functions.h line ~2795) ----
		PRUnitTestMethod(ScaledOriFromDirTests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			// Scaled direction — axis column should have the length of the input direction
			auto dir = Vec(S(0), S(0), S(3));
			auto ori = ScaledOriFromDir<mat_t>(dir, AxisId::PosZ);
			PR_EXPECT(FEql(Length(vec(ori).z), S(3)));

			// Zero direction gives zero matrix
			PR_EXPECT(ScaledOriFromDir<mat_t>(Vec(S(0)), AxisId::PosZ) == mat_t::Zero());
		}

		// ---- RotationVectorApprox (functions.h line ~2805) ----
		PRUnitTestMethod(RotationVectorApproxTests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			auto rv = RotationVectorApprox(Identity<mat_t>(), Identity<mat_t>());
			PR_EXPECT(FEql(rv, Vec(S(0))));
		}

		// ---- CPM (functions.h line ~2824) ----
		PRUnitTestMethod(CPMTests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			// CPM(v) * u == Cross(v, u)
			auto v = Vec::XAxis();
			auto u = Vec::YAxis();
			PR_EXPECT(FEql(CPM<mat_t>(v) * u, Cross(v, u)));

			// CPM is anti-symmetric
			PR_EXPECT(IsAntiSymmetric(CPM<mat_t>(v)));
		}

		// ---- ExpMap3x3 (functions.h line ~2841) ----
		PRUnitTestMethod(ExpMap3x3Tests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using Vec = typename vt::component_t;
			using S = typename vt::element_t;

			auto R = ExpMap3x3<mat_t>(Vec(S(0)));
			PR_EXPECT(FEql(R, Identity<mat_t>()));
		}

		// ---- LogMap3x3 (functions.h line ~2857) ----
		PRUnitTestMethod(LogMap3x3Tests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using Vec = typename vt::component_t;
			using S = typename vt::element_t;

			auto omega = LogMap3x3(Identity<mat_t>());
			PR_EXPECT(FEql(omega, Vec(S(0))));
		}

		// ---- RotationAt (functions.h line ~2882) ----
		PRUnitTestMethod(RotationAtTests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			// Zero angular velocity and acceleration — orientation unchanged
			auto ori = Identity<mat_t>();
			auto result = RotationAt(0.0f, ori, Vec(S(0)), Vec(S(0)));
			PR_EXPECT(FEql(result, ori));

			// Constant angular velocity about Z, zero acceleration
			auto avel = Vec(S(0), S(0), S(1)); // 1 rad/s about Z
			auto R1 = RotationAt(0.0f, Identity<mat_t>(), avel, Vec(S(0)));
			PR_EXPECT(IsOrthonormal(R1));
		}

		// ---- Lerp (functions.h line ~1554) ----
		PRUnitTestMethod(LerpTests
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			auto a = vec_t(S(0));
			auto b = vec_t(S(10));

			PR_EXPECT(FEql(Lerp(a, b, S(0.0)), a));
			PR_EXPECT(FEql(Lerp(a, b, S(1.0)), b));
			PR_EXPECT(FEql(Lerp(a, b, S(0.5)), vec_t(S(5))));
			PR_EXPECT(FEql(Lerp(a, b, S(0.25)), vec_t(S(2.5))));

			// Scalar Lerp
			PR_EXPECT(FEql(Lerp(S(0), S(10), S(0.5)), S(5)));
			PR_EXPECT(FEql(Lerp(S(-5), S(5), S(0.5)), S(0)));
		}

		// ---- Slerp (functions.h line ~1565) ----
		PRUnitTestMethod(SlerpTests
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;
			using vt = vector_traits<vec_t>;

			auto a = vec_t(S(0)); vec(a).x = S(1);
			auto b = vec_t(S(0)); vec(b).y = S(1);

			// Endpoints
			PR_EXPECT(FEql(Slerp(a, b, S(0.0)), a));
			PR_EXPECT(FEql(Slerp(a, b, S(1.0)), b));

			// Midpoint should be normalised (equal length vectors)
			auto mid = Slerp(a, b, S(0.5));
			PR_EXPECT(FEql(Length(mid), S(1)));
		}

		// ---- Quantise (functions.h line ~1577) ----
		PRUnitTestMethod(QuantiseTests
		, Vec2<float>, Vec2<double>
		, Vec3<float>, Vec3<double>
		, Vec4<float>, Vec4<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			// Scalar quantise
			PR_EXPECT(Quantise(S(0.123456), 100) == S(0.12));
			PR_EXPECT(Quantise(S(0.999), 10) == S(0.9));

			// Vector quantise
			auto v = vec_t(S(0.123456));
			auto q = Quantise(v, 100);
			PR_EXPECT(FEql(q, vec_t(S(0.12))));
		}

		// ---- SmoothStep (functions.h line ~1637) ----
		PRUnitTestMethod(SmoothStepTests
		, float, double
		) {
			using S = T;

			// SmoothStep: 0 at lo, 1 at hi, smooth in between
			PR_EXPECT(FEql(SmoothStep(S(0), S(1), S(0)), S(0)));
			PR_EXPECT(FEql(SmoothStep(S(0), S(1), S(1)), S(1)));
			PR_EXPECT(FEql(SmoothStep(S(0), S(1), S(0.5)), S(0.5)));

			// Clamped outside range
			PR_EXPECT(FEql(SmoothStep(S(0), S(1), S(-1)), S(0)));
			PR_EXPECT(FEql(SmoothStep(S(0), S(1), S(2)), S(1)));

			// SmoothStep2: same boundary conditions
			PR_EXPECT(FEql(SmoothStep2(S(0), S(1), S(0)), S(0)));
			PR_EXPECT(FEql(SmoothStep2(S(0), S(1), S(1)), S(1)));
			PR_EXPECT(FEql(SmoothStep2(S(0), S(1), S(0.5)), S(0.5)));
		}

		// ---- Step (functions.h line ~1631) ----
		PRUnitTestMethod(StepTests
		, float, double, int32_t, int64_t
		) {
			using S = T;

			PR_EXPECT(Step(S(0), S(1)) == S(0)); // lo <= hi → 0
			PR_EXPECT(Step(S(1), S(0)) == S(1)); // lo > hi → 1
			PR_EXPECT(Step(S(5), S(5)) == S(0)); // lo == hi → 0
		}

		// ---- Sigmoid (functions.h line ~1653) ----
		PRUnitTestMethod(SigmoidTests
		, float, double
		) {
			using S = T;

			// Sigmoid(0) should be 0
			PR_EXPECT(FEql(Sigmoid(S(0)), S(0)));

			// Sigmoid should be odd: Sigmoid(-x) == -Sigmoid(x)
			PR_EXPECT(FEql(Sigmoid(S(1)) + Sigmoid(S(-1)), S(0)));

			// Sigmoid should be bounded in (-1, 1)
			PR_EXPECT(Sigmoid(S(1000)) < S(1));
			PR_EXPECT(Sigmoid(S(-1000)) > S(-1));
		}

		// ---- UnitCubic (functions.h line ~1662) ----
		PRUnitTestMethod(UnitCubicTests
		, float, double
		) {
			using S = T;

			// f(0) = 0, f(1) = 1, df(0.5) = 0
			PR_EXPECT(FEql(UnitCubic(S(0)), S(0)));
			PR_EXPECT(FEql(UnitCubic(S(1)), S(1)));
			PR_EXPECT(FEql(UnitCubic(S(0.5)), S(0.5)));
		}

		// ---- Rsqrt (functions.h line ~1669) ----
		PRUnitTestMethod(RsqrtTests
		, float, double
		) {
			using S = T;

			auto v = S(4);
			auto expected = S(0.5); // 1/sqrt(4) = 0.5

			// Rsqrt0: low precision
			PR_EXPECT(FEqlRelative(Rsqrt0(v), expected, S(0.01)));

			// Rsqrt1: higher precision
			PR_EXPECT(FEqlRelative(Rsqrt1(v), expected, S(0.0001)));
		}

		// ---- Log (functions.h line ~1477) ----
		PRUnitTestMethod(LogTests
		, float, double
		) {
			using S = T;

			// Log(1) == 0
			PR_EXPECT(FEql(Log(S(1)), S(0)));

			// Log(e) == 1
			PR_EXPECT(FEqlRelative(Log(Exp(S(1))), S(1), S(0.0001)));

			// Log10(100) == 2
			PR_EXPECT(FEqlRelative(Log10(S(100)), S(2), S(0.0001)));
		}

		// ---- Sector (functions.h line ~3638) ----
		PRUnitTestMethod(SectorTests
		, Vec2<float>, Vec2<double>
		) {
			using vec_t = T;
			using S = typename vector_traits<vec_t>::element_t;

			// 4 sectors: right=0, up=1, left=2, down=3
			vec_t right = {S(1), S(0)};
			vec_t up = {S(0), S(1)};
			vec_t left = {S(-1), S(0)};
			vec_t down = {S(0), S(-1)};

			PR_EXPECT(Sector(right, 4) == 0);
			PR_EXPECT(Sector(up, 4) == 1);
			PR_EXPECT(Sector(left, 4) == 2);
			PR_EXPECT(Sector(down, 4) == 3);
		}

		// ---- RandomN (functions.h line ~2922) ----
		PRUnitTestMethod(RandomNTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = vt::element_t;

			std::default_random_engine rng(42);
			auto v = RandomN<vec_t>(rng);
			if constexpr (IsRank2<vec_t>)
			{
				if constexpr (std::floating_point<S>)
				{
					if constexpr (vt::dimension > 0) PR_EXPECT(IsNormalised(vec(v).x));
					if constexpr (vt::dimension > 1) PR_EXPECT(IsNormalised(vec(v).y));
					if constexpr (vt::dimension > 2) PR_EXPECT(IsNormalised(vec(v).z));
					if constexpr (vt::dimension > 3) PR_EXPECT(IsNormalised(vec(v).w));
				}
				else
				{
					// For integer matrices, each column is an axis-aligned unit vector
					if constexpr (vt::dimension > 0) PR_EXPECT(LengthSq(vec(v).x) == S(1));
					if constexpr (vt::dimension > 1) PR_EXPECT(LengthSq(vec(v).y) == S(1));
					if constexpr (vt::dimension > 2) PR_EXPECT(LengthSq(vec(v).z) == S(1));
					if constexpr (vt::dimension > 3) PR_EXPECT(LengthSq(vec(v).w) == S(1));
				}
			}
			else
			{
				if constexpr (std::floating_point<S>)
					PR_EXPECT(IsNormalised(v));
				else
					PR_EXPECT(LengthSq(v) == S(1));
			}
		}

		// ---- Random (functions.h line ~2955) ----
		PRUnitTestMethod(RandomRangeTests
		, Vec2<float>, Vec2<double>, Vec2<int32_t>, Vec2<int64_t>
		, Vec3<float>, Vec3<double>, Vec3<int32_t>, Vec3<int64_t>
		, Vec4<float>, Vec4<double>, Vec4<int32_t>, Vec4<int64_t>
		, Mat2x2<float>, Mat2x2<double>, Mat2x2<int32_t>, Mat2x2<int64_t>
		, Mat3x4<float>, Mat3x4<double>, Mat3x4<int32_t>, Mat3x4<int64_t>
		, Mat4x4<float>, Mat4x4<double>, Mat4x4<int32_t>, Mat4x4<int64_t>
		) {
			using vec_t = T;
			using vt = vector_traits<vec_t>;
			using S = typename vt::element_t;

			std::default_random_engine rng(42);
			auto vmin = vec_t(S(-1));
			auto vmax = vec_t(S(1));
			auto v = Random<vec_t>(rng, vmin, vmax);

			// Result should be within [vmin, vmax] per element
			if constexpr (IsRank1<vec_t>)
			{
				for (int i = 0; i != vt::dimension; ++i)
				{
					PR_EXPECT(v[i] >= S(-1));
					PR_EXPECT(v[i] <= S(1));
				}
			}
			else if constexpr (IsRank2<vec_t>)
			{
				using C = typename vt::component_t;
				for (int i = 0; i != vt::dimension; ++i)
				{
					auto comp = v[i];
					for (int j = 0; j != vector_traits<C>::dimension; ++j)
					{
						PR_EXPECT(comp[j] >= S(-1));
						PR_EXPECT(comp[j] <= S(1));
					}
				}
			}
		}

		// ---- Random rotation (functions.h line ~3035) ----
		PRUnitTestMethod(RandomRotationTests
		, Mat3x4<float>, Mat3x4<double>
		) {
			using mat_t = T;
			using vt = vector_traits<mat_t>;
			using S = typename vt::element_t;
			using Vec = typename vt::component_t;

			std::default_random_engine rng(42);

			// 3D random rotation about a given axis
			auto R = Random<mat_t>(rng, Vec::ZAxis(), S(0), constants<S>::tau);
			PR_EXPECT(IsOrthonormal(R));
		}

		// ---- Random 2D rotation (functions.h line ~3058) ----
		PRUnitTestMethod(Random2DRotationTests
		, Mat2x2<float>, Mat2x2<double>
		) {
			using mat_t = T;
			using S = typename vector_traits<mat_t>::element_t;

			std::default_random_engine rng(42);

			// 2D random rotation (no args — full [0, tau))
			auto R = Random<mat_t>(rng);
			PR_EXPECT(IsOrthonormal(R));

			// 2D random rotation with angle range
			auto R2 = Random<mat_t>(rng, S(0), constants<S>::tau);
			PR_EXPECT(IsOrthonormal(R2));
		}
	};
}
#endif
