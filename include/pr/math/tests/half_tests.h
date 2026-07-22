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
	PRUnitTestClass(HalfTypeTests)
	{
		PRUnitTestMethod(ScalarRoundTrip)
		{
			// Basic values round-trip through half
			auto test = [](float f)
			{
				auto h = F32toF16(f);
				auto r = F16toF32(h);
				return r == f || (f != f && r != r); // NaN != NaN
			};
			PR_EXPECT(test(0.0f));
			PR_EXPECT(test(-0.0f));
			PR_EXPECT(test(1.0f));
			PR_EXPECT(test(-1.0f));
			PR_EXPECT(test(0.5f));
			PR_EXPECT(test(0.25f));
			PR_EXPECT(test(65504.0f)); // max finite half
		}

		PRUnitTestMethod(SpecialValues)
		{
			// Positive and negative infinity
			auto pos_inf = F32toF16(std::numeric_limits<float>::infinity());
			auto neg_inf = F32toF16(-std::numeric_limits<float>::infinity());
			PR_EXPECT(F16toF32(pos_inf) == std::numeric_limits<float>::infinity());
			PR_EXPECT(F16toF32(neg_inf) == -std::numeric_limits<float>::infinity());

			// NaN
			auto nan_h = F32toF16(std::numeric_limits<float>::quiet_NaN());
			PR_EXPECT(std::isnan(F16toF32(nan_h)));

			// Zero
			auto zero = F32toF16(0.0f);
			PR_EXPECT(F16toF32(zero) == 0.0f);
		}

		PRUnitTestMethod(CompileTimeConversion)
		{
			// Constexpr uses the public API so the test fails if constant evaluation ever falls back to the runtime-only path again.
			constexpr auto h1 = F32toF16(1.0f);
			constexpr auto f1 = F16toF32(h1);
			static_assert(f1 == 1.0f);

			constexpr auto h0 = F32toF16(0.0f);
			constexpr auto f0 = F16toF32(h0);
			static_assert(f0 == 0.0f);

			constexpr auto hm = F32toF16(-0.5f);
			constexpr auto fm = F16toF32(hm);
			static_assert(fm == -0.5f);

			static_assert(F32toF16(-0.0f) == half_t{0x8000u});
			static_assert(F32toF16(std::bit_cast<float>(0x33000000u)) == half_t{0x0000u});
			static_assert(F32toF16(std::bit_cast<float>(0x33000001u)) == half_t{0x0001u});
			static_assert(F32toF16(std::bit_cast<float>(0x337fffffu)) == half_t{0x0001u});
			static_assert(F32toF16(0x1.0p-24f) == half_t{0x0001u});
			static_assert(F32toF16(0x1.0p-14f) == half_t{0x0400u});
			static_assert(F32toF16(65504.0f) == half_t{0x7bffu});
			static_assert(F32toF16(std::bit_cast<float>(0x7fc00000u)) == half_t{0x7e00u});
			static_assert(std::bit_cast<uint32_t>(F16toF32(half_t{0x0001u})) == 0x33800000u);
			static_assert(std::bit_cast<uint32_t>(F16toF32(half_t{0x03ffu})) == 0x387fc000u);
			static_assert(std::bit_cast<uint32_t>(F16toF32(half_t{0x7e00u})) == 0x7fc00000u);
			static_assert(std::bit_cast<uint32_t>(F16toF32(half_t{0x7c01u})) == 0x7fc02000u);
			static_assert(std::bit_cast<uint32_t>(F16toF32(half_t{0xfc01u})) == 0xffc02000u);

			// This reproduces the original regression: constexpr vector conversion must stay on the constexpr scalar path.
			constexpr auto hv = F32toF16(Vec4<float>{1.0f, -0.5f, 0.25f, -0.0f});
			static_assert(hv.x == half_t{0x3c00u});
			static_assert(hv.y == half_t{0xb800u});
			static_assert(hv.z == half_t{0x3400u});
			static_assert(hv.w == half_t{0x8000u});

			constexpr auto fv = F16toF32<Vec4<float>>(hv);
			static_assert(fv.x == 1.0f);
			static_assert(fv.y == -0.5f);
			static_assert(fv.z == 0.25f);
			static_assert(std::bit_cast<uint32_t>(fv.w) == 0x80000000u);
		}

		PRUnitTestMethod(VectorConversion, float, double)
		{
			using V4 = Vec4<T>;

			auto v = V4(T(1.0), T(-0.5), T(0.25), T(0.0));
			auto h = F32toF16(v);
			auto r = F16toF32<V4>(h);
			PR_EXPECT(FEql(v, r));
		}

		PRUnitTestMethod(Overflow)
		{
			// Values too large for half should become infinity
			auto h = F32toF16(100000.0f);
			PR_EXPECT(F16toF32(h) == std::numeric_limits<float>::infinity());

			auto hn = F32toF16(-100000.0f);
			PR_EXPECT(F16toF32(hn) == -std::numeric_limits<float>::infinity());
		}

		PRUnitTestMethod(Denormals)
		{
			// Small values near the denormal range
			float small_val = 5.96046e-8f; // smallest positive half denormal
			auto h = F32toF16(small_val);
			auto r = F16toF32(h);

			// Should be close to zero or the denormal value
			PR_EXPECT(r >= 0.0f);
			PR_EXPECT(r <= small_val * 2.0f);
		}

		PRUnitTestMethod(UserDefinedLiteral)
		{
			auto h = 1.0_hf;
			PR_EXPECT(F16toF32(h) == 1.0f);

			auto h2 = F32toF16(-0.5f);
			PR_EXPECT(F16toF32(h2) == -0.5f);
		}
		PRUnitTestMethod(RuntimeMatchesConstexpr)
		{
			struct F32Case
			{
				uint32_t f32_bits;
				half_t half_bits;
				uint32_t round_trip_bits;
			};

			// Use exact bit patterns so rounding, signed zero, subnormal, infinity, and quiet-NaN cases are all compared without depending on formatted literals.
			auto const f32_cases = std::array
			{
				F32Case{0x00000000u, half_t{0x0000u}, 0x00000000u},
				F32Case{0x80000000u, half_t{0x8000u}, 0x80000000u},
				F32Case{0x3f800000u, half_t{0x3c00u}, 0x3f800000u},
				F32Case{0xbf000000u, half_t{0xb800u}, 0xbf000000u},
				F32Case{0x3e800000u, half_t{0x3400u}, 0x3e800000u},
				F32Case{0x33000000u, half_t{0x0000u}, 0x00000000u},
				F32Case{0x33000001u, half_t{0x0001u}, 0x33800000u},
				F32Case{0x337fffffu, half_t{0x0001u}, 0x33800000u},
				F32Case{0xb3000000u, half_t{0x8000u}, 0x80000000u},
				F32Case{0xb3000001u, half_t{0x8001u}, 0xb3800000u},
				F32Case{0xb37fffffu, half_t{0x8001u}, 0xb3800000u},
				F32Case{0x33800000u, half_t{0x0001u}, 0x33800000u},
				F32Case{0x38800000u, half_t{0x0400u}, 0x38800000u},
				F32Case{0x3f801000u, half_t{0x3c00u}, 0x3f800000u},
				F32Case{0x3f801800u, half_t{0x3c01u}, 0x3f802000u},
				F32Case{0x477fe000u, half_t{0x7bffu}, 0x477fe000u},
				F32Case{0x47c35000u, half_t{0x7c00u}, 0x7f800000u},
				F32Case{0x7f800000u, half_t{0x7c00u}, 0x7f800000u},
				F32Case{0x7fc00000u, half_t{0x7e00u}, 0x7fc00000u},
			};

			for (auto const& test_case : f32_cases)
			{
				auto const value = std::bit_cast<float>(test_case.f32_bits);
				auto const constexpr_half = F32toF16CT(value);
				auto const runtime_half = F32toF16(value);
				PR_EXPECT(constexpr_half == test_case.half_bits);
				PR_EXPECT(runtime_half == test_case.half_bits);

				auto const constexpr_round_trip = F16toF32CT(test_case.half_bits);
				auto const runtime_round_trip = F16toF32(test_case.half_bits);
				PR_EXPECT(std::bit_cast<uint32_t>(constexpr_round_trip) == test_case.round_trip_bits);
				PR_EXPECT(std::bit_cast<uint32_t>(runtime_round_trip) == test_case.round_trip_bits);
			}

			struct F16Case
			{
				half_t half_bits;
				uint32_t f32_bits;
			};

			// Decode-only coverage keeps the largest subnormal and signed special values checked even when they are not produced by the forward conversion inputs above.
			auto const f16_cases = std::array
			{
				F16Case{half_t{0x03ffu}, 0x387fc000u},
				F16Case{half_t{0x7c01u}, 0x7fc02000u},
				F16Case{half_t{0xfc01u}, 0xffc02000u},
				F16Case{half_t{0xfc00u}, 0xff800000u},
				F16Case{half_t{0xfe00u}, 0xffc00000u},
			};

			for (auto const& test_case : f16_cases)
			{
				auto const constexpr_value = F16toF32CT(test_case.half_bits);
				auto const runtime_value = F16toF32(test_case.half_bits);
				PR_EXPECT(std::bit_cast<uint32_t>(constexpr_value) == test_case.f32_bits);
				PR_EXPECT(std::bit_cast<uint32_t>(runtime_value) == test_case.f32_bits);
			}
		}
	};
}
#endif
