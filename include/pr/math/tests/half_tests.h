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
			// constexpr round-trip
			constexpr auto h1 = F32toF16CT(1.0f);
			constexpr auto f1 = F16toF32CT(h1);
			static_assert(f1 == 1.0f);

			constexpr auto h0 = F32toF16CT(0.0f);
			constexpr auto f0 = F16toF32CT(h0);
			static_assert(f0 == 0.0f);

			constexpr auto hm = F32toF16CT(-0.5f);
			constexpr auto fm = F16toF32CT(hm);
			static_assert(fm == -0.5f);
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
	};
}
#endif
