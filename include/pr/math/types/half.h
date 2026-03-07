//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/core/traits.h"
#include "pr/math/types/vector4.h"

namespace pr::math
{
	using half_t = unsigned short;
	using Half4 = Vec4<half_t>;

	#define PR_MATH_DEFINE_TYPE(element)\
	template <> struct vector_traits<Vec4<element>>\
		: vector_traits_base<element, element, 4>\
		, vector_access_member<Vec4<element>, element, 4>\
	{};\
	\
	static_assert(VectorType<Vec4<element>>, "Vec4<"#element"> is not a valid vector type");\
	static_assert(IsRank1<Vec4<element>>, "Vec4<"#element"> is not rank 1");\
	static_assert(sizeof(Vec4<element>) == 4*sizeof(element), "Vec4<"#element"> has the wrong size");\
	static_assert(std::is_trivially_copyable_v<Vec4<element>>, "Vec4<"#element"> is not trivially copyable");
	PR_MATH_DEFINE_TYPE(half_t);
	#undef PR_MATH_DEFINE_TYPE

	// Convert between 32-bit float (1s7e24m) and 16-bit float (1s5e10m) at compile time
	constexpr half_t F32toF16CT(float f32) noexcept
	{
		// see https://gist.github.com/martin-kallman/5049614 (Note comments though, Martin's implementation has a bug)
		// see https://github.com/numpy/numpy/blob/master/numpy/core/src/npymath/halffloat.c#L466

		// Martin Kallman (mostly. Some parts nicked from numpy)
		//
		// Fast single-precision to half-precision floating point conversion
		//  - Supports signed zero, denormals-as-zero (DAZ), flush-to-zero (FTZ),
		//    clamp-to-max
		//  - Does not support infinities or NaN
		//  - Few, partially pipeline-able, non-branching instructions,
		//  - Core operations ~10 clock cycles on modern x86-64
		auto u = std::bit_cast<uint32_t>(f32);
		auto t1 = static_cast<uint32_t>(u & 0x7fffffff);        // Non-sign bits
		auto t2 = static_cast<uint32_t>(u & 0x80000000);        // Sign bit
		auto t3 = static_cast<uint32_t>(u & 0x7f800000);        // Exponent
		auto t4 = static_cast<uint32_t>(u & 0x007fffffu) >> 13; // NaN signal >> 13

		t1 >>= 13;                                     // Align mantissa on MSB
		t2 >>= 16;                                     // Shift sign bit into position
		t1 -= 0x1c000;                                 // Adjust bias
		t1 = (t3 < 0x38800000) ? 0 : t1;               // Flush-to-zero
		t1 = (t3 > 0x47000000) ? 0x7c00u : t1;         // Clamp-to-max (inf = 0x7c00u, max = 0x7bffu)
		t1 = (t3 == 0x7f800000) ? (0x7c00u + t4) : t1; // NaN or Inf (t4 == 0)
		t1 = (t3 == 0) ? 0 : t1;                       // Denormals-as-zero
		t1 |= t2;                                      // Re-insert sign bit
		return static_cast<half_t>(t1);
	}
	constexpr float F16toF32CT(half_t f16) noexcept
	{
		// Martin Kallman (mostly. Some parts nicked from numpy)
		//
		// Fast half-precision to single-precision floating point conversion
		//  - Supports signed zero and denormals-as-zero (DAZ)
		//  - Does not support infinities or NaN
		//  - Few, partially pipeline-able, non-branching instructions,
		//  - Core operations ~6 clock cycles on modern x86-64

		auto t1 = static_cast<uint32_t>(f16 & 0x7fff);      // Non-sign bits
		auto t2 = static_cast<uint32_t>(f16 & 0x8000);      // Sign bit
		auto t3 = static_cast<uint32_t>(f16 & 0x7c00);      // Exponent
		auto t4 = static_cast<uint32_t>(t1 - 0x7c00) << 13; // NaN signal

		t1 <<= 13;                                         // Align mantissa on MSB
		t2 <<= 16;                                         // Shift sign bit into position
		t1 += 0x38000000;                                  // Adjust bias
		t1 = t3 == 0 ? 0 : t1;                             // Denormals-as-zero
		t1 = t3 >= 0x7c00u ? 0x7f800000u + t4 : t1;        // NaN or Inf (t4 == 0)
		t1 |= t2;                                          // Re-insert sign bit
		return std::bit_cast<float>(t1);
	}

	// Convert between 32-bit float (1s7e24m) and 16-bit float (1s5e10m)
	inline half_t F32toF16(float f32) noexcept
	{
		#if PR_MATHS_USE_INTRINSICS
		auto vecf32 = _mm_set_ps1(f32);
		auto vecf16 = _mm_cvtps_ph(vecf32, _MM_FROUND_TO_NEAREST_INT);
		return static_cast<half_t>(vecf16.m128i_u16[0]);
		#else
		return F32toF16CT(f32);
		#endif
	}
	inline float F16toF32(half_t f16) noexcept
	{
		#if PR_MATHS_USE_INTRINSICS
		auto vecf16 = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, static_cast<short>(f16));
		auto vecf32 = _mm_cvtph_ps(vecf16);
		return vecf32.m128_f32[0];
		#else
		return F16toF32CT(f16);
		#endif
	}

	// Return 'v' converted to half size floats
	template <VectorTypeFP Vec> constexpr Half4 pr_vectorcall F32toF16(Vec v) noexcept
	{
		using vt = vector_traits<Vec>;

		auto fallback = [&]() constexpr { return Half4{ F32toF16(static_cast<float>(vec(v).x)), F32toF16(static_cast<float>(vec(v).y)), F32toF16(static_cast<float>(vec(v).z)), F32toF16(static_cast<float>(vec(v).w)) }; };
		if consteval
		{
			return fallback();
		}
		else
		{
			if constexpr (Vec::IntrinsicF)
			{
				auto f16 = _mm_cvtps_ph(v.vec, _MM_FROUND_TO_NEAREST_INT); //|_MM_FROUND_NO_EXC - emits a warning
				return Half4{ f16.m128i_u16[0], f16.m128i_u16[1], f16.m128i_u16[2], f16.m128i_u16[3] };
			}
			else
			{
				return fallback();
			}
		}
	}

	// Return 'v' to full size floats/doubles
	template <VectorTypeFP Vec> constexpr Vec pr_vectorcall F16toF32(Half4 v) noexcept
	{
		using vt = vector_traits<Vec>;

		auto fallback = [&]() constexpr { return Vec{ static_cast<typename vt::element_t>(F16toF32(v.x)), static_cast<typename vt::element_t>(F16toF32(v.y)), static_cast<typename vt::element_t>(F16toF32(v.z)), static_cast<typename vt::element_t>(F16toF32(v.w)) }; };
		if consteval
		{
			return fallback();
		}
		else
		{
			if constexpr (Vec::IntrinsicF)
			{
				auto f16 = _mm_set_epi16(0, 0, 0, 0, v.w, v.z, v.y, v.x);
				auto res = Vec{ _mm_cvtph_ps(f16) };
				return res;
			}
			else
			{
				return fallback();
			}
		}
	}

	// float literal to half_t
	constexpr half_t operator ""_hf(long double x) noexcept
	{
		return F32toF16CT(static_cast<float>(x));
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::math::tests
{
	PRUnitTestClass(HalfTests)
	{
		PRUnitTestMethod(LiteralTests)
		{
			constexpr half_t h0 = 1.2345_hf;
			static_assert(sizeof(h0) == sizeof(half_t));

			auto h1 = F32toF16(1.2345f);
			PR_EXPECT(h0 == h1);
		}
		PRUnitTestMethod(ScalarRoundTripTests)
		{
			{// Zero
				auto x0 = 0.0f;
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32(x1);
				PR_EXPECT(x0 == x2);
			}
			{// Tau
				auto x0 = 6.28318530f;
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32(x1);
				PR_EXPECT(FEqlRelative(x0, x2, 0.005f));
			}
			{// Negative one
				auto x0 = -1.0f;
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32(x1);
				PR_EXPECT(FEqlRelative(x0, x2, 0.005f));
			}
			{// Large negative
				auto x0 = -4000.0f;
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32(x1);
				PR_EXPECT(FEqlRelative(x0, x2, 0.005f));
			}
			{// Medium positive
				auto x0 = 200.0f;
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32(x1);
				PR_EXPECT(FEqlRelative(x0, x2, 0.005f));
			}
			{// Small denormal
				auto x0 = -4.125e-6f;
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32(x1);
				PR_EXPECT(FEqlRelative(x0, x2, 0.005f));
			}
		}
		PRUnitTestMethod(SpecialValueTests)
		{
			{// +Inf
				auto x0 = limits<float>::infinity();
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32(x1);
				PR_EXPECT(x0 == x2);
			}
			{// -Inf
				auto x0 = -limits<float>::infinity();
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32(x1);
				PR_EXPECT(x0 == x2);
			}
			{// NaN (NaN != NaN by definition)
				auto x0 = limits<float>::quiet_NaN();
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32(x1);
				PR_EXPECT((x0 == x2) == false);
			}
		}
		PRUnitTestMethod(VectorRoundTripTests)
		{
			{// Zero vector
				auto x0 = Vec4<float>{};
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32<Vec4<float>>(x1);
				PR_EXPECT(x2 == x0);
			}
			{// Integer-valued components
				auto x0 = Vec4<float>{1, 2, 3, 4};
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32<Vec4<float>>(x1);
				PR_EXPECT(x2 == x0);
			}
			{// Mixed values
				auto x0 = Vec4<float>{-4000.0f, -200.0f, 0.003f, -4.125e-6f};
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32<Vec4<float>>(x1);
				PR_EXPECT(FEql(x2, x0));
			}
		}
		PRUnitTestMethod(ConstexprTests)
		{
			// Constexpr round-trip
			constexpr auto h = F32toF16CT(1.0f);
			constexpr auto f = F16toF32CT(h);
			static_assert(f == 1.0f);

			// Constexpr zero
			constexpr auto hz = F32toF16CT(0.0f);
			constexpr auto fz = F16toF32CT(hz);
			static_assert(fz == 0.0f);

			// Constexpr negative
			constexpr auto hn = F32toF16CT(-1.0f);
			constexpr auto fn = F16toF32CT(hn);
			static_assert(fn == -1.0f);
		}
		PRUnitTestMethod(BoundaryTests)
		{
			// Max representable half value (~65504)
			auto h_max = F32toF16(65504.0f);
			auto f_max = F16toF32(h_max);
			PR_EXPECT(FEqlRelative(f_max, 65504.0f, 0.001f));

			// Overflow clamps to inf
			auto h_over = F32toF16(100000.0f);
			auto f_over = F16toF32(h_over);
			PR_EXPECT(f_over == limits<float>::infinity());

			// Smallest normal half (~6.1e-5)
			auto h_min = F32toF16(6.1035e-5f);
			auto f_min = F16toF32(h_min);
			PR_EXPECT(FEqlRelative(f_min, 6.1035e-5f, 0.01f));
		}
	};
}
#endif
