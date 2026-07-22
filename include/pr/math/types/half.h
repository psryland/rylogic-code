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

	namespace half_impl
	{
		// Round an integer right shift using IEEE round-to-nearest-even so the constexpr path matches the runtime F16C conversion.
		constexpr uint32_t RoundShiftRightNearestEven(uint32_t value, uint32_t shift) noexcept
		{
			if (shift == 0)
				return value;

			auto const truncated = value >> shift;
			auto const remainder_mask = (uint32_t{1} << shift) - 1u;
			auto const remainder = value & remainder_mask;
			auto const halfway = uint32_t{1} << (shift - 1u);
			auto const round_up = remainder > halfway || (remainder == halfway && (truncated & 1u) != 0u);
			return truncated + static_cast<uint32_t>(round_up);
		}
	}

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

	// Convert a 32-bit float to IEEE binary16 during constant evaluation. The runtime path keeps using F16C so this code only needs to mirror the hardware bit pattern.
	constexpr half_t F32toF16CT(float f32) noexcept
	{
		auto const bits = std::bit_cast<uint32_t>(f32);
		auto const sign_bits = static_cast<half_t>((bits >> 16) & 0x8000u);
		auto const exponent_bits = static_cast<uint32_t>((bits >> 23) & 0xffu);
		auto const mantissa_bits = static_cast<uint32_t>(bits & 0x007fffffu);

		// Preserve infinities and carry a quiet NaN payload into the half mantissa so constant evaluation classifies the same way as the F16C path.
		if (exponent_bits == 0xffu)
		{
			if (mantissa_bits == 0u)
				return static_cast<half_t>(sign_bits | 0x7c00u);

			auto nan_payload = static_cast<half_t>(mantissa_bits >> 13);
			nan_payload = static_cast<half_t>(nan_payload | 0x0200u);
			return static_cast<half_t>(sign_bits | 0x7c00u | nan_payload);
		}

		// Float32 subnormals are far smaller than the binary16 range, so they only contribute the sign when rounded.
		if (exponent_bits == 0u)
			return sign_bits;

		auto const exponent = static_cast<int>(exponent_bits) - 127;
		if (exponent > 15)
			return static_cast<half_t>(sign_bits | 0x7c00u);

		auto const significand_bits = static_cast<uint32_t>(0x00800000u | mantissa_bits);
		if (exponent >= -14)
		{
			// Round the 24-bit float significand into the 11-bit half significand, then fold any carry into the exponent.
			auto rounded_significand = half_impl::RoundShiftRightNearestEven(significand_bits, 13);
			auto half_exponent = static_cast<uint32_t>(exponent + 15);
			if (rounded_significand == 0x0800u)
			{
				rounded_significand = 0x0400u;
				++half_exponent;
				if (half_exponent >= 0x1fu)
					return static_cast<half_t>(sign_bits | 0x7c00u);
			}

			return static_cast<half_t>(sign_bits | (half_exponent << 10) | (rounded_significand & 0x03ffu));
		}

		// Values below the normal range become subnormals or signed zero, still using round-to-nearest-even.
		if (exponent < -24)
			return sign_bits;

		auto rounded_mantissa = half_impl::RoundShiftRightNearestEven(significand_bits, static_cast<uint32_t>(-exponent - 1));
		if (rounded_mantissa == 0x0400u)
			return static_cast<half_t>(sign_bits | 0x0400u);

		return static_cast<half_t>(sign_bits | rounded_mantissa);
	}
	// Convert an IEEE binary16 value to 32-bit float during constant evaluation so constexpr code sees the same categories and payload layout as the runtime path.
	constexpr float F16toF32CT(half_t f16) noexcept
	{
		auto const sign_bits = static_cast<uint32_t>(f16 & 0x8000u) << 16;
		auto const exponent_bits = static_cast<uint32_t>((f16 >> 10) & 0x1fu);
		auto const mantissa_bits = static_cast<uint32_t>(f16 & 0x03ffu);

		if (exponent_bits == 0x1fu)
			return std::bit_cast<float>(sign_bits | 0x7f800000u | (mantissa_bits << 13));

		if (exponent_bits != 0u)
			return std::bit_cast<float>(sign_bits | ((exponent_bits + 112u) << 23) | (mantissa_bits << 13));

		if (mantissa_bits == 0u)
			return std::bit_cast<float>(sign_bits);

		// Binary16 subnormals need renormalising before the float exponent can be reconstructed.
		auto normalised_mantissa = mantissa_bits;
		auto exponent = -14;
		while ((normalised_mantissa & 0x0400u) == 0u)
		{
			normalised_mantissa <<= 1;
			--exponent;
		}
		normalised_mantissa &= 0x03ffu;
		return std::bit_cast<float>(sign_bits | (static_cast<uint32_t>(exponent + 127) << 23) | (normalised_mantissa << 13));
	}

	// Convert between 32-bit float (1s8e23m) and IEEE binary16 (1s5e10m). Constant evaluation uses the scalar implementation while runtime keeps the F16C fast path.
	inline constexpr half_t F32toF16(float f32) noexcept
	{
		if consteval
		{
			return F32toF16CT(f32);
		}

		#if PR_MATHS_USE_INTRINSICS
		auto vecf32 = _mm_set_ps1(f32);
		auto vecf16 = _mm_cvtps_ph(vecf32, _MM_FROUND_TO_NEAREST_INT);
		return static_cast<half_t>(vecf16.m128i_u16[0]);
		#else
		return F32toF16CT(f32);
		#endif
	}
	// Convert between IEEE binary16 (1s5e10m) and 32-bit float (1s8e23m). The runtime path still uses F16C where available.
	inline constexpr float F16toF32(half_t f16) noexcept
	{
		if consteval
		{
			return F16toF32CT(f16);
		}

		#if PR_MATHS_USE_INTRINSICS
		auto vecf16 = _mm_set_epi16(0, 0, 0, 0, 0, 0, 0, static_cast<short>(f16));
		auto vecf32 = _mm_cvtph_ps(vecf16);
		return vecf32.m128_f32[0];
		#else
		return F16toF32CT(f16);
		#endif
	}

	// Return 'v' converted to half size floats
	template <VectorTypeFP Vec> requires (vector_traits<std::remove_cv_t<Vec>>::dimension == 4) constexpr Half4 pr_vectorcall F32toF16(Vec v) noexcept
	{
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
	template <VectorTypeFP Vec> requires (vector_traits<std::remove_cv_t<Vec>>::dimension == 4) constexpr Vec pr_vectorcall F16toF32(Half4 v) noexcept
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
				PR_EXPECT(All(x2 == x0));
			}
			{// Integer-valued components
				auto x0 = Vec4<float>{1, 2, 3, 4};
				auto x1 = F32toF16(x0);
				auto x2 = F16toF32<Vec4<float>>(x1);
				PR_EXPECT(All(x2 == x0));
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
			constexpr auto h = F32toF16(1.0f);
			constexpr auto f = F16toF32(h);
			static_assert(f == 1.0f);

			// Constexpr zero
			constexpr auto hz = F32toF16(0.0f);
			constexpr auto fz = F16toF32(hz);
			static_assert(fz == 0.0f);

			// Constexpr negative
			constexpr auto hn = F32toF16(-1.0f);
			constexpr auto fn = F16toF32(hn);
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
