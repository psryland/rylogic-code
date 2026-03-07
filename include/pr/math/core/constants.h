//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once

namespace pr::math
{
	// Alias the numeric_limits to a shorter name
	template <typename Type> using limits = std::numeric_limits<Type>;

	// Mathematical constants.
	template <typename Type> struct constants {};
	template <> struct constants<double>
	{
		static constexpr double tiny = 1.0000000000000002e-12;
		static constexpr double tiny_sq = 1.0000000000000003e-24;
		static constexpr double tiny_sqrt = 1.0000000000000002e-06;

		static constexpr double tau = 6.283185307179586476925286766559; // circle constant
		static constexpr double inv_tau = 1.0 / tau;
		static constexpr double tau_by_2 = tau / 2.0;
		static constexpr double tau_by_3 = tau / 3.0;
		static constexpr double tau_by_4 = tau / 4.0;
		static constexpr double tau_by_5 = tau / 5.0;
		static constexpr double tau_by_6 = tau / 6.0;
		static constexpr double tau_by_7 = tau / 7.0;
		static constexpr double tau_by_8 = tau / 8.0;
		static constexpr double tau_by_360 = tau / 360.0;
		static constexpr double E60_by_tau = 360.0 / tau;
		static constexpr double root2 = 1.4142135623730950488;
		static constexpr double root3 = 1.7320508075688772935274463415059;
		static constexpr double root5 = 2.23606797749978969641;
		static constexpr double root3_by_2 = root3 / 2.0; // 0.86602540378443864676
		static constexpr double inv_root2 = 1.0 / root2;
		static constexpr double inv_root3 = 1.0 / root3;
		static constexpr double golden_ratio = 1.618033988749894848204586834;    // "Golden Ratio" = (1 + root5) / 2
		static constexpr double golden_angle = 2.39996322972865332223;  // (rad) "Golden Angle" = pi*(3-root(5)) = the angle the divides the circumference of a circle with ratio equal to the golden ratio

		// Trig
		static constexpr double cos_30 = root3_by_2;
		static constexpr double cos_45 = inv_root2;
		static constexpr double cos_60 = 0.5;
		static constexpr double sin_30 = cos_60;
		static constexpr double sin_45 = cos_45;
		static constexpr double sin_60 = cos_30;

		// The maximum integer value that can be exactly represented by a float,double
		static constexpr int64_t max_representable_int = 9007199254740994LL; // 2^53
	};
	template <> struct constants<float>
	{
		static constexpr float tiny = 1.00000007e-05f;
		static constexpr float tiny_sq = 1.00000015e-10f;
		static constexpr float tiny_sqrt = 3.16227786e-03f;

		static constexpr float tau = 6.283185307179586476925286766559f;    // circle constant
		static constexpr float inv_tau = 1.0f / tau;
		static constexpr float tau_by_2 = tau / 2.0f;
		static constexpr float tau_by_3 = tau / 3.0f;
		static constexpr float tau_by_4 = tau / 4.0f;
		static constexpr float tau_by_5 = tau / 5.0f;
		static constexpr float tau_by_6 = tau / 6.0f;
		static constexpr float tau_by_7 = tau / 7.0f;
		static constexpr float tau_by_8 = tau / 8.0f;
		static constexpr float tau_by_360 = tau / 360.0f;
		static constexpr float E60_by_tau = 360.0f / tau;
		static constexpr float root2 = 1.4142135623730950488f;
		static constexpr float root3 = 1.7320508075688772935274463415059f;
		static constexpr float root5 = 2.23606797749978969641f;
		static constexpr float root3_by_2 = root3 / 2.0f;
		static constexpr float inv_root2 = 1.0f / root2;
		static constexpr float inv_root3 = 1.0f / root3;
		static constexpr float golden_ratio = 1.618033988749894848204586834f;  // "Golden Ratio"
		static constexpr float golden_angle = 2.39996322972865332223f; //(rad) "Golden Angle" = pi*(3-root(5)) = the angle the divides the circumference of a circle with ratio equal to the golden ratio

		// Trig
		static constexpr float cos_30 = root3_by_2;
		static constexpr float cos_45 = inv_root2;
		static constexpr float cos_60 = 0.5f;
		static constexpr float sin_30 = cos_60;
		static constexpr float sin_45 = cos_45;
		static constexpr float sin_60 = cos_30;

		// The maximum integer value that can be exactly represented by a float,double
		static constexpr int32_t max_representable_int = 16777216; // 2^24
	};
	template <> struct constants<int64_t>
	{
		static constexpr int64_t tiny = 0LL;
	};
	template <> struct constants<int32_t>
	{
		static constexpr int32_t tiny = 0;
	};

	// static constants
	template <typename T> constexpr std::decay_t<T> tiny;
	template <> constexpr double  tiny<double const>  = constants<double>::tiny;
	template <> constexpr float   tiny<float const>   = constants<float>::tiny;
	template <> constexpr int64_t tiny<int64_t const> = constants<int64_t>::tiny;
	template <> constexpr int32_t tiny<int32_t const> = constants<int32_t>::tiny;
	template <> constexpr double  tiny<double>        = constants<double>::tiny;
	template <> constexpr float   tiny<float>         = constants<float>::tiny;
	template <> constexpr int64_t tiny<int64_t>       = constants<int64_t>::tiny;
	template <> constexpr int32_t tiny<int32_t>       = constants<int32_t>::tiny;

	// The maximum integer value that can be exactly represented by a float,double
	inline constexpr int32_t max_int_in_float  = 16777216;           // 2^24
	inline constexpr int64_t max_int_in_double = 9007199254740994LL; // 2^53
	static_assert(static_cast<int32_t>(static_cast<float >(max_int_in_float     )) == max_int_in_float    );
	static_assert(static_cast<int32_t>(static_cast<float >(max_int_in_float  + 1)) != max_int_in_float + 1);
	static_assert(static_cast<int64_t>(static_cast<double>(max_int_in_double    )) == max_int_in_double    );
	static_assert(static_cast<int64_t>(static_cast<double>(max_int_in_double + 1)) != max_int_in_double + 1);
}