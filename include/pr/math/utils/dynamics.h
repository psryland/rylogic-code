//*****************************************************************************
// Maths library
//  Copyright (c) Rylogic Ltd 2002
//*****************************************************************************
#pragma once
#include "pr/math/core/forward.h"
#include "pr/math/types/vector4.h"
#include "pr/math/types/quaternion.h"

namespace pr::math
{
	// From a sample of 5 consecutive transforms (spaced by dt), returns the orientation, angular velocity, and angular acceleration
	// Note: Ensure the quaternions within 'samples' are all the shortest arc
	template <ScalarTypeFP S>
	inline std::tuple<Quat<S>, Vec4<S>, Vec4<S>> CalculateRotationalDynamics(std::span<Xform<S> const, 5> samples, S dt) noexcept
	{ 
		// Indices into 'samples'
		enum : int32_t { p2 = 0, p1 = 1, c0 = 2, n1 = 3, n2 = 4, count };
		pr_assert(ssize(samples) >= count && "'samples' must be at least length >= 5");

		auto ori_p2 = samples[p2].rot;
		auto ori_p1 = samples[p1].rot;
		auto ori_c0 = samples[c0].rot;
		auto ori_n1 = samples[n1].rot;
		auto ori_n2 = samples[n2].rot;

		auto ori_p2c0 = ori_c0 * ~ori_p2; // Change in orientation from p2 to c0
		auto ori_p1n1 = ori_n1 * ~ori_p1; // Change in orientation from p1 to n1
		auto ori_c0n2 = ori_n2 * ~ori_c0; // Change in orientation from c0 to n2

		auto avel_p1 = LogMap<Vec4<S>>(ori_p2c0) / dt; // Avl at p1
		auto avel_n1 = LogMap<Vec4<S>>(ori_c0n2) / dt; // Avl at n1

		auto value = ori_c0;
		
		// Angular velocity
		auto avel_c0 = LogMap<Vec4<S>>(ori_p1n1) / dt; // (N1 - P1) / 2dT = angular velocity at C0. Remember log space uses Angle/2 for length
		auto velocity = avel_c0;

		// Angular acceleration (central difference of angular velocities)
		auto aacc_c0 = (avel_n1 - avel_p1) / (2*dt); // Angular acceleration from central difference of angular velocities
		auto acceleration = aacc_c0;

		return {value, velocity, acceleration};
	}

	// From a sample of 5 consecutive transforms (spaced by dt), returns the position, linear velocity, and linear acceleration
	template <ScalarTypeFP S>
	inline std::tuple<Vec4<S>, Vec4<S>, Vec4<S>> CalculateTranslationalDynamics(std::span<Xform<S> const, 5> samples, S dt) noexcept
	{
		// Indices into 'samples'
		enum : int32_t { p2 = 0, p1 = 1, c0 = 2, n1 = 3, n2 = 4, count };
		pr_assert(ssize(samples) >= count && "'samples' must be at least length >= 5");

		auto pos_p2 = samples[p2].pos;
		auto pos_p1 = samples[p1].pos;
		auto pos_c0 = samples[c0].pos;
		auto pos_n1 = samples[n1].pos;
		auto pos_n2 = samples[n2].pos;

		auto vel_p1 = (pos_c0 - pos_p2) / (2*dt);
		auto vel_c0 = (pos_n1 - pos_p1) / (2*dt);
		auto vel_n1 = (pos_n2 - pos_c0) / (2*dt);

		auto acc_p = (vel_c0 - vel_p1) / (dt);
		auto acc_n = (vel_n1 - vel_c0) / (dt);

		auto value = pos_c0;
	
		// Velocity (central difference)
		auto velocity = S(0.25) * (vel_p1 + S(2)*vel_c0 + vel_n1);

		// Linear acceleration (central difference of velocities)
		auto acceleration = S(0.5) * (acc_p + acc_n);

		return {value, velocity, acceleration};
	}

	// From a sample of 5 consecutive transforms (spaced by dt), returns the position, linear velocity, and linear acceleration
	template <ScalarTypeFP S>
	inline std::tuple<Vec4<S>, Vec4<S>, Vec4<S>> CalculateScaleDynamics(std::span<Xform<S> const, 5> samples, S dt) noexcept
	{
		// Indices into 'samples'
		enum : int32_t { p2 = 0, p1 = 1, c0 = 2, n1 = 3, n2 = 4, count };
		pr_assert(ssize(samples) >= count && "'samples' must be at least length >= 5");

		auto scale_c0 = samples[c0].scl;
		auto value = scale_c0;

		// velocity of scale
		auto scale_p1 = samples[p1].scl;
		auto scale_n1 = samples[n1].scl;
		auto scalevel_c0 = (scale_n1 - scale_p1) / (2*dt); // central difference of scale
		auto velocity = scalevel_c0;

		// acceleration of scale (central difference of scale velocities)
		auto scale_p2 = samples[p2].scl;
		auto scale_n2 = samples[n2].scl;
		auto scalevel_p1 = (scale_c0 - scale_p2) / (2*dt);
		auto scalevel_n1 = (scale_n2 - scale_c0) / (2*dt);
		auto scaleacc_c0 = (scalevel_n1 - scalevel_p1) / (2*dt); // central difference of scale velocity
		auto acceleration = scaleacc_c0;

		return {value, velocity, acceleration};
	}

	// From a sample of 5 consecutive scalar values (spaced by dt), returns the value, dvalue/dt, d�value/dt�
	template <ScalarTypeFP S>
	inline std::tuple<S, S, S> CalculateScalarDynamics(std::span<S const> samples, S dt) noexcept
	{
		// Indices into 'samples'
		enum : int32_t { p2 = 0, p1 = 1, c0 = 2, n1 = 3, n2 = 4, count };
		pr_assert(ssize(samples) >= count && "'samples' must be at least length >= 5");

		// Value
		auto value = samples[c0];

		// dValue/dt (central difference)
		auto vel_c0 = (samples[n1] - samples[p1]) / (2*dt); // Central difference of position
		auto velocity = vel_c0;

		// d�Value/dt� (central difference of velocities)
		auto vel_p1 = (samples[c0] - samples[p2]) / (2*dt);
		auto vel_n1 = (samples[n2] - samples[c0]) / (2*dt);
		auto acc_c0 = (vel_n1 - vel_p1) / (2*dt); // central difference of velocity
		auto acceleration = acc_c0;

		return { value, velocity, acceleration };
	}
}
