//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/terrain/surface_sample.h"

namespace pr::physics::terrain
{
	// Bounds the search distance, sample spacing, refinement interval, and total surface evaluations.
	struct RayCastOptions
	{
		double m_max_distance = 10000.0;
		double m_max_step = 1.0;
		double m_tolerance = 0.01;
		int m_max_samples = 32768;
	};

	// Describes the first detected entry into the solid below a height surface; a miss has m_hit false.
	struct RayCastResult
	{
		bool m_hit = false;
		double m_distance = 0.0;
		v4d m_position = v4d::Origin();
		SurfaceSample m_surface = {};
		int m_samples = 0;
	};

	// Search along a finite ray, normalizing its direction so distances are in surface units.
	// Surface must provide Sample(v2d). An origin on or below ground hits immediately. Detected above/below brackets
	// are refined to m_tolerance in ray distance, returning the below-ground endpoint. Narrow or grazing intersections
	// between samples can be missed: this is not a conservative collision query or a guaranteed earliest intersection.
	// Uses O(1) storage and at most m_max_samples evaluations; exhaustion and invalid surface values throw, not report a miss.
	template <typename Surface>
	RayCastResult RayCast(Surface const& surface, v4d origin, v4d direction, RayCastOptions const& options = {})
	{
		if (!IsFinite(origin) || origin.w != 1 || !IsFinite(direction) || direction.w != 0 ||
			!std::isfinite(options.m_max_distance) || options.m_max_distance < 0 ||
			!std::isfinite(options.m_max_step) || options.m_max_step <= 0 ||
			!std::isfinite(options.m_tolerance) || options.m_tolerance <= 0 ||
			options.m_max_samples < 1 || options.m_max_samples > 1'000'000)
			throw std::invalid_argument("Invalid terrain ray or search limits");

		auto const length = std::hypot(direction.x, direction.y, direction.z);
		if (!std::isfinite(length) || length == 0)
			throw std::invalid_argument("Terrain ray direction must have finite nonzero length");

		// Keep the work budget shared between coarse search and bracket refinement.
		direction /= length;
		if (!IsFinite(direction))
			throw std::invalid_argument("Terrain ray direction cannot be normalized at this precision");

		auto result = RayCastResult{};
		auto evaluate = [&](double distance)
		{
			if (result.m_samples == options.m_max_samples)
				throw std::runtime_error("Terrain raycast exhausted its surface evaluation budget");

			++result.m_samples;
			auto const position = origin + direction * distance;
			if (!IsFinite(position))
				throw std::out_of_range("Terrain ray position exceeds coordinate precision");

			auto sample = surface.Sample(v2d{position.x, position.y});
			if (!std::isfinite(sample.m_height) || !IsFinite(sample.m_gradient_xy))
				throw std::runtime_error("Terrain raycast received a non-finite surface sample");

			return sample;
		};
		auto lower = 0.0;
		auto upper = 0.0;
		auto sample = evaluate(upper);
		while (origin.z + direction.z * upper > sample.m_height)
		{
			if (upper == options.m_max_distance)
				return result;

			lower = upper;
			upper = std::min(upper + options.m_max_step, options.m_max_distance);
			if (upper == lower)
				throw std::runtime_error("Terrain raycast step is below coordinate precision");

			sample = evaluate(upper);
		}

		// Refine the first detected entry without implying that unobserved intersections were excluded.
		while (upper - lower > options.m_tolerance)
		{
			auto const middle = lower + (upper - lower) * 0.5;
			if (middle == lower || middle == upper)
				throw std::runtime_error("Terrain raycast tolerance is below coordinate precision");

			auto const midpoint_sample = evaluate(middle);
			if (origin.z + direction.z * middle > midpoint_sample.m_height)
				lower = middle;
			else
			{
				upper = middle;
				sample = midpoint_sample;
			}
		}
		result.m_hit = true;
		result.m_distance = upper;
		result.m_position = origin + direction * upper;
		result.m_surface = sample;
		return result;
	}
}
