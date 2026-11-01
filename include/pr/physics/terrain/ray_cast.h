//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/terrain/surface_sample.h"

namespace pr::physics::terrain
{
	// Bounds the search distance, horizontal sample spacing, refinement interval, and total surface evaluations.
	struct RayCastOptions
	{
		double m_max_distance = 10000.0;
		double m_max_horizontal_step = 1.0; // The largest horizontal distance between successive search samples
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
	// Surface must provide Sample(v2d). An origin on or below ground hits immediately. Search samples are at most
	// m_max_horizontal_step apart horizontally, so a steep ray needs few samples. Within that spacing, each search sample is placed
	// just past the crossing predicted from the previous sample's height and slope, so a locally planar surface is usually found with one search sample.
	// Detected above/below brackets are refined to m_tolerance in ray distance, returning the below-ground endpoint.
	// A vertical ray sees one surface height along its whole length, so it needs one sample and returns its exact crossing.
	// Surface features narrower than the horizontal step can be missed: this is not a conservative collision query or a
	// guaranteed earliest intersection. Uses O(1) storage and at most m_max_samples evaluations; exhaustion and invalid
	// surface values throw, not report a miss.
	template <typename Surface>
	RayCastResult RayCast(Surface const& surface, v4d origin, v4d direction, RayCastOptions const& options = {})
	{
		if (!IsFinite(origin) || origin.w != 1 || !IsFinite(direction) || direction.w != 0 ||
			!std::isfinite(options.m_max_distance) || options.m_max_distance < 0 ||
			!std::isfinite(options.m_max_horizontal_step) || options.m_max_horizontal_step <= 0 ||
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
		auto clearance = [&](double distance, SurfaceSample const& at)
		{
			// Height of the ray above the surface; positive means the ray point is above ground
			return origin.z + direction.z * distance - at.m_height;
		};

		// A height surface only varies horizontally, so the ray distance between search samples is the horizontal step
		// scaled by the ray's horizontal speed.
		auto const horizontal_speed = std::hypot(direction.x, direction.y);
		auto const step = horizontal_speed * options.m_max_distance > options.m_max_horizontal_step
			? options.m_max_horizontal_step / horizontal_speed
			: options.m_max_distance;

		auto lower = 0.0;
		auto upper = 0.0;
		auto sample = evaluate(upper);
		auto lower_clearance = clearance(upper, sample);
		auto upper_clearance = lower_clearance;

		// A vertical ray stays over the origin's surface point, so the origin's clearance gives the exact crossing
		if (horizontal_speed == 0 && upper_clearance > 0)
		{
			// A ray that rises, or falls less than the clearance, misses
			if (direction.z >= 0 || upper_clearance > -direction.z * options.m_max_distance)
				return result;

			auto const crossing = upper_clearance / -direction.z;
			result.m_hit = true;
			result.m_distance = crossing;
			result.m_position = origin + direction * crossing;
			result.m_surface = sample;
			return result;
		}

		// March until a sample is below ground. The surface's tangent plane at the last sample predicts where the ray meets it. Sampling slightly
		// past that point, but never further than the step, usually brackets the crossing within a tolerance without weakening the step's spacing.
		while (upper_clearance > 0)
		{
			if (upper == options.m_max_distance)
				return result;

			// The clearance changes by the ray's rise less the surface's rise beneath it, per unit of ray distance
			auto next = std::min(upper + step, options.m_max_distance);
			auto const descent = direction.z - (sample.m_gradient_xy.x * direction.x + sample.m_gradient_xy.y * direction.y);
			if (descent < 0)
				next = std::min(next, upper + upper_clearance / -descent + options.m_tolerance * 0.4);

			lower = upper;
			lower_clearance = upper_clearance;
			upper = next;
			if (upper == lower)
				throw std::runtime_error("Terrain raycast step is below coordinate precision");

			sample = evaluate(upper);
			upper_clearance = clearance(upper, sample);
		}

		// Refine the detected entry without implying that unobserved intersections were excluded. The secant estimate is
		// exact for a planar surface, and keeping it at least half a tolerance inside the bracket lets the next estimate
		// close the bracket from the other side. An estimate within half a tolerance of one end probes nearly a tolerance from
		// that end instead, so that the crossing is likely between the probe and that end, which closes the bracket at once.
		// Bisect after two estimates in a row fail to halve the bracket, which bounds the cost on badly curved surfaces to a
		// small multiple of plain bisection.
		auto slow_estimates = 0;
		while (upper - lower > options.m_tolerance)
		{
			auto const width = upper - lower;
			auto middle = lower + width * 0.5;
			if (slow_estimates < 2)
			{
				auto const margin = options.m_tolerance * 0.5;
				auto const closing = options.m_tolerance * 0.9;
				auto const secant = lower + width * lower_clearance / (lower_clearance - upper_clearance);
				middle =
					upper - secant < margin ? upper - closing :
					secant - lower < margin ? lower + closing :
					std::clamp(secant, lower + margin, upper - margin);
			}
			if (middle <= lower || middle >= upper)
				throw std::runtime_error("Terrain raycast tolerance is below coordinate precision");

			auto const middle_sample = evaluate(middle);
			auto const middle_clearance = clearance(middle, middle_sample);
			if (middle_clearance > 0)
			{
				lower = middle;
				lower_clearance = middle_clearance;
			}
			else
			{
				upper = middle;
				upper_clearance = middle_clearance;
				sample = middle_sample;
			}
			slow_estimates = upper - lower > width * 0.5 ? slow_estimates + 1 : 0;
		}
		result.m_hit = true;
		result.m_distance = upper;
		result.m_position = origin + direction * upper;
		result.m_surface = sample;
		return result;
	}
}
