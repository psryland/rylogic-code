//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/terrain/forward.h"
#include "pr/physics/terrain/water/water_field_types.hlsli"

namespace pr::physics::terrain::water
{
	using WaterFieldElement = shared::WaterFieldElement;
	inline static int constexpr MaxElementCount = shared::WaterFieldMaxElementCount;

	// Water-surface height and slope at one XY position.
	struct WaterSample
	{
		double m_height;
		v2d m_gradient_xy;
	};

	// Return a sine wave element with height A*sin(k*dot(d, xy) + omega*t). 'direction' need not be unit length.
	WaterFieldElement SineWave(v2 direction, float amplitude, float wavelength, float angular_frequency);

	// Return a Gerstner wave element with height A*sin(k*dot(d, xy) - k*c*t). 'steepness' in [0,1] only affects rendered horizontal displacement.
	WaterFieldElement GerstnerWave(v2 direction, float amplitude, float wavelength, float phase_speed, float steepness);

	// Return a finite radial packet travelling outward from 'source'. See water_field_types.hlsli for the parameter meaning.
	WaterFieldElement RadialPacket(v2 source, float amplitude, float wavelength, float half_width, float propagation_speed, float age, float lifetime, float attack_time, float attenuation_scale);

	// A water surface defined as a still-water level plus a bounded set of fixed-stride elements.
	// The surface is a height field over world XY with +Z up. Sampling returns the water surface height even where terrain lies above it;
	// callers decide wetness by comparing against their own terrain or body geometry.
	// The same elements are evaluated by the stage-neutral HLSL in water_field.hlsli, so CPU and GPU consumers see the same surface.
	class WaterField
	{
		double m_level;
		std::vector<WaterFieldElement> m_elements;
		double m_amplitude_bound;

	public:

		// Construct a validated water field. Throws invalid_argument for non-finite values, unknown element types,
		// non-unit wave directions, non-positive wavelengths, or more than MaxElementCount elements.
		explicit WaterField(double level, std::span<WaterFieldElement const> elements = {});

		// Construct still water at level zero.
		WaterField();

		// The still-water level in metres.
		double Level() const noexcept;
		void Level(double level);

		// The active elements, in upload order.
		std::span<WaterFieldElement const> Elements() const noexcept;
		void Elements(std::span<WaterFieldElement const> elements);

		// True when no element can change the height, so the surface is the still-water level everywhere.
		bool IsFlat() const noexcept;

		// Conservative bounds on the surface height over all positions and times.
		double MaxHeight() const noexcept;
		double MinHeight() const noexcept;

		// Sample the surface height at a world-space XY position and simulation time.
		double Height(v2d xy, double time_s) const;

		// Sample the surface height and slope at a world-space XY position and simulation time.
		WaterSample Sample(v2d xy, double time_s) const;

		// Sample the dimensionless lateral pressure gradient used by buoyancy forces. Zero for non-positive gravity.
		v2 PressureGradient(v2 xy, float time_s, float gravity) const;

		// Sample the world-space water particle velocity (w = 0) at a world-space position.
		v4 Velocity(v4 pos_ws, float time_s) const;
	};
}
