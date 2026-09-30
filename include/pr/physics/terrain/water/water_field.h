//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/terrain/forward.h"
#include "pr/physics/terrain/water/water_field_types.hlsli"
#include "pr/physics/terrain/water/bathymetry.h"

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

	// Return a sine wave element with height A*sin(k*dot(d, xy) + omega*t + phase). 'direction' need not be unit length.
	WaterFieldElement SineWave(v2 direction, float amplitude, float wavelength, float angular_frequency, float phase = 0.0f);

	// Return a Gerstner wave element with height A*sin(k*dot(d, xy) - k*c*t + phase). 'steepness' in [0,1] only affects rendered horizontal displacement.
	WaterFieldElement GerstnerWave(v2 direction, float amplitude, float wavelength, float phase_speed, float steepness, float phase = 0.0f);

	// Return a finite radial packet travelling outward from 'source'. See water_field_types.hlsli for the parameter meaning.
	WaterFieldElement RadialPacket(v2 source, float amplitude, float wavelength, float half_width, float propagation_speed, float age, float lifetime, float attack_time, float attenuation_scale);

	// A water surface defined as a still-water level plus a bounded set of fixed-stride elements.
	// The surface is a height field over world XY with +Z up. Sampling returns the water surface height even where terrain lies above it;
	// callers decide wetness by comparing against their own terrain or body geometry.
	// The same elements are evaluated by the stage-neutral HLSL in water_field.hlsli, so CPU and GPU consumers see the same surface.
	// With a bathymetry, element amplitudes are corrected for the local water depth (see water_depth.hlsli): waves grow in shallow water,
	// are limited to the breaking height, run a little way up the shore, and vanish on dry land beyond the swash allowance.
	// Times are simulation seconds. With a repeat period, times are wrapped into [0, period) before evaluation (see LocalTime).
	class WaterField
	{
		double m_level;
		std::vector<WaterFieldElement> m_elements;
		double m_amplitude_bound;
		double m_swash_depth;
		std::shared_ptr<Bathymetry const> m_bathymetry;
		float m_breaking_ratio;
		double m_repeat_period;

	public:

		// Default ratio of breaking wave height to water depth.
		static constexpr float DefaultBreakingRatio = 0.78f;

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

		// The terrain heights used for depth correction, or null for deep water everywhere.
		std::shared_ptr<Bathymetry const> const& TerrainHeights() const noexcept;
		void TerrainHeights(std::shared_ptr<Bathymetry const> bathymetry);

		// Ratio of the largest wave height (crest to trough) to the water depth. Only used with a bathymetry.
		float BreakingRatio() const noexcept;
		void BreakingRatio(float ratio);

		// The period after which every element repeats exactly, or zero when elements are not periodic.
		// Callers own the periodicity: every element's time term must repeat after this period.
		double RepeatPeriod() const noexcept;
		void RepeatPeriod(double period);

		// Wrap a simulation time into [0, RepeatPeriod) so it keeps full float precision. Returns the time unchanged without a repeat period.
		float LocalTime(double time_s) const noexcept;

		// The water depth used for wave corrections at a world-space position: the still-water level minus the terrain height, plus the
		// swash allowance of the current elements (see shared::WaterFieldSwashDepth). shared::WaterFieldDeepWater without a bathymetry.
		float WaveDepth(v2d xy) const;

		// True when no element can change the height, so the surface is the still-water level everywhere.
		bool IsFlat() const noexcept;

		// Conservative bounds on the surface height over all positions and times.
		double MaxHeight() const noexcept;
		double MinHeight() const noexcept;

		// Conservative bounds on the surface height over all times and all positions in the world-space rectangle [lo, hi].
		// With a bathymetry the bound shrinks with the depth; it is the still-water level where the terrain is above the reach of the swash.
		double MaxHeight(v2d lo, v2d hi) const;
		double MinHeight(v2d lo, v2d hi) const;

		// Sample the surface height at a world-space XY position and simulation time.
		double Height(v2d xy, double time_s) const;

		// Sample the surface height and slope at a world-space XY position and simulation time.
		WaterSample Sample(v2d xy, double time_s) const;

		// Sample the dimensionless lateral pressure gradient used by buoyancy forces. Zero for non-positive gravity.
		v2 PressureGradient(v2 xy, double time_s, float gravity) const;

		// Sample the world-space water particle velocity (w = 0) at a world-space position.
		v4 Velocity(v4 pos_ws, double time_s) const;

	private:

		// Return the amplitude bound near a point whose deepest water, including the swash allowance, is 'max_depth'.
		double AmplitudeBound(double max_depth) const noexcept;

		// Return the breaking scale for 'depth'. See water_depth.hlsli.
		float BreakingScale(float depth) const;
	};
}
