//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/terrain/forward.h"
#include "pr/physics/terrain/water/water_field.h"

namespace pr::physics::terrain::water
{
	// Wind conditions that drive a wave spectrum. The wind direction is not needed because spectrum directions are relative to downwind.
	struct WaveWeather
	{
		// Wind speed at 10 m above the water, in m/s. Speeds below 0.1 m/s produce no waves.
		float m_wind_speed;

		// Distance over which the wind has blown across open water, in metres. Longer fetches produce longer, higher waves.
		float m_fetch;
	};

	// The fixed layout of a wave spectrum's components.
	struct WaveSpectrumLayout
	{
		// Gravity in m/s², positive. It sets each component's speed.
		float m_gravity = 9.81f;

		// Every component repeats exactly after this many seconds, positive.
		float m_repeat_period = 1024.0f;

		// The range of wavelengths the components cover, in metres (0 < min < max). The spectrum energy outside the range is not represented.
		float m_min_wavelength = 0.5f;
		float m_max_wavelength = 256.0f;

		// The wavelength range is split into 'm_bands' equal log-wavelength bands, each with 'm_directions' components (both positive).
		// The component count, m_bands * m_directions, must not exceed MaxElementCount.
		int m_bands = 16;
		int m_directions = 4;
	};

	// One fixed wave component of a spectrum.
	struct WaveComponent
	{
		v2 m_direction;            // Unit travel direction in the XY plane, relative to downwind (+X).
		float m_wavelength;        // Metres.
		float m_angular_frequency; // rad/s, a whole multiple of 2*pi/repeat period.
		float m_phase;             // Constant phase offset in radians.
	};

	// A fixed set of wave components whose amplitudes follow the weather.
	// The components never change, so changing the wind strength only changes amplitudes and never makes the surface jump. Component
	// directions are relative to downwind; callers rotate them to the actual wind direction (the heading) when building elements.
	// Each band places its directions at equal-share points of a cos² spread within ±90° of downwind, so most components travel roughly
	// with the wind. The share points are offset differently in each band, so neighbouring bands do not travel in the same directions.
	// Within a band each direction has a different wavelength, and phases come from a fixed hash, so the surface has no obvious repeating
	// pattern. Each component carries the energy of its whole band and direction sector, so the total energy does not depend on where the
	// spectrum peak falls between components.
	// Every angular frequency is a whole multiple of 2*pi/repeat_period, so the surface repeats exactly after the repeat period and callers
	// can wrap a long simulation clock into [0, repeat_period) without a visible jump (see WaterField::LocalTime).
	class WaveSpectrum
	{
		WaveSpectrumLayout m_layout;
		std::vector<WaveComponent> m_components;

	public:

		// Build the component table for 'layout'.
		explicit WaveSpectrum(WaveSpectrumLayout const& layout = {});

		// The layout used to build the components.
		WaveSpectrumLayout const& Layout() const noexcept;

		// The number of components, and the length of every amplitude array.
		int ComponentCount() const noexcept;

		// The fixed components, band by band from the shortest to the longest wavelength.
		std::span<WaveComponent const> Components() const noexcept;

		// Write the equilibrium amplitude (m) of every component for 'weather' into 'amplitudes' (ComponentCount values).
		// Amplitudes follow a fetch-limited wind-sea spectrum. The energy is spread about downwind most narrowly at the spectrum peak and more
		// widely for longer and shorter waves, so the dominant waves have a clear direction. Above 8 m/s every band is held to a minimum
		// alignment that grows with the wind, so a gale has a clear dominant direction even when its peak is longer than every band.
		void Targets(WaveWeather const& weather, std::span<float> amplitudes) const;

		// Move 'amplitudes' towards 'targets' over 'dt' seconds with an exponential time constant of 'time_constant' seconds.
		static void Relax(std::span<float> amplitudes, std::span<float const> targets, float dt, float time_constant);

		// Return the crest sharpness of wind-driven waves for 'wind_speed' (m/s, finite and non-negative). See shared::WaterFieldWaveProfile.
		// Light winds (up to 4 m/s) make sine-shaped waves. Stronger winds make choppier seas with narrower crests and flatter troughs; the
		// sharpness approaches MaxCrestSharpness in a storm.
		static float CrestSharpness(float wind_speed);

		// Largest sharpness returned by CrestSharpness.
		static constexpr float MaxCrestSharpness = 0.6f;

		// Write Gerstner elements for the components with a positive amplitude and a wavelength of at least 'min_wavelength'.
		// 'heading' is the direction the wind blows towards, in radians anticlockwise from +X; component directions are rotated by it.
		// 'sharpness' is the crest sharpness of every element, in [0, shared::WaterFieldMaxCrestSharpness] (see CrestSharpness).
		// Steepness is zero; renderers choose their own. Returns the number of elements written, at most ComponentCount.
		int Elements(std::span<float const> amplitudes, float heading, float sharpness, float min_wavelength, std::span<WaterFieldElement> elements) const;
	};
}
