//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/terrain/water/wave_spectrum.h"

namespace pr::physics::terrain::water
{
	namespace
	{
		// Fractional part of the golden ratio. Successive multiples of it spread evenly over [0,1), so successive bands get different offsets.
		constexpr double GoldenFraction = 0.61803398874989485;

		// Full circle in radians.
		constexpr double Tau = math::constants<double>::tau;

		// Directional spreading exponent at the spectrum peak. Larger values concentrate the wave energy closer to the wind direction.
		constexpr double PeakSpreading = 12.0;

		// Return the angle from the wind that divides a cos² spread over ±90° so that a fraction 'u' (0 < u < 1) of it lies below the angle.
		double SpreadQuantile(double u)
		{
			// The spread's integral F(delta) = 0.5 + (delta + 0.5*sin(2*delta))/pi is monotonic, so bisection always converges.
			auto lo = -Tau / 4;
			auto hi = +Tau / 4;
			for (int i = 0; i != 60; ++i)
			{
				// Keep the half of the interval that contains the target fraction.
				auto const mid = 0.5 * (lo + hi);
				auto const f = 0.5 + (mid + 0.5 * std::sin(2.0 * mid)) / (Tau / 2);
				if (f < u)
					lo = mid;
				else
					hi = mid;
			}
			return 0.5 * (lo + hi);
		}

		// Return a well-mixed 32-bit hash of 'value'.
		uint32_t Hash(uint32_t value)
		{
			// Integer finaliser from MurmurHash3; every input bit affects every output bit.
			value ^= value >> 16;
			value *= 0x85EBCA6Bu;
			value ^= value >> 13;
			value *= 0xC2B2AE35u;
			value ^= value >> 16;
			return value;
		}
	}

	// Build the fixed component table.
	WaveSpectrum::WaveSpectrum(WaveSpectrumLayout const& layout)
		: m_layout(layout)
		, m_components()
	{
		if (!std::isfinite(layout.m_gravity) || !(layout.m_gravity > 0.0f))
			throw std::invalid_argument("Wave spectrum gravity must be finite and positive");
		if (!std::isfinite(layout.m_repeat_period) || !(layout.m_repeat_period > 0.0f))
			throw std::invalid_argument("Wave spectrum repeat period must be finite and positive");
		if (!std::isfinite(layout.m_min_wavelength) || !std::isfinite(layout.m_max_wavelength) || !(layout.m_min_wavelength > 0.0f) || !(layout.m_max_wavelength > layout.m_min_wavelength))
			throw std::invalid_argument("Wave spectrum wavelengths must be finite with 0 < min < max");
		if (layout.m_bands < 1 || layout.m_directions < 1 || static_cast<int64_t>(layout.m_bands) * layout.m_directions > MaxElementCount)
			throw std::invalid_argument(std::format("Wave spectrum bands and directions must be positive with at most {} components", MaxElementCount));

		// Deep-water dispersion gives omega = sqrt(g*k). Rounding omega to a whole multiple of the base frequency makes the surface periodic;
		// the change in speed is under one percent for every component in the supported range.
		auto const gravity = static_cast<double>(layout.m_gravity);
		auto const base_frequency = Tau / static_cast<double>(layout.m_repeat_period);
		auto const ratio = static_cast<double>(layout.m_max_wavelength) / layout.m_min_wavelength;
		auto const bands = layout.m_bands;
		auto const directions = layout.m_directions;
		m_components.resize(static_cast<size_t>(bands) * directions);
		for (int b = 0; b != bands; ++b)
		{
			// Each band places its directions at equal-share points of a cos² spread about downwind, so most directions lie near the wind.
			// The share points are jittered by a different amount in each band so that neighbouring bands do not travel in the same directions.
			auto const jitter = 0.8 * (std::fmod(b * GoldenFraction, 1.0) - 0.5);
			for (int d = 0; d != directions; ++d)
			{
				// Give each direction its own wavelength within the band.
				auto const i = b * directions + d;
				auto const wavelength = layout.m_min_wavelength * std::pow(ratio, (b + (d + 0.5) / directions) / bands);
				auto const k = Tau / wavelength;
				auto const harmonic = std::max(std::round(std::sqrt(gravity * k) / base_frequency), 1.0);
				auto const angle = SpreadQuantile((d + 0.5 + jitter) / directions);
				m_components[i] = WaveComponent{
					.m_direction = v2{static_cast<float>(std::cos(angle)), static_cast<float>(std::sin(angle))},
					.m_wavelength = static_cast<float>(wavelength),
					.m_angular_frequency = static_cast<float>(harmonic * base_frequency),
					.m_phase = static_cast<float>(Tau * (Hash(static_cast<uint32_t>(i) + 0x9E3779B9u) / 4294967296.0)),
				};
			}
		}
	}

	// The layout used to build the components.
	WaveSpectrumLayout const& WaveSpectrum::Layout() const noexcept
	{
		return m_layout;
	}

	// The number of components.
	int WaveSpectrum::ComponentCount() const noexcept
	{
		return static_cast<int>(m_components.size());
	}

	// The fixed components, band by band from the shortest to the longest wavelength.
	std::span<WaveComponent const> WaveSpectrum::Components() const noexcept
	{
		return m_components;
	}

	// Write the equilibrium amplitude of every component for 'weather'.
	void WaveSpectrum::Targets(WaveWeather const& weather, std::span<float> amplitudes) const
	{
		if (std::ssize(amplitudes) != ComponentCount())
			throw std::invalid_argument(std::format("Wave spectrum amplitudes must contain {} values", ComponentCount()));
		if (!std::isfinite(weather.m_wind_speed) || weather.m_wind_speed < 0.0f)
			throw std::invalid_argument("Wind speed must be finite and non-negative");
		if (!std::isfinite(weather.m_fetch) || !(weather.m_fetch > 0.0f))
			throw std::invalid_argument("Wind fetch must be finite and positive");

		// Calm air raises no waves.
		auto const wind = static_cast<double>(weather.m_wind_speed);
		if (wind < 0.1)
		{
			std::fill(amplitudes.begin(), amplitudes.end(), 0.0f);
			return;
		}

		// Fetch-limited wind-sea (JONSWAP) parameters from the dimensionless fetch X = g*F/U². Short fetches give a higher, sharper peak frequency.
		// Long fetches approach the fully developed sea, which bounds both the peak frequency and the energy scale from below.
		auto const g = static_cast<double>(m_layout.m_gravity);
		auto const fetch = g * weather.m_fetch / (wind * wind);
		auto const peak = std::max(22.0 * (g / wind) * std::pow(fetch, -1.0 / 3.0), 0.855 * g / wind);
		auto const alpha = std::max(0.076 * std::pow(fetch, -0.22), 0.0081);
		auto const gamma = 3.3;
		auto const density = [=](double omega)
		{
			// Spectral density S(omega) in m²·s.
			auto const sigma = omega <= peak ? 0.07 : 0.09;
			auto const r = std::exp(-(omega - peak) * (omega - peak) / (2.0 * sigma * sigma * peak * peak));
			return alpha * g * g * std::pow(omega, -5.0) * std::exp(-1.25 * std::pow(peak / omega, 4.0)) * std::pow(gamma, r);
		};

		// The directional spread at frequency 'omega' is proportional to cos(delta/2)^(2s) for angles 'delta' from downwind (the Mitsuyasu form).
		// The exponent 's' is largest at the spectrum peak, so waves near the peak are the most closely aligned with the wind; longer swell and
		// shorter ripples spread more widely. The spread is integrated numerically over the full circle, and each direction takes the share
		// between the midpoints to its neighbours. The outermost directions take everything beyond them, so no energy is lost.
		auto const directions = m_layout.m_directions;
		constexpr int SpreadSamples = 720;
		std::vector<double> shares(directions);
		auto const spread_shares = [&](int band, double omega)
		{
			// Accumulate the spread into the sector of each direction. Directions are sorted by angle within a band and the samples run in
			// increasing angle, so the sector index only ever moves forward.
			auto const exponent = 2.0 * PeakSpreading * (omega <= peak ? std::pow(omega / peak, 5.0) : std::pow(omega / peak, -2.5));
			auto const angle = [&](int d)
			{
				auto const& direction = m_components[band * directions + d].m_direction;
				return std::atan2(static_cast<double>(direction.y), static_cast<double>(direction.x));
			};
			std::fill(shares.begin(), shares.end(), 0.0);
			auto total = 0.0;
			auto d = 0;
			auto boundary = directions > 1 ? 0.5 * (angle(0) + angle(1)) : Tau;
			for (int s = 0; s != SpreadSamples; ++s)
			{
				// Move to the next sector once the sample passes the midpoint between neighbouring directions.
				auto const delta = -Tau / 2 + (s + 0.5) * Tau / SpreadSamples;
				while (delta >= boundary)
				{
					++d;
					boundary = d + 1 != directions ? 0.5 * (angle(d) + angle(d + 1)) : Tau;
				}

				auto const weight = std::pow(std::cos(0.5 * delta), exponent);
				shares[d] += weight;
				total += weight;
			}

			// Normalise so the band's variance is shared out exactly.
			for (auto& share : shares)
				share /= total;
		};

		// Each band carries the variance of its whole frequency range, found by midpoint integration in log frequency. The spectrum peak can
		// be narrower than a band, so the integration samples each band finely rather than evaluating the density once per component.
		auto const bands = m_layout.m_bands;
		auto const ratio = static_cast<double>(m_layout.m_max_wavelength) / m_layout.m_min_wavelength;
		constexpr int SamplesPerBand = 32;
		for (int b = 0; b != bands; ++b)
		{
			// Longer wavelengths have lower frequencies, so the band's frequency range runs from its long end to its short end.
			auto const omega_lo = std::sqrt(g * Tau / (m_layout.m_min_wavelength * std::pow(ratio, (b + 1.0) / bands)));
			auto const omega_hi = std::sqrt(g * Tau / (m_layout.m_min_wavelength * std::pow(ratio, static_cast<double>(b) / bands)));
			auto const step = std::log(omega_hi / omega_lo) / SamplesPerBand;
			auto band_variance = 0.0;
			for (int s = 0; s != SamplesPerBand; ++s)
			{
				// d(omega) = omega * d(ln omega).
				auto const omega = omega_lo * std::exp((s + 0.5) * step);
				band_variance += density(omega) * omega * step;
			}

			// Share the band's variance between its directions. A sine wave of amplitude A has variance A²/2.
			spread_shares(b, std::sqrt(omega_lo * omega_hi));
			for (int d = 0; d != directions; ++d)
			{
				auto const variance = band_variance * shares[d];
				amplitudes[b * directions + d] = static_cast<float>(std::sqrt(2.0 * std::max(variance, 0.0)));
			}
		}
	}
	// Move 'amplitudes' towards 'targets'.
	void WaveSpectrum::Relax(std::span<float> amplitudes, std::span<float const> targets, float dt, float time_constant)
	{
		if (amplitudes.size() != targets.size())
			throw std::invalid_argument("Wave amplitudes and targets must have the same length");
		if (!std::isfinite(dt) || dt < 0.0f)
			throw std::invalid_argument("Relaxation time step must be finite and non-negative");
		if (!std::isfinite(time_constant) || time_constant < 0.0f)
			throw std::invalid_argument("Relaxation time constant must be finite and non-negative");

		// An exact exponential step is stable for any time step, and a zero time constant jumps straight to the targets.
		auto const blend = time_constant > 0.0f ? 1.0f - std::exp(-dt / time_constant) : 1.0f;
		for (size_t i = 0; i != amplitudes.size(); ++i)
			amplitudes[i] += (targets[i] - amplitudes[i]) * blend;
	}

	// Return the crest sharpness of wind-driven waves.
	float WaveSpectrum::CrestSharpness(float wind_speed)
	{
		if (!std::isfinite(wind_speed) || wind_speed < 0.0f)
			throw std::invalid_argument("Wind speed must be finite and non-negative");

		// Sharpness starts at the calm-sea threshold and approaches its limit with an 8 m/s scale, so moderate winds already add visible chop.
		constexpr auto calm_wind_speed = 4.0f;
		constexpr auto wind_speed_scale = 8.0f;
		if (wind_speed <= calm_wind_speed)
			return 0.0f;

		return MaxCrestSharpness * (1.0f - std::exp(-(wind_speed - calm_wind_speed) / wind_speed_scale));
	}

	// Write Gerstner elements for the selected components.
	int WaveSpectrum::Elements(std::span<float const> amplitudes, float heading, float sharpness, float min_wavelength, std::span<WaterFieldElement> elements) const
	{
		if (std::ssize(amplitudes) != ComponentCount())
			throw std::invalid_argument(std::format("Wave spectrum amplitudes must contain {} values", ComponentCount()));
		if (!std::isfinite(heading))
			throw std::invalid_argument("Wave heading must be finite");
		if (!(sharpness >= 0.0f && sharpness <= shared::WaterFieldMaxCrestSharpness))
			throw std::invalid_argument("Crest sharpness is out of range");
		if (!std::isfinite(min_wavelength))
			throw std::invalid_argument("Minimum wavelength must be finite");

		// Component directions are relative to downwind, so rotate them by the heading.
		auto const cos_heading = std::cos(heading);
		auto const sin_heading = std::sin(heading);

		// Skip components that cannot contribute so consumers loop over as few elements as possible.
		auto count = 0;
		for (int i = 0; i != ComponentCount(); ++i)
		{
			auto const& component = m_components[i];
			auto const amplitude = amplitudes[i];
			if (!std::isfinite(amplitude) || amplitude < 0.0f)
				throw std::invalid_argument("Wave amplitudes must be finite and non-negative");
			if (amplitude == 0.0f || component.m_wavelength < min_wavelength)
				continue;
			if (count == std::ssize(elements))
				throw std::invalid_argument("Wave element output is too small");

			// The phase speed is omega/k so the element's time term is exactly the quantised frequency.
			auto const phase_speed = component.m_angular_frequency * component.m_wavelength / static_cast<float>(Tau);
			auto const direction = v2{
				cos_heading * component.m_direction.x - sin_heading * component.m_direction.y,
				sin_heading * component.m_direction.x + cos_heading * component.m_direction.y,
			};
			auto element = GerstnerWave(direction, amplitude, component.m_wavelength, phase_speed, 0.0f, sharpness);
			element.position.z = component.m_phase;
			elements[count++] = element;
		}
		return count;
	}
}
