//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/terrain/water/wave_spectrum.h"

namespace pr::physics::terrain::water
{
	namespace
	{
		// Rotation between consecutive component directions. Successive multiples of the golden angle spread evenly around the circle.
		constexpr double GoldenAngle = 2.39996322972865332;

		// Full circle in radians.
		constexpr double Tau = math::constants<double>::tau;

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
			// Rotating each band by the golden angle keeps the directions of neighbouring bands apart.
			for (int d = 0; d != directions; ++d)
			{
				// Spread the band's directions evenly around the circle, and give each one its own wavelength within the band.
				auto const i = b * directions + d;
				auto const wavelength = layout.m_min_wavelength * std::pow(ratio, (b + (d + 0.5) / directions) / bands);
				auto const k = Tau / wavelength;
				auto const harmonic = std::max(std::round(std::sqrt(gravity * k) / base_frequency), 1.0);
				auto const angle = std::fmod(b * GoldenAngle + d * Tau / directions, Tau);
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
		if (!std::isfinite(weather.m_wind_direction))
			throw std::invalid_argument("Wind direction must be finite");
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

		// The share of the directional spread (2/pi)cos²(delta) between angles 'lo' and 'hi' from the wind. The spread integrates to one over
		// the half circle facing downwind and is zero upwind. The sector is also tested one turn either side because it may wrap around.
		auto const spread_share = [](double lo, double hi)
		{
			// The integral of the spread from zero to 'delta', for delta within a quarter turn of the wind.
			auto const integral = [](double delta)
			{
				auto const clamped = std::clamp(delta, -Tau / 4, Tau / 4);
				return (clamped + 0.5 * std::sin(2.0 * clamped)) / (Tau / 2);
			};
			auto share = 0.0;
			for (auto turn = -1; turn != 2; ++turn)
				share += integral(hi + turn * Tau) - integral(lo + turn * Tau);

			return share;
		};

		// Each band carries the variance of its whole frequency range, found by midpoint integration in log frequency. The spectrum peak can
		// be narrower than a band, so the integration samples each band finely rather than evaluating the density once per component.
		auto const bands = m_layout.m_bands;
		auto const directions = m_layout.m_directions;
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

			// Share the band's variance between its directions by the spread over each direction's sector. A sine wave of amplitude A has variance A²/2.
			for (int d = 0; d != directions; ++d)
			{
				auto const& component = m_components[b * directions + d];
				auto const centre = std::remainder(std::atan2(component.m_direction.y, component.m_direction.x) - weather.m_wind_direction, Tau);
				auto const half_width = 0.5 * Tau / directions;
				auto const variance = band_variance * spread_share(centre - half_width, centre + half_width);
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

	// Write Gerstner elements for the selected components.
	int WaveSpectrum::Elements(std::span<float const> amplitudes, float min_wavelength, std::span<WaterFieldElement> elements) const
	{
		if (std::ssize(amplitudes) != ComponentCount())
			throw std::invalid_argument(std::format("Wave spectrum amplitudes must contain {} values", ComponentCount()));
		if (!std::isfinite(min_wavelength))
			throw std::invalid_argument("Minimum wavelength must be finite");

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
			auto element = GerstnerWave(component.m_direction, amplitude, component.m_wavelength, phase_speed, 0.0f);
			element.position.z = component.m_phase;
			elements[count++] = element;
		}
		return count;
	}
}
