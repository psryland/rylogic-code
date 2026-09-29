//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/terrain/water/water_field.h"
#include "pr/physics/terrain/water/water_field.hlsli"

namespace pr::physics::terrain::water
{
	namespace
	{
		// Require one scalar water-field value to be finite.
		void RequireFinite(double value, char const* name)
		{
			if (!std::isfinite(value))
				throw std::invalid_argument(std::format("Water field '{}' must be finite", name));
		}

		// Require one wave or packet scalar to be finite and strictly positive.
		void RequirePositive(float value, char const* name)
		{
			RequireFinite(value, name);
			if (!(value > 0.0f))
				throw std::invalid_argument(std::format("Water field '{}' must be greater than zero", name));
		}

		// Validate one element so CPU and GPU evaluation never meet a division by zero or an unknown type.
		void Validate(WaterFieldElement const& element)
		{
			// Every payload component must be finite, whatever the element type.
			for (auto i = 0; i != 4; ++i)
			{
				RequireFinite(element.position[i], "position");
				RequireFinite(element.wave[i], "wave");
				RequireFinite(element.timing[i], "timing");
			}

			switch (element.info.x)
			{
				case shared::WaterFieldElementNone:
				{
					return;
				}
				case shared::WaterFieldElementSineWave:
				case shared::WaterFieldElementGerstnerWave:
				{
					// Waves need a unit direction so amplitude, gradient, and phase keep their documented meaning.
					RequirePositive(element.wave.y, "wavelength");
					auto const length = Length(v2{element.position.x, element.position.y});
					if (std::abs(length - 1.0f) > 1e-4f)
						throw std::invalid_argument("Water field wave direction must be unit length");

					return;
				}
				case shared::WaterFieldElementRadialPacket:
				{
					// Packets with non-positive shape values are inactive in the evaluator, so only the wavelength must be usable.
					RequirePositive(element.wave.y, "wavelength");
					return;
				}
				default:
				{
					throw std::invalid_argument(std::format("Water field element type {} is not supported", element.info.x));
				}
			}
		}

		// Return a unit wave direction, rejecting a zero-length input.
		v2 UnitDirection(v2 direction)
		{
			auto const length = Length(direction);
			if (!(length > tiny<float>) || !std::isfinite(length))
				throw std::invalid_argument("Water field wave direction must be non-zero and finite");

			return direction / length;
		}
	}

	// Return a sine wave element with height A*sin(k*dot(d, xy) + omega*t).
	WaterFieldElement SineWave(v2 direction, float amplitude, float wavelength, float angular_frequency)
	{
		auto const d = UnitDirection(direction);
		return WaterFieldElement{
			.info = {shared::WaterFieldElementSineWave, 0, 0, 0},
			.position = {d.x, d.y, 0, 0},
			.wave = {amplitude, wavelength, angular_frequency, 0},
			.timing = {0, 0, 0, 0},
		};
	}

	// Return a Gerstner wave element with height A*sin(k*dot(d, xy) - k*c*t).
	WaterFieldElement GerstnerWave(v2 direction, float amplitude, float wavelength, float phase_speed, float steepness)
	{
		auto const d = UnitDirection(direction);
		return WaterFieldElement{
			.info = {shared::WaterFieldElementGerstnerWave, 0, 0, 0},
			.position = {d.x, d.y, 0, 0},
			.wave = {amplitude, wavelength, phase_speed, steepness},
			.timing = {0, 0, 0, 0},
		};
	}

	// Return a finite radial packet travelling outward from 'source'.
	WaterFieldElement RadialPacket(v2 source, float amplitude, float wavelength, float half_width, float propagation_speed, float age, float lifetime, float attack_time, float attenuation_scale)
	{
		return WaterFieldElement{
			.info = {shared::WaterFieldElementRadialPacket, 0, 0, 0},
			.position = {source.x, source.y, 0, 0},
			.wave = {amplitude, wavelength, half_width, propagation_speed},
			.timing = {age, lifetime, attack_time, attenuation_scale},
		};
	}

	// Construct a validated water field.
	WaterField::WaterField(double level, std::span<WaterFieldElement const> elements)
		: m_level()
		, m_elements()
		, m_amplitude_bound()
	{
		Level(level);
		Elements(elements);
	}
	WaterField::WaterField()
		: m_level()
		, m_elements()
		, m_amplitude_bound()
	{
		// Zero level with no elements is already a valid flat field.
	}

	// The still-water level in metres.
	double WaterField::Level() const noexcept
	{
		return m_level;
	}
	void WaterField::Level(double level)
	{
		RequireFinite(level, "level");
		m_level = level;
	}

	// The active elements, in upload order.
	std::span<WaterFieldElement const> WaterField::Elements() const noexcept
	{
		return m_elements;
	}
	void WaterField::Elements(std::span<WaterFieldElement const> elements)
	{
		// Validate everything before replacing the current elements so a failed update leaves the field unchanged.
		if (std::ssize(elements) > MaxElementCount)
			throw std::invalid_argument(std::format("Water field supports at most {} elements", MaxElementCount));

		auto amplitude_bound = 0.0;
		for (auto const& element : elements)
		{
			Validate(element);
			amplitude_bound += shared::WaterFieldElementAmplitudeBound(element);
		}

		m_elements.assign(elements.begin(), elements.end());
		m_amplitude_bound = amplitude_bound;
	}

	// True when no element can change the height.
	bool WaterField::IsFlat() const noexcept
	{
		return m_amplitude_bound == 0.0;
	}

	// Conservative bounds on the surface height over all positions and times.
	double WaterField::MaxHeight() const noexcept
	{
		return m_level + m_amplitude_bound;
	}
	double WaterField::MinHeight() const noexcept
	{
		return m_level - m_amplitude_bound;
	}

	// Sample the surface height at a world-space XY position and simulation time.
	double WaterField::Height(v2d xy, double time_s) const
	{
		// Element contributions are evaluated in float, matching the GPU, and added to the double still-water level.
		auto const xy_f = v2{static_cast<float>(xy.x), static_cast<float>(xy.y)};
		auto const t = static_cast<float>(time_s);
		auto height = 0.0;
		for (auto const& element : m_elements)
			height += shared::WaterFieldElementHeight(element, xy_f, t);

		return m_level + height;
	}

	// Sample the surface height and slope at a world-space XY position and simulation time.
	WaterSample WaterField::Sample(v2d xy, double time_s) const
	{
		// Accumulate the float element contributions onto the double still-water level.
		auto const xy_f = v2{static_cast<float>(xy.x), static_cast<float>(xy.y)};
		auto const t = static_cast<float>(time_s);
		auto sample = WaterSample{.m_height = m_level, .m_gradient_xy = v2d::Zero()};
		for (auto const& element : m_elements)
		{
			// Each contribution is (height, dh/dx, dh/dy).
			auto const contribution = shared::WaterFieldElementHeightAndGradient(element, xy_f, t);
			sample.m_height += contribution.x;
			sample.m_gradient_xy += v2d{contribution.y, contribution.z};
		}
		return sample;
	}

	// Sample the dimensionless lateral pressure gradient used by buoyancy forces.
	v2 WaterField::PressureGradient(v2 xy, float time_s, float gravity) const
	{
		// Without gravity there is no hydrostatic pressure to form a gradient.
		if (!(gravity > tiny<float>))
			return v2::Zero();

		auto gradient = v2::Zero();
		for (auto const& element : m_elements)
		{
			// Each contribution is (height, gradient.x, gradient.y); only the gradient is needed.
			auto const contribution = shared::WaterFieldElementHeightAndPressureGradient(element, xy, time_s, gravity);
			gradient += v2{contribution.y, contribution.z};
		}
		return gradient;
	}

	// Sample the world-space water particle velocity at a world-space position.
	v4 WaterField::Velocity(v4 pos_ws, float time_s) const
	{
		// Orbital flow decays with depth below the still-water level.
		auto velocity = v4::Zero();
		auto const level = static_cast<float>(m_level);
		for (auto const& element : m_elements)
		{
			// Each contribution is a world-space velocity.
			auto const contribution = shared::WaterFieldElementVelocity(element, pos_ws.xyz, time_s, level);
			velocity += v4{contribution.x, contribution.y, contribution.z, 0};
		}
		return velocity;
	}
}
