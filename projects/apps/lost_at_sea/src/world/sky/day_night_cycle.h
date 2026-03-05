//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Day/night cycle: computes sun position, colour, and intensity from time of day.
#pragma once
#include "src/forward.h"

namespace las
{
	struct DayNightCycle
	{
		float m_time_of_day;  // Hours [0, 24)
		float m_day_speed;    // Game seconds per real second (e.g., 60 = 1 real second per game minute)

		// Maximum sun elevation angle (~63° for mid-latitudes)
		static constexpr float MaxElevation = 1.1f; // radians

		DayNightCycle()
			: m_time_of_day(10.0f)  // Start at 10 AM
			, m_day_speed(60.0f)    // 1 real second = 1 game minute
		{}

		void Update(float dt)
		{
			m_time_of_day += dt * m_day_speed / 3600.0f;
			m_time_of_day = std::fmod(m_time_of_day, 24.0f);
			if (m_time_of_day < 0) m_time_of_day += 24.0f;
		}

		// Sun position as a normalised direction vector pointing toward the sun.
		// Sunrise at 06:00, noon at 12:00, sunset at 18:00.
		v4 SunDirection() const
		{
			auto azimuth = (m_time_of_day - 12.0f) / 24.0f * constants<float>::tau;
			auto elevation = MaxElevation * std::sin(constants<float>::tau * 0.5f * (m_time_of_day - 6.0f) / 12.0f);
			return Normalise(v4(
				std::cos(azimuth) * std::cos(elevation),
				std::sin(azimuth) * std::cos(elevation),
				std::sin(elevation),
				0.0f));
		}

		// Sun light colour: warm white at noon, orange at sunrise/sunset, dim at night
		v4 SunColour() const
		{
			auto elev = SunDirection().z;
			auto intensity = std::clamp(elev * 2.0f + 0.2f, 0.05f, 1.2f);

			// Sunset band: warm tones when sun is near horizon
			auto sunset = std::clamp(1.0f - std::abs(elev) * 3.0f, 0.0f, 1.0f);
			sunset *= sunset;

			return v4(
				intensity,
				intensity * (1.0f - sunset * 0.4f),
				intensity * (1.0f - sunset * 0.7f),
				1.0f);
		}

		// Scalar intensity: 0 at night, 1 at noon
		float SunIntensity() const
		{
			return std::clamp(SunDirection().z * 2.0f + 0.1f, 0.0f, 1.0f);
		}
	};
}
