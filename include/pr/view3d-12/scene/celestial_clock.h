//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Optional Sun–Earth–Moon model that places the procedural sky's sun, moon and stars.
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// The sun, moon and star state for one moment, in the procedural sky's frame (+Z up, +Y north, +X east).
	// The directions and rotation can be copied directly into ProceduralSkySettings (see procedural_sky.h).
	struct CelestialState
	{
		// Unit direction toward the sun.
		v4 m_sun_direction;

		// Unit direction toward the moon.
		v4 m_moon_direction;

		// Unit direction from the moon toward the sun, which sets the moon's lit side.
		v4 m_moon_light_direction;

		// Rotation from the sky frame to the star frame (+Z toward the north celestial pole, +X toward right ascension zero).
		m4x4 m_sky_to_stars;

		// Fraction of the moon's visible face that is lit: 0 for a new moon, 1 for a full moon.
		float m_lit_fraction;
	};

	// A simple Sun–Earth–Moon model for an observer on the Earth. The Earth follows a circular orbit with its axis tilted, so the date sets the sun's
	// path through the sky and the time of day sets where the sun is on that path. The moon goes around the Earth once per synodic month on an orbit
	// tilted to the Earth's orbit, so its phase and position both follow from its age and the sun. The model is accurate to a few degrees, which suits
	// a rendered sky but not navigation. The caller decides how fast time passes by calling Advance.
	// The default moment is midnight at the start of 1 January, at a new moon, seen from the equator.
	struct CelestialClock
	{
		// Days in the model's year.
		static constexpr double DaysPerYear = 365.0;

		// Days from one new moon to the next.
		static constexpr double SynodicMonthDays = 29.53;

		// Day of the year, counted from zero at the start of 1 January, of the March equinox, where the sun crosses the celestial equator heading north.
		static constexpr double MarchEquinoxDay = 79.0;

		// Tilt of the Earth's axis relative to its orbit, in degrees.
		static constexpr double ObliquityDegrees = 23.44;

		// Tilt of the moon's orbit relative to the Earth's orbit, in degrees.
		static constexpr double MoonInclinationDegrees = 5.14;

		// Latitude of the observer, in degrees within [-90, 90]. Negative is the southern hemisphere.
		float m_latitude = 0.0f;

		// Days since midnight at the start of 1 January. The fraction is the time of day. Any finite value is valid because the year repeats.
		double m_day = 0.0;

		// Days since the last new moon. About 14.8 is full. Any finite value is valid because the phases repeat.
		double m_moon_age = 0.0;

		// Move time forward by 'days'. The moon ages by the same amount. Both values are kept within one period so precision does not drift.
		void Advance(double days) noexcept
		{
			// The date and the moon share one clock, so the moon's phase stays tied to the sun.
			m_day = Wrap(m_day + days, DaysPerYear);
			m_moon_age = Wrap(m_moon_age + days, SynodicMonthDays);
		}

		// The sun, moon and stars at the current moment.
		CelestialState Evaluate() const noexcept
		{
			// The celestial sphere's frame at the observer: the north celestial pole, the point where the celestial equator crosses the meridian highest,
			// and west. The pole's height above the horizon equals the latitude, so in the south it is below the northern horizon.
			auto latitude = DegreesToRadians(double(m_latitude));
			auto pole = v4(0, float(std::cos(latitude)), float(std::sin(latitude)), 0);
			auto meridian = v4(0, float(-std::sin(latitude)), float(std::cos(latitude)), 0);
			auto west = v4(-1, 0, 0, 0);

			// The Earth moves evenly around a circular orbit, so the sun's longitude along the ecliptic grows evenly from zero at the March equinox.
			// The moon is ahead of the sun by its age, as a fraction of a month, and moves above and below the ecliptic on its tilted orbit.
			// Its orbit's nodes are fixed, so the same age on the same date always gives the same moon.
			auto sun_longitude = constants<double>::tau * (m_day - MarchEquinoxDay) / DaysPerYear;
			auto moon_longitude = sun_longitude + constants<double>::tau * m_moon_age / SynodicMonthDays;
			auto moon_latitude = DegreesToRadians(MoonInclinationDegrees) * std::sin(moon_longitude);
			auto sun_stars = EclipticToStars(sun_longitude, 0);
			auto moon_stars = EclipticToStars(moon_longitude, moon_latitude);

			// The time of day is the sun's hour angle: 12:00 puts the sun on the meridian. The stars' turn, the local sidereal time, is the sun's
			// right ascension plus its hour angle, so the stars keep their place relative to the sun at a given date and time.
			auto time_of_day = m_day - std::floor(m_day);
			auto sun_hour_angle = constants<double>::tau * (time_of_day - 0.5);
			auto sidereal = std::atan2(double(sun_stars.y), double(sun_stars.x)) + sun_hour_angle;

			// A star at right ascension 'ra' is at hour angle (sidereal time - ra), measured west from the meridian. So the star frame's X and Y axes
			// are the meridian and west directions turned by the sidereal time, which keeps the frame right-handed.
			auto sin_st = float(std::sin(sidereal));
			auto cos_st = float(std::cos(sidereal));
			auto stars_x = meridian * cos_st + west * sin_st;
			auto stars_y = meridian * sin_st - west * cos_st;
			auto stars_to_sky = m4x4(stars_x, stars_y, pole, v4::Origin());

			// The sun is far enough away that the moon sees it in the same direction as the observer does, so the sun's direction lights the moon.
			auto sun = Normalise(stars_to_sky * sun_stars);
			auto moon = Normalise(stars_to_sky * moon_stars);
			return CelestialState{
				.m_sun_direction = sun,
				.m_moon_direction = moon,
				.m_moon_light_direction = sun,
				.m_sky_to_stars = Transpose3x3(stars_to_sky),
				.m_lit_fraction = 0.5f - 0.5f * Dot3(sun, moon),
			};
		}

	private:

		// Return the star-frame unit direction of the point at ecliptic 'longitude' and 'latitude', in radians.
		static v4 EclipticToStars(double longitude, double latitude) noexcept
		{
			// The ecliptic is tilted from the celestial equator about their shared X axis, the March equinox.
			auto obliquity = DegreesToRadians(ObliquityDegrees);
			auto sin_e = std::sin(obliquity);
			auto cos_e = std::cos(obliquity);
			auto x = std::cos(latitude) * std::cos(longitude);
			auto y = std::cos(latitude) * std::sin(longitude);
			auto z = std::sin(latitude);
			return v4(float(x), float(y * cos_e - z * sin_e), float(y * sin_e + z * cos_e), 0);
		}

		// Return 'value' wrapped into [0, 'period').
		static double Wrap(double value, double period) noexcept
		{
			// Floor keeps negative values in range too.
			return value - period * std::floor(value / period);
		}
	};
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
namespace pr::rdr12::tests
{
	PRUnitTest(CelestialClockTests, Quick)
	{
		// At the March equinox, the noon sun at 35 degrees south is 55 degrees up, due north.
		auto clock = CelestialClock{ .m_latitude = -35.0f, .m_day = CelestialClock::MarchEquinoxDay + 0.5, .m_moon_age = 0.0 };
		auto sky = clock.Evaluate();
		PR_EXPECT(FEqlAbsolute(sky.m_sun_direction.z, float(std::sin(DegreesToRadians(55.0))), 0.01f));
		PR_EXPECT(FEqlAbsolute(sky.m_sun_direction.x, 0.0f, 0.01f));
		PR_EXPECT(sky.m_sun_direction.y > 0);

		// A new moon is dark and near the sun.
		PR_EXPECT(sky.m_lit_fraction < 0.01f);

		// At midnight on the same date the sun is below the horizon.
		clock.m_day = CelestialClock::MarchEquinoxDay;
		PR_EXPECT(clock.Evaluate().m_sun_direction.z < -0.5f);

		// A full moon is lit and opposite the sun, and is lit from the sun's direction.
		clock.m_day = CelestialClock::MarchEquinoxDay + 0.5;
		clock.m_moon_age = CelestialClock::SynodicMonthDays / 2;
		sky = clock.Evaluate();
		PR_EXPECT(sky.m_lit_fraction > 0.99f);
		PR_EXPECT(FEql(sky.m_moon_light_direction, sky.m_sun_direction));

		// Advancing moves the date and the moon together, and wraps both into one period.
		clock.Advance(0.5);
		PR_EXPECT(FEqlAbsolute(clock.m_day, CelestialClock::MarchEquinoxDay + 1, 1e-9));
		PR_EXPECT(FEqlAbsolute(clock.m_moon_age, CelestialClock::SynodicMonthDays / 2 + 0.5, 1e-9));
		clock.m_day = 0.25;
		clock.Advance(-0.5);
		PR_EXPECT(FEqlAbsolute(clock.m_day, CelestialClock::DaysPerYear - 0.25, 1e-9));
	}
}
#endif
