//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#pragma once
#include "src/forward.h"

namespace las
{
	// Parameters for a single Gerstner wave component
	struct GerstnerWave
	{
		v4 m_direction;     // Normalised wave travel direction (XY plane, Z=0, w=0)
		float m_amplitude;  // Wave height (peak to mean), in metres
		float m_wavelength; // Distance between crests, in metres
		float m_speed;      // Phase speed, in m/s
		float m_steepness;  // Gerstner steepness Q [0..1], controls sharpness of peaks

		float Frequency() const
		{
			return constants<float>::tau / m_wavelength;
		}
		float WaveNumber() const
		{
			return constants<float>::tau / m_wavelength;
		}
	};
}
