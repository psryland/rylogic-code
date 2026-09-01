//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
#include "src/forward.h"
#include "src/world/terrain/height_field.h"

namespace las
{
	HeightField::HeightField(uint32_t seed)
		:m_octaves(6)
		,m_base_frequency(0.001f)  // ~1000m feature scale
		,m_persistence(0.5f)
		,m_amplitude(300.0f)       // Islands peak ~200m above sea level
		,m_sea_level_bias(-0.3f)   // Ensures ~65% water coverage
		,m_noise(seed)
	{}

	float HeightField::HeightAt(float world_x, float world_y) const
	{
		auto value = 0.0f;
		auto freq = m_base_frequency;
		auto amp = 1.0f;
		auto max_amp = 0.0f;

		for (int i = 0; i < m_octaves; ++i)
		{
			value += m_noise.Noise(world_x * freq, world_y * freq, 0.0f) * amp;
			max_amp += amp;
			amp *= m_persistence;
			freq *= 2.0f;
		}

		// Normalise to [-1, 1] and apply amplitude + bias
		value = value / max_amp;
		return (value + m_sea_level_bias) * m_amplitude;
	}

	v4 HeightField::NormalAt(float world_x, float world_y) const
	{
		auto eps = 1.0f; // 1m sample spacing
		auto hL = HeightAt(world_x - eps, world_y);
		auto hR = HeightAt(world_x + eps, world_y);
		auto hD = HeightAt(world_x, world_y - eps);
		auto hU = HeightAt(world_x, world_y + eps);
		return Normalise(v4(hL - hR, hD - hU, 2.0f * eps, 0.0f));
	}

	bool HeightField::IsLand(float world_x, float world_y) const
	{
		return HeightAt(world_x, world_y) > 0.0f;
	}

	v4 HeightField::FindHighPoint(float centre_x, float centre_y, float radius, float step) const
	{
		auto best_x = centre_x;
		auto best_y = centre_y;
		auto best_z = HeightAt(centre_x, centre_y);

		for (auto y = centre_y - radius; y <= centre_y + radius; y += step)
		{
			for (auto x = centre_x - radius; x <= centre_x + radius; x += step)
			{
				auto z = HeightAt(x, y);
				if (z > best_z)
				{
					best_x = x;
					best_y = y;
					best_z = z;
				}
			}
		}

		return v4{best_x, best_y, best_z, 1};
	}
}
