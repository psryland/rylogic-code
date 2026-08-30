//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
// Perlin noise height field for world terrain generation.
// Height > 0 = land, height < 0 = ocean floor, ocean surface at z = 0.
#pragma once
#include "src/forward.h"

namespace las
{
	// Multi-octave Perlin noise height field
	struct HeightField
	{
		int m_octaves;
		float m_base_frequency;
		float m_persistence;
		float m_amplitude;
		float m_sea_level_bias;

		std::default_random_engine m_rng;
		pr::algorithm::PerlinNoiseGenerator<std::default_random_engine> m_noise;

		explicit HeightField(uint32_t seed = 42);

		float HeightAt(float world_x, float world_y) const;
		v4 NormalAt(float world_x, float world_y) const;
		bool IsLand(float world_x, float world_y) const;

		// Search for a high terrain point near 'centre', sampling in a grid of 'radius' extent.
		v4 FindHighPoint(float centre_x, float centre_y, float radius = 2000.0f, float step = 50.0f) const;
	};
}