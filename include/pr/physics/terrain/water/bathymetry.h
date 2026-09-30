//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/terrain/forward.h"
#include "pr/physics/terrain/water/water_depth.hlsli"

namespace pr::physics::terrain::water
{
	// A regular grid of terrain heights used to find the water depth under the waves.
	// It stores terrain heights rather than depths, so it stays valid when the water level changes.
	// Heights between nodes are bilinear, and positions outside the grid use the nearest edge value. See water_depth.hlsli for the GPU equivalent.
	class Bathymetry
	{
		v2d m_origin;
		double m_cell_size;
		int m_width;
		int m_height;
		std::vector<float> m_heights;
		float m_min_height;
		float m_max_height;

	public:

		// Copy 'width' x 'height' row-major node heights. Node (i, j) is at origin + cell_size * (i, j).
		// Throws invalid_argument unless the grid has at least 2x2 nodes, a positive cell size, and finite values.
		Bathymetry(v2d origin, double cell_size, int width, int height, std::span<float const> heights);

		// Grid placement and size.
		v2d Origin() const noexcept;
		double CellSize() const noexcept;
		int Width() const noexcept;
		int Height() const noexcept;

		// The row-major node heights.
		std::span<float const> Heights() const noexcept;

		// The grid description used by the shared HLSL helpers, in float world coordinates.
		shared::WaterBathymetryGrid Grid() const noexcept;

		// The bilinear terrain height at a world-space position.
		double HeightAt(v2d xy) const;

		// A lower bound on the interpolated terrain height anywhere in the world-space rectangle [lo, hi].
		double MinHeight(v2d lo, v2d hi) const;

		// The lowest and highest node heights over the whole grid.
		double MinHeight() const noexcept;
		double MaxHeight() const noexcept;
	};
}
