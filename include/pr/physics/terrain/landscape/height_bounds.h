//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/terrain/forward.h"
#include "pr/physics/terrain/landscape/baseline_surface.h"

namespace pr::physics::terrain::landscape
{
	// An inclusive range of terrain heights in metres.
	struct HeightRange
	{
		double m_min = 0.0;
		double m_max = 0.0;
	};

	// A min/max terrain-height pyramid over a square region of a baseline surface.
	// Level 0 has one entry per grid cell and each higher level merges 2x2 entries, up to one root entry for the whole region.
	// Each cell's range comes from the heights and slopes at its four corners, widened by MarginScale, so it covers the true surface
	// wherever the surface is close to linear across a cell. Features much narrower than the cell size can escape the range, so choose a
	// cell size below the smallest terrain wavelength of interest. Queries are cheap, so callers can re-query when a threshold height changes.
	class HeightBounds
	{
	public:

		// One square pyramid entry.
		struct Tile
		{
			v2d m_min_xy = v2d::Zero();
			double m_size = 0.0;
			HeightRange m_range;
		};

		// Scales the first-order slope margin to allow for curvature within a cell.
		inline static double constexpr MarginScale = 1.5;

		// The largest supported grid, in cells per side.
		inline static int constexpr MaxCellsPerSide = 4096;

		// Sample 'surface' on a square grid of 'cells_per_side' cells of 'cell_size' metres whose lowest corner is 'origin_xy'.
		// 'cells_per_side' must be a power of two no greater than MaxCellsPerSide, and the region must lie within the surface's supported bounds.
		// Throws invalid_argument for an invalid grid, and propagates surface sampling errors.
		HeightBounds(BaselineSurface const& surface, v2d origin_xy, double cell_size, int cells_per_side);

		// Return the lowest corner of the covered region.
		v2d Origin() const noexcept;

		// Return the edge length of a level-0 cell in metres.
		double CellSize() const noexcept;

		// Return the number of level-0 cells along each side.
		int CellsPerSide() const noexcept;

		// Return the number of levels, including level 0 and the single-entry root.
		int LevelCount() const noexcept;

		// Return the height range of the whole region.
		HeightRange Total() const noexcept;

		// Return the height range over the part of the rectangle [min_xy, max_xy] that lies inside the region, or nullopt if they do not overlap.
		// The result is the union of the smallest pyramid entries that cover the overlap, so it can be wider than the exact range.
		std::optional<HeightRange> Query(v2d min_xy, v2d max_xy) const;

		// Append the entries at 'level' whose minimum height is below 'height'. 'visible' is called for entries at every level on the way down;
		// returning false skips that entry and everything inside it, which lets callers cull large areas (for example by view frustum) early.
		void TilesBelow(double height, int level, std::function<bool(Tile const&)> const& visible, std::vector<Tile>& tiles) const;

	private:

		v2d m_origin_xy;
		double m_cell_size;
		int m_cells_per_side;

		// m_levels[k] holds (m_cells_per_side >> k)² ranges in row-major order, with rows along +Y.
		std::vector<std::vector<HeightRange>> m_levels;

		// Return the tile for entry (x, y) at 'level'.
		Tile TileAt(int level, int x, int y) const;
	};
}
