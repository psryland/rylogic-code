//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/terrain/landscape/height_bounds.h"

namespace pr::physics::terrain::landscape
{
	// Sample the grid corners in parallel, then build level 0 from the corners and each higher level from the one below.
	HeightBounds::HeightBounds(BaselineSurface const& surface, v2d origin_xy, double cell_size, int cells_per_side)
		: m_origin_xy(origin_xy)
		, m_cell_size(cell_size)
		, m_cells_per_side(cells_per_side)
		, m_levels()
	{
		// The pyramid halves each level exactly, so the grid side must be a power of two.
		if (!std::isfinite(origin_xy.x) || !std::isfinite(origin_xy.y))
			throw std::invalid_argument("Height bounds origin must be finite");
		if (!std::isfinite(cell_size) || !(cell_size > 0.0))
			throw std::invalid_argument("Height bounds cell size must be finite and positive");
		if (cells_per_side < 1 || cells_per_side > MaxCellsPerSide || (cells_per_side & (cells_per_side - 1)) != 0)
			throw std::invalid_argument(std::format("Height bounds cells per side must be a power of two in [1, {}]", MaxCellsPerSide));

		// Sample every corner. Rows are independent, and the surface is immutable, so rows run in parallel.
		auto const corners_per_side = cells_per_side + 1;
		auto corners = std::vector<SurfaceSample>(static_cast<size_t>(corners_per_side) * corners_per_side);
		auto rows = std::vector<int>(corners_per_side);
		std::iota(rows.begin(), rows.end(), 0);
		std::for_each(std::execution::par, rows.begin(), rows.end(), [&](int y)
		{
			// Sample one row of corners along +X.
			auto positions = std::vector<v2d>(corners_per_side);
			for (int x = 0; x != corners_per_side; ++x)
				positions[x] = v2d{ origin_xy.x + x * cell_size, origin_xy.y + y * cell_size };

			surface.Sample(positions, std::span{ corners }.subspan(static_cast<size_t>(y) * corners_per_side, corners_per_side));
		});

		// Every point in a cell is within half a cell along each axis of its nearest corner, so a corner's first-order reach is
		// half a cell times the sum of its absolute slopes. MarginScale widens that reach to allow for curvature.
		auto& cells = m_levels.emplace_back(static_cast<size_t>(cells_per_side) * cells_per_side);
		for (int y = 0; y != cells_per_side; ++y)
		{
			for (int x = 0; x != cells_per_side; ++x)
			{
				// Merge the reach of the cell's four corners.
				auto range = HeightRange{ .m_min = std::numeric_limits<double>::infinity(), .m_max = -std::numeric_limits<double>::infinity() };
				for (auto [cx, cy] : { std::pair{x, y}, std::pair{x + 1, y}, std::pair{x, y + 1}, std::pair{x + 1, y + 1} })
				{
					auto const& corner = corners[static_cast<size_t>(cy) * corners_per_side + cx];
					auto const reach = 0.5 * cell_size * MarginScale * (std::abs(corner.m_gradient_xy.x) + std::abs(corner.m_gradient_xy.y));
					range.m_min = std::min(range.m_min, corner.m_height - reach);
					range.m_max = std::max(range.m_max, corner.m_height + reach);
				}
				cells[static_cast<size_t>(y) * cells_per_side + x] = range;
			}
		}

		// Each coarser entry is the union of the 2x2 entries it covers.
		for (int side = cells_per_side / 2; side >= 1; side /= 2)
		{
			auto const& fine = m_levels.back();
			auto coarse = std::vector<HeightRange>(static_cast<size_t>(side) * side);
			for (int y = 0; y != side; ++y)
			{
				for (int x = 0; x != side; ++x)
				{
					// Union the four children.
					auto const fine_side = side * 2;
					auto const& a = fine[static_cast<size_t>(2 * y + 0) * fine_side + 2 * x + 0];
					auto const& b = fine[static_cast<size_t>(2 * y + 0) * fine_side + 2 * x + 1];
					auto const& c = fine[static_cast<size_t>(2 * y + 1) * fine_side + 2 * x + 0];
					auto const& d = fine[static_cast<size_t>(2 * y + 1) * fine_side + 2 * x + 1];
					coarse[static_cast<size_t>(y) * side + x] = HeightRange{
						.m_min = std::min({ a.m_min, b.m_min, c.m_min, d.m_min }),
						.m_max = std::max({ a.m_max, b.m_max, c.m_max, d.m_max }),
					};
				}
			}
			m_levels.push_back(std::move(coarse));
		}
	}

	// Return the lowest corner of the covered region.
	v2d HeightBounds::Origin() const noexcept
	{
		return m_origin_xy;
	}

	// Return the edge length of a level-0 cell in metres.
	double HeightBounds::CellSize() const noexcept
	{
		return m_cell_size;
	}

	// Return the number of level-0 cells along each side.
	int HeightBounds::CellsPerSide() const noexcept
	{
		return m_cells_per_side;
	}

	// Return the number of levels, including level 0 and the root.
	int HeightBounds::LevelCount() const noexcept
	{
		return static_cast<int>(m_levels.size());
	}

	// Return the root range.
	HeightRange HeightBounds::Total() const noexcept
	{
		return m_levels.back().front();
	}

	// Descend from the root, taking whole entries that lie inside the rectangle and splitting entries that straddle its edge.
	std::optional<HeightRange> HeightBounds::Query(v2d min_xy, v2d max_xy) const
	{
		// Convert the rectangle to an inclusive level-0 cell range and clip it to the grid.
		auto const to_cell = [&](double value, double origin)
		{
			return static_cast<int>(std::floor((value - origin) / m_cell_size));
		};
		auto const x0 = std::max(to_cell(min_xy.x, m_origin_xy.x), 0);
		auto const y0 = std::max(to_cell(min_xy.y, m_origin_xy.y), 0);
		auto const x1 = std::min(to_cell(max_xy.x, m_origin_xy.x), m_cells_per_side - 1);
		auto const y1 = std::min(to_cell(max_xy.y, m_origin_xy.y), m_cells_per_side - 1);
		if (!(min_xy.x <= max_xy.x && min_xy.y <= max_xy.y) || x0 > x1 || y0 > y1)
			return std::nullopt;

		// Visit entries with an explicit stack; an entry wholly inside the cell range contributes its own range.
		auto result = HeightRange{ .m_min = std::numeric_limits<double>::infinity(), .m_max = -std::numeric_limits<double>::infinity() };
		struct Node { int level, x, y; };
		auto stack = std::vector<Node>{ Node{ LevelCount() - 1, 0, 0 } };
		while (!stack.empty())
		{
			// Skip entries outside the range, take entries inside it, and split the rest.
			auto const node = stack.back();
			stack.pop_back();
			auto const span = 1 << node.level;
			auto const nx0 = node.x * span, ny0 = node.y * span;
			auto const nx1 = nx0 + span - 1, ny1 = ny0 + span - 1;
			if (nx1 < x0 || nx0 > x1 || ny1 < y0 || ny0 > y1)
				continue;

			if ((nx0 >= x0 && nx1 <= x1 && ny0 >= y0 && ny1 <= y1) || node.level == 0)
			{
				auto const& range = m_levels[node.level][static_cast<size_t>(node.y) * (m_cells_per_side >> node.level) + node.x];
				result.m_min = std::min(result.m_min, range.m_min);
				result.m_max = std::max(result.m_max, range.m_max);
				continue;
			}
			for (int j = 0; j != 2; ++j)
			{
				for (int i = 0; i != 2; ++i)
					stack.push_back(Node{ node.level - 1, node.x * 2 + i, node.y * 2 + j });
			}
		}
		return result;
	}

	// Descend from the root, pruning entries that are entirely at or above 'height' or rejected by 'visible'.
	void HeightBounds::TilesBelow(double height, int level, std::function<bool(Tile const&)> const& visible, std::vector<Tile>& tiles) const
	{
		// The requested level must exist in the pyramid.
		if (level < 0 || level >= LevelCount())
			throw std::invalid_argument(std::format("Height bounds level {} is outside [0, {})", level, LevelCount()));

		// Visit entries with an explicit stack so deep pyramids cannot exhaust the call stack.
		struct Node { int level, x, y; };
		auto stack = std::vector<Node>{ Node{ LevelCount() - 1, 0, 0 } };
		while (!stack.empty())
		{
			// Drop dry or rejected entries, emit entries at the target level, and split the rest.
			auto const node = stack.back();
			stack.pop_back();
			auto const tile = TileAt(node.level, node.x, node.y);
			if (!(tile.m_range.m_min < height))
				continue;
			if (visible && !visible(tile))
				continue;

			if (node.level == level)
			{
				tiles.push_back(tile);
				continue;
			}
			for (int j = 0; j != 2; ++j)
			{
				for (int i = 0; i != 2; ++i)
					stack.push_back(Node{ node.level - 1, node.x * 2 + i, node.y * 2 + j });
			}
		}
	}

	// Return the tile for entry (x, y) at 'level'.
	HeightBounds::Tile HeightBounds::TileAt(int level, int x, int y) const
	{
		// Entries at 'level' span 2^level cells.
		auto const size = m_cell_size * (1 << level);
		return Tile{
			.m_min_xy = v2d{ m_origin_xy.x + x * size, m_origin_xy.y + y * size },
			.m_size = size,
			.m_range = m_levels[level][static_cast<size_t>(y) * (m_cells_per_side >> level) + x],
		};
	}
}
