//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/terrain/water/bathymetry.h"

namespace pr::physics::terrain::water
{
	// Copy and validate the node heights.
	Bathymetry::Bathymetry(v2d origin, double cell_size, int width, int height, std::span<float const> heights)
		: m_origin(origin)
		, m_cell_size(cell_size)
		, m_width(width)
		, m_height(height)
		, m_heights(heights.begin(), heights.end())
		, m_min_height()
		, m_max_height()
	{
		// Interpolation needs at least one whole cell.
		if (!std::isfinite(origin.x) || !std::isfinite(origin.y))
			throw std::invalid_argument("Bathymetry origin must be finite");
		if (!std::isfinite(cell_size) || !(cell_size > 0.0))
			throw std::invalid_argument("Bathymetry cell size must be finite and positive");
		if (width < 2 || height < 2)
			throw std::invalid_argument("Bathymetry grid must have at least 2x2 nodes");
		if (std::ssize(heights) != static_cast<int64_t>(width) * height)
			throw std::invalid_argument("Bathymetry height count must equal width * height");

		// Record the global range for whole-field bounds.
		m_min_height = +std::numeric_limits<float>::infinity();
		m_max_height = -std::numeric_limits<float>::infinity();
		for (auto h : m_heights)
		{
			if (!std::isfinite(h))
				throw std::invalid_argument("Bathymetry heights must be finite");

			m_min_height = std::min(m_min_height, h);
			m_max_height = std::max(m_max_height, h);
		}
	}

	// Grid placement and size.
	v2d Bathymetry::Origin() const noexcept
	{
		return m_origin;
	}
	double Bathymetry::CellSize() const noexcept
	{
		return m_cell_size;
	}
	int Bathymetry::Width() const noexcept
	{
		return m_width;
	}
	int Bathymetry::Height() const noexcept
	{
		return m_height;
	}

	// The row-major node heights.
	std::span<float const> Bathymetry::Heights() const noexcept
	{
		return m_heights;
	}

	// The grid description used by the shared HLSL helpers.
	shared::WaterBathymetryGrid Bathymetry::Grid() const noexcept
	{
		return shared::WaterBathymetryGrid{
			.origin = {static_cast<float>(m_origin.x), static_cast<float>(m_origin.y)},
			.cell_size = static_cast<float>(m_cell_size),
			.pad = 0.0f,
			.dims = {m_width, m_height},
			.pad2 = {0, 0},
		};
	}

	// The bilinear terrain height at a world-space position.
	double Bathymetry::HeightAt(v2d xy) const
	{
		// Clamp to the grid so positions outside it use the edge heights.
		auto const u = std::clamp((xy.x - m_origin.x) / m_cell_size, 0.0, static_cast<double>(m_width - 1));
		auto const v = std::clamp((xy.y - m_origin.y) / m_cell_size, 0.0, static_cast<double>(m_height - 1));
		auto const i0 = std::min(static_cast<int>(u), m_width - 2);
		auto const j0 = std::min(static_cast<int>(v), m_height - 2);
		auto const fu = u - i0;
		auto const fv = v - j0;

		// Blend the four surrounding nodes.
		auto const* row0 = m_heights.data() + static_cast<size_t>(j0) * m_width + i0;
		auto const* row1 = row0 + m_width;
		auto const lo = row0[0] + (row0[1] - row0[0]) * fu;
		auto const hi = row1[0] + (row1[1] - row1[0]) * fu;
		return lo + (hi - lo) * fv;
	}

	// A lower bound on the interpolated terrain height anywhere in the world-space rectangle [lo, hi].
	double Bathymetry::MinHeight(v2d lo, v2d hi) const
	{
		// Bilinear heights never go below the lowest node of their cell, so the minimum over every node touching the rectangle is a bound.
		auto const node_range = [this](double lo_ws, double hi_ws, double origin, int count)
		{
			// Clamp each end to the grid; positions outside the grid use the edge nodes.
			auto const first = std::clamp(static_cast<int>(std::floor((lo_ws - origin) / m_cell_size)), 0, count - 1);
			auto const last = std::clamp(static_cast<int>(std::ceil((hi_ws - origin) / m_cell_size)), 0, count - 1);
			return std::pair{first, last};
		};
		auto const [i0, i1] = node_range(lo.x, hi.x, m_origin.x, m_width);
		auto const [j0, j1] = node_range(lo.y, hi.y, m_origin.y, m_height);

		// Scan the covered nodes.
		auto result = std::numeric_limits<float>::infinity();
		for (auto j = j0; j <= j1; ++j)
		{
			auto const* row = m_heights.data() + static_cast<size_t>(j) * m_width;
			for (auto i = i0; i <= i1; ++i)
				result = std::min(result, row[i]);
		}
		return result;
	}

	// The lowest and highest node heights over the whole grid.
	double Bathymetry::MinHeight() const noexcept
	{
		return m_min_height;
	}
	double Bathymetry::MaxHeight() const noexcept
	{
		return m_max_height;
	}
}
