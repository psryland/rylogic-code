//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/terrain/landscape/baseline_surface.h"
#include "pr/physics/terrain/landscape/baseline_surface.hlsli"
#include "pr/physics/terrain/landscape/terrain_decals.hlsli"
#include "pr/physics/materials/material.h"

namespace pr::physics::terrain::landscape
{
	// A uniform grid over a set of decal discs. Each cell lists every decal whose blend disc can reach a point in that cell.
	struct DecalCells
	{
		v2d m_origin = {};                 // Minimum corner of the grid, in the frame the grid was built in.
		double m_cell_size = 1;            // Side length of one cell, in metres.
		uint32_t m_cells_x = 0;            // Number of cells along X.
		uint32_t m_cells_y = 0;            // Number of cells along Y.
		std::vector<uint32_t> m_cells;     // (first, count) pairs into 'm_indices', row-major from the minimum corner.
		std::vector<uint32_t> m_indices;   // Decal numbers, grouped by cell.

		// Return the cell coordinate containing 'value' along one axis, clamped to the grid.
		static uint32_t Cell(double value, double origin, double cell_size, uint32_t count)
		{
			// The same monotonic arithmetic is used for decal bounds and for samples, so a sample inside a decal's bounds finds that decal's cells.
			auto const cell = std::floor((value - origin) / cell_size);
			return static_cast<uint32_t>(std::clamp(cell, 0.0, static_cast<double>(count - 1)));
		}
	};

	// The decals on one surface and the grid that indexes them in world space.
	struct BaselineSurface::DecalGrid
	{
		std::vector<TerrainDecal> m_decals;
		DecalCells m_cells;
		v2d m_lower; // World-space minimum corner of every blend disc's bounds.
		v2d m_upper; // World-space maximum corner of every blend disc's bounds.
	};

	namespace
	{
		// Upper bound on the number of grid cells, which bounds the size of the local decal blob.
		constexpr uint32_t MaxDecalGridCells = 1 << 14;

		// Build a grid over 'decals', with centres measured from 'frame_origin'. Each decal's bounds are enlarged by 'margin' metres.
		// When 'fp32' is true, the grid origin and cell size are exact single-precision values so a single-precision reader computes the same cells.
		DecalCells BuildDecalCells(std::span<TerrainDecal const> decals, v2d frame_origin, double margin, bool fp32)
		{
			// Bound every blend disc, enlarged by the margin, and find the largest blend radius.
			auto lower = v2d{+std::numeric_limits<double>::infinity(), +std::numeric_limits<double>::infinity()};
			auto upper = v2d{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
			auto max_radius = 0.0;
			for (auto const& decal : decals)
			{
				auto const centre = decal.m_centre - frame_origin;
				auto const reach = decal.m_blend_radius + margin;
				lower = v2d{std::min(lower.x, centre.x - reach), std::min(lower.y, centre.y - reach)};
				upper = v2d{std::max(upper.x, centre.x + reach), std::max(upper.y, centre.y + reach)};
				max_radius = std::max(max_radius, reach);
			}

			// Single-precision readers need an origin and cell size that are exact floats. Rounding the origin down keeps every disc inside the grid.
			auto cells = DecalCells{};
			cells.m_origin = lower;
			cells.m_cell_size = 2 * max_radius;
			if (fp32)
			{
				cells.m_origin = v2d{
					static_cast<double>(std::nextafter(static_cast<float>(lower.x), -std::numeric_limits<float>::infinity())),
					static_cast<double>(std::nextafter(static_cast<float>(lower.y), -std::numeric_limits<float>::infinity()))};
				cells.m_cell_size = static_cast<double>(std::nextafter(static_cast<float>(cells.m_cell_size), std::numeric_limits<float>::infinity()));
			}

			// A disc spans at most two cells per axis at this size. Double the size while the grid has too many cells, so widely spread decals stay compact.
			for (;;)
			{
				auto const count_x = std::max(1.0, std::ceil((upper.x - cells.m_origin.x) / cells.m_cell_size));
				auto const count_y = std::max(1.0, std::ceil((upper.y - cells.m_origin.y) / cells.m_cell_size));
				if (count_x * count_y <= MaxDecalGridCells)
				{
					cells.m_cells_x = static_cast<uint32_t>(count_x);
					cells.m_cells_y = static_cast<uint32_t>(count_y);
					break;
				}
				cells.m_cell_size *= 2;
			}

			// Count the decals that touch each cell, then turn the counts into first-index offsets.
			auto const cell_count = size_t(cells.m_cells_x) * cells.m_cells_y;
			auto ranges = std::vector<std::array<uint32_t, 4>>(decals.size()); // Inclusive cell range (x0, y0, x1, y1) of each decal.
			auto counts = std::vector<uint32_t>(cell_count + 1, 0);
			for (auto i = size_t{}; i != decals.size(); ++i)
			{
				// Each decal touches the cells covering its enlarged bounds.
				auto const centre = decals[i].m_centre - frame_origin;
				auto const reach = decals[i].m_blend_radius + margin;
				auto& range = ranges[i];
				range[0] = DecalCells::Cell(centre.x - reach, cells.m_origin.x, cells.m_cell_size, cells.m_cells_x);
				range[1] = DecalCells::Cell(centre.y - reach, cells.m_origin.y, cells.m_cell_size, cells.m_cells_y);
				range[2] = DecalCells::Cell(centre.x + reach, cells.m_origin.x, cells.m_cell_size, cells.m_cells_x);
				range[3] = DecalCells::Cell(centre.y + reach, cells.m_origin.y, cells.m_cell_size, cells.m_cells_y);
				for (auto y = range[1]; y <= range[3]; ++y)
					for (auto x = range[0]; x <= range[2]; ++x)
						++counts[size_t(y) * cells.m_cells_x + x + 1];
			}
			for (auto c = size_t{}; c != cell_count; ++c)
				counts[c + 1] += counts[c];

			// Fill the index list in decal order and record each cell's (first, count) pair.
			cells.m_indices.resize(counts[cell_count]);
			auto cursor = std::vector<uint32_t>(counts.begin(), counts.end() - 1);
			for (auto i = size_t{}; i != decals.size(); ++i)
			{
				auto const& range = ranges[i];
				for (auto y = range[1]; y <= range[3]; ++y)
					for (auto x = range[0]; x <= range[2]; ++x)
						cells.m_indices[cursor[size_t(y) * cells.m_cells_x + x]++] = static_cast<uint32_t>(i);
			}
			cells.m_cells.resize(cell_count * 2);
			for (auto c = size_t{}; c != cell_count; ++c)
			{
				cells.m_cells[c * 2 + 0] = counts[c];
				cells.m_cells[c * 2 + 1] = counts[c + 1] - counts[c];
			}
			return cells;
		}
		// Return a deterministic decorrelated field seed derived from the user-visible seed.
		uint32_t MixSeed(uint32_t seed, uint32_t salt)
		{
			auto hash = uint64_t{seed} + 0x9E3779B97F4A7C15ull + (uint64_t{salt} << 1);
			hash ^= hash >> 30;
			hash *= 0xBF58476D1CE4E5B9ull;
			hash ^= hash >> 27;
			hash *= 0x94D049BB133111EBull;
			hash ^= hash >> 31;
			return static_cast<uint32_t>(hash);
		}

		// Require one scalar configuration value to be finite.
		void RequireFinite(double value, char const* name)
		{
			if (!std::isfinite(value))
				throw std::invalid_argument(std::format("Terrain config '{}' must be finite", name));
		}

		// Require one positive scalar configuration value to be finite and strictly positive.
		void RequirePositive(double value, char const* name)
		{
			RequireFinite(value, name);
			if (value <= 0.0)
				throw std::invalid_argument(std::format("Terrain config '{}' must be greater than zero", name));
		}

		// Require one bounded octave count to stay within the supported contract.
		void RequireOctaves(int octave_count, char const* name)
		{
			if (octave_count < 1 || octave_count > BaselineSurface::MaxOctaveCount)
				throw std::invalid_argument(std::format("Terrain config '{}' must be in the range [1, {}]", name, BaselineSurface::MaxOctaveCount));
		}

		// Validate one ordinary fractal band.
		void Validate(FractalConfig const& config, char const* prefix)
		{
			RequireFinite(config.m_amplitude, std::format("{}.amplitude", prefix).c_str());
			RequirePositive(config.m_wavelength_m, std::format("{}.wavelength_m", prefix).c_str());
			RequireOctaves(config.m_octave_count, std::format("{}.octave_count", prefix).c_str());
			RequirePositive(config.m_lacunarity, std::format("{}.lacunarity", prefix).c_str());
			RequireFinite(config.m_persistence, std::format("{}.persistence", prefix).c_str());
		}

		// Validate one domain-warp band.
		void Validate(DomainWarpConfig const& config, char const* prefix)
		{
			RequireFinite(config.m_amplitude_m, std::format("{}.amplitude_m", prefix).c_str());
			RequirePositive(config.m_wavelength_m, std::format("{}.wavelength_m", prefix).c_str());
			RequireOctaves(config.m_octave_count, std::format("{}.octave_count", prefix).c_str());
			RequirePositive(config.m_lacunarity, std::format("{}.lacunarity", prefix).c_str());
			RequireFinite(config.m_persistence, std::format("{}.persistence", prefix).c_str());
		}

		// Validate one rounded-ridge fractal band.
		void Validate(RidgedFractalConfig const& config, char const* prefix)
		{
			RequireFinite(config.m_amplitude, std::format("{}.amplitude", prefix).c_str());
			RequirePositive(config.m_wavelength_m, std::format("{}.wavelength_m", prefix).c_str());
			RequireOctaves(config.m_octave_count, std::format("{}.octave_count", prefix).c_str());
			RequirePositive(config.m_lacunarity, std::format("{}.lacunarity", prefix).c_str());
			RequireFinite(config.m_persistence, std::format("{}.persistence", prefix).c_str());
			RequirePositive(config.m_roundness, std::format("{}.roundness", prefix).c_str());
			RequirePositive(config.m_weight_gain, std::format("{}.weight_gain", prefix).c_str());
		}

		// Validate the supported public baseline-surface contract before any sampling occurs.
		void Validate(BaselineSurfaceConfig const& config)
		{
			if (config.m_material_id < 0 || config.m_material_id >= Material::MaxMaterialId)
				throw std::invalid_argument(std::format("Terrain config 'material_id' must be in the range [0, {})", Material::MaxMaterialId));

			RequirePositive(config.m_supported_coordinate_abs_m, "supported_coordinate_abs_m");
			RequireFinite(config.m_sea_level_bias_m, "sea_level_bias_m");
			RequireFinite(config.m_uplift_height_m, "uplift_height_m");
			RequireFinite(config.m_mountain_base_height_m, "mountain_base_height_m");
			RequireFinite(config.m_basin_depth_m, "basin_depth_m");
			RequireFinite(config.m_basin_threshold, "basin_threshold");
			Validate(config.m_regional_base, "regional_base");
			Validate(config.m_region_selector, "region_selector");
			Validate(config.m_region_uplift, "region_uplift");
			Validate(config.m_domain_warp, "domain_warp");
			Validate(config.m_plains, "plains");
			Validate(config.m_hills, "hills");
			Validate(config.m_mountains, "mountains");
			Validate(config.m_basin_selector, "basin_selector");
		}

		// Require one query position to stay finite and within the documented supported range.
		void Validate(BaselineSurfaceConfig const& config, BaselineSurface::Position position_xy)
		{
			if (!std::isfinite(position_xy.x) || !std::isfinite(position_xy.y))
				throw std::invalid_argument("Terrain query coordinates must be finite");
			if (std::abs(position_xy.x) > config.m_supported_coordinate_abs_m || std::abs(position_xy.y) > config.m_supported_coordinate_abs_m)
				throw std::out_of_range("Terrain query coordinates exceed the supported absolute range");
		}

		// Copy a validated fractal band without quantization or a change to its seed.
		shared::BaselineBand Prepare(FractalConfig const& config, uint32_t seed)
		{
			return {
				.m_amplitude = config.m_amplitude, .m_wavelength_m = config.m_wavelength_m,
				.m_lacunarity = config.m_lacunarity, .m_persistence = config.m_persistence,
				.m_roundness = 0, .m_weight_gain = 0, .m_seed = seed, .m_octave_count = config.m_octave_count,
			};
		}
	}

	// Construct one immutable surface after validating its bounded configuration.
	BaselineSurface::BaselineSurface(BaselineSurfaceConfig config)
		: m_config(std::move(config))
		, m_recipe()
	{
		Validate(m_config);

		// Expand the public seed in the original field order and preserve every double-valued recipe parameter.
		m_recipe.m_fields[0] = Prepare(config.m_regional_base, MixSeed(config.m_seed, 1));
		m_recipe.m_fields[1] = Prepare(config.m_region_selector, MixSeed(config.m_seed, 2));
		m_recipe.m_fields[2] = Prepare(config.m_region_uplift, MixSeed(config.m_seed, 3));
		m_recipe.m_fields[3] = Prepare(config.m_plains, MixSeed(config.m_seed, 4));
		m_recipe.m_fields[4] = Prepare(config.m_hills, MixSeed(config.m_seed, 5));
		auto const& mountain = config.m_mountains;
		m_recipe.m_fields[5] = Prepare({mountain.m_amplitude, mountain.m_wavelength_m, mountain.m_octave_count, mountain.m_lacunarity, mountain.m_persistence}, MixSeed(config.m_seed, 6));
		m_recipe.m_fields[5].m_roundness = mountain.m_roundness;
		m_recipe.m_fields[5].m_weight_gain = mountain.m_weight_gain;
		auto const& warp = config.m_domain_warp;
		m_recipe.m_fields[6] = Prepare({warp.m_amplitude_m, warp.m_wavelength_m, warp.m_octave_count, warp.m_lacunarity, warp.m_persistence}, MixSeed(config.m_seed, 7));
		m_recipe.m_fields[7] = m_recipe.m_fields[6];
		m_recipe.m_fields[7].m_seed = MixSeed(config.m_seed, 8);
		m_recipe.m_fields[8] = Prepare(config.m_basin_selector, MixSeed(config.m_seed, 9));
		m_recipe.m_supported_coordinate_abs_m = config.m_supported_coordinate_abs_m;
		m_recipe.m_sea_level_bias_m = config.m_sea_level_bias_m;
		m_recipe.m_uplift_height_m = config.m_uplift_height_m;
		m_recipe.m_mountain_base_height_m = config.m_mountain_base_height_m;
		m_recipe.m_basin_depth_m = config.m_basin_depth_m;
		m_recipe.m_basin_threshold = config.m_basin_threshold;
		m_recipe.m_material_id = config.m_material_id;
	}

	// Return the validated immutable configuration that defines this surface.
	BaselineSurfaceConfig const& BaselineSurface::Config() const noexcept
	{
		return m_config;
	}

	// Return the immutable upload representation without allocating or creating a GPU context.
	shared::BaselineRecipe const& BaselineSurface::Recipe() const noexcept
	{
		return m_recipe;
	}

	// Prepare a single-precision recipe that samples positions relative to 'origin_xy'.
	shared::BaselineLocalRecipe BaselineSurface::LocalRecipe(Position origin_xy, double radius_m) const
	{
		// The sampled square must lie in the range that the CPU surface supports.
		if (!std::isfinite(origin_xy.x) || !std::isfinite(origin_xy.y))
			throw std::invalid_argument("Terrain local recipe origin must be finite");

		RequirePositive(radius_m, "local recipe radius_m");
		if (std::abs(origin_xy.x) + radius_m > m_config.m_supported_coordinate_abs_m || std::abs(origin_xy.y) + radius_m > m_config.m_supported_coordinate_abs_m)
			throw std::out_of_range("Terrain local recipe exceeds the supported absolute range");

		// Bound the domain warp offset. Each noise octave is within [-2, 2], so this bound is conservative.
		auto const& warp = m_recipe.m_fields[6];
		auto warp_reach_m = 0.0;
		auto warp_amplitude = warp.m_amplitude;
		for (int octave = 0; octave != warp.m_octave_count; ++octave)
		{
			warp_reach_m += 2 * std::abs(warp_amplitude);
			warp_amplitude *= warp.m_persistence;
		}
		auto const reach_m = std::max(std::abs(origin_xy.x), std::abs(origin_xy.y)) + radius_m + warp_reach_m;

		// Repeat BaselineFractal's double-precision frequency, amplitude and slice sequences, and split the origin's lattice phase at each octave.
		auto local = shared::BaselineLocalRecipe{};
		for (int field = 0; field != _countof(m_recipe.m_fields); ++field)
		{
			// Only the mountain band is ridged; see BaselineEvaluate.
			auto const& band = m_recipe.m_fields[field];
			auto& out = local.m_fields[field];
			auto const ridged = field == 5;
			out.m_roundness = static_cast<float>(band.m_roundness);
			out.m_weight_gain = static_cast<float>(band.m_weight_gain);
			out.m_seed = band.m_seed;
			out.m_octave_count = band.m_octave_count;
			auto frequency = double(1) / band.m_wavelength_m;
			auto amplitude = band.m_amplitude;
			auto slice = ridged
				? 0.38196601125010515 + static_cast<double>(band.m_seed) * (double(1) / double(65539))
				: 0.17320508075688773 + static_cast<double>(band.m_seed) * (double(1) / double(65537));
			for (int octave = 0; octave != band.m_octave_count; ++octave)
			{
				// Every lattice cell the shader can reach must fit in a 32-bit integer.
				if (!(reach_m * frequency < 1073741824.0))
					throw std::out_of_range("Terrain local recipe lattice coordinates exceed 32 bits");

				// Split the origin and slice into exact integer cells and single-precision fractions. A fraction that rounds up to one moves to the next cell.
				auto const z = slice + octave * (ridged ? 0.4142135623730950 : 0.6180339887498949);
				auto const cell_x = std::floor(origin_xy.x * frequency);
				auto const cell_y = std::floor(origin_xy.y * frequency);
				auto cell_z = std::floor(z);
				auto frac_z = static_cast<float>(z - cell_z);
				if (frac_z == 1.0f)
				{
					cell_z += 1;
					frac_z = 0.0f;
				}
				out.m_octaves[octave] = {
					.m_frequency = static_cast<float>(frequency),
					.m_amplitude = static_cast<float>(amplitude),
					.m_origin_cell_x = static_cast<int>(cell_x),
					.m_origin_cell_y = static_cast<int>(cell_y),
					.m_origin_frac_x = static_cast<float>(origin_xy.x * frequency - cell_x),
					.m_origin_frac_y = static_cast<float>(origin_xy.y * frequency - cell_y),
					.m_cell_z = static_cast<int>(cell_z),
					.m_frac_z = frac_z,
				};
				frequency *= band.m_lacunarity;
				amplitude *= band.m_persistence;
			}
		}

		// The height datum and blend constants are small, so single precision keeps them within the shader's rounding.
		local.m_sea_level_bias_m = static_cast<float>(m_recipe.m_sea_level_bias_m);
		local.m_uplift_height_m = static_cast<float>(m_recipe.m_uplift_height_m);
		local.m_mountain_base_height_m = static_cast<float>(m_recipe.m_mountain_base_height_m);
		local.m_basin_depth_m = static_cast<float>(m_recipe.m_basin_depth_m);
		local.m_basin_threshold = static_cast<float>(m_recipe.m_basin_threshold);
		local.m_material_id = m_recipe.m_material_id;
		return local;
	}

	// Sample one world-space XY position and return its double-precision terrain result.
	SurfaceSample BaselineSurface::Sample(Position position_xy) const
	{
		Validate(m_config, position_xy);
		auto const sample = shared::BaselineEvaluate(m_recipe, position_xy);
		if (sample.m_status != 0)
			throw std::runtime_error("Terrain evaluation exceeds supported double-precision arithmetic");

		auto result = SurfaceSample{.m_height = sample.m_height, .m_gradient_xy = v2d{sample.m_dx, sample.m_dy}, .m_material_id = sample.m_material_id};
		if (!m_decals ||
			position_xy.x < m_decals->m_lower.x || position_xy.x > m_decals->m_upper.x ||
			position_xy.y < m_decals->m_lower.y || position_xy.y > m_decals->m_upper.y)
			return result;

		// Combine every decal listed in the sample's cell; decals that do not reach the sample add nothing.
		auto const& cells = m_decals->m_cells;
		auto const cell_x = DecalCells::Cell(position_xy.x, cells.m_origin.x, cells.m_cell_size, cells.m_cells_x);
		auto const cell_y = DecalCells::Cell(position_xy.y, cells.m_origin.y, cells.m_cell_size, cells.m_cells_y);
		auto const cell = size_t(cell_y) * cells.m_cells_x + cell_x;
		auto blend = shared::BaselineDecalBlendInit();
		for (auto i = cells.m_cells[cell * 2], end = i + cells.m_cells[cell * 2 + 1]; i != end; ++i)
		{
			auto const& decal = m_decals->m_decals[cells.m_indices[i]];
			blend = shared::BaselineDecalAccumulate(blend, position_xy.x - decal.m_centre.x, position_xy.y - decal.m_centre.y, decal.m_pad_radius, decal.m_blend_radius, decal.m_height);
		}

		// Replace the height and gradient with the decorated field. The material belongs to the undecorated surface.
		auto const field = shared::BaselineDecalResolve(blend, shared::BaselineField(result.m_height, result.m_gradient_xy.x, result.m_gradient_xy.y));
		result.m_height = field.x;
		result.m_gradient_xy = v2d{field.y, field.z};
		return result;
	}

	// Return a copy of this surface whose decals are 'decals'.
	BaselineSurface BaselineSurface::WithDecals(std::span<TerrainDecal const> decals) const
	{
		// Every decal must be well formed and inside the supported range, so samples and local blobs can rely on them.
		if (decals.size() > size_t(MaxDecalCount))
			throw std::invalid_argument(std::format("Terrain decal count must be at most {}", MaxDecalCount));
		for (auto const& decal : decals)
		{
			if (!std::isfinite(decal.m_centre.x) || !std::isfinite(decal.m_centre.y) || !std::isfinite(decal.m_pad_radius) || !std::isfinite(decal.m_blend_radius) || !std::isfinite(decal.m_height))
				throw std::invalid_argument("Terrain decal values must be finite");
			if (!(decal.m_pad_radius >= 0 && decal.m_pad_radius < decal.m_blend_radius))
				throw std::invalid_argument("Terrain decal radii must satisfy 0 <= pad radius < blend radius");
			if (std::abs(decal.m_centre.x) + decal.m_blend_radius > m_config.m_supported_coordinate_abs_m || std::abs(decal.m_centre.y) + decal.m_blend_radius > m_config.m_supported_coordinate_abs_m)
				throw std::invalid_argument("Terrain decal extends beyond the supported absolute range");
		}

		// Share the recipe and replace only the decal set. An empty set leaves the surface undecorated.
		auto surface = *this;
		surface.m_decals = nullptr;
		if (decals.empty())
			return surface;

		// Index the decals in world space. The double-precision grid needs no margin because samples use the same arithmetic as the bounds.
		auto grid = std::make_shared<DecalGrid>();
		grid->m_decals.assign(decals.begin(), decals.end());
		grid->m_cells = BuildDecalCells(grid->m_decals, v2d{0, 0}, 0.0, false);
		grid->m_lower = v2d{+std::numeric_limits<double>::infinity(), +std::numeric_limits<double>::infinity()};
		grid->m_upper = v2d{-std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
		for (auto const& decal : grid->m_decals)
		{
			grid->m_lower = v2d{std::min(grid->m_lower.x, decal.m_centre.x - decal.m_blend_radius), std::min(grid->m_lower.y, decal.m_centre.y - decal.m_blend_radius)};
			grid->m_upper = v2d{std::max(grid->m_upper.x, decal.m_centre.x + decal.m_blend_radius), std::max(grid->m_upper.y, decal.m_centre.y + decal.m_blend_radius)};
		}
		surface.m_decals = std::move(grid);
		return surface;
	}

	// Return this surface's decals in the order given to WithDecals.
	std::span<TerrainDecal const> BaselineSurface::Decals() const noexcept
	{
		if (!m_decals)
			return {};

		return m_decals->m_decals;
	}

	// Prepare the single-precision decal blob for BaselineApplyDecalsLocal.
	std::vector<std::byte> BaselineSurface::LocalDecals(Position origin_xy) const
	{
		if (!std::isfinite(origin_xy.x) || !std::isfinite(origin_xy.y))
			throw std::invalid_argument("Terrain local decal origin must be finite");

		// A surface without decals has a header-only blob with a zero decal count.
		auto header = shared::BaselineDecalLocalHeader{};
		if (!m_decals)
		{
			auto blob = std::vector<std::byte>(sizeof(header));
			memcpy(blob.data(), &header, sizeof(header));
			return blob;
		}

		// The shader's local positions and the records are rounded to single precision, so enlarge each decal's cells by a margin well beyond
		// that rounding. A few float ULPs of the largest local coordinate is enough; the extra 0.1 mm covers decals near the origin.
		auto const& decals = m_decals->m_decals;
		auto max_abs_m = 0.0;
		for (auto const& decal : decals)
			max_abs_m = std::max({max_abs_m, std::abs(decal.m_centre.x - origin_xy.x) + decal.m_blend_radius, std::abs(decal.m_centre.y - origin_xy.y) + decal.m_blend_radius});

		auto const margin = max_abs_m * (1.0 / 65536.0) + 1e-4;
		auto const cells = BuildDecalCells(decals, origin_xy, margin, true);

		// Lay out the header, cells, index list and records as consecutive 32-bit words.
		header.m_decal_count = static_cast<uint32_t>(decals.size());
		header.m_cells_x = cells.m_cells_x;
		header.m_cells_y = cells.m_cells_y;
		header.m_cell_size = static_cast<float>(cells.m_cell_size);
		header.m_grid_origin_x = static_cast<float>(cells.m_origin.x);
		header.m_grid_origin_y = static_cast<float>(cells.m_origin.y);
		header.m_indices_offset = static_cast<uint32_t>(shared::BaselineDecalHeaderSize + cells.m_cells.size() * sizeof(uint32_t));
		header.m_decals_offset = static_cast<uint32_t>(header.m_indices_offset + cells.m_indices.size() * sizeof(uint32_t));
		auto blob = std::vector<std::byte>(header.m_decals_offset + decals.size() * shared::BaselineDecalRecordSize);
		memcpy(blob.data(), &header, sizeof(header));
		memcpy(blob.data() + shared::BaselineDecalHeaderSize, cells.m_cells.data(), cells.m_cells.size() * sizeof(uint32_t));
		memcpy(blob.data() + header.m_indices_offset, cells.m_indices.data(), cells.m_indices.size() * sizeof(uint32_t));

		// Records hold positions relative to the origin, so single precision keeps sub-millimetre accuracy near the origin.
		for (auto i = size_t{}; i != decals.size(); ++i)
		{
			auto const& decal = decals[i];
			auto const record = shared::BaselineDecalLocalRecord{
				.m_centre_x = static_cast<float>(decal.m_centre.x - origin_xy.x),
				.m_centre_y = static_cast<float>(decal.m_centre.y - origin_xy.y),
				.m_pad_radius = static_cast<float>(decal.m_pad_radius),
				.m_blend_radius = static_cast<float>(decal.m_blend_radius),
				.m_height = static_cast<float>(decal.m_height),
				.m_user = decal.m_user,
				.m_reserved0 = 0,
				.m_reserved1 = 0,
			};
			memcpy(blob.data() + header.m_decals_offset + i * shared::BaselineDecalRecordSize, &record, sizeof(record));
		}
		return blob;
	}

	// Sample a caller-owned batch in stable order without retaining caller buffers.
	void BaselineSurface::Sample(std::span<Position const> positions_xy, std::span<SurfaceSample> samples) const
	{
		if (positions_xy.size() != samples.size())
			throw std::invalid_argument("Terrain batch sampling requires equal input and output sizes");

		// Reuse the scalar evaluator so the batch boundary preserves exact scalar behaviour.
		for (auto index = size_t{}; index != positions_xy.size(); ++index)
			samples[index] = Sample(positions_xy[index]);
	}
}
