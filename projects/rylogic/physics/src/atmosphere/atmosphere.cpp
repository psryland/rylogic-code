//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/physics/atmosphere/atmosphere.h"
#include "pr/compute/compute_pso.h"
#include "pr/compute/compute_step.h"
#include "pr/compute/shaders/shader_compiler.h"
#include "pr/compute/utility/root_signature.h"
#include "src/utility/gpu.h"

namespace pr::physics::atmosphere
{
	using namespace pr::compute;

	namespace
	{
		// The kernels cover cells and all three face arrays with one conservative dispatch extent.
		constexpr auto AtmosphereThreadGroup = iv3{ 8, 8, 4 };

		// Largest column count along X and Y of the coarsest multigrid level. Must match ATMOSPHERE_COARSE_COLUMNS in atmosphere.hlsl,
		// because the fused small-level kernel uses it to find the coarsest level.
		constexpr auto MgCoarseColumns = 4;

		// Levels with at most this many columns along X and Y run in one fused thread group. Must match ATMOSPHERE_FUSED_COLUMNS in atmosphere.hlsl.
		constexpr auto MgFusedColumns = 32;

		// Thread group of the column kernels, which run one thread per vertical column. Must match ATMOSPHERE_COLUMN_THREAD_X/Y in atmosphere.hlsl.
		constexpr auto AtmosphereColumnThreadGroup = iv3{ 8, 8, 1 };

		// Largest supported layer count. Must match ATMOSPHERE_MAX_LAYERS in atmosphere.hlsl, which sizes the per-thread vertical solve arrays.
		constexpr auto MaxLayers = 32;
		constexpr auto MaxHeatSources = 64;
		constexpr auto AtmosphereTracerThreadGroup = 64;

		// Root constants shared by every atmosphere kernel. Must match CBufAtmosphere in atmosphere.hlsl, which documents each field.
		// HLSL constant packing does not let a vector cross a 16-byte boundary, so the field order keeps every vector inside one 16-byte row.
		struct CBufAtmosphere
		{
			// Grid shape.
			iv3 m_cell_count;              // fine-grid cell counts in X, Y (columns) and Z (layers)
			int m_boundary_mask;           // one bit per open side: bit 0 = x-, 1 = x+, 2 = y-, 3 = y+

			// Domain placement.
			v2 m_origin;                   // world-space XY of the low corner of cell (0,0), metres
			float m_lid_z;                 // world-space Z of the flat domain top, metres
			float m_dx;                    // horizontal cell size in both X and Y, metres

			// Step and layer shape.
			float m_dt;                    // duration of this solver step, seconds
			float m_gravity;               // positive gravitational acceleration, m/s^2
			float m_first_layer_thickness; // smallest allowed column height per layer, metres
			float m_layer_power;           // sigma stretch exponent

			// Reference temperature profile: T_ref(z) = max(temp0 + lapse * z, min_temp).
			float m_temp0;                 // reference temperature at world Z = 0, K
			float m_lapse;                 // change in reference temperature per metre of height, K/m
			float m_min_temp;              // lower clamp for the reference temperature, K

			// Floor and lid heat exchange.
			float m_floor_exchange_rate;   // lowest-layer relaxation rate toward the floor temperature, 1/s
			float m_lid_temperature;       // top-layer relaxation target, K
			float m_lid_relaxation_rate;   // top-layer relaxation rate; zero disables it, 1/s

			// Caller-supplied buffers.
			int m_source_count;            // valid heat sources
			int m_use_floor_temp_buffer;   // non-zero when the floor temperature buffer has one value per column

			// Multigrid level selection. The child (next coarser) level is derived from these in the shader.
			iv2 m_mg_size;                 // column counts of the active level
			int m_mg_offset;               // index of the active level's first cell in the packed pressure buffers
			int m_mg_scale;                // fine-grid columns per active-level column
			int m_mg_phase;                // red-black colour for smoothing, or the pressure-normalisation pass

			// Open edges and swirl restoration.
			int m_open_edge_band;          // columns over which the outside wind is blended in
			float m_vorticity_confinement; // swirl-restoring strength; zero disables it, 1/s

			// Fused small-level V-cycle.
			int m_mg_passes;               // red-black smoothing passes on the coarsest level
			uint32_t m_mg_smooth;          // red-black smoothing passes on each level: before restriction in the low 16 bits, after prolongation in the high 16 bits

			// Vertical mixing of horizontal momentum.
			float m_vertical_viscosity;    // vertical eddy viscosity, m^2/s

			// Solid columns.
			float m_origin_z;              // world-space Z of the flat floor used for the layer heights of solid columns, metres

			// Wall drag coefficients as pairs of 16-bit floats, low side in the low half. Packing keeps the root signature within 64 DWORDs.
			uint32_t m_drag_x;
			uint32_t m_drag_y;
			uint32_t m_drag_z;
		};
		static_assert(sizeof(CBufAtmosphere) == 34 * sizeof(uint32_t));
		static_assert(offsetof(CBufAtmosphere, m_origin) == 16 && offsetof(CBufAtmosphere, m_source_count) == 72);
		static_assert(offsetof(CBufAtmosphere, m_mg_size) == 80 && offsetof(CBufAtmosphere, m_mg_phase) == 96 && offsetof(CBufAtmosphere, m_mg_passes) == 108 && offsetof(CBufAtmosphere, m_mg_smooth) == 112);

		// Root constants shared by tracer kernels. Must match CBufAtmosphereTracers in atmosphere.hlsl.
		struct CBufAtmosphereTracers
		{
			int m_tracer_count;
			uint32_t m_tracer_seed;
			uint32_t m_tracer_frame;
			float m_tracer_max_age;
			float m_tracer_ground_density; // normalised tracer height profile. See AtmosphereTracerConfig
			float m_tracer_break_density;
			float m_tracer_upper_density;
			float m_tracer_break_height;
		};
		static_assert(sizeof(CBufAtmosphereTracers) == 8 * sizeof(uint32_t));

		// GPU heat source layout. Must match HeatSource in atmosphere.hlsl.
		struct GpuHeatSource
		{
			v4 m_centre;
			float m_radius;
			float m_heating_rate;
			float m_target_temperature;
			float m_relaxation_rate;
		};

		// GPU outside-air layout. Must match OutsideAir in atmosphere.hlsl.
		struct GpuOutsideAir
		{
			v2 m_wind;
			float m_temperature_offset;
			float m_pad;
		};

		// GPU tracer particle layout. Must match TracerParticle in atmosphere.hlsl.
		struct GpuTracerParticle
		{
			v4 m_position;
			float m_temperature;
			float m_age;
			float m_speed;
			float m_pad;
		};

		// Create a structured UAV buffer in the requested default state.
		template <typename Type> D3DPtr<ID3D12Resource> CreateBuffer(Gpu& gpu, GpuJob& job, int count, char const* name)
		{
			// All solver fields are plain structured buffers used by raw GPU virtual address root descriptors.
			auto desc = ResDesc::Buf<Type>(count, {}).usage(EUsage::UnorderedAccess).def_state(D3D12_RESOURCE_STATE_UNORDERED_ACCESS);
			return gpu.CreateResource(desc, job.m_cmd_list, name);
		}

		// Return one valid placeholder element because D3D12 root SRVs need a non-null resource address.
		template <typename Type> D3DPtr<ID3D12Resource> CreateSrvSentinel(Gpu& gpu, GpuJob& job, char const* name)
		{
			// Empty caller spans bind to this harmless one-element buffer.
			auto desc = ResDesc::Buf<Type>(1, {}).def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE);
			return gpu.CreateResource(desc, job.m_cmd_list, name);
		}

		// Pack two non-negative drag coefficients as 16-bit floats, 'lo' in the low half, to match UnpackDragPair in atmosphere.hlsl.
		uint32_t PackDragPair(float lo, float hi)
		{
			// Half precision is ample for drag coefficients, which are small tuning values.
			return static_cast<uint32_t>(math::F32toF16(lo)) | (static_cast<uint32_t>(math::F32toF16(hi)) << 16);
		}

		// Convert a public boundary enum to a packed open-boundary bit.
		int BoundaryValue(EAtmosphereBoundary boundary)
		{
			// Keep the switch exhaustive so new boundary modes cannot silently inherit an old shader meaning.
			switch (boundary)
			{
			case EAtmosphereBoundary::Solid:
			{
				return 0;
			}
			case EAtmosphereBoundary::Open:
			{
				return 1;
			}
			default:
			{
				throw std::invalid_argument("Unknown atmosphere boundary type");
			}
			}
		}

		// Return the column used for a slope sample, matching the shader's MetricNeighbour. Edge and solid neighbours fall back to 'cell'
		// itself, giving a one-sided difference.
		iv2 MetricNeighbour(AtmosphereGrid const& grid, iv2 cell, iv2 neighbour)
		{
			// Clamp to the grid, then reject solid columns because they hold no air.
			auto const n = iv2{
				std::clamp(neighbour.x, 0, grid.m_cell_count.x - 1),
				std::clamp(neighbour.y, 0, grid.m_cell_count.y - 1),
			};
			return grid.ColumnSolid(n) ? cell : n;
		}

		// Return the W-face value, deriving floor faces of air columns from the ground slope. Matches LoadW in atmosphere.hlsl.
		float LoadW(AtmosphereGrid const& grid, AtmosphereState const& state, iv3 face)
		{
			// Interior and lid faces are stored directly.
			if (face.z != 0 || grid.ColumnSolid(iv2{ face.x, face.y }))
				return state.m_w_faces[grid.WFaceIndex(face)];

			// Air at the floor moves along the ground, so its vertical speed is the bottom-layer horizontal velocity times the floor slope.
			auto const c = iv2{ face.x, face.y };
			auto const x0 = MetricNeighbour(grid, c, c + iv2{ -1, 0 });
			auto const x1 = MetricNeighbour(grid, c, c + iv2{ +1, 0 });
			auto const y0 = MetricNeighbour(grid, c, c + iv2{ 0, -1 });
			auto const y1 = MetricNeighbour(grid, c, c + iv2{ 0, +1 });
			auto const sx = (grid.FloorHeight(x1) - grid.FloorHeight(x0)) / (grid.m_dx * (static_cast<float>(std::abs(x1.x - x0.x)) + 1.0e-6f));
			auto const sy = (grid.FloorHeight(y1) - grid.FloorHeight(y0)) / (grid.m_dx * (static_cast<float>(std::abs(y1.y - y0.y)) + 1.0e-6f));
			auto const u = 0.5f * (state.m_u_faces[grid.UFaceIndex(iv3{ c.x, c.y, 0 })] + state.m_u_faces[grid.UFaceIndex(iv3{ c.x + 1, c.y, 0 })]);
			auto const v = 0.5f * (state.m_v_faces[grid.VFaceIndex(iv3{ c.x, c.y, 0 })] + state.m_v_faces[grid.VFaceIndex(iv3{ c.x, c.y + 1, 0 })]);
			return u * sx + v * sy;
		}
	}

	// Return true when every outside face is a solid wall.
	bool AtmosphereBoundaries::AllSolid() const
	{
		// A fully closed domain uses a pure-Neumann pressure solve.
		return m_x_min == EAtmosphereBoundary::Solid && m_x_max == EAtmosphereBoundary::Solid && m_y_min == EAtmosphereBoundary::Solid && m_y_max == EAtmosphereBoundary::Solid && m_z_min == EAtmosphereBoundary::Solid && m_z_max == EAtmosphereBoundary::Solid;
	}

	// Reject unknown boundary values at the caller boundary.
	void AtmosphereBoundaries::Validate() const
	{
		// Each value is passed through the conversion switch so invalid enum casts fail before GPU work is recorded.
		(void)BoundaryValue(m_x_min);
		(void)BoundaryValue(m_x_max);
		(void)BoundaryValue(m_y_min);
		(void)BoundaryValue(m_y_max);
		(void)BoundaryValue(m_z_min);
		(void)BoundaryValue(m_z_max);
	}

	// Reject invalid coefficients, and drag on sides that are not solid, at the caller boundary.
	void AtmosphereWallDrag::Validate(AtmosphereBoundaries const& boundaries) const
	{
		// Each side is checked with its boundary mode, because air leaving through an open side has no wall to rub against.
		auto check = [](float drag, EAtmosphereBoundary boundary)
		{
			// A coefficient must be a finite number in [0, 1], and only a wall can have drag. The GPU stores coefficients as 16-bit floats.
			if (!std::isfinite(drag) || drag < 0.0f || drag > 1.0f)
				throw std::invalid_argument("Atmosphere wall drag must be in the range [0, 1]");
			if (drag != 0.0f && boundary != EAtmosphereBoundary::Solid)
				throw std::invalid_argument("Atmosphere wall drag is only allowed on solid sides");
		};
		check(m_x_min, boundaries.m_x_min);
		check(m_x_max, boundaries.m_x_max);
		check(m_y_min, boundaries.m_y_min);
		check(m_y_max, boundaries.m_y_max);
		check(m_z_min, boundaries.m_z_min);
		check(m_z_max, boundaries.m_z_max);
	}

	// Reject invalid tracer configuration at the caller boundary.
	void AtmosphereTracerConfig::Validate() const
	{
		// Tracer buffers are optional, but a created tracer set needs a finite positive lifetime.
		if (m_particle_count < 0)
			throw std::invalid_argument("Atmosphere tracer count cannot be negative");
		if (!std::isfinite(m_max_age) || m_max_age <= 0.0f)
			throw std::invalid_argument("Atmosphere tracer lifetime must be finite and positive");
		if (!std::isfinite(m_ground_density) || !std::isfinite(m_break_density) || !std::isfinite(m_upper_density) || m_ground_density < 0.0f || m_break_density < 0.0f || m_upper_density < 0.0f)
			throw std::invalid_argument("Atmosphere tracer height densities must be finite and non-negative");
		if (!std::isfinite(m_break_height) || m_break_height <= 0.0f || m_break_height > 1.0f)
			throw std::invalid_argument("Atmosphere tracer break height must be in (0, 1]");
		if (!(0.5f * m_break_height * (m_ground_density + m_break_density) + (1.0f - m_break_height) * m_upper_density > 0.0f))
			throw std::invalid_argument("Atmosphere tracer height profile must have a positive total density");
	}

	// Build one floor height per column from row-major caller data.
	std::vector<float> AtmosphereGrid::BuildFloorHeights(iv2 cell_count, std::span<float const> floor_heights)
	{
		// Copy the caller buffer so later solver work does not depend on caller-owned memory.
		auto const column_count = cell_count.x * cell_count.y;
		if (cell_count.x <= 1 || cell_count.y <= 1)
			throw std::invalid_argument("Atmosphere floor grid dimensions must be greater than one");
		if (isize(floor_heights) != column_count)
			throw std::invalid_argument("Atmosphere floor buffer must contain one height per column");

		// Validate every height at the trust boundary because shader coordinates depend on them.
		auto result = std::vector<float>(floor_heights.begin(), floor_heights.end());
		for (auto floor_height : result)
		{
			if (!std::isfinite(floor_height))
				throw std::invalid_argument("Atmosphere floor heights must be finite");
		}
		return result;
	}

	// Build one floor height per column by sampling a caller-owned floor-height function at cell centres.
	std::vector<float> AtmosphereGrid::BuildFloorHeights(iv2 cell_count, v4 origin, float dx, std::function<float(v2)> const& floor_height_function)
	{
		// Validate the sampling grid before calling the supplied function.
		if (cell_count.x <= 1 || cell_count.y <= 1)
			throw std::invalid_argument("Atmosphere floor grid dimensions must be greater than one");
		if (!std::isfinite(dx) || dx <= 0.0f || !IsFinite(origin))
			throw std::invalid_argument("Atmosphere floor builder received invalid domain values");
		if (!floor_height_function)
			throw std::invalid_argument("Atmosphere floor builder needs a floor-height function");

		// Sample the function at every column centre and store the returned height directly.
		auto floors = std::vector<float>(cell_count.x * cell_count.y, 0.0f);
		for (int y = 0; y != cell_count.y; ++y)
		{
			// Rows share the same Y centre.
			for (int x = 0; x != cell_count.x; ++x)
			{
				// The caller owns all domain-specific rules used to choose the floor height.
				auto const xy = v2{ origin.x + (x + 0.5f) * dx, origin.y + (y + 0.5f) * dx };
				auto const floor_height = floor_height_function(xy);
				if (!std::isfinite(floor_height))
					throw std::invalid_argument("Atmosphere floor-height function returned a non-finite value");

				floors[y * cell_count.x + x] = floor_height;
			}
		}
		return floors;
	}

	// Return true when the grid uses a caller-supplied floor per column.
	bool AtmosphereGrid::TerrainFollowing() const
	{
		// A non-empty floor vector is the only supported terrain-following mode.
		return !m_floor_heights.empty();
	}

	// Return the total number of cell-centred samples.
	int AtmosphereGrid::CellCount() const
	{
		// Multiplication is valid after Validate has rejected negative dimensions.
		return m_cell_count.x * m_cell_count.y * m_cell_count.z;
	}

	// Return the number of x-face velocity samples in the MAC grid.
	int AtmosphereGrid::UFaceCount() const
	{
		// U samples lie on the two x walls and every interior x face.
		return (m_cell_count.x + 1) * m_cell_count.y * m_cell_count.z;
	}

	// Return the number of y-face velocity samples in the MAC grid.
	int AtmosphereGrid::VFaceCount() const
	{
		// V samples lie on the two y walls and every interior y face.
		return m_cell_count.x * (m_cell_count.y + 1) * m_cell_count.z;
	}

	// Return the number of z-face velocity samples in the MAC grid.
	int AtmosphereGrid::WFaceCount() const
	{
		// W samples lie on the floor, lid, and every interior z face.
		return m_cell_count.x * m_cell_count.y * (m_cell_count.z + 1);
	}

	// Return the number of horizontal columns.
	int AtmosphereGrid::ColumnCount() const
	{
		// Columns are the row-major XY part of the grid.
		return m_cell_count.x * m_cell_count.y;
	}

	// Return the packed column index for 'cell'.
	int AtmosphereGrid::ColumnIndex(iv2 cell) const
	{
		// Public column arrays use row-major XY order.
		return cell.y * m_cell_count.x + cell.x;
	}

	// Return the number of boundary columns.
	int AtmosphereGrid::BoundaryColumnCount() const
	{
		// Each x side has one column per row, and each y side has one column per x position.
		return 2 * (m_cell_count.x + m_cell_count.y);
	}

	// Build outside air for every boundary column by sampling a caller-owned function at the centre of each column's outside face.
	std::vector<AtmosphereOutsideAir> AtmosphereGrid::BuildOutsideAir(std::function<AtmosphereOutsideAir(v2)> const& outside_air_function) const
	{
		// Validate the sampling function before calling it.
		if (!outside_air_function)
			throw std::invalid_argument("Atmosphere outside-air builder needs an outside-air function");

		// Sample in the packed side order used by the solver: x- by y, x+ by y, y- by x, then y+ by x.
		auto const lo = m_origin.xy;
		auto const hi = lo + v2{ m_cell_count.x * m_dx, m_cell_count.y * m_dx };
		auto result = std::vector<AtmosphereOutsideAir>{};
		result.reserve(BoundaryColumnCount());
		for (int y = 0; y != m_cell_count.y; ++y)
			result.push_back(outside_air_function(v2{ lo.x, lo.y + (y + 0.5f) * m_dx }));
		for (int y = 0; y != m_cell_count.y; ++y)
			result.push_back(outside_air_function(v2{ hi.x, lo.y + (y + 0.5f) * m_dx }));
		for (int x = 0; x != m_cell_count.x; ++x)
			result.push_back(outside_air_function(v2{ lo.x + (x + 0.5f) * m_dx, lo.y }));
		for (int x = 0; x != m_cell_count.x; ++x)
			result.push_back(outside_air_function(v2{ lo.x + (x + 0.5f) * m_dx, hi.y }));

		return result;
	}

	// Return the packed cell-centre index for 'cell'.
	int AtmosphereGrid::CellIndex(iv3 cell) const
	{
		// Public arrays use z-major row order for cell-centred values.
		return (cell.z * m_cell_count.y + cell.y) * m_cell_count.x + cell.x;
	}

	// Return the packed x-face index for 'face'.
	int AtmosphereGrid::UFaceIndex(iv3 face) const
	{
		// U rows have nx + 1 entries because they include both outside x faces.
		return (face.z * m_cell_count.y + face.y) * (m_cell_count.x + 1) + face.x;
	}

	// Return the packed y-face index for 'face'.
	int AtmosphereGrid::VFaceIndex(iv3 face) const
	{
		// V layers have ny + 1 rows because they include both outside y faces.
		return (face.z * (m_cell_count.y + 1) + face.y) * m_cell_count.x + face.x;
	}

	// Return the packed z-face index for 'face'.
	int AtmosphereGrid::WFaceIndex(iv3 face) const
	{
		// W arrays have nz + 1 layers because they include the floor and lid faces.
		return (face.z * m_cell_count.y + face.y) * m_cell_count.x + face.x;
	}

	// Return the floor height for a column.
	float AtmosphereGrid::FloorHeight(iv2 cell) const
	{
		// Uniform legacy boxes use the origin Z as their flat floor.
		return TerrainFollowing() ? m_floor_heights[ColumnIndex(cell)] : m_origin.z;
	}

	// Return true when a column holds no air.
	bool AtmosphereGrid::ColumnSolid(iv2 cell) const
	{
		// The solver treats every face of a column whose floor reaches the lid as a wall.
		return FloorHeight(cell) >= m_lid_z;
	}

	// Return the sigma face fraction for a vertical face index.
	float AtmosphereGrid::SigmaFace(int z) const
	{
		// A power below one packs faces near the floor while preserving the lid exactly.
		auto const raw = std::clamp(static_cast<float>(z) / std::max(1, m_cell_count.z), 0.0f, 1.0f);
		return std::pow(raw, m_layer_stretch_power);
	}

	// Return the world-space height of a sigma face in one column.
	float AtmosphereGrid::FaceZ(iv2 cell, int z) const
	{
		// The lid is flat, so column height is the difference between the lid and the local floor. The height is guarded by the
		// requested first-layer thickness, matching the shader metric, so very thin and solid columns still have positive layer heights.
		auto const floor_height = FloorHeight(cell);
		auto const column_height = std::max(m_lid_z - floor_height, m_first_layer_thickness * std::max(m_cell_count.z, 1));
		return floor_height + SigmaFace(z) * column_height;
	}

	// Return the world-space centre height of a cell in one column.
	float AtmosphereGrid::CellZ(iv2 cell, int z) const
	{
		// Cell centres are midpoints in physical height, not only in sigma, which keeps CPU diagnostics aligned with shader volumes.
		return 0.5f * (FaceZ(cell, z) + FaceZ(cell, z + 1));
	}

	// Return the physical height of one cell.
	float AtmosphereGrid::CellHeight(iv2 cell, int z) const
	{
		// Local layer thickness is used for volume-weighted remapping and divergence diagnostics.
		return FaceZ(cell, z + 1) - FaceZ(cell, z);
	}

	// Return the world-space centre of cell 'cell'.
	v4 AtmosphereGrid::CellCentre(iv3 cell) const
	{
		// Terrain-following columns keep X and Y regular while Z follows the local floor-to-lid mapping.
		return v4{ m_origin.x + (cell.x + 0.5f) * m_dx, m_origin.y + (cell.y + 0.5f) * m_dx, CellZ(iv2{ cell.x, cell.y }, cell.z), 1.0f };
	}

	// Reject invalid grid geometry at the caller boundary.
	void AtmosphereGrid::Validate() const
	{
		// A real 3D domain is required because the projection and plume tests need all axes.
		if (m_cell_count.x <= 1 || m_cell_count.y <= 1 || m_cell_count.z <= 1)
			throw std::invalid_argument("Atmosphere grid dimensions must all be greater than one");

		// The pressure smoother solves each column's layers with fixed-size per-thread arrays.
		if (m_cell_count.z > MaxLayers)
			throw std::invalid_argument(std::format("Atmosphere grid layer count must not exceed {}", MaxLayers));

		// Spacing and lid geometry define the metric used by advection, divergence and pressure projection.
		if (!std::isfinite(m_dx) || m_dx <= 0.0f)
			throw std::invalid_argument("Atmosphere horizontal cell size must be finite and positive");
		if (!std::isfinite(m_lid_z))
			throw std::invalid_argument("Atmosphere lid height must be finite");
		if (!std::isfinite(m_first_layer_thickness) || m_first_layer_thickness <= 0.0f)
			throw std::invalid_argument("Atmosphere first layer thickness must be finite and positive");
		if (!std::isfinite(m_layer_stretch_power) || m_layer_stretch_power <= 0.0f)
			throw std::invalid_argument("Atmosphere layer stretch power must be finite and positive");
		if (!IsFinite(m_origin))
			throw std::invalid_argument("Atmosphere origin must be finite");
		if (!m_floor_heights.empty() && isize(m_floor_heights) != ColumnCount())
			throw std::invalid_argument("Atmosphere floor height count must match the grid columns");

		// Every floor must be finite. A floor at or above the lid marks a solid column. See ColumnSolid.
		for (auto floor_height : m_floor_heights)
		{
			if (!std::isfinite(floor_height))
				throw std::invalid_argument("Atmosphere floor heights must be finite");
		}
	}

	// Return the reference temperature at world height 'z'.
	float AtmosphereReferenceProfile::Temperature(float z) const
	{
		// The lower bound keeps buoyancy finite for caller-selected lapse profiles.
		return std::max(m_min_temperature, m_temperature_at_origin + m_lapse_rate * z);
	}

	// Reject invalid reference-profile values at the caller boundary.
	void AtmosphereReferenceProfile::Validate() const
	{
		// The reference profile is in Kelvin and appears in the buoyancy denominator.
		if (!std::isfinite(m_temperature_at_origin) || m_temperature_at_origin <= 0.0f)
			throw std::invalid_argument("Atmosphere reference temperature must be finite and positive");
		if (!std::isfinite(m_lapse_rate))
			throw std::invalid_argument("Atmosphere lapse rate must be finite");
		if (!std::isfinite(m_min_temperature) || m_min_temperature <= 0.0f)
			throw std::invalid_argument("Atmosphere minimum reference temperature must be finite and positive");
	}

	// Reject invalid solver configuration at the caller boundary.
	void AtmosphereConfig::Validate() const
	{
		// Validate owned sub-objects first so error messages identify the failing contract.
		m_grid.Validate();
		m_boundaries.Validate();
		m_wall_drag.Validate(m_boundaries);
		m_reference.Validate();
		if (!std::isfinite(m_gravity) || m_gravity < 0.0f)
			throw std::invalid_argument("Atmosphere gravity must be finite and non-negative");
		if (!std::isfinite(m_floor_exchange_rate) || m_floor_exchange_rate < 0.0f)
			throw std::invalid_argument("Atmosphere floor exchange rate must be finite and non-negative");
		if (!std::isfinite(m_lid_temperature) || m_lid_temperature <= 0.0f)
			throw std::invalid_argument("Atmosphere lid temperature must be finite and positive");
		if (!std::isfinite(m_lid_relaxation_rate) || m_lid_relaxation_rate < 0.0f)
			throw std::invalid_argument("Atmosphere lid relaxation rate must be finite and non-negative");
		if (m_pressure_vcycles < 0)
			throw std::invalid_argument("Atmosphere pressure V-cycle count must be non-negative");
		if (m_pressure_pre_smooth < 0 || m_pressure_post_smooth < 0 || m_pressure_coarse_smooth < 0)
			throw std::invalid_argument("Atmosphere multigrid smoothing counts must be non-negative");
		if (m_pressure_pre_smooth > 0xFFFF || m_pressure_post_smooth > 0xFFFF)
			throw std::invalid_argument("Atmosphere multigrid pre- and post-smoothing counts must fit in 16 bits");
		if (m_open_edge_band < 0)
			throw std::invalid_argument("Atmosphere open-edge band must be non-negative");
		if (!std::isfinite(m_vorticity_confinement) || m_vorticity_confinement < 0.0f)
			throw std::invalid_argument("Atmosphere vorticity confinement must be finite and non-negative");
		if (!std::isfinite(m_vertical_viscosity) || m_vertical_viscosity < 0.0f)
			throw std::invalid_argument("Atmosphere vertical viscosity must be finite and non-negative");
	}


	// Private implementation kept out of the public header so shader plumbing can change without API churn.
	struct AtmosphereSolver::Impl
	{
		// Root parameter indices of the root layout shared by every atmosphere kernel.
		enum class ERootParam
		{
			Constants = 0,
			FieldsOut = 1,  // u0..u6
			FieldsIn = 8,   // t0..t7
		};

		// Buffers a kernel writes, used to emit UAV barriers only where a later kernel can read the result.
		enum class EFieldWrites
		{
			None = 0,
			Velocity = 1 << 0,
			Pressure = 1 << 1,
			Divergence = 1 << 2,
			Residual = 1 << 3,
			_flags_enum = 0,
		};

		Gpu& m_gpu;
		AtmosphereConfig m_config;
		D3DPtr<ID3D12RootSignature> m_sig;
		ID3D12PipelineState* m_bound_pso;
		ComputeStep m_initialise;
		ComputeStep m_advect;
		ComputeStep m_vorticity;
		ComputeStep m_forces_heat;
		ComputeStep m_divergence;
		ComputeStep m_mg_smooth;
		ComputeStep m_mg_small_levels;
		ComputeStep m_mg_residual;
		ComputeStep m_mg_restrict;
		ComputeStep m_mg_prolongate;
		ComputeStep m_normalise_pressure;
		ComputeStep m_project;
		D3DPtr<ID3D12Resource> m_u[2];
		D3DPtr<ID3D12Resource> m_v[2];
		D3DPtr<ID3D12Resource> m_w[2];
		D3DPtr<ID3D12Resource> m_temperature[2];
		D3DPtr<ID3D12Resource> m_pressure;
		D3DPtr<ID3D12Resource> m_divergence_buffer;
		D3DPtr<ID3D12Resource> m_residual_buffer;
		D3DPtr<ID3D12Resource> m_floor_temp_sentinel;
		D3DPtr<ID3D12Resource> m_source_sentinel;
		D3DPtr<ID3D12Resource> m_floor_height;
		D3DPtr<ID3D12Resource> m_outside_air_sentinel;

		// Description of one horizontally coarsened multigrid level.
		struct MgLevel
		{
			int m_nx;
			int m_ny;
			int m_offset;
			int m_scale;
		};

		std::vector<MgLevel> m_levels;
		int m_current;

		// Compile kernels and allocate persistent field buffers.
		Impl(Gpu& gpu, AtmosphereConfig config, IShaderCache* cache)
			: m_gpu(gpu)
			, m_config(std::move(config))
			, m_sig()
			, m_bound_pso()
			, m_initialise()
			, m_advect()
			, m_vorticity()
			, m_forces_heat()
			, m_divergence()
			, m_mg_smooth()
			, m_mg_small_levels()
			, m_mg_residual()
			, m_mg_restrict()
			, m_mg_prolongate()
			, m_normalise_pressure()
			, m_project()
			, m_u{}
			, m_v{}
			, m_w{}
			, m_temperature{}
			, m_pressure()
			, m_divergence_buffer()
			, m_residual_buffer()
			, m_floor_temp_sentinel()
			, m_source_sentinel()
			, m_floor_height()
			, m_outside_air_sentinel()
			, m_current(0)
		{
			// Validate once before any GPU resources are created.
			m_config.Validate();
			BuildMultigridLevels();
			auto resolver = shader_cache::ResourceSourceResolver{};
			auto compile = [&](wchar_t const* entry_point)
			{
				// Runtime compilation matches the other physics GPU modules and uses the embedded resource resolver. The optional cache skips DXC when the source is unchanged.
				return ShaderCompiler{}.Source("src/atmosphere/atmosphere.hlsl", resolver).Cache(cache).HlslVersion(EHlslVersion::Hlsl2021).Define(L"SHADER_BUILD").Optimise(true).ShaderModel(L"cs_6_6").EntryPoint(entry_point).Compile();
			};

			// Every kernel shares one root signature so the field views stay bound across pipeline changes within a step. The order must match ERootParam.
			m_sig = RootSig(ERootSigFlags::ComputeOnly)
				.U32<CBufAtmosphere>(hlsl::ECBufReg::b0)
				.UAV(hlsl::EUAVReg::u0).UAV(hlsl::EUAVReg::u1).UAV(hlsl::EUAVReg::u2).UAV(hlsl::EUAVReg::u3).UAV(hlsl::EUAVReg::u4).UAV(hlsl::EUAVReg::u5).UAV(hlsl::EUAVReg::u6)
				.SRV(hlsl::ESRVReg::t0).SRV(hlsl::ESRVReg::t1).SRV(hlsl::ESRVReg::t2).SRV(hlsl::ESRVReg::t3).SRV(hlsl::ESRVReg::t4).SRV(hlsl::ESRVReg::t5).SRV(hlsl::ESRVReg::t6).SRV(hlsl::ESRVReg::t7)
				.Create(gpu, "Physics.Atmosphere.RootSig");
			auto make_step = [&](wchar_t const* entry_point, char const* name)
			{
				// Each kernel only needs its own pipeline state on top of the shared root signature.
				auto step = ComputeStep{};
				auto code = compile(entry_point);
				auto pso_name = std::format("Physics.Atmosphere.{}.PSO", name);
				step.m_sig = m_sig;
				step.m_pso = ComputePSO(m_sig.get(), code).Create(gpu, pso_name.c_str());
				return step;
			};

			// Allocate buffers using the solver's job because resource creation can record initial transitions.
			m_initialise = make_step(L"CSInitialise", "Initialise");
			m_advect = make_step(L"CSAdvect", "Advect");
			m_vorticity = make_step(L"CSVorticity", "Vorticity");
			m_forces_heat = make_step(L"CSForcesHeat", "ForcesHeat");
			m_divergence = make_step(L"CSDivergence", "Divergence");
			m_mg_smooth = make_step(L"CSMgSmooth", "MgSmooth");
			m_mg_small_levels = make_step(L"CSMgSmallLevels", "MgSmallLevels");
			m_mg_residual = make_step(L"CSMgResidual", "MgResidual");
			m_mg_restrict = make_step(L"CSMgRestrict", "MgRestrict");
			m_mg_prolongate = make_step(L"CSMgProlongate", "MgProlongate");
			m_normalise_pressure = make_step(L"CSNormalisePressure", "NormalisePressure");
			m_project = make_step(L"CSProject", "Project");
			Allocate(gpu.m_job);
			InitialiseReference(gpu.m_job);
			gpu.m_job.RetireRecordedWork();
		}

		// Return the dispatch count for all cell and face grids.
		iv3 DispatchSize() const
		{
			// The extra element on each axis covers the high-side face arrays; kernels ignore coordinates outside each specific array.
			auto const& n = m_config.m_grid.m_cell_count;
			return DispatchCount(iv3{ n.x + 1, n.y + 1, n.z + 1 }, AtmosphereThreadGroup);
		}

		// Build semi-coarsened pressure levels used by the V-cycle.
		void BuildMultigridLevels()
		{
			// Only the horizontal axes are coarsened because 32 m columns with 5 m near-floor layers are strongly anisotropic; keeping all vertical layers on every level avoids amplifying the stiff vertical coupling with coarse vertical stencils.
			auto const& n = m_config.m_grid.m_cell_count;
			m_levels.clear();
			auto nx = n.x;
			auto ny = n.y;
			auto offset = 0;
			auto scale = 1;
			for (;;)
			{
				m_levels.push_back(MgLevel{ .m_nx = nx, .m_ny = ny, .m_offset = offset, .m_scale = scale });
				offset += nx * ny * n.z;
				if (nx <= MgCoarseColumns && ny <= MgCoarseColumns)
					break;

				nx = std::max(1, (nx + 1) / 2);
				ny = std::max(1, (ny + 1) / 2);
				scale *= 2;
			}
		}

		// Return the total cell slots needed by all pressure levels.
		int MultigridCellCount() const
		{
			// The pressure, residual, and right-hand side buffers share the same packed level layout.
			auto const& n = m_config.m_grid.m_cell_count;
			auto result = 0;
			for (auto const& level : m_levels)
				result += level.m_nx * level.m_ny * n.z;
			return result;
		}

		// Return dispatch counts for one multigrid level.
		iv3 LevelDispatch(MgLevel const& level) const
		{
			// Level kernels only touch pressure nodes on the selected level.
			return DispatchCount(iv3{ level.m_nx, level.m_ny, m_config.m_grid.m_cell_count.z }, AtmosphereThreadGroup);
		}

		// Build constants for a selected multigrid level. The shader derives the child level from these (see MgChildLevel in atmosphere.hlsl).
		CBufAtmosphere LevelConstants(CBufAtmosphere cb, int level_index, int phase) const
		{
			// Level metadata selects the packed pressure range and the floor metric scale for the shader.
			auto const& level = m_levels[level_index];
			cb.m_mg_phase = phase;
			cb.m_mg_size = iv2{ level.m_nx, level.m_ny };
			cb.m_mg_offset = level.m_offset;
			cb.m_mg_scale = level.m_scale;
			return cb;
		}

		// Dispatch one kernel over a selected multigrid level using the fields bound by BindFields.
		void DispatchLevel(GpuJob& job, ComputeStep& step, CBufAtmosphere const& cb, MgLevel const& level)
		{
			// Multigrid kernels share resources with the main step but use smaller dispatch extents on coarse levels.
			auto const dispatch = LevelDispatch(level);
			Run(job, step, cb, dispatch);
		}

		// Run red-black Gauss-Seidel smoothing on a pressure level.
		void SmoothLevel(GpuJob& job, CBufAtmosphere cb, int level_index, int passes)
		{
			// Each pass performs both colours so the result is independent of thread order within one colour.
			// The smoother runs one thread per column of the selected colour, so each row needs only half its columns' threads.
			auto const& level = m_levels[level_index];
			auto const dispatch = DispatchCount(iv3{ (level.m_nx + 1) / 2, level.m_ny, 1 }, AtmosphereColumnThreadGroup);
			for (int iter = 0; iter != passes; ++iter)
			{
				for (int phase = 0; phase != 2; ++phase)
				{
					// Each colour reads the other colour's latest pressures, so a barrier separates them.
					Run(job, m_mg_smooth, LevelConstants(cb, level_index, phase), dispatch);
					Barrier(job, EFieldWrites::Pressure);
				}
			}
		}

		// Run one recursive multigrid V-cycle from 'level_index'.
		void VCycle(GpuJob& job, CBufAtmosphere cb, int level_index)
		{
			// Coarse levels receive residual equations; the finest level starts with the physical divergence from the MAC field.
			auto const& level = m_levels[level_index];
			if (level.m_nx <= MgFusedColumns && level.m_ny <= MgFusedColumns)
			{
				// Small levels cannot fill the GPU, so the rest of the V-cycle runs in one thread group with group barriers between passes.
				auto level_cb = LevelConstants(cb, level_index, 0);
				level_cb.m_mg_passes = m_config.m_pressure_coarse_smooth;
				level_cb.m_mg_smooth = s_cast<uint32_t>(m_config.m_pressure_pre_smooth) | (s_cast<uint32_t>(m_config.m_pressure_post_smooth) << 16);
				Run(job, m_mg_small_levels, level_cb, iv3{ 1, 1, 1 });
				Barrier(job, EFieldWrites::Pressure | EFieldWrites::Residual | EFieldWrites::Divergence);
				return;
			}

			// Pre-smoothing damps cell-scale errors before residual restriction.
			SmoothLevel(job, cb, level_index, m_config.m_pressure_pre_smooth);
			DispatchLevel(job, m_mg_residual, LevelConstants(cb, level_index, 0), m_levels[level_index]);
			Barrier(job, EFieldWrites::Residual);
			DispatchLevel(job, m_mg_restrict, LevelConstants(cb, level_index, 0), m_levels[level_index + 1]); // dispatched over the child level's cells
			Barrier(job, EFieldWrites::Divergence | EFieldWrites::Pressure);

			// The coarse correction removes long wavelengths that local smoothing cannot reach on large grids.
			VCycle(job, cb, level_index + 1);
			DispatchLevel(job, m_mg_prolongate, LevelConstants(cb, level_index, 0), m_levels[level_index]);
			Barrier(job, EFieldWrites::Pressure);
			SmoothLevel(job, cb, level_index, m_config.m_pressure_post_smooth);
		}

		// Create all persistent buffers.
		void Allocate(GpuJob& job)
		{
			// The MAC layout stores normal velocity on faces and scalar values at cell centres.
			auto const& grid = m_config.m_grid;
			for (int slot = 0; slot != 2; ++slot)
			{
				m_u[slot] = CreateBuffer<float>(m_gpu, job, grid.UFaceCount(), slot == 0 ? "Atmosphere:U0" : "Atmosphere:U1");
				m_v[slot] = CreateBuffer<float>(m_gpu, job, grid.VFaceCount(), slot == 0 ? "Atmosphere:V0" : "Atmosphere:V1");
				m_w[slot] = CreateBuffer<float>(m_gpu, job, grid.WFaceCount(), slot == 0 ? "Atmosphere:W0" : "Atmosphere:W1");
				m_temperature[slot] = CreateBuffer<float>(m_gpu, job, grid.CellCount(), slot == 0 ? "Atmosphere:Temperature0" : "Atmosphere:Temperature1");
			}
			m_pressure = CreateBuffer<float>(m_gpu, job, MultigridCellCount(), "Atmosphere:Pressure");
			m_divergence_buffer = CreateBuffer<float>(m_gpu, job, MultigridCellCount(), "Atmosphere:Divergence");
			m_residual_buffer = CreateBuffer<float>(m_gpu, job, MultigridCellCount(), "Atmosphere:Residual");
			m_floor_temp_sentinel = CreateSrvSentinel<float>(m_gpu, job, "Atmosphere:FloorTemperatureSentinel");
			m_source_sentinel = CreateSrvSentinel<GpuHeatSource>(m_gpu, job, "Atmosphere:SourceSentinel");
			m_floor_height = m_gpu.CreateResource(ResDesc::Buf<float>(grid.ColumnCount(), {}).def_state(D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE), job.m_cmd_list, "Atmosphere:FloorHeight");
			m_outside_air_sentinel = CreateSrvSentinel<GpuOutsideAir>(m_gpu, job, "Atmosphere:OutsideAirSentinel");
			UploadFloors(job, grid.m_floor_heights.empty() ? FlatFloors() : grid.m_floor_heights);
		}

		// Return flat floors for a uniform-box configuration.
		std::vector<float> FlatFloors() const
		{
			// A missing floor array means every column starts at the grid origin height.
			return std::vector<float>(m_config.m_grid.ColumnCount(), m_config.m_grid.m_origin.z);
		}

		// Upload the floor-height buffer used by terrain-following shader metrics.
		void UploadFloors(GpuJob& job, std::span<float const> floor_heights)
		{
			// Floor heights change rarely, so a simple whole-buffer copy keeps state management clear.
			auto const& grid = m_config.m_grid;
			if (isize(floor_heights) != grid.ColumnCount())
				throw std::invalid_argument("Atmosphere floor height upload count must match the grid columns");

			job.m_barriers.Transition(m_floor_height.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto upload = job.m_upload.Alloc<float>(grid.ColumnCount());
			memcpy(upload.ptr<float>(), floor_heights.data(), floor_heights.size_bytes());
			job.m_cmd_list.CopyBufferRegion(m_floor_height.get(), 0, upload);
			job.m_barriers.Transition(m_floor_height.get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE).Commit();
		}

		// Build constants shared by all kernels.
		CBufAtmosphere Constants(float dt, AtmosphereStepSources const& sources, int pressure_iteration) const
		{
			// Pack scalar config into a 32-bit root constant block for cheap per-dispatch updates.
			auto const& grid = m_config.m_grid;
			auto const& boundaries = m_config.m_boundaries;
			return CBufAtmosphere{
				.m_cell_count = grid.m_cell_count,
				.m_boundary_mask = (BoundaryValue(boundaries.m_x_min) << 0) | (BoundaryValue(boundaries.m_x_max) << 1) | (BoundaryValue(boundaries.m_y_min) << 2) | (BoundaryValue(boundaries.m_y_max) << 3),
				.m_origin = grid.m_origin.xy,
				.m_lid_z = grid.m_lid_z,
				.m_dx = grid.m_dx,
				.m_dt = dt,
				.m_gravity = m_config.m_gravity,
				.m_first_layer_thickness = grid.m_first_layer_thickness,
				.m_layer_power = grid.m_layer_stretch_power,
				.m_temp0 = m_config.m_reference.m_temperature_at_origin,
				.m_lapse = m_config.m_reference.m_lapse_rate,
				.m_min_temp = m_config.m_reference.m_min_temperature,
				.m_floor_exchange_rate = m_config.m_floor_exchange_rate,
				.m_lid_temperature = m_config.m_lid_temperature,
				.m_lid_relaxation_rate = m_config.m_lid_relaxation_rate,
				.m_source_count = isize(sources.m_heat_sources),
				.m_use_floor_temp_buffer = sources.m_floor_temperatures.empty() ? 0 : 1,
				.m_mg_size = grid.m_cell_count.xy,
				.m_mg_offset = 0,
				.m_mg_scale = 1,
				.m_mg_phase = pressure_iteration,
				.m_open_edge_band = m_config.m_open_edge_band,
				.m_vorticity_confinement = m_config.m_vorticity_confinement,
				.m_mg_passes = 0,
				.m_mg_smooth = 0,
				.m_vertical_viscosity = m_config.m_vertical_viscosity,
				.m_origin_z = grid.m_origin.z,
				.m_drag_x = PackDragPair(m_config.m_wall_drag.m_x_min, m_config.m_wall_drag.m_x_max),
				.m_drag_y = PackDragPair(m_config.m_wall_drag.m_y_min, m_config.m_wall_drag.m_y_max),
				.m_drag_z = PackDragPair(m_config.m_wall_drag.m_z_min, m_config.m_wall_drag.m_z_max),
			};
		}

		// Transition the ping-pong fields for one 'src'/'dst' arrangement and bind the shared root signature and every field view.
		// Field views stay bound until the next call, so call this only when the arrangement changes. Run then only changes the pipeline and constants.
		void BindFields(GpuJob& job, int src, int dst, D3D12_GPU_VIRTUAL_ADDRESS floor_temp_buffer, D3D12_GPU_VIRTUAL_ADDRESS source_buffer, D3D12_GPU_VIRTUAL_ADDRESS outside_air_buffer)
		{
			// The ping-pong fields default to UAV state but 'src' is bound as root SRVs. The barrier batch tracks the current state so repeated transitions are dropped.
			// A UAV-to-SRV transition also orders the previous kernel's writes before the next kernel's reads, so velocity writes need no separate UAV barrier.
			assert(src != dst && "Ping-pong fields cannot be bound as both input and output");
			job.m_barriers
				.Transition(m_u[dst].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
				.Transition(m_v[dst].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
				.Transition(m_w[dst].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
				.Transition(m_temperature[dst].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
				.Transition(m_u[src].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
				.Transition(m_v[src].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
				.Transition(m_w[src].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
				.Transition(m_temperature[src].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
				.Commit();

			// Root descriptors use GPU virtual addresses, so no descriptor heap pressure is added by the solver.
			// Setting the root signature clears all root arguments, so the constants are left for Run to set.
			auto const uav = [&](int i, ID3D12Resource* res)
			{
				// UAV root parameters follow the constants in register order.
				job.m_cmd_list.SetComputeRootUnorderedAccessView(s_cast<int>(ERootParam::FieldsOut) + i, res->GetGPUVirtualAddress());
			};
			auto const srv = [&](int i, D3D12_GPU_VIRTUAL_ADDRESS address)
			{
				// SRV root parameters follow the UAVs in register order.
				job.m_cmd_list.SetComputeRootShaderResourceView(s_cast<int>(ERootParam::FieldsIn) + i, address);
			};
			job.m_cmd_list.SetComputeRootSignature(m_sig.get());
			uav(0, m_u[dst].get());
			uav(1, m_v[dst].get());
			uav(2, m_w[dst].get());
			uav(3, m_temperature[dst].get());
			uav(4, m_pressure.get());
			uav(5, m_divergence_buffer.get());
			uav(6, m_residual_buffer.get());
			srv(0, m_u[src]->GetGPUVirtualAddress());
			srv(1, m_v[src]->GetGPUVirtualAddress());
			srv(2, m_w[src]->GetGPUVirtualAddress());
			srv(3, m_temperature[src]->GetGPUVirtualAddress());
			srv(4, floor_temp_buffer);
			srv(5, source_buffer);
			srv(6, m_floor_height->GetGPUVirtualAddress());
			srv(7, outside_air_buffer);
		}

		// Forget the recorded pipeline state. Call before recording into a command list that other code may have used since the last atmosphere dispatch.
		void ResetBinding()
		{
			// The next Run then sets its pipeline unconditionally.
			m_bound_pso = nullptr;
		}

		// Dispatch 'step' with 'cb' over 'dispatch' thread groups using the fields bound by BindFields.
		void Run(GpuJob& job, ComputeStep& step, CBufAtmosphere const& cb, iv3 dispatch)
		{
			// Consecutive dispatches of the same kernel (for example the smoother passes) only need new constants.
			if (step.m_pso.get() != m_bound_pso)
			{
				job.m_cmd_list.SetPipelineState(step.m_pso.get());
				m_bound_pso = step.m_pso.get();
			}
			job.m_cmd_list.SetComputeRoot32BitConstants(ERootParam::Constants, cb);
			job.m_cmd_list.Dispatch(dispatch.x, dispatch.y, dispatch.z);
		}

		// Dispatch one kernel over the full grid using the fields bound by BindFields.
		void Dispatch(GpuJob& job, ComputeStep& step, CBufAtmosphere const& cb)
		{
			// Full-grid kernels cover every cell and face.
			Run(job, step, cb, DispatchSize());
		}

		// Insert UAV barriers for the buffers in 'writes' so the next kernel sees the completed results.
		void Barrier(GpuJob& job, EFieldWrites writes)
		{
			// Velocity barriers cover both ping-pong slots. Inside Step, velocity writes are ordered by the next BindFields transition instead.
			if (AllSet(writes, EFieldWrites::Velocity))
			{
				for (int slot = 0; slot != 2; ++slot)
					job.m_barriers.UAV(m_u[slot].get()).UAV(m_v[slot].get()).UAV(m_w[slot].get()).UAV(m_temperature[slot].get());
			}
			if (AllSet(writes, EFieldWrites::Pressure))
				job.m_barriers.UAV(m_pressure.get());

			if (AllSet(writes, EFieldWrites::Divergence))
				job.m_barriers.UAV(m_divergence_buffer.get());

			if (AllSet(writes, EFieldWrites::Residual))
				job.m_barriers.UAV(m_residual_buffer.get());

			job.m_barriers.Commit();
		}

		// Reset velocity to zero, pressure to zero, and temperature to the reference profile.
		void InitialiseReference(GpuJob& job)
		{
			// Both ping-pong buffers are reset so the next Step has no hidden stale state. The initialise kernel does not read 'src'.
			auto const empty = AtmosphereStepSources{};
			auto const cb = Constants(0.0f, empty, 0);
			auto const all = EFieldWrites::Velocity | EFieldWrites::Pressure | EFieldWrites::Divergence | EFieldWrites::Residual;
			ResetBinding();
			BindFields(job, 1, 0, m_floor_temp_sentinel->GetGPUVirtualAddress(), m_source_sentinel->GetGPUVirtualAddress(), m_outside_air_sentinel->GetGPUVirtualAddress());
			Dispatch(job, m_initialise, cb);
			Barrier(job, all);
			BindFields(job, 0, 1, m_floor_temp_sentinel->GetGPUVirtualAddress(), m_source_sentinel->GetGPUVirtualAddress(), m_outside_air_sentinel->GetGPUVirtualAddress());
			Dispatch(job, m_initialise, cb);
			Barrier(job, all);
			m_current = 0;
		}

		// Reject a public state whose arrays do not match the MAC layout.
		void ValidateState(AtmosphereState const& state) const
		{
			// The API exposes the real staggered layout, so every array size is part of the caller contract.
			auto const& grid = m_config.m_grid;
			if (isize(state.m_u_faces) != grid.UFaceCount() || isize(state.m_v_faces) != grid.VFaceCount() || isize(state.m_w_faces) != grid.WFaceCount() || isize(state.m_temperature) != grid.CellCount() || (!state.m_pressure.empty() && isize(state.m_pressure) != grid.CellCount()))
				throw std::invalid_argument("Atmosphere state arrays do not match the configured MAC grid");
		}
	};

	// Create GPU buffers and initialise the field to the reference profile at rest.
	AtmosphereSolver::AtmosphereSolver(Gpu& gpu, AtmosphereConfig config, IShaderCache* shader_cache)
		: m_impl(std::make_unique<Impl>(gpu, std::move(config), shader_cache))
	{
		// Construction is fully delegated to the implementation object.
	}

	// Release GPU resources owned by the solver.
	AtmosphereSolver::~AtmosphereSolver() = default;

	// Return the immutable solver configuration.
	AtmosphereConfig const& AtmosphereSolver::Config() const
	{
		// Expose the validated config for callers and tests.
		return m_impl->m_config;
	}

	// Reset velocity to zero, pressure to zero, and temperature to the configured reference profile.
	void AtmosphereSolver::InitialiseReference(GpuJob& job)
	{
		// Queue reset work into the caller's job so it can share the same queue sequencing.
		m_impl->InitialiseReference(job);
	}

	// Upload a full staggered field state. Face and cell arrays must match the configured MAC layout.
	void AtmosphereSolver::UploadState(GpuJob& job, AtmosphereState const& state)
	{
		// Validate the full-state contract before recording copies.
		m_impl->ValidateState(state);
		auto const& grid = m_impl->m_config.m_grid;

		// Copy data through upload memory and leave both ping-pong slots consistent.
		for (int slot = 0; slot != 2; ++slot)
		{
			job.m_barriers.Transition(m_impl->m_u[slot].get(), D3D12_RESOURCE_STATE_COPY_DEST).Transition(m_impl->m_v[slot].get(), D3D12_RESOURCE_STATE_COPY_DEST).Transition(m_impl->m_w[slot].get(), D3D12_RESOURCE_STATE_COPY_DEST).Transition(m_impl->m_temperature[slot].get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
			auto u_upload = job.m_upload.Alloc<float>(grid.UFaceCount());
			auto v_upload = job.m_upload.Alloc<float>(grid.VFaceCount());
			auto w_upload = job.m_upload.Alloc<float>(grid.WFaceCount());
			auto temperature_upload = job.m_upload.Alloc<float>(grid.CellCount());
			memcpy(u_upload.ptr<float>(), state.m_u_faces.data(), state.m_u_faces.size() * sizeof(float));
			memcpy(v_upload.ptr<float>(), state.m_v_faces.data(), state.m_v_faces.size() * sizeof(float));
			memcpy(w_upload.ptr<float>(), state.m_w_faces.data(), state.m_w_faces.size() * sizeof(float));
			memcpy(temperature_upload.ptr<float>(), state.m_temperature.data(), state.m_temperature.size() * sizeof(float));
			job.m_cmd_list.CopyBufferRegion(m_impl->m_u[slot].get(), 0, u_upload);
			job.m_cmd_list.CopyBufferRegion(m_impl->m_v[slot].get(), 0, v_upload);
			job.m_cmd_list.CopyBufferRegion(m_impl->m_w[slot].get(), 0, w_upload);
			job.m_cmd_list.CopyBufferRegion(m_impl->m_temperature[slot].get(), 0, temperature_upload);
			job.m_barriers.Transition(m_impl->m_u[slot].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Transition(m_impl->m_v[slot].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Transition(m_impl->m_w[slot].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Transition(m_impl->m_temperature[slot].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
		}

		// Pressure is persistent across steps for warm starting. For a closed box, pressure has a constant null space. The solver warm-starts from the previous pressure, then subtracts one reference cell after each solve so the offset stays bounded without changing gradients.
		job.m_barriers.Transition(m_impl->m_pressure.get(), D3D12_RESOURCE_STATE_COPY_DEST).Commit();
		auto pressure_upload = job.m_upload.Alloc<float>(grid.CellCount());
		if (state.m_pressure.empty())
		{
			// Missing pressure means a caller is importing physical state without a useful warm start.
			std::fill_n(pressure_upload.ptr<float>(), grid.CellCount(), 0.0f);
		}
		else
		{
			// Provided pressure preserves the caller's warm-start state.
			memcpy(pressure_upload.ptr<float>(), state.m_pressure.data(), state.m_pressure.size() * sizeof(float));
		}
		job.m_cmd_list.CopyBufferRegion(m_impl->m_pressure.get(), 0, pressure_upload);
		job.m_barriers.Transition(m_impl->m_pressure.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
		m_impl->m_current = 0;
	}

	// Change column floors and remap only changed columns into the new terrain-following layers.
	void AtmosphereSolver::RemapFloors(GpuJob& job, std::span<float const> floor_heights)
	{
		// Read back the old field because floor changes are rare and conservative CPU remapping is simpler and auditable.
		auto old_grid = m_impl->m_config.m_grid;
		auto state = ReadBack(job);
		auto new_floors = AtmosphereGrid::BuildFloorHeights(iv2{ old_grid.m_cell_count.x, old_grid.m_cell_count.y }, floor_heights);
		auto changed = std::vector<uint8_t>(old_grid.ColumnCount(), 0);
		auto any_changed = false;
		for (int i = 0; i != old_grid.ColumnCount(); ++i)
		{
			changed[i] = std::abs(new_floors[i] - old_grid.FloorHeight(iv2{ i % old_grid.m_cell_count.x, i / old_grid.m_cell_count.x })) > 1.0e-5f ? 1 : 0;
			any_changed = any_changed || changed[i] != 0;
		}
		if (!any_changed)
			return;

		// Update the grid before using its new metric, then remap only columns whose floor changed.
		auto new_grid = old_grid;
		new_grid.m_floor_heights = std::move(new_floors);
		m_impl->m_config.m_grid = new_grid;
		for (int y = 0; y != new_grid.m_cell_count.y; ++y)
		{
			for (int x = 0; x != new_grid.m_cell_count.x; ++x)
			{
				auto const column = iv2{ x, y };
				if (changed[new_grid.ColumnIndex(column)] == 0)
					continue;

				// Conserving layer-integrated heat in the affected column avoids adding or removing energy when floor geometry changes.
				// Solid columns hold no air, so they neither give nor receive heat. They take the reference temperature at their
				// flat-floor layer height, matching the shader initialisation, so trilinear samples near walls stay plausible.
				auto const old_solid = old_grid.ColumnSolid(column);
				auto const new_solid = new_grid.ColumnSolid(column);
				auto remapped = std::vector<float>(new_grid.m_cell_count.z, 0.0f);
				for (int z = 0; z != new_grid.m_cell_count.z; ++z)
				{
					// Solid cells use the reference profile at the flat-floor height of their layer.
					if (new_solid)
					{
						auto const sigma = 0.5f * (new_grid.SigmaFace(z) + new_grid.SigmaFace(z + 1));
						remapped[z] = m_impl->m_config.m_reference.Temperature(new_grid.m_origin.z + sigma * (new_grid.m_lid_z - new_grid.m_origin.z));
						continue;
					}

					// Air cells take the overlap-weighted heat of the old air layers.
					auto const lo = new_grid.FaceZ(column, z);
					auto const hi = new_grid.FaceZ(column, z + 1);
					auto heat = 0.0;
					auto weight = 0.0;
					for (int oz = 0; oz != old_grid.m_cell_count.z && !old_solid; ++oz)
					{
						auto const old_lo = old_grid.FaceZ(column, oz);
						auto const old_hi = old_grid.FaceZ(column, oz + 1);
						auto const overlap = std::max(0.0f, std::min(hi, old_hi) - std::max(lo, old_lo));
						if (overlap > 0.0f)
						{
							heat += overlap * state.m_temperature[old_grid.CellIndex(iv3{ x, y, oz })];
							weight += overlap;
						}
					}
					remapped[z] = weight > 0.0 ? static_cast<float>(heat / weight) : m_impl->m_config.m_reference.Temperature(new_grid.CellZ(column, z));
				}
				for (int z = 0; z != new_grid.m_cell_count.z; ++z)
					state.m_temperature[new_grid.CellIndex(iv3{ x, y, z })] = remapped[z];
			}
		}

		// Upload the remapped physical state and the new floor metric for subsequent GPU steps.
		m_impl->UploadFloors(job, m_impl->m_config.m_grid.m_floor_heights);
		UploadState(job, state);
	}

	// Record one solver step into 'job'. The caller owns command submission and completion.
	void AtmosphereSolver::Step(GpuJob& job, float dt, AtmosphereStepSources const& sources)
	{
		// Reject invalid per-step values where they enter the solver.
		if (!std::isfinite(dt) || dt < 0.0f)
			throw std::invalid_argument("Atmosphere step dt must be finite and non-negative");
		if (isize(sources.m_heat_sources) > MaxHeatSources)
			throw std::invalid_argument("Atmosphere heat-source count exceeds the per-step limit");
		auto const& grid = m_impl->m_config.m_grid;
		if (!sources.m_floor_temperatures.empty() && isize(sources.m_floor_temperatures) != grid.ColumnCount())
			throw std::invalid_argument("Atmosphere floor temperature buffer must contain one value per column");
		if (!sources.m_outside_air.empty() && isize(sources.m_outside_air) != grid.BoundaryColumnCount())
			throw std::invalid_argument("Atmosphere outside air must contain one value per boundary column");

		// Stage floor temperatures and heat sources for the recorded frame.
		auto floor_buffer = m_impl->m_floor_temp_sentinel->GetGPUVirtualAddress();
		if (!sources.m_floor_temperatures.empty())
		{
			auto upload = job.m_upload.Alloc<float>(isize(sources.m_floor_temperatures));
			memcpy(upload.ptr<float>(), sources.m_floor_temperatures.data(), sources.m_floor_temperatures.size_bytes());
			floor_buffer = upload.m_res->GetGPUVirtualAddress() + upload.m_ofs;
		}
		else
		{
			auto upload = job.m_upload.Alloc<float>(1);
			*upload.ptr<float>() = sources.m_uniform_floor_temperature;
			floor_buffer = upload.m_res->GetGPUVirtualAddress() + upload.m_ofs;
		}

		// Convert public sources to the shader layout in upload memory.
		auto source_buffer = m_impl->m_source_sentinel->GetGPUVirtualAddress();
		if (!sources.m_heat_sources.empty())
		{
			auto upload = job.m_upload.Alloc<GpuHeatSource>(isize(sources.m_heat_sources));
			for (int i = 0; i != isize(sources.m_heat_sources); ++i)
			{
				auto const& src = sources.m_heat_sources[i];
				if (!IsFinite(src.m_centre) || !std::isfinite(src.m_radius) || src.m_radius < 0.0f || !std::isfinite(src.m_heating_rate) || !std::isfinite(src.m_target_temperature) || !std::isfinite(src.m_relaxation_rate) || src.m_relaxation_rate < 0.0f)
					throw std::invalid_argument("Atmosphere heat source contains invalid values");

				upload.ptr<GpuHeatSource>()[i] = GpuHeatSource{ .m_centre = src.m_centre, .m_radius = src.m_radius, .m_heating_rate = src.m_heating_rate, .m_target_temperature = src.m_target_temperature, .m_relaxation_rate = src.m_relaxation_rate };
			}
			source_buffer = upload.m_res->GetGPUVirtualAddress() + upload.m_ofs;
		}

		// Stage one outside-air entry per boundary column. Calm reference air fills the buffer when the caller supplies none,
		// so the kernels always read a full-size buffer.
		auto outside_air_upload = job.m_upload.Alloc<GpuOutsideAir>(grid.BoundaryColumnCount());
		for (int i = 0; i != grid.BoundaryColumnCount(); ++i)
		{
			// Validate caller values where they enter the GPU layout.
			auto const air = sources.m_outside_air.empty() ? AtmosphereOutsideAir{} : sources.m_outside_air[i];
			if (!IsFinite(air.m_wind) || !std::isfinite(air.m_temperature_offset))
				throw std::invalid_argument("Atmosphere outside air contains invalid values");

			outside_air_upload.ptr<GpuOutsideAir>()[i] = GpuOutsideAir{ .m_wind = air.m_wind, .m_temperature_offset = air.m_temperature_offset, .m_pad = 0.0f };
		}
		auto const outside_air_buffer = outside_air_upload.m_res->GetGPUVirtualAddress() + outside_air_upload.m_ofs;

		// Run advection, heat/forces, MAC divergence, multigrid pressure, and metric face-gradient subtraction.
		auto src = m_impl->m_current;
		auto dst = 1 - src;
		auto cb = m_impl->Constants(dt, sources, 0);
		auto& impl = *m_impl;
		impl.ResetBinding();
		if (dt != 0.0f)
		{
			// A zero-duration step is a pure projection of the caller's MAC field; semi-Lagrangian sampling is intentionally skipped because sampling a terrain-following face at its world position is not an exact identity on sloped columns.
			impl.BindFields(job, src, dst, floor_buffer, source_buffer, outside_air_buffer);
			impl.Dispatch(job, impl.m_advect, cb);
			src = dst;
			dst = 1 - src;
			impl.BindFields(job, src, dst, floor_buffer, source_buffer, outside_air_buffer);
			if (impl.m_config.m_vorticity_confinement != 0.0f)
			{
				// Store the swirl magnitude of the advected field in the divergence scratch buffer for the forces pass. The divergence pass overwrites it afterwards.
				impl.Dispatch(job, impl.m_vorticity, cb);
				impl.Barrier(job, Impl::EFieldWrites::Divergence);
			}
			impl.Dispatch(job, impl.m_forces_heat, cb);
			src = dst;
			dst = 1 - src;
		}

		// The pressure solve and projection all read the same 'src' fields and write 'dst' only in the final projection.
		impl.BindFields(job, src, dst, floor_buffer, source_buffer, outside_air_buffer);
		impl.Dispatch(job, impl.m_divergence, cb);
		impl.Barrier(job, Impl::EFieldWrites::Divergence);
		for (int cycle = 0; cycle != impl.m_config.m_pressure_vcycles; ++cycle)
		{
			// Warm-started V-cycles preserve the previous pressure on the finest level and rebuild coarse corrections from the current residual.
			impl.VCycle(job, cb, 0);
		}
		impl.Dispatch(job, impl.m_normalise_pressure, cb);
		impl.Barrier(job, Impl::EFieldWrites::Pressure | Impl::EFieldWrites::Residual);
		if (impl.m_config.m_boundaries.m_x_min == EAtmosphereBoundary::Solid && impl.m_config.m_boundaries.m_x_max == EAtmosphereBoundary::Solid && impl.m_config.m_boundaries.m_y_min == EAtmosphereBoundary::Solid && impl.m_config.m_boundaries.m_y_max == EAtmosphereBoundary::Solid)
		{
			auto normalise_cb = cb;
			normalise_cb.m_mg_phase = 1;
			impl.Dispatch(job, impl.m_normalise_pressure, normalise_cb);
			impl.Barrier(job, Impl::EFieldWrites::Pressure);
		}

		// Later readers bind the projected fields as SRVs or copy sources, and that transition orders these writes.
		impl.Dispatch(job, impl.m_project, cb);
		impl.m_current = dst;
	}

	// Read the full staggered field after all previously recorded work in 'job' has completed.
	AtmosphereState AtmosphereSolver::ReadBack(GpuJob& job)
	{
		// Copy each MAC array separately so callers can inspect the physical storage layout.
		auto const& grid = m_impl->m_config.m_grid;
		auto const current = m_impl->m_current;
		job.m_barriers.Transition(m_impl->m_u[current].get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Transition(m_impl->m_v[current].get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Transition(m_impl->m_w[current].get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Transition(m_impl->m_temperature[current].get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Transition(m_impl->m_pressure.get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
		auto u_readback = job.m_readback.Alloc<float>(grid.UFaceCount());
		auto v_readback = job.m_readback.Alloc<float>(grid.VFaceCount());
		auto w_readback = job.m_readback.Alloc<float>(grid.WFaceCount());
		auto temperature_readback = job.m_readback.Alloc<float>(grid.CellCount());
		auto pressure_readback = job.m_readback.Alloc<float>(grid.CellCount());
		job.m_cmd_list.CopyBufferRegion(u_readback, m_impl->m_u[current].get(), 0);
		job.m_cmd_list.CopyBufferRegion(v_readback, m_impl->m_v[current].get(), 0);
		job.m_cmd_list.CopyBufferRegion(w_readback, m_impl->m_w[current].get(), 0);
		job.m_cmd_list.CopyBufferRegion(temperature_readback, m_impl->m_temperature[current].get(), 0);
		job.m_cmd_list.CopyBufferRegion(pressure_readback, m_impl->m_pressure.get(), 0);
		job.m_barriers.Transition(m_impl->m_u[current].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Transition(m_impl->m_v[current].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Transition(m_impl->m_w[current].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Transition(m_impl->m_temperature[current].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Transition(m_impl->m_pressure.get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
		job.Run();

		// Pack readback spans into owned vectors so the caller can keep the result after the job allocator advances.
		auto state = AtmosphereState{};
		state.m_u_faces.assign(u_readback.ptr<float>(), u_readback.ptr<float>() + grid.UFaceCount());
		state.m_v_faces.assign(v_readback.ptr<float>(), v_readback.ptr<float>() + grid.VFaceCount());
		state.m_w_faces.assign(w_readback.ptr<float>(), w_readback.ptr<float>() + grid.WFaceCount());
		state.m_temperature.assign(temperature_readback.ptr<float>(), temperature_readback.ptr<float>() + grid.CellCount());
		state.m_pressure.assign(pressure_readback.ptr<float>(), pressure_readback.ptr<float>() + grid.CellCount());
		return state;
	}

	// Return cell-centred convenience samples from a staggered field.
	std::vector<AtmosphereCellState> AtmosphereSolver::CellStates(AtmosphereState const& state) const
	{
		// Average adjacent face velocities onto cell centres for consumers that do not need the MAC layout.
		m_impl->ValidateState(state);
		auto const& grid = m_impl->m_config.m_grid;
		auto cells = std::vector<AtmosphereCellState>(grid.CellCount());
		for (int z = 0; z != grid.m_cell_count.z; ++z)
		{
			// Scan one horizontal layer at a time for locality in the packed state.
			for (int y = 0; y != grid.m_cell_count.y; ++y)
			{
				// Rows use contiguous x cells.
				for (int x = 0; x != grid.m_cell_count.x; ++x)
				{
					// A cell-centred sample is a convenience view, not the authoritative storage layout.
					auto const cell = iv3{ x, y, z };
					auto const idx = grid.CellIndex(cell);
					auto const u = 0.5f * (state.m_u_faces[grid.UFaceIndex(iv3{ x, y, z })] + state.m_u_faces[grid.UFaceIndex(iv3{ x + 1, y, z })]);
					auto const v = 0.5f * (state.m_v_faces[grid.VFaceIndex(iv3{ x, y, z })] + state.m_v_faces[grid.VFaceIndex(iv3{ x, y + 1, z })]);
					auto const w = 0.5f * (LoadW(grid, state, iv3{ x, y, z }) + LoadW(grid, state, iv3{ x, y, z + 1 }));
					cells[idx] = AtmosphereCellState{ .m_velocity = v4{ u, v, w, 0.0f }, .m_temperature = state.m_temperature[idx] };
				}
			}
		}
		return cells;
	}

	// Return simple CPU diagnostics for a staggered field.
	AtmosphereFieldStats AtmosphereSolver::Stats(AtmosphereState const& state) const
	{
		// Validate the readback span before indexing it as a MAC grid.
		m_impl->ValidateState(state);
		auto const& grid = m_impl->m_config.m_grid;

		// Accumulate speed, top-layer temperature, and finite-volume divergence from face fluxes.
		auto stats = AtmosphereFieldStats{};
		auto sum_div2 = 0.0;
		auto top_sum = 0.0;
		auto top_count = 0;
		auto air_cell_count = 0;
		for (int z = 0; z != grid.m_cell_count.z; ++z)
		{
			// Scan one horizontal layer at a time for locality in the packed state.
			for (int y = 0; y != grid.m_cell_count.y; ++y)
			{
				// Rows use contiguous x cells.
				for (int x = 0; x != grid.m_cell_count.x; ++x)
				{
					// Solid cells hold no air, so they do not contribute to flow statistics.
					auto const column = iv2{ x, y };
					if (grid.ColumnSolid(column))
						continue;

					// MAC divergence uses the two faces that bound each cell on every axis.
					++air_cell_count;
					auto const cell = iv3{ x, y, z };
					auto const idx = grid.CellIndex(cell);
					auto const u0 = state.m_u_faces[grid.UFaceIndex(iv3{ x, y, z })];
					auto const u1 = state.m_u_faces[grid.UFaceIndex(iv3{ x + 1, y, z })];
					auto const v0 = state.m_v_faces[grid.VFaceIndex(iv3{ x, y, z })];
					auto const v1 = state.m_v_faces[grid.VFaceIndex(iv3{ x, y + 1, z })];
					auto const w0 = LoadW(grid, state, iv3{ x, y, z });
					auto const w1 = LoadW(grid, state, iv3{ x, y, z + 1 });
					auto const u = 0.5f * (u0 + u1);
					auto const v = 0.5f * (v0 + v1);
					auto const w = 0.5f * (w0 + w1);
					auto const dz = grid.CellHeight(column, z);
					auto const base_div = (u1 - u0) / grid.m_dx + (v1 - v0) / grid.m_dx + (w1 - w0) / dz;
					auto const x0 = MetricNeighbour(grid, column, column + iv2{ -1, 0 });
					auto const x1 = MetricNeighbour(grid, column, column + iv2{ +1, 0 });
					auto const y0 = MetricNeighbour(grid, column, column + iv2{ 0, -1 });
					auto const y1 = MetricNeighbour(grid, column, column + iv2{ 0, +1 });
					auto const z_x = (grid.CellZ(x1, z) - grid.CellZ(x0, z)) / (grid.m_dx * static_cast<float>(std::abs(x1.x - x0.x) + 1.0e-6f));
					auto const z_y = (grid.CellZ(y1, z) - grid.CellZ(y0, z)) / (grid.m_dx * static_cast<float>(std::abs(y1.y - y0.y) + 1.0e-6f));
					auto const z0 = std::max(0, z - 1);
					auto const z1 = std::min(grid.m_cell_count.z - 1, z + 1);
					auto const z_span = std::max(grid.CellZ(column, z1) - grid.CellZ(column, z0), 0.001f);
					auto const u_lo = 0.5f * (state.m_u_faces[grid.UFaceIndex(iv3{ x, y, z0 })] + state.m_u_faces[grid.UFaceIndex(iv3{ x + 1, y, z0 })]);
					auto const u_hi = 0.5f * (state.m_u_faces[grid.UFaceIndex(iv3{ x, y, z1 })] + state.m_u_faces[grid.UFaceIndex(iv3{ x + 1, y, z1 })]);
					auto const v_lo = 0.5f * (state.m_v_faces[grid.VFaceIndex(iv3{ x, y, z0 })] + state.m_v_faces[grid.VFaceIndex(iv3{ x, y + 1, z0 })]);
					auto const v_hi = 0.5f * (state.m_v_faces[grid.VFaceIndex(iv3{ x, y, z1 })] + state.m_v_faces[grid.VFaceIndex(iv3{ x, y + 1, z1 })]);
					auto const div = base_div - z_x * (u_hi - u_lo) / z_span - z_y * (v_hi - v_lo) / z_span;
					stats.m_max_speed = std::max(stats.m_max_speed, Length(v4{ u, v, w, 0.0f }.xyz));
					stats.m_peak_vertical_velocity = std::max(stats.m_peak_vertical_velocity, w);
					stats.m_max_divergence = std::max(stats.m_max_divergence, std::abs(div));
					sum_div2 += div * div;
					if (z == grid.m_cell_count.z - 1)
					{
						// The lid-layer mean is used by plume tests as a stable warming signal.
						top_sum += state.m_temperature[idx];
						++top_count;
					}
				}
			}
		}
		stats.m_rms_divergence = static_cast<float>(std::sqrt(sum_div2 / std::max(1, air_cell_count)));
		stats.m_mean_top_temperature = static_cast<float>(top_sum / std::max(1, top_count));
		return stats;
	}

	// Private implementation for GPU tracer particles.
	struct AtmosphereTracers::Impl
	{
		AtmosphereSolver& m_solver;
		Gpu& m_gpu;
		AtmosphereTracerConfig m_config;
		ComputeStep m_initialise;
		ComputeStep m_advect;
		D3DPtr<ID3D12Resource> m_particles[2];
		GpuReadbackBuffer::Allocation m_pending_readback; // Destination of the last recorded particle copy, valid until resolved
		int m_current;
		uint32_t m_frame;

		// Compile tracer kernels and allocate particle buffers.
		Impl(AtmosphereSolver& solver, Gpu& gpu, AtmosphereTracerConfig config, IShaderCache* cache)
			: m_solver(solver)
			, m_gpu(gpu)
			, m_config(config)
			, m_initialise()
			, m_advect()
			, m_particles{}
			, m_pending_readback()
			, m_current(0)
			, m_frame(0)
		{
			// Tracer state is valid only when its solver and buffers have a defined lifetime.
			m_config.Validate();
			auto resolver = shader_cache::ResourceSourceResolver{};
			auto compile = [&](wchar_t const* entry_point)
			{
				// Runtime compilation uses the same embedded source and optional cache as the solver kernels.
				return ShaderCompiler{}.Source("src/atmosphere/atmosphere.hlsl", resolver).Cache(cache).HlslVersion(EHlslVersion::Hlsl2021).Define(L"SHADER_BUILD").Optimise(true).ShaderModel(L"cs_6_6").EntryPoint(entry_point).Compile();
			};
			auto make_step = [&](wchar_t const* entry_point, char const* name)
			{
				// The tracer kernels share the atmosphere root layout so they can sample the solver buffers directly.
				auto step = ComputeStep{};
				auto code = compile(entry_point);
				auto sig_name = std::format("Physics.Atmosphere.Tracers.{}.RootSig", name);
				auto pso_name = std::format("Physics.Atmosphere.Tracers.{}.PSO", name);
				step.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<CBufAtmosphere>(hlsl::ECBufReg::b0)
					.U32<CBufAtmosphereTracers>(hlsl::ECBufReg::b1)
					.UAV(hlsl::EUAVReg::u7)
					.SRV(hlsl::ESRVReg::t0).SRV(hlsl::ESRVReg::t1).SRV(hlsl::ESRVReg::t2).SRV(hlsl::ESRVReg::t3).SRV(hlsl::ESRVReg::t6).SRV(hlsl::ESRVReg::t8)
					.Create(gpu, sig_name.c_str());
				step.m_pso = ComputePSO(step.m_sig.get(), code).Create(gpu, pso_name.c_str());
				return step;
			};

			// Allocate both ping-pong buffers, then fill them deterministically.
			m_initialise = make_step(L"CSInitialiseTracers", "Initialise");
			m_advect = make_step(L"CSAdvectTracers", "Advect");
			for (int slot = 0; slot != 2; ++slot)
				m_particles[slot] = CreateBuffer<GpuTracerParticle>(m_gpu, m_gpu.m_job, m_config.m_particle_count, slot == 0 ? "Atmosphere:Tracer0" : "Atmosphere:Tracer1");
			Initialise(m_gpu.m_job);
			m_gpu.m_job.RetireRecordedWork();
		}

		// Build tracer-specific constants while preserving the solver's domain constants.
		CBufAtmosphereTracers TracerConstants() const
		{
			// Tracer constants control deterministic respawn and lifetime.
			// Normalise the height profile so its integral over the column is 1.
			auto const& c = m_config;
			auto const total = 0.5f * c.m_break_height * (c.m_ground_density + c.m_break_density) + (1.0f - c.m_break_height) * c.m_upper_density;
			return CBufAtmosphereTracers{
				.m_tracer_count = c.m_particle_count,
				.m_tracer_seed = c.m_seed,
				.m_tracer_frame = m_frame,
				.m_tracer_max_age = c.m_max_age,
				.m_tracer_ground_density = c.m_ground_density / total,
				.m_tracer_break_density = c.m_break_density / total,
				.m_tracer_upper_density = c.m_upper_density / total,
				.m_tracer_break_height = c.m_break_height,
			};
		}

		// Bind the shared root layout and dispatch a tracer kernel.
		void Dispatch(GpuJob& job, ComputeStep& step, CBufAtmosphere const& cb, CBufAtmosphereTracers const& tracer_cb, int src, int dst)
		{
			// The solver buffers are read-only here; only the tracer output buffer is written. Transition each binding into the state its root descriptor requires.
			assert(src != dst && "Ping-pong particles cannot be bound as both input and output");
			auto const solver_src = m_solver.m_impl->m_current;
			job.m_barriers
				.Transition(m_particles[dst].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS)
				.Transition(m_particles[src].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
				.Transition(m_solver.m_impl->m_u[solver_src].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
				.Transition(m_solver.m_impl->m_v[solver_src].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
				.Transition(m_solver.m_impl->m_w[solver_src].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
				.Transition(m_solver.m_impl->m_temperature[solver_src].get(), D3D12_RESOURCE_STATE_NON_PIXEL_SHADER_RESOURCE)
				.Commit();

			// Bind the shared root layout and dispatch.
			auto const dispatch_x = (m_config.m_particle_count + AtmosphereTracerThreadGroup - 1) / AtmosphereTracerThreadGroup;
			job.m_cmd_list.SetPipelineState(step.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			job.m_cmd_list.AddComputeRoot32BitConstants(tracer_cb);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_particles[dst]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_solver.m_impl->m_u[solver_src]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_solver.m_impl->m_v[solver_src]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_solver.m_impl->m_w[solver_src]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_solver.m_impl->m_temperature[solver_src]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_solver.m_impl->m_floor_height->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_particles[src]->GetGPUVirtualAddress());
			job.m_cmd_list.Dispatch(dispatch_x, 1, 1);
		}

		// Insert a UAV barrier for the tracer ping-pong buffers.
		void Barrier(GpuJob& job)
		{
			// The next tracer dispatch may read the buffer just written.
			for (auto const& resource : m_particles)
				job.m_barriers.UAV(resource.get());
			job.m_barriers.Commit();
		}

		// Reset all tracer particles to deterministic domain positions.
		void Initialise(GpuJob& job)
		{
			// Write both slots so the first advect has no hidden dependency on old data. The initialise kernel does not read 'src'.
			++m_frame;
			auto cb = m_solver.m_impl->Constants(0.0f, AtmosphereStepSources{}, 0);
			auto tracer_cb = TracerConstants();
			Dispatch(job, m_initialise, cb, tracer_cb, 1, 0);
			Barrier(job);
			Dispatch(job, m_initialise, cb, tracer_cb, 0, 1);
			Barrier(job);
			m_current = 0;
		}
	};

	// Create GPU buffers for deterministic tracer particles associated with 'solver'.
	AtmosphereTracers::AtmosphereTracers(AtmosphereSolver& solver, Gpu& gpu, AtmosphereTracerConfig config, IShaderCache* shader_cache)
		: m_impl(std::make_unique<Impl>(solver, gpu, config, shader_cache))
	{
		// Construction is fully delegated to the implementation object.
	}

	// Release GPU resources owned by the tracer set.
	AtmosphereTracers::~AtmosphereTracers() = default;

	// Return the immutable tracer configuration.
	AtmosphereTracerConfig const& AtmosphereTracers::Config() const
	{
		// Expose the validated config for callers and tests.
		return m_impl->m_config;
	}

	// Reset all particles to deterministic positions inside the solver domain.
	void AtmosphereTracers::Initialise(GpuJob& job)
	{
		// Queue reset work into the caller-owned job.
		m_impl->Initialise(job);
	}

	// Advect all particles through the solver's current velocity and temperature fields.
	void AtmosphereTracers::Advect(GpuJob& job, float dt)
	{
		// Reject invalid integration intervals at the API boundary.
		if (!std::isfinite(dt) || dt < 0.0f)
			throw std::invalid_argument("Atmosphere tracer dt must be finite and non-negative");

		// Ping-pong particle state so the kernel reads a stable previous particle set.
		++m_impl->m_frame;
		auto src = m_impl->m_current;
		auto dst = 1 - src;
		auto cb = m_impl->m_solver.m_impl->Constants(dt, AtmosphereStepSources{}, 0);
		auto tracer_cb = m_impl->TracerConstants();
		m_impl->Dispatch(job, m_impl->m_advect, cb, tracer_cb, src, dst);
		m_impl->Barrier(job);
		m_impl->m_current = dst;
	}

	// Read all particles after all previously recorded tracer work in 'job' has completed.
	std::vector<AtmosphereTracerParticle> AtmosphereTracers::ReadBack(GpuJob& job)
	{
		// Copy the particles and wait for the job so the copy can be resolved immediately.
		RecordReadBack(job);
		job.Run();
		return ResolveReadBack();
	}

	// Record a copy of all particles into 'job' without submitting it.
	void AtmosphereTracers::RecordReadBack(GpuJob& job)
	{
		// Copy the current GPU particle buffer to readback memory. The particles return to UAV state for the next advect.
		auto const count = m_impl->m_config.m_particle_count;
		auto const current = m_impl->m_current;
		job.m_barriers.Transition(m_impl->m_particles[current].get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
		m_impl->m_pending_readback = job.m_readback.Alloc<GpuTracerParticle>(count);
		job.m_cmd_list.CopyBufferRegion(m_impl->m_pending_readback, m_impl->m_particles[current].get(), 0);
		job.m_barriers.Transition(m_impl->m_particles[current].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
	}

	// Return the particles copied by the last 'RecordReadBack'.
	std::vector<AtmosphereTracerParticle> AtmosphereTracers::ResolveReadBack()
	{
		// Convert from the shader layout to the public CPU layout, then release the pending copy.
		auto const count = m_impl->m_config.m_particle_count;
		assert(m_impl->m_pending_readback.m_mem != nullptr && "No particle read back has been recorded");
		auto particles = std::vector<AtmosphereTracerParticle>(count);
		auto const* src = m_impl->m_pending_readback.ptr<GpuTracerParticle>();
		for (int i = 0; i != count; ++i)
		{
			particles[i] = AtmosphereTracerParticle{
				.m_position = src[i].m_position,
				.m_temperature = src[i].m_temperature,
				.m_age = src[i].m_age,
				.m_speed = src[i].m_speed,
			};
		}
		m_impl->m_pending_readback = {};
		return particles;
	}

}
