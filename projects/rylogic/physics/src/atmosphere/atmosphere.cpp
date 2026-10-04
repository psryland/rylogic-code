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
			int m_open_edge_band;          // columns over which inflow outside wind is blended in
			float m_vorticity_confinement; // swirl-restoring strength; zero disables it, 1/s

			// Fused small-level V-cycle.
			int m_mg_passes;               // red-black smoothing passes on the coarsest level
			int m_mg_pre_smooth;           // red-black smoothing passes on each level before restriction
			int m_mg_post_smooth;          // red-black smoothing passes on each level after prolongation
		};
		static_assert(sizeof(CBufAtmosphere) == 30 * sizeof(uint32_t));
		static_assert(offsetof(CBufAtmosphere, m_origin) == 16 && offsetof(CBufAtmosphere, m_source_count) == 72);
		static_assert(offsetof(CBufAtmosphere, m_mg_size) == 80 && offsetof(CBufAtmosphere, m_mg_phase) == 96 && offsetof(CBufAtmosphere, m_mg_passes) == 108 && offsetof(CBufAtmosphere, m_mg_post_smooth) == 116);

		// Root constants shared by tracer kernels. Must match CBufAtmosphereTracers in atmosphere.hlsl.
		struct CBufAtmosphereTracers
		{
			int m_tracer_count;
			uint32_t m_tracer_seed;
			uint32_t m_tracer_frame;
			float m_tracer_max_age;
		};
		static_assert(sizeof(CBufAtmosphereTracers) == 4 * sizeof(uint32_t));

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
			v2 m_pad;
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

	// Reject invalid tracer configuration at the caller boundary.
	void AtmosphereTracerConfig::Validate() const
	{
		// Tracer buffers are optional, but a created tracer set needs a finite positive lifetime.
		if (m_particle_count < 0)
			throw std::invalid_argument("Atmosphere tracer count cannot be negative");
		if (!std::isfinite(m_max_age) || m_max_age <= 0.0f)
			throw std::invalid_argument("Atmosphere tracer lifetime must be finite and positive");
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
		// The lid is flat, so column height is the difference between the lid and the local floor.
		auto const floor_height = FloorHeight(cell);
		return floor_height + SigmaFace(z) * (m_lid_z - floor_height);
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

		// Every column must have positive air height and enough room for the requested first layer.
		for (auto floor_height : m_floor_heights)
		{
			if (!std::isfinite(floor_height))
				throw std::invalid_argument("Atmosphere floor heights must be finite");
			if (floor_height >= m_lid_z)
				throw std::invalid_argument("Atmosphere floor heights must be below the lid");
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
		if (m_open_edge_band < 0)
			throw std::invalid_argument("Atmosphere open-edge band must be non-negative");
		if (!std::isfinite(m_vorticity_confinement) || m_vorticity_confinement < 0.0f)
			throw std::invalid_argument("Atmosphere vorticity confinement must be finite and non-negative");
	}


	// Private implementation kept out of the public header so shader plumbing can change without API churn.
	struct AtmosphereSolver::Impl
	{
		Gpu& m_gpu;
		AtmosphereConfig m_config;
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
			auto make_step = [&](wchar_t const* entry_point, char const* name)
			{
				// Every kernel uses the same root layout so Step can bind buffers consistently.
				auto step = ComputeStep{};
				auto code = compile(entry_point);
				auto sig_name = std::format("Physics.Atmosphere.{}.RootSig", name);
				auto pso_name = std::format("Physics.Atmosphere.{}.PSO", name);
				step.m_sig = RootSig(ERootSigFlags::ComputeOnly)
					.U32<CBufAtmosphere>(hlsl::ECBufReg::b0)
					.UAV(hlsl::EUAVReg::u0).UAV(hlsl::EUAVReg::u1).UAV(hlsl::EUAVReg::u2).UAV(hlsl::EUAVReg::u3).UAV(hlsl::EUAVReg::u4).UAV(hlsl::EUAVReg::u5).UAV(hlsl::EUAVReg::u6)
					.SRV(hlsl::ESRVReg::t0).SRV(hlsl::ESRVReg::t1).SRV(hlsl::ESRVReg::t2).SRV(hlsl::ESRVReg::t3).SRV(hlsl::ESRVReg::t4).SRV(hlsl::ESRVReg::t5).SRV(hlsl::ESRVReg::t6).SRV(hlsl::ESRVReg::t7)
					.Create(gpu, sig_name.c_str());
				step.m_pso = ComputePSO(step.m_sig.get(), code).Create(gpu, pso_name.c_str());
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

		// Bind the common root layout and dispatch one kernel over a selected multigrid level.
		void DispatchLevel(GpuJob& job, ComputeStep& step, CBufAtmosphere const& cb, MgLevel const& level, int src, int dst, D3D12_GPU_VIRTUAL_ADDRESS floor_temp_buffer, D3D12_GPU_VIRTUAL_ADDRESS source_buffer, D3D12_GPU_VIRTUAL_ADDRESS outside_air_buffer)
		{
			// Multigrid kernels share resources with the main step but use smaller dispatch extents on coarse levels.
			auto const dispatch = LevelDispatch(level);
			BindFields(job, step, cb, src, dst, floor_temp_buffer, source_buffer, outside_air_buffer);
			job.m_cmd_list.Dispatch(dispatch.x, dispatch.y, dispatch.z);
		}

		// Run red-black Gauss-Seidel smoothing on a pressure level.
		void SmoothLevel(GpuJob& job, CBufAtmosphere cb, int level_index, int passes, int src, int dst, D3D12_GPU_VIRTUAL_ADDRESS floor_temp_buffer, D3D12_GPU_VIRTUAL_ADDRESS source_buffer, D3D12_GPU_VIRTUAL_ADDRESS outside_air_buffer)
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
					BindFields(job, m_mg_smooth, LevelConstants(cb, level_index, phase), src, dst, floor_temp_buffer, source_buffer, outside_air_buffer);
					job.m_cmd_list.Dispatch(dispatch.x, dispatch.y, dispatch.z);
					BarrierFields(job);
				}
			}
		}

		// Run one recursive multigrid V-cycle from 'level_index'.
		void VCycle(GpuJob& job, CBufAtmosphere cb, int level_index, int src, int dst, D3D12_GPU_VIRTUAL_ADDRESS floor_temp_buffer, D3D12_GPU_VIRTUAL_ADDRESS source_buffer, D3D12_GPU_VIRTUAL_ADDRESS outside_air_buffer)
		{
			// Coarse levels receive residual equations; the finest level starts with the physical divergence from the MAC field.
			auto const& level = m_levels[level_index];
			if (level.m_nx <= MgFusedColumns && level.m_ny <= MgFusedColumns)
			{
				// Small levels cannot fill the GPU, so the rest of the V-cycle runs in one thread group with group barriers between passes.
				auto level_cb = LevelConstants(cb, level_index, 0);
				level_cb.m_mg_passes = m_config.m_pressure_coarse_smooth;
				level_cb.m_mg_pre_smooth = m_config.m_pressure_pre_smooth;
				level_cb.m_mg_post_smooth = m_config.m_pressure_post_smooth;
				BindFields(job, m_mg_small_levels, level_cb, src, dst, floor_temp_buffer, source_buffer, outside_air_buffer);
				job.m_cmd_list.Dispatch(1, 1, 1);
				BarrierFields(job);
				return;
			}

			// Pre-smoothing damps cell-scale errors before residual restriction.
			SmoothLevel(job, cb, level_index, m_config.m_pressure_pre_smooth, src, dst, floor_temp_buffer, source_buffer, outside_air_buffer);
			DispatchLevel(job, m_mg_residual, LevelConstants(cb, level_index, 0), m_levels[level_index], src, dst, floor_temp_buffer, source_buffer, outside_air_buffer);
			BarrierFields(job);
			DispatchLevel(job, m_mg_restrict, LevelConstants(cb, level_index, 0), m_levels[level_index + 1], src, dst, floor_temp_buffer, source_buffer, outside_air_buffer); // dispatched over the child level's cells
			BarrierFields(job);

			// The coarse correction removes long wavelengths that local smoothing cannot reach on large grids.
			VCycle(job, cb, level_index + 1, src, dst, floor_temp_buffer, source_buffer, outside_air_buffer);
			DispatchLevel(job, m_mg_prolongate, LevelConstants(cb, level_index, 0), m_levels[level_index], src, dst, floor_temp_buffer, source_buffer, outside_air_buffer);
			BarrierFields(job);
			SmoothLevel(job, cb, level_index, m_config.m_pressure_post_smooth, src, dst, floor_temp_buffer, source_buffer, outside_air_buffer);
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
				.m_mg_pre_smooth = 0,
				.m_mg_post_smooth = 0,
			};
		}

		// Transition the fields into the states the common root layout requires, then bind 'step' and its resources.
		void BindFields(GpuJob& job, ComputeStep& step, CBufAtmosphere const& cb, int src, int dst, D3D12_GPU_VIRTUAL_ADDRESS floor_temp_buffer, D3D12_GPU_VIRTUAL_ADDRESS source_buffer, D3D12_GPU_VIRTUAL_ADDRESS outside_air_buffer)
		{
			// The ping-pong fields default to UAV state but 'src' is bound as root SRVs. The barrier batch tracks the current state so repeated transitions are dropped.
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
			job.m_cmd_list.SetPipelineState(step.m_pso.get());
			job.m_cmd_list.SetComputeRootSignature(step.m_sig.get());
			job.m_cmd_list.AddComputeRoot32BitConstants(cb);
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_u[dst]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_v[dst]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_w[dst]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_temperature[dst]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_pressure->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_divergence_buffer->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootUnorderedAccessView(m_residual_buffer->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_u[src]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_v[src]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_w[src]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(m_temperature[src]->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(floor_temp_buffer);
			job.m_cmd_list.AddComputeRootShaderResourceView(source_buffer);
			job.m_cmd_list.AddComputeRootShaderResourceView(m_floor_height->GetGPUVirtualAddress());
			job.m_cmd_list.AddComputeRootShaderResourceView(outside_air_buffer);
		}

		// Bind the common root layout and dispatch one kernel over the full grid.
		void Dispatch(GpuJob& job, ComputeStep& step, CBufAtmosphere const& cb, int src, int dst, D3D12_GPU_VIRTUAL_ADDRESS floor_temp_buffer, D3D12_GPU_VIRTUAL_ADDRESS source_buffer, D3D12_GPU_VIRTUAL_ADDRESS outside_air_buffer)
		{
			// Full-grid kernels cover every cell and face.
			auto const dispatch = DispatchSize();
			BindFields(job, step, cb, src, dst, floor_temp_buffer, source_buffer, outside_air_buffer);
			job.m_cmd_list.Dispatch(dispatch.x, dispatch.y, dispatch.z);
		}

		// Insert UAV barriers for every field that may be read by the next kernel.
		void BarrierFields(GpuJob& job)
		{
			// Conservative barriers keep kernel ordering obvious while the solver is still small.
			for (auto const& resource : m_u)
				job.m_barriers.UAV(resource.get());
			for (auto const& resource : m_v)
				job.m_barriers.UAV(resource.get());
			for (auto const& resource : m_w)
				job.m_barriers.UAV(resource.get());
			for (auto const& resource : m_temperature)
				job.m_barriers.UAV(resource.get());
			job.m_barriers.UAV(m_pressure.get()).UAV(m_divergence_buffer.get()).UAV(m_residual_buffer.get()).Commit();
		}

		// Reset velocity to zero, pressure to zero, and temperature to the reference profile.
		void InitialiseReference(GpuJob& job)
		{
			// Both ping-pong buffers are reset so the next Step has no hidden stale state. The initialise kernel does not read 'src'.
			auto const empty = AtmosphereStepSources{};
			auto const cb = Constants(0.0f, empty, 0);
			Dispatch(job, m_initialise, cb, 1, 0,  m_floor_temp_sentinel->GetGPUVirtualAddress(), m_source_sentinel->GetGPUVirtualAddress(), m_outside_air_sentinel->GetGPUVirtualAddress());
			BarrierFields(job);
			Dispatch(job, m_initialise, cb, 0, 1, m_floor_temp_sentinel->GetGPUVirtualAddress(), m_source_sentinel->GetGPUVirtualAddress(), m_outside_air_sentinel->GetGPUVirtualAddress());
			BarrierFields(job);
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
				auto remapped = std::vector<float>(new_grid.m_cell_count.z, 0.0f);
				for (int z = 0; z != new_grid.m_cell_count.z; ++z)
				{
					auto const lo = new_grid.FaceZ(column, z);
					auto const hi = new_grid.FaceZ(column, z + 1);
					auto heat = 0.0;
					auto weight = 0.0;
					for (int oz = 0; oz != old_grid.m_cell_count.z; ++oz)
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
		if (dt != 0.0f)
		{
			// A zero-duration step is a pure projection of the caller's MAC field; semi-Lagrangian sampling is intentionally skipped because sampling a terrain-following face at its world position is not an exact identity on sloped columns.
			m_impl->Dispatch(job, m_impl->m_advect, cb, src, dst, floor_buffer, source_buffer, outside_air_buffer);
			m_impl->BarrierFields(job);
			src = dst;
			dst = 1 - src;
			if (m_impl->m_config.m_vorticity_confinement != 0.0f)
			{
				// Store the swirl magnitude of the advected field in the divergence scratch buffer for the forces pass. The divergence pass overwrites it afterwards.
				m_impl->Dispatch(job, m_impl->m_vorticity, cb, src, dst, floor_buffer, source_buffer, outside_air_buffer);
				m_impl->BarrierFields(job);
			}
			m_impl->Dispatch(job, m_impl->m_forces_heat, cb, src, dst, floor_buffer, source_buffer, outside_air_buffer);
			m_impl->BarrierFields(job);
			src = dst;
			dst = 1 - src;
		}
		m_impl->Dispatch(job, m_impl->m_divergence, cb, src, dst, floor_buffer, source_buffer, outside_air_buffer);
		m_impl->BarrierFields(job);
		for (int cycle = 0; cycle != m_impl->m_config.m_pressure_vcycles; ++cycle)
		{
			// Warm-started V-cycles preserve the previous pressure on the finest level and rebuild coarse corrections from the current residual.
			m_impl->VCycle(job, cb, 0, src, dst, floor_buffer, source_buffer, outside_air_buffer);
		}
		m_impl->Dispatch(job, m_impl->m_normalise_pressure, cb, src, dst, floor_buffer, source_buffer, outside_air_buffer);
		m_impl->BarrierFields(job);
		if (m_impl->m_config.m_boundaries.m_x_min == EAtmosphereBoundary::Solid && m_impl->m_config.m_boundaries.m_x_max == EAtmosphereBoundary::Solid && m_impl->m_config.m_boundaries.m_y_min == EAtmosphereBoundary::Solid && m_impl->m_config.m_boundaries.m_y_max == EAtmosphereBoundary::Solid)
		{
			auto normalise_cb = cb;
			normalise_cb.m_mg_phase = 1;
			m_impl->Dispatch(job, m_impl->m_normalise_pressure, normalise_cb, src, dst, floor_buffer, source_buffer, outside_air_buffer);
			m_impl->BarrierFields(job);
		}
		m_impl->Dispatch(job, m_impl->m_project, cb, src, dst, floor_buffer, source_buffer, outside_air_buffer);
		m_impl->BarrierFields(job);
		m_impl->m_current = dst;
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
					auto const w = 0.5f * (state.m_w_faces[grid.WFaceIndex(iv3{ x, y, z })] + state.m_w_faces[grid.WFaceIndex(iv3{ x, y, z + 1 })]);
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
		auto const clamp_column = [&](iv2 cell)
		{
			// Slope samples at the outermost cells use the nearest valid column, matching the shader metric helper.
			return iv2{
				std::clamp(cell.x, 0, grid.m_cell_count.x - 1),
				std::clamp(cell.y, 0, grid.m_cell_count.y - 1),
			};
		};
		for (int z = 0; z != grid.m_cell_count.z; ++z)
		{
			// Scan one horizontal layer at a time for locality in the packed state.
			for (int y = 0; y != grid.m_cell_count.y; ++y)
			{
				// Rows use contiguous x cells.
				for (int x = 0; x != grid.m_cell_count.x; ++x)
				{
					// MAC divergence uses the two faces that bound each cell on every axis.
					auto const cell = iv3{ x, y, z };
					auto const idx = grid.CellIndex(cell);
					auto const u0 = state.m_u_faces[grid.UFaceIndex(iv3{ x, y, z })];
					auto const u1 = state.m_u_faces[grid.UFaceIndex(iv3{ x + 1, y, z })];
					auto const v0 = state.m_v_faces[grid.VFaceIndex(iv3{ x, y, z })];
					auto const v1 = state.m_v_faces[grid.VFaceIndex(iv3{ x, y + 1, z })];
					auto const w0 = state.m_w_faces[grid.WFaceIndex(iv3{ x, y, z })];
					auto const w1 = state.m_w_faces[grid.WFaceIndex(iv3{ x, y, z + 1 })];
					auto const u = 0.5f * (u0 + u1);
					auto const v = 0.5f * (v0 + v1);
					auto const w = 0.5f * (w0 + w1);
					auto const column = iv2{ x, y };
					auto const dz = grid.CellHeight(column, z);
					auto const base_div = (u1 - u0) / grid.m_dx + (v1 - v0) / grid.m_dx + (w1 - w0) / dz;
					auto const x0 = clamp_column(column + iv2{ -1, 0 });
					auto const x1 = clamp_column(column + iv2{ +1, 0 });
					auto const y0 = clamp_column(column + iv2{ 0, -1 });
					auto const y1 = clamp_column(column + iv2{ 0, +1 });
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
		stats.m_rms_divergence = static_cast<float>(std::sqrt(sum_div2 / std::max(1, grid.CellCount())));
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
			return CBufAtmosphereTracers{ .m_tracer_count = m_config.m_particle_count, .m_tracer_seed = m_config.m_seed, .m_tracer_frame = m_frame, .m_tracer_max_age = m_config.m_max_age };
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
		// Copy the current GPU particle buffer to readback memory and synchronise through the supplied job.
		auto const count = m_impl->m_config.m_particle_count;
		auto const current = m_impl->m_current;
		job.m_barriers.Transition(m_impl->m_particles[current].get(), D3D12_RESOURCE_STATE_COPY_SOURCE).Commit();
		auto readback = job.m_readback.Alloc<GpuTracerParticle>(count);
		job.m_cmd_list.CopyBufferRegion(readback, m_impl->m_particles[current].get(), 0);
		job.m_barriers.Transition(m_impl->m_particles[current].get(), D3D12_RESOURCE_STATE_UNORDERED_ACCESS).Commit();
		job.Run();

		// Convert from the shader layout to the public CPU layout.
		auto particles = std::vector<AtmosphereTracerParticle>(count);
		auto const* src = readback.ptr<GpuTracerParticle>();
		for (int i = 0; i != count; ++i)
		{
			particles[i] = AtmosphereTracerParticle{
				.m_position = src[i].m_position,
				.m_temperature = src[i].m_temperature,
				.m_age = src[i].m_age,
			};
		}
		return particles;
	}

}
