//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics::atmosphere
{
	// Boundary condition applied to one outside face of the atmosphere domain.
	enum class EAtmosphereBoundary
	{
		Solid,
		Open,
	};

	// Boundary conditions for the six outside faces of the atmosphere domain.
	struct AtmosphereBoundaries
	{
		EAtmosphereBoundary m_x_min = EAtmosphereBoundary::Solid;
		EAtmosphereBoundary m_x_max = EAtmosphereBoundary::Solid;
		EAtmosphereBoundary m_y_min = EAtmosphereBoundary::Solid;
		EAtmosphereBoundary m_y_max = EAtmosphereBoundary::Solid;
		EAtmosphereBoundary m_z_min = EAtmosphereBoundary::Solid;
		EAtmosphereBoundary m_z_max = EAtmosphereBoundary::Solid;

		// Return true when every outside face is a solid wall.
		bool AllSolid() const;

		// Reject unknown boundary values at the caller boundary.
		void Validate() const;
	};

	// Air outside one boundary column. Where its wind blows into the domain through an open side, it sets the inflow wind and temperature.
	// Where its wind blows out of the domain, the inside air leaves freely.
	struct AtmosphereOutsideAir
	{
		v2 m_wind = v2::Zero();            // horizontal wind of the outside air, m/s
		float m_temperature_offset = 0.0f; // outside air temperature relative to the reference profile, K
	};

	// Grid dimensions and terrain-following metric conversion for the atmosphere solver.
	struct AtmosphereGrid
	{
		// Cell counts in X, Y (columns) and Z (layers). Every axis must exceed one, and there may be at most 32 layers.
		iv3 m_cell_count = iv3{ 0, 0, 0 };
		v4 m_origin = v4::Zero();
		float m_dx = 0.0f;
		float m_lid_z = 1.0f;
		float m_first_layer_thickness = 1.0f;
		float m_layer_stretch_power = 1.0f;
		std::vector<float> m_floor_heights;

		// Build one floor height per column from row-major caller data.
		static std::vector<float> BuildFloorHeights(iv2 cell_count, std::span<float const> floor_heights);

		// Build one floor height per column by sampling a caller-owned floor-height function at cell centres.
		static std::vector<float> BuildFloorHeights(iv2 cell_count, v4 origin, float dx, std::function<float(v2)> const& floor_height_function);

		// Return true when the grid uses a caller-supplied floor per column.
		bool TerrainFollowing() const;

		// Return the total number of cell-centred samples.
		int CellCount() const;

		// Return the number of x-face velocity samples in the MAC grid.
		int UFaceCount() const;

		// Return the number of y-face velocity samples in the MAC grid.
		int VFaceCount() const;

		// Return the number of z-face velocity samples in the MAC grid.
		int WFaceCount() const;

		// Return the number of horizontal columns.
		int ColumnCount() const;

		// Return the packed column index for 'cell'.
		int ColumnIndex(iv2 cell) const;

		// Return the number of boundary columns. Each side has one boundary column per grid column along it, packed in this order:
		// the x- side by y, the x+ side by y, the y- side by x, then the y+ side by x.
		int BoundaryColumnCount() const;

		// Build outside air for every boundary column by sampling a caller-owned function at the centre of each column's outside face.
		std::vector<AtmosphereOutsideAir> BuildOutsideAir(std::function<AtmosphereOutsideAir(v2)> const& outside_air_function) const;

		// Return the packed cell-centre index for 'cell'.
		int CellIndex(iv3 cell) const;

		// Return the packed x-face index for 'face'.
		int UFaceIndex(iv3 face) const;

		// Return the packed y-face index for 'face'.
		int VFaceIndex(iv3 face) const;

		// Return the packed z-face index for 'face'.
		int WFaceIndex(iv3 face) const;

		// Return the floor height for a column.
		float FloorHeight(iv2 cell) const;

		// Return the sigma face fraction for a vertical face index.
		float SigmaFace(int z) const;

		// Return the world-space height of a sigma face in one column.
		float FaceZ(iv2 cell, int z) const;

		// Return the world-space centre height of a cell in one column.
		float CellZ(iv2 cell, int z) const;

		// Return the physical height of one cell.
		float CellHeight(iv2 cell, int z) const;

		// Return the world-space centre of cell 'cell'.
		v4 CellCentre(iv3 cell) const;

		// Reject invalid grid geometry at the caller boundary.
		void Validate() const;
	};

	// Linear reference potential-temperature profile used by buoyancy and reset initialisation.
	struct AtmosphereReferenceProfile
	{
		float m_temperature_at_origin = 288.0f;
		float m_lapse_rate = -0.0065f;
		float m_min_temperature = 180.0f;

		// Return the reference temperature at world height 'z'.
		float Temperature(float z) const;

		// Reject invalid reference-profile values at the caller boundary.
		void Validate() const;
	};

	// Runtime constants for the atmosphere solver.
	struct AtmosphereConfig
	{
		AtmosphereGrid m_grid = {};
		AtmosphereBoundaries m_boundaries = {};
		AtmosphereReferenceProfile m_reference = {};
		float m_gravity = 9.80665f;
		float m_floor_exchange_rate = 0.0f;
		float m_lid_temperature = 270.0f;
		float m_lid_relaxation_rate = 0.0f;

		// Pressure solve effort per step. The pressure is warm-started from the previous step, so a few smoothing passes are enough to keep the field stable.
		// Two V-cycles with two pre- and post-smoothing passes are the smallest settings that remain stable on steep terrain; fewer passes let errors grow.
		int m_pressure_vcycles = 2;
		int m_pressure_pre_smooth = 2;
		int m_pressure_post_smooth = 2;
		int m_pressure_coarse_smooth = 8;

		// Width, in columns, of the band inside each open side where the wind is nudged toward the inflowing outside air.
		// The nudge is strongest at the side and fades to zero across the band. Zero applies the outside wind at the boundary faces only.
		int m_open_edge_band = 8;

		// Strength of the force that restores small swirls lost to numerical smoothing, 1/s. The added acceleration is this value times
		// the cell size times the local swirl rate, pushed toward the swirl centre. Zero disables it.
		float m_vorticity_confinement = 0.0f;

		// Reject invalid solver configuration at the caller boundary.
		void Validate() const;
	};

	// A spherical source that heats or relaxes cells inside its radius.
	struct AtmosphereHeatSource
	{
		v4 m_centre = v4::Zero();
		float m_radius = 0.0f;
		float m_heating_rate = 0.0f;
		float m_target_temperature = 0.0f;
		float m_relaxation_rate = 0.0f;
	};

	// Per-step forcing and heat data supplied by the caller.
	struct AtmosphereStepSources
	{
		std::span<AtmosphereHeatSource const> m_heat_sources = {};
		std::span<float const> m_floor_temperatures = {};
		float m_uniform_floor_temperature = 288.0f;

		// Air outside the open sides; it drives large-scale wind through the domain. Either empty, for calm outside air at the
		// reference temperature, or one entry per boundary column (see AtmosphereGrid::BoundaryColumnCount and BuildOutsideAir).
		std::span<AtmosphereOutsideAir const> m_outside_air = {};
	};

	// Full staggered MAC field state. U, V, and W are stored on x, y, and z faces; temperature and pressure are stored at cell centres.
	struct AtmosphereState
	{
		std::vector<float> m_u_faces;
		std::vector<float> m_v_faces;
		std::vector<float> m_w_faces;
		std::vector<float> m_temperature;
		std::vector<float> m_pressure;
	};

	// One cell-centred convenience sample built from the staggered field. Velocity is the average of the two adjacent faces on each axis.
	struct AtmosphereCellState
	{
		v4 m_velocity = v4::Zero();
		float m_temperature = 0.0f;
	};

	// Diagnostics returned after a CPU readback of the MAC field.
	struct AtmosphereFieldStats
	{
		float m_max_speed = 0.0f;
		float m_max_divergence = 0.0f;
		float m_rms_divergence = 0.0f;
		float m_mean_top_temperature = 0.0f;
		float m_peak_vertical_velocity = 0.0f;
	};

	// Configuration for deterministic GPU tracer particles that follow the atmosphere velocity field.
	struct AtmosphereTracerConfig
	{
		int m_particle_count = 0; // number of tracer particles
		uint32_t m_seed = 0;      // seed for the deterministic start and respawn positions
		float m_max_age = 20.0f;  // seconds before a particle respawns, so tracers do not collect in stagnant regions

		// Reject invalid tracer configuration at the caller boundary.
		void Validate() const;
	};

	// CPU-visible state for one atmosphere tracer particle.
	struct AtmosphereTracerParticle
	{
		v4 m_position = v4::Origin();
		float m_temperature = 0.0f;
		float m_age = 0.0f;
	};

	class AtmosphereTracers;

	// Standalone GPU atmosphere solver for a terrain-following MAC-grid domain.
	class AtmosphereSolver
	{
	public:
		// Create GPU buffers and initialise the field to the reference profile at rest.
		// 'shader_cache' is optional; when given, compiled kernels are reused across runs.
		AtmosphereSolver(Gpu& gpu, AtmosphereConfig config, IShaderCache* shader_cache = nullptr);

		// Release GPU resources owned by the solver.
		~AtmosphereSolver();

		// Return the immutable solver configuration.
		AtmosphereConfig const& Config() const;

		// Reset velocity to zero, pressure to zero, and temperature to the configured reference profile.
		void InitialiseReference(GpuJob& job);

		// Upload a full staggered field state. Face and cell arrays must match the configured MAC layout.
		void UploadState(GpuJob& job, AtmosphereState const& state);

		// Change column floors and remap only changed columns into the new terrain-following layers.
		void RemapFloors(GpuJob& job, std::span<float const> floor_heights);

		// Record one solver step into 'job'. The caller owns command submission and completion.
		void Step(GpuJob& job, float dt, AtmosphereStepSources const& sources);

		// Read the full staggered field after all previously recorded work in 'job' has completed.
		AtmosphereState ReadBack(GpuJob& job);

		// Return cell-centred convenience samples from a staggered field.
		std::vector<AtmosphereCellState> CellStates(AtmosphereState const& state) const;

		// Return simple CPU diagnostics for a staggered field.
		AtmosphereFieldStats Stats(AtmosphereState const& state) const;

	private:
		friend class AtmosphereTracers;
		struct Impl;
		std::unique_ptr<Impl> m_impl;
	};

	// GPU tracer particle set that advects through an AtmosphereSolver field.
	class AtmosphereTracers
	{
	public:
		// Create GPU buffers for deterministic tracer particles associated with 'solver'.
		// Tracers that leave the domain re-enter through open sides where the solved wind flows inward, roughly in proportion to the local inflow.
		// Tracers that exceed their maximum age, or leave when no side has inflow, respawn anywhere in the domain.
		// 'shader_cache' is optional; when given, compiled kernels are reused across runs.
		AtmosphereTracers(AtmosphereSolver& solver, Gpu& gpu, AtmosphereTracerConfig config, IShaderCache* shader_cache = nullptr);

		// Release GPU resources owned by the tracer set.
		~AtmosphereTracers();

		// Return the immutable tracer configuration.
		AtmosphereTracerConfig const& Config() const;

		// Reset all particles to deterministic positions inside the solver domain.
		void Initialise(GpuJob& job);

		// Advect all particles through the solver's current velocity and temperature fields.
		void Advect(GpuJob& job, float dt);

		// Read all particles after all previously recorded tracer work in 'job' has completed.
		std::vector<AtmosphereTracerParticle> ReadBack(GpuJob& job);

	private:
		struct Impl;
		std::unique_ptr<Impl> m_impl;
	};
}
