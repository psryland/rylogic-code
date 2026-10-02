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

	// Grid dimensions and terrain-following metric conversion for the atmosphere solver.
	struct AtmosphereGrid
	{
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
		float m_surface_forcing_height = 600.0f;
		int m_pressure_vcycles = 3;
		int m_pressure_pre_smooth = 4;
		int m_pressure_post_smooth = 4;
		int m_pressure_coarse_smooth = 96;

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

	// One large-scale pressure node used to build the forcing surface.
	struct AtmospherePressureNode
	{
		v2 m_centre = v2::Zero();
		v2 m_drift = v2::Zero();
		float m_strength = 0.0f;
		float m_radius = 1.0f;
		float m_age = 0.0f;
		float m_lifetime = 1.0f;
		float m_growth_rate = 0.0f;
		float m_temperature_offset = 0.0f;
	};

	// Deterministic pressure-node state that callers can copy into their own save format.
	struct AtmospherePressureForcingState
	{
		uint32_t m_seed = 0;
		float m_time_s = 0.0f;
		std::vector<AtmospherePressureNode> m_nodes;

		// Build a deterministic initial node set from a seed.
		static AtmospherePressureForcingState Create(uint32_t seed, int node_count, float domain_radius, float pressure_scale, float temperature_scale);

		// Advance nodes deterministically without sampling external state.
		void Evolve(float dt, float domain_radius, float pressure_scale, float temperature_scale);
	};

	// Per-step forcing and heat data supplied by the caller.
	struct AtmosphereStepSources
	{
		std::span<AtmosphereHeatSource const> m_heat_sources = {};
		std::span<float const> m_floor_temperatures = {};
		std::span<AtmospherePressureNode const> m_pressure_nodes = {};
		float m_uniform_floor_temperature = 288.0f;
		v4 m_reservoir_wind = v4::Zero();
		float m_reservoir_temperature_offset = 0.0f;
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

	// Standalone GPU atmosphere solver for a terrain-following MAC-grid domain.
	class AtmosphereSolver
	{
	public:
		// Create GPU buffers and initialise the field to the reference profile at rest.
		AtmosphereSolver(Gpu& gpu, AtmosphereConfig config);

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
		struct Impl;
		std::unique_ptr<Impl> m_impl;
	};
}
