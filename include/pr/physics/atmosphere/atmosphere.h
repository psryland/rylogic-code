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

	// Surface drag coefficients for the six outside faces of the atmosphere domain. Each value is a dimensionless quadratic drag coefficient in [0, 1],
	// typically 0.001 to 0.01 for land and water. Zero means the wall is frictionless. Only solid sides can have drag.
	// Drag slows the flow along the wall in the layer of cells that touches it, at the rate 'coefficient * speed / cell thickness'. The floor
	// (z_min) follows the terrain. Solid columns inside the domain do not apply drag.
	struct AtmosphereWallDrag
	{
		float m_x_min = 0.0f;
		float m_x_max = 0.0f;
		float m_y_min = 0.0f;
		float m_y_max = 0.0f;
		float m_z_min = 0.0f;
		float m_z_max = 0.0f;

		// Reject invalid coefficients, and drag on sides that are not solid, at the caller boundary.
		void Validate(AtmosphereBoundaries const& boundaries) const;
	};

	// Air outside one column. Where its wind blows into the domain through an open side or inactive mask neighbour, it sets the inflow wind and temperature.
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

		// Thickness of the bottom layer in every column, metres. Layers above it grow thicker with height so that each column reaches the lid in
		// 'm_cell_count.z' layers. A column shorter than 'm_cell_count.z' layers of this thickness uses uniform layers of this thickness instead,
		// so its top faces lie above the lid. See ColumnHeight and SigmaFace.
		float m_first_layer_thickness = 1.0f;

		// Optional floor height per column, row-major. Empty means a flat floor at 'm_origin.z'.
		// A floor at or above 'm_lid_z' makes an active column solid, which models obstacles that reach the lid. See ColumnSolid.
		std::vector<float> m_floor_heights;

		// Optional active mask per column, row-major. Empty means every column is active. A zero entry removes that column from the solve and makes
		// neighbouring active columns treat its faces as open boundaries. Floor heights in inactive columns are ignored.
		std::vector<uint8_t> m_active_columns;

		// Build one floor height per column from row-major caller data.
		static std::vector<float> BuildFloorHeights(iv2 cell_count, std::span<float const> floor_heights);

		// Build one active-mask value per column from row-major caller data. Non-zero values are active.
		static std::vector<uint8_t> BuildActiveColumns(iv2 cell_count, std::span<uint8_t const> active_columns);

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

		// Build outside air for every column by sampling a caller-owned function at the column centre. Open square sides use their edge-column entry,
		// and active/inactive mask faces use the inactive column's entry.
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

		// Return true when a column is included in the solve. Inactive columns act as open outside air to neighbouring active columns.
		bool ColumnActive(iv2 cell) const;

		// Return true when a column holds no air because it is active and its floor is at or above the lid. Solid columns are walls to the flow;
		// their layer heights are not meaningful and must not be used.
		bool ColumnSolid(iv2 cell) const;

		// Return the height spanned by the layers of a column whose floor is at 'floor_height'. This is the floor-to-lid height,
		// but never less than 'm_cell_count.z' layers of 'm_first_layer_thickness', so every layer has a positive height.
		float ColumnHeight(float floor_height) const;

		// Return the fraction of the height of a column, from 0 at the floor to 1 at the top, at vertical face index 'z'.
		// The layers of the column follow a power curve whose exponent makes the bottom layer exactly 'm_first_layer_thickness' thick.
		// 'column_height' must come from ColumnHeight.
		float SigmaFace(float column_height, int z) const;

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
		AtmosphereWallDrag m_wall_drag = {};
		AtmosphereReferenceProfile m_reference = {};
		float m_gravity = 9.80665f;
		float m_floor_exchange_rate = 0.0f;
		float m_lid_temperature = 270.0f;
		float m_lid_relaxation_rate = 0.0f;

		// Pressure solve effort per step. The pressure is warm-started from the previous step, so a few smoothing passes are enough to keep the field stable.
		// Fewer than two pre- and post-smoothing passes let errors grow. Two V-cycles are a conservative default; grids whose flow changes little
		// between steps may give the same result with one V-cycle and fewer coarse-level passes.
		int m_pressure_vcycles = 2;
		int m_pressure_pre_smooth = 2;
		int m_pressure_post_smooth = 2;
		int m_pressure_coarse_smooth = 8;

		// Width, in columns, of the band inside each open side where the wind is nudged toward the inflowing outside air.
		// The nudge is strongest at the side and fades to zero across the band. Zero applies the outside wind at the boundary faces only.
		// The outside wind is the same at every height, which only conserves mass where the floor is level. A floor that rises or falls
		// across the band changes the column depth under a fixed wind, and the pressure solve then creates false vertical flow to balance it.
		// Callers should keep the floor level across the band of each open side.
		int m_open_edge_band = 8;

		// Strength of the force that restores small swirls lost to numerical smoothing, 1/s. The added acceleration is this value times
		// the cell size times the local swirl rate, pushed toward the swirl centre. Zero disables it.
		float m_vorticity_confinement = 0.0f;

		// Vertical eddy viscosity of the horizontal wind, m^2/s. It mixes horizontal momentum between neighbouring layers, so drag at the
		// floor or lid reaches the layers above or below it, and fast layers share their speed with slow ones. Zero disables it.
		float m_vertical_viscosity = 0.0f;

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
		float m_max_age = 20.0f;  // seconds before a particle respawns, so tracers do not collect in stagnant regions. Initial ages are spread over [0, m_max_age) so tracers do not all respawn together

		// Height profile for new tracers, as relative densities over the column fraction (0 at the floor, 1 at the lid). The density falls linearly
		// from 'm_ground_density' at the floor to 'm_break_density' at 'm_break_height', then is 'm_upper_density' up to the lid. The profile is
		// normalised, so only the ratios matter and the total count is unchanged. Equal densities give an even spread.
		float m_ground_density = 1.0f;
		float m_break_density = 1.0f;
		float m_upper_density = 1.0f;
		float m_break_height = 0.5f;   // column fraction in (0, 1]

		// Reject invalid tracer configuration at the caller boundary.
		void Validate() const;
	};

	// CPU-visible state for one atmosphere tracer particle.
	struct AtmosphereTracerParticle
	{
		v4 m_position = v4::Origin();
		float m_temperature = 0.0f;
		float m_age = 0.0f;
		float m_speed = 0.0f;     // air speed that moved the particle in the last step, m/s
	};

	// The air sampled at one world-space point. Points outside the air of the domain have 'm_inside' false and zero velocity and temperature.
	struct AtmosphereProbeSample
	{
		v4 m_velocity = v4::Zero(); // interpolated air velocity, m/s
		float m_temperature = 0.0f; // interpolated air temperature, K
		bool m_inside = false;      // true when the point is in an air column, between its floor and the lid
	};

	class AtmosphereTracers;
	class AtmosphereProbes;

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

		// Set the temperature of the floor under each column, K, in AtmosphereGrid column order. The lowest layer relaxes toward it at
		// AtmosphereConfig::m_floor_exchange_rate. Initially the reference temperature at each column's floor height; RemapFloors does not change it.
		// The values are kept on the CPU and uploaded by the next Step, so this may be called while a recorded step is still running.
		void SetFloorTemperatures(std::span<float const> floor_temperatures);

		// Set the air outside the open faces, one entry per column (see AtmosphereGrid::ColumnCount and BuildOutsideAir); it drives large-scale
		// wind through the domain. Square sides use the edge-column entry. Active/inactive mask faces use the inactive column's entry.
		// The open-edge sponge is still measured from the square domain sides. Initially calm air at the reference temperature.
		// The values are kept on the CPU and uploaded by the next Step, so this may be called while a recorded step is still running.
		void SetOutsideAir(std::span<AtmosphereOutsideAir const> outside_air);

		// Record one solver step into 'job', with at most 64 'heat_sources' active for this step only. The caller owns command submission and completion.
		void Step(GpuJob& job, float dt, std::span<AtmosphereHeatSource const> heat_sources = {});

		// Read the full staggered field after all previously recorded work in 'job' has completed.
		AtmosphereState ReadBack(GpuJob& job);

		// Return cell-centred convenience samples from a staggered field.
		std::vector<AtmosphereCellState> CellStates(AtmosphereState const& state) const;

		// Return simple CPU diagnostics for a staggered field.
		AtmosphereFieldStats Stats(AtmosphereState const& state) const;

	private:
		friend class AtmosphereTracers;
		friend class AtmosphereProbes;
		struct Impl;
		std::unique_ptr<Impl> m_impl;
	};

	// GPU tracer particle set that advects through an AtmosphereSolver field.
	// Particles live in a ring of 'SlotCount' GPU buffers. Each 'Initialise' or 'Advect' writes a new slot, which then becomes the current slot.
	// Other GPU consumers, such as a renderer on another queue, may read a slot directly: 'Hold' the slot so that later steps do not write it, then
	// 'Release' it with a fence value that is signalled when the consumer's reads have finished. Each slot buffer is an array of 'GpuParticle'.
	class AtmosphereTracers
	{
	public:
		// The number of particle buffers in the ring. One is current and the step writes one, so at most 'SlotCount - 2' slots can be held.
		static constexpr int SlotCount = 4;

		// GPU layout of one particle in a slot buffer.
		struct GpuParticle
		{
			v4 m_position;       // world position, w = 1
			float m_temperature; // air temperature at the particle, K
			float m_age;         // seconds since the particle last spawned
			float m_speed;       // air speed that moved the particle in the last step, m/s
			float m_pad;
		};
		static_assert(sizeof(GpuParticle) == 32);

		// Create GPU buffers for deterministic tracer particles associated with 'solver'.
		// Tracers that leave the domain re-enter through open sides where the solved wind flows inward, roughly in proportion to the local inflow.
		// Tracers that exceed their maximum age, or leave when no side has inflow, respawn anywhere in the domain.
		// 'shader_cache' is optional; when given, compiled kernels are reused across runs.
		AtmosphereTracers(AtmosphereSolver& solver, Gpu& gpu, AtmosphereTracerConfig config, IShaderCache* shader_cache = nullptr);

		// Release GPU resources owned by the tracer set.
		~AtmosphereTracers();

		// Return the immutable tracer configuration.
		AtmosphereTracerConfig const& Config() const;

		// Reset all particles to deterministic positions inside the solver domain. Writes a new current slot; see 'Advect'.
		void Initialise(GpuJob& job);

		// Advect all particles through the solver's current velocity and temperature fields.
		// The result is written to a slot that is neither current nor held, which then becomes current. If that slot was released with a fence,
		// 'job's queue waits on the GPU for the fence value before running later submissions. Throws if every non-current slot is held.
		void Advect(GpuJob& job, float dt);

		// Return the index of the slot that holds the particles written by the last recorded 'Initialise' or 'Advect'.
		int CurrentSlot() const;

		// Return the particle buffer of 'slot'. The buffer holds 'Config().m_particle_count' 'GpuParticle' records.
		ID3D12Resource* Slot(int slot) const;

		// Prevent later steps from writing 'slot' until it is released. A slot may be held more than once; each hold needs its own release.
		void Hold(int slot);

		// Release one hold on 'slot'. 'fence' and 'value' identify when the consumer's GPU reads finish; a null 'fence' means they already have.
		// The next step that writes the slot waits on the GPU for every fence value released since the slot was last written.
		void Release(int slot, ID3D12Fence* fence, uint64_t value);

		// Read all particles after all previously recorded tracer work in 'job' has completed. Blocks until 'job' has run.
		std::vector<AtmosphereTracerParticle> ReadBack(GpuJob& job);

		// Record a copy of all particles into 'job' without submitting it. Collect the copy with 'ResolveReadBack' once the submission has completed.
		void RecordReadBack(GpuJob& job);

		// Return the particles copied by the last 'RecordReadBack'. Call after that submission has completed and before 'job' records another read back.
		std::vector<AtmosphereTracerParticle> ResolveReadBack();

	private:
		struct Impl;
		std::unique_ptr<Impl> m_impl;
	};

	// Samples the air of an AtmosphereSolver at a few world-space points on the GPU, without reading back the whole field.
	// 'Record' adds the sampling and a small copy to a caller-owned job, so the results can be collected without waiting once that job has run.
	class AtmosphereProbes
	{
	public:
		// The largest number of points that one 'Record' can sample.
		static constexpr int MaxPoints = 64;

		// Compile the sampling kernel and allocate the result buffer for 'solver'.
		// 'shader_cache' is optional; when given, the compiled kernel is reused across runs.
		AtmosphereProbes(AtmosphereSolver& solver, Gpu& gpu, IShaderCache* shader_cache = nullptr);

		// Release GPU resources owned by the probes.
		~AtmosphereProbes();

		// Record a sample of the solver's current field at each of 'points' (world space, at most 'MaxPoints', all finite), and a copy of the results, into 'job'.
		// The samples see all solver work recorded into 'job' before this call. Collect them with 'Resolve' once the submission has completed.
		void Record(GpuJob& job, std::span<v4 const> points);

		// Return the samples copied by the last 'Record', in point order. Call after that submission has completed and before 'job' records another read back.
		std::vector<AtmosphereProbeSample> Resolve();

	private:
		struct Impl;
		std::unique_ptr<Impl> m_impl;
	};
}
