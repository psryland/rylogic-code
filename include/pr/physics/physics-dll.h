//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
// Dependency-minimal, versioned C ABI for the native physics engine.
#pragma once

#ifdef PHYSICS_EXPORTS
#define PHYSICS_API __declspec(dllexport)
#else
#define PHYSICS_API __declspec(dllimport)
#endif

#include <cstdint>
#include <type_traits>
#include <utility>
#include <windows.h>

namespace pr::physics
{
	using DllHandle = unsigned char const*;

	template <typename FuncType>
	struct Callback
	{
		using FuncCB = FuncType;
		using CtxPtr = union { void const* cp; void* p; };

		CtxPtr m_ctx = {};
		FuncCB m_cb = {};

		template <typename... Args>
		auto operator()(Args&&... args) const
		{
			return m_cb(m_ctx.p, std::forward<Args>(args)...);
		}
		explicit operator bool() const
		{
			return m_cb != nullptr;
		}
		friend bool operator == (Callback lhs, Callback rhs)
		{
			return lhs.m_cb == rhs.m_cb && lhs.m_ctx.cp == rhs.m_ctx.cp;
		}
	};
	using ReportErrorCB = Callback<void(__stdcall*)(void* ctx, char const* msg, char const* filepath, int line, int64_t pos)>;
}

namespace pr::physics
{
	inline constexpr std::uint32_t PHYSICS_API_VERSION = 0x00030300U;
	inline constexpr std::uint32_t PHYSICS_STRUCT_VERSION = 2U;
	inline constexpr std::uint32_t PHYSICS_CHECKPOINT_VERSION = 3U;

	// Reported in place of a compound child index when the shape involved has no child identity.
	inline constexpr std::uint32_t PHYSICS_NO_CHILD = 0xFFFFFFFFU;

	// Maximum convex leaves addressable by one compound shape.
	inline constexpr std::uint32_t PHYSICS_MAX_COMPOUND_CHILDREN = 1024U;

	using EngineHandle = std::uint64_t;
	using ShapeHandle = std::uint64_t;
	using BodyHandle = std::uint64_t;
	using ArticulationHandle = std::uint64_t;
	using PersistentConstraintHandle = std::uint64_t;
	using AtmosphereHandle = std::uint64_t;

	enum class EStatus : std::int32_t
	{
		Success = 0,
		InvalidArgument = 1,
		InvalidStruct = 2,
		InvalidHandle = 3,
		StaleHandle = 4,
		WrongThread = 5,
		StepPending = 6,
		NoStepPending = 7,
		BufferTooSmall = 8,
		IncompatibleVersion = 9,
		DeviceRemoved = 10,
		InternalError = 11,
	};

	enum class EStructId : std::int32_t
	{
		EngineConfig = 1,
		ShapeCommon = 2,
		SphereShape = 3,
		BoxShape = 4,
		LineShape = 5,
		TriangleShape = 6,
		BodyDesc = 7,
		BodyState = 8,
		BodyCommand = 9,
		BodySnapshot = 10,
		Event = 11,
		Diagnostics = 12,
		Material = 13,
		ArticulationDesc = 14,
		ArticulationLink = 15,
		ArticulationJoint = 16,
		ArticulationState = 17,
		ArticulationLinkState = 18,
		D6Constraint = 19,
		Terrain = 20,
		CylindricalBoundary = 21,
		Water = 22,
		WaterBathymetry = 23,
		AtmosphereDesc = 24,
		AtmosphereStep = 25,
		AtmosphereStats = 26,
	};

	// One explicit terrain frequency band; roundness and weight gain apply to the mountain band.
	struct TerrainBand
	{
		double amplitude, wavelength, lacunarity, persistence, roundness, weight_gain;
		std::int32_t octaves, reserved;
	};

	enum class EMotionType : std::int32_t
	{
		Static = 0,
		Dynamic = 1,
		Kinematic = 2,
	};

	enum class EMassMode : std::int32_t
	{
		ExplicitInertia = 0,
		Mass = 1,
		Density = 2,
	};

	enum class ECommand : std::int32_t
	{
		SetTransform = 0,
		SetVelocity = 1,
		SetMomentum = 2,
		SetForce = 3,
		ApplyForce = 4,
		ApplyImpulse = 5,
		SetGravity = 6,
		SetKinematicTransform = 7,
		SetEnabled = 8,
		Wake = 9,
		Sleep = 10,
	};

	enum class EEvent : std::int32_t
	{
		Contact = 0,
		Wake = 1,
		Sleep = 2,
		ConstraintBreak = 3,
		CoupledConstraintFailure = 4,
		WorldContact = 5,
	};

	// Selects whether an articulation root is fixed to world or contributes a floating six-velocity base.
	enum class EArticulationRoot : std::int32_t
	{
		Fixed = 0,
		Floating = 1,
	};

	// Selects the scalar screw motion represented by one articulation joint coordinate.
	enum class EArticulationAxis : std::int32_t
	{
		Revolute = 0,
		Prismatic = 1,
	};

	// Selects the dynamics owner addressed by a persistent constraint endpoint.
	enum class EConstraintEndpoint : std::int32_t
	{
		World = 0,
		RigidBody = 1,
		ArticulationLink = 2,
	};

	// Selects how one translational or rotational D6 coordinate contributes to the projected solve.
	enum class EConstraintMode : std::int32_t
	{
		Free = 0,
		Locked = 1,
		Limited = 2,
		Driven = 3,
	};

	enum class EBodyFlags : std::uint32_t
	{
		None = 0,
		Enabled = 1U << 0,
		Sleeping = 1U << 1,
		NeverSleep = 1U << 2,
	};

	// Bit flags controlling articulation participation and whole-tree sleeping.
	enum class EArticulationFlags : std::uint32_t
	{
		None = 0,
		Enabled = 1U << 0,
		Sleeping = 1U << 1,
		NeverSleep = 1U << 2,
	};

	// Bit flags controlling immutable per-link collision policy.
	enum class EArticulationLinkFlags : std::uint32_t
	{
		None = 0,
		CollideParent = 1U << 0,
		CollideSelf = 1U << 1,
	};

	// Bit flags controlling persistent-constraint participation and connected-body collision policy.
	enum class EConstraintFlags : std::uint32_t
	{
		None = 0,
		Enabled = 1U << 0,
		CollideConnected = 1U << 1,
	};

	struct StructHeader
	{
		std::uint32_t size;
		std::uint32_t version;
	};

	// Complete immutable baseline terrain settings. Zero spacing selects the engine's shared surface-sampling default.
	struct TerrainDesc
	{
		StructHeader header;
		std::uint32_t seed;
		std::int32_t material_id;
		double supported_coordinate, sea_level_bias, uplift_height, mountain_base, basin_depth, basin_threshold;
		TerrainBand regional_base, region_selector, region_uplift, domain_warp, plains, hills, mountains, basin_selector;
		float surface_spacing;
		std::uint32_t reserved;
	};

	// Infinite-height inward cylinder and surface-sample spacing; all distances are metres.
	struct CylindricalBoundaryDesc
	{
		StructHeader header;
		double centre_x, centre_y, radius;
		std::int32_t material_id;
		float surface_spacing;
	};
	static_assert(sizeof(CylindricalBoundaryDesc) == 40);

	// One water-surface element with the same layout as 'terrain::water::shared::WaterFieldElement'; see water_field_types.hlsli for the fields.
	struct WaterElement
	{
		std::int32_t info[4];
		float position[4];
		float wave[4];
		float timing[4];
	};
	static_assert(sizeof(WaterElement) == 64);

	// A water surface with buoyancy and drag for every dynamic body. Density is kg/m³ and drag rates are 1/s.
	// The surface is the still-water 'level' plus 'element_count' elements from 'elements'; the engine copies them.
	// Wave amplitudes are corrected for depth only after terrain heights are supplied with Physics_EngineWaterBathymetrySet.
	// 'breaking_ratio' limits the total wave amplitude to half this fraction of the depth. 'repeat_period' (s) is the period
	// of the wave motion; the engine samples waves at time modulo this period, so element frequencies should repeat within it.
	// Zero means no wrap.
	struct WaterDesc
	{
		StructHeader header;
		double level;
		float density;
		float linear_drag_rate;
		float quadratic_drag_coefficient;
		float angular_drag_rate;
		float breaking_ratio;
		float repeat_period;
		WaterElement const* elements;
		std::int32_t element_count;
		std::int32_t reserved;
	};
	static_assert(sizeof(WaterDesc) == 56);

	// Terrain heights sampled on a regular world-space grid, used to correct waves for water depth.
	// Node (i, j) is at origin + (i, j) * cell_size and its height is heights[j * width + i]. The engine copies the heights.
	struct WaterBathymetryDesc
	{
		StructHeader header;
		double origin_x, origin_y, cell_size;
		std::int32_t width, height;
		float const* heights;
	};
	static_assert(sizeof(WaterBathymetryDesc) == 48);

	// The fixed component layout of a wind-driven wave spectrum. Gravity (m/s²) and the repeat period (s) are positive. The wavelength range
	// [min_wavelength, max_wavelength] (m, 0 < min < max) is split into 'bands' equal log-wavelength bands of 'directions' components each.
	// The component count, bands * directions, is at most 64.
	struct WaveSpectrumDesc
	{
		float gravity, repeat_period;
		float min_wavelength, max_wavelength;
		std::int32_t bands, directions;
	};
	static_assert(sizeof(WaveSpectrumDesc) == 24);

	// Boundary condition of one outside face of an atmosphere domain.
	enum class EAtmosphereBoundary : std::int32_t
	{
		Solid = 0,
		Open = 1,
	};

	// Indices of the six outside faces of an atmosphere domain, used by 'AtmosphereDesc::boundaries' and 'AtmosphereDesc::wall_drag'.
	enum class EAtmosphereSide : std::int32_t
	{
		XMin = 0,
		XMax = 1,
		YMin = 2,
		YMax = 3,
		ZMin = 4,
		ZMax = 5,
	};

	// A terrain-following GPU air solver and its optional tracer particles. Distances are metres, temperatures are kelvin, and rates are 1/s.
	// The domain has 'cell_count_x * cell_count_y' columns of 'cell_count_z' layers (each count above one, at most 32 layers) with square columns
	// of size 'dx' starting at 'origin'. Each column runs from its floor to 'lid_z'. 'floor_heights' is null for a flat floor at 'origin_z', or
	// one height per column in row-major order (x fastest); a floor at or above the lid makes the column solid. The heights are copied.
	// 'boundaries' and 'wall_drag' are indexed by EAtmosphereSide; drag is a quadratic coefficient in [0, 1] and only solid sides may have drag.
	// The reference profile 'reference_temperature + lapse_rate * (z - origin_z)', limited below by 'min_temperature', sets the initial air at rest.
	// 'tracer_count' particles follow the wind for diagnostics; zero disables them. Tracer heights are spread with the relative densities
	// 'tracer_ground_density' at the floor, 'tracer_break_density' at the column fraction 'tracer_break_height', and 'tracer_upper_density' above it.
	// The pressure and stability fields match 'atmosphere::AtmosphereConfig', which documents their effect.
	struct AtmosphereDesc
	{
		StructHeader header;
		std::int32_t cell_count_x, cell_count_y, cell_count_z;
		float origin_x, origin_y, origin_z;
		float dx, lid_z, first_layer_thickness;
		float const* floor_heights;
		std::uint8_t const* active_columns;
		EAtmosphereBoundary boundaries[6];
		float wall_drag[6];
		float reference_temperature, lapse_rate, min_temperature;
		float gravity, floor_exchange_rate, lid_temperature, lid_relaxation_rate;
		std::int32_t pressure_vcycles, pressure_pre_smooth, pressure_post_smooth, pressure_coarse_smooth, open_edge_band;
		float vorticity_confinement, vertical_viscosity;
		std::int32_t tracer_count;
		std::uint32_t tracer_seed;
		float tracer_max_age, tracer_ground_density, tracer_break_density, tracer_upper_density, tracer_break_height;
		std::int32_t reserved;
	};
	static_assert(sizeof(AtmosphereDesc) == 200);

	// A sphere that heats air at 'heating_rate' (K/s) and relaxes it towards 'target_temperature' at 'relaxation_rate' (1/s).
	struct AtmosphereHeatSource
	{
		float centre_x, centre_y, centre_z, radius;
		float heating_rate, target_temperature, relaxation_rate;
		float reserved;
	};
	static_assert(sizeof(AtmosphereHeatSource) == 32);

	// Air outside one column: its horizontal wind (m/s) and its temperature relative to the reference profile (K).
	struct AtmosphereOutsideAir
	{
		float wind_x, wind_y, temperature_offset;
		float reserved;
	};
	static_assert(sizeof(AtmosphereOutsideAir) == 16);

	// The inputs to one atmosphere step of 'dt' seconds; the heat sources are copied and apply to this step only. At most 64 heat sources.
	struct AtmosphereStepDesc
	{
		StructHeader header;
		float dt;
		std::int32_t heat_source_count;
		AtmosphereHeatSource const* heat_sources;
	};
	static_assert(sizeof(AtmosphereStepDesc) == 24);

	// One atmosphere tracer particle: world position (m), air temperature (K), age (s), and the air speed that last moved it (m/s).
	struct AtmosphereTracerParticle
	{
		float x, y, z;
		float temperature, age, speed;
	};
	static_assert(sizeof(AtmosphereTracerParticle) == 24);

	// The air at one cell centre: velocity (m/s, averaged from the cell faces) and temperature (K).
	struct AtmosphereCellState
	{
		float velocity_x, velocity_y, velocity_z;
		float temperature;
	};
	static_assert(sizeof(AtmosphereCellState) == 16);

	// Simple diagnostics of a completed atmosphere field.
	struct AtmosphereStats
	{
		StructHeader header;
		float max_speed, max_divergence, rms_divergence, mean_top_temperature, peak_vertical_velocity;
		std::int32_t reserved;
	};
	static_assert(sizeof(AtmosphereStats) == 32);

	struct Vector4
	{
		float x;
		float y;
		float z;
		float w;
	};

	struct Matrix4
	{
		Vector4 x;
		Vector4 y;
		Vector4 z;
		Vector4 w;
	};

	struct SpatialVector
	{
		Vector4 angular;
		Vector4 linear;
	};

	struct InertiaProperties
	{
		Vector4 diagonal;
		Vector4 products;
		Vector4 centre_of_mass_and_mass;
	};

	struct Config
	{
		StructHeader header;
		std::int32_t max_collision_pairs;
		std::int32_t sleeping_enabled;
		float sleep_velocity_threshold_linear;
		float sleep_velocity_threshold_angular;
		float sleep_delay_seconds;
		std::int32_t solver_iterations;
		std::int32_t push_out_iterations;
		float broadphase_aabb_margin;
		float contact_sort_propagation_scale;
		std::int32_t contact_sort_shock_iterations;
		float contact_sort_shock_alignment;
		float contact_sort_shock_min_strength;
		float contact_sort_shock_decay;
		float penetration_slop;
		float velocity_baumgarte;
		float position_slop;
		float position_baumgarte;
		float contact_slop_scale;
		float support_contact_slop_scale;
		float warm_start_scale;
		float deep_penetration_threshold;
		float deep_penetration_range;
		float deep_penetration_baumgarte_min;
		float deep_penetration_baumgarte_max;
		std::int32_t selective_refresh_passes;
		std::int32_t selective_refresh_max_pairs;
		std::int32_t selective_refresh_body_limit;
		std::int32_t selective_refresh_contact_limit;
		std::int32_t selective_refresh_solver_iterations;
		std::int32_t selective_refresh_position_iterations;
		float selective_refresh_bias_scale;
		float selective_refresh_restitution_scale;
		std::int32_t selective_refresh_adaptive_body_limit;
		std::int32_t selective_refresh_adaptive_solver_iterations;
		std::int32_t selective_refresh_support_only;
		std::int32_t selective_refresh_resolve_support_only;
		float selective_refresh_depth_slop;
		float selective_refresh_support_depth_slop;
		float selective_refresh_closing_speed_slop;
		float selective_refresh_support_alignment;
		float selective_refresh_aabb_margin;
		std::int32_t max_collision_events;
		std::int32_t max_internal_substeps;
		float constraint_relaxation;
		float constraint_coupled_relaxation;
		std::int32_t constraint_coupled_backtrack_limit;
		float constraint_position_relaxation;
		float constraint_position_beta;
		float constraint_max_position_speed;
		float constraint_regularization;
		float constraint_warm_start_factor;
	};

	struct MaterialProperties
	{
		StructHeader header;
		std::int32_t id;
		float static_friction;
		float normal_elasticity;
		float tangential_elasticity;
		float torsional_elasticity;
		float density;
	};

	struct ShapeCommon
	{
		StructHeader header;
		Matrix4 shape_to_root;
		std::int32_t material_id;
		std::uint32_t flags;
	};

	struct SphereShape
	{
		ShapeCommon common;
		float radius;
		std::int32_t hollow;
	};

	struct BoxShape
	{
		ShapeCommon common;
		Vector4 dimensions;
	};

	struct LineShape
	{
		ShapeCommon common;
		float length;
		float radius;
	};

	struct TriangleShape
	{
		ShapeCommon common;
		Vector4 a;
		Vector4 b;
		Vector4 c;
	};

	struct BodyDesc
	{
		StructHeader header;
		ShapeHandle shape;
		Matrix4 object_to_world;
		InertiaProperties inertia;
		SpatialVector momentum;
		Vector4 gravity;
		std::uint64_t user_tag;
		EMotionType motion_type;
		EMassMode mass_mode;
		float mass_or_density;
		EBodyFlags flags;
	};

	struct BodyState
	{
		StructHeader header;
		BodyHandle body;
		ShapeHandle shape;
		Matrix4 object_to_world;
		InertiaProperties inertia;
		SpatialVector momentum;
		SpatialVector velocity;
		SpatialVector force;
		Vector4 gravity;
		std::uint64_t user_tag;
		EMotionType motion_type;
		EBodyFlags flags;
	};

	struct BodyCommand
	{
		StructHeader header;
		BodyHandle body;
		ECommand type;
		std::uint32_t flags;
		Matrix4 transform;
		SpatialVector value;
		Vector4 at;
	};

	struct BodySnapshot
	{
		StructHeader header;
		BodyHandle body;
		ShapeHandle shape;
		Matrix4 object_to_world;
		SpatialVector momentum;
		SpatialVector velocity;
		std::uint64_t user_tag;
		EMotionType motion_type;
		EBodyFlags flags;
	};

	// Immutable articulation topology metadata supplied with ordered link and joint arrays.
	struct ArticulationDesc
	{
		StructHeader header;
		Matrix4 root_to_world;
		SpatialVector root_velocity;
		std::uint64_t user_tag;
		std::uint32_t link_count;
		EArticulationRoot root_type;
		EArticulationFlags flags;
		std::uint32_t reserved;
	};

	// Immutable mass, shape, parent, and collision policy for one topologically ordered articulation link.
	struct ArticulationLinkProperties
	{
		StructHeader header;
		ShapeHandle shape;
		InertiaProperties inertia;
		Matrix4 shape_to_link;
		std::int32_t parent_index;
		EArticulationLinkFlags flags;
	};

	// Immutable reduced-coordinate joint joining ordered link i+1 to its earlier parent link.
	struct ArticulationJointProperties
	{
		StructHeader header;
		Matrix4 joint_to_parent;
		Matrix4 joint_to_child;
		Vector4 axes[6];
		float initial_positions[6];
		float initial_velocities[6];
		EArticulationAxis axis_types[6];
		std::uint32_t dof_count;
		std::uint32_t reserved;
	};

	// Whole-tree mutable state plus the dimensions of the flattened non-root joint arrays.
	struct ArticulationState
	{
		StructHeader header;
		ArticulationHandle articulation;
		Matrix4 root_to_world;
		SpatialVector root_velocity;
		SpatialVector root_force;
		std::uint64_t user_tag;
		std::uint32_t link_count;
		std::uint32_t joint_dof_count;
		EArticulationFlags flags;
		std::uint32_t reserved;
	};

	// Current state and persistent external fields for one topologically indexed articulation link.
	struct ArticulationLinkState
	{
		StructHeader header;
		ArticulationHandle articulation;
		std::uint32_t link_index;
		std::int32_t parent_index;
		ShapeHandle shape;
		Matrix4 link_to_world;
		SpatialVector velocity;
		SpatialVector acceleration;
		SpatialVector external_force;
		Vector4 gravity;
	};

	// One endpoint-local constraint frame; object_handle is a body or articulation handle according to type.
	struct ConstraintFrameProperties
	{
		EConstraintEndpoint type;
		std::uint32_t link_index;
		std::uint64_t object_handle;
		Matrix4 constraint_to_body;
	};

	// Scalar D6 coordinate configuration using force units for linear rows and torque units for angular rows.
	struct ConstraintAxisProperties
	{
		EConstraintMode mode;
		float lower_limit;
		float upper_limit;
		float target_position;
		float target_velocity;
		float stiffness;
		float damping;
		float max_force;
	};

	// General persistent six-degree-of-freedom constraint between two stable engine-owned endpoints.
	struct D6ConstraintProperties
	{
		StructHeader header;
		ConstraintFrameProperties frame_a;
		ConstraintFrameProperties frame_b;
		ConstraintAxisProperties linear[3];
		ConstraintAxisProperties angular[3];
		float break_force;
		float break_torque;
		EConstraintFlags flags;
		std::uint32_t reserved;
	};

	// Completed contact geometry and lifecycle diagnostics. WorldContact has exactly one zero body handle for the
	// engine-owned terrain/boundary endpoint; Contact has two caller-owned handles. Normals point from A towards B.
	struct Event
	{
		StructHeader header;
		EEvent type;
		std::uint32_t point_count;
		BodyHandle body_a;
		BodyHandle body_b;
		Vector4 normal;
		Vector4 points[4];
		float depth;
		std::int32_t material_a;
		std::int32_t material_b;

		// Identify which child of a compound shape produced the contact, in the declaration order the compound
		// was built with, so a caller can attribute a contact to a specific part rather than only to a material.
		// A body whose shape is a primitive root has no child identity and reports PHYSICS_NO_CHILD, as do
		// events that are not contacts.
		std::uint32_t child_a;
		std::uint32_t child_b;
		PersistentConstraintHandle constraint;
		float break_force;
		float break_torque;
		std::int32_t substep_index;
		std::uint32_t reserved;
		std::uint32_t failure_flags;
		std::int32_t failure_phase;
		std::int32_t failure_island_index;
		std::int32_t failure_iteration_count;
		float failure_relaxation;
		float failure_merit_change;
	};

	struct StepProfile
	{
		double new_frame_ms;
		double pack_ms;
		double upload_ms;
		double external_forces_ms;
		double integrate_ms;
		double sleep_wake_ms;
		double broadphase_ms;
		double collide_ms;
		double resolve_ms;
		double selective_ms;
		double sleep_update_ms;
		double readback_ms;
		double gpu_run_ms;
		double gpu_prepare_ms;
		double gpu_execute_ms;
		double gpu_wait_ms;
		double gpu_reset_ms;
		double unpack_ms;
	};

	// Common work and storage accounting for one optional GPU feature lane.
	struct FeatureResourceDiagnostics
	{
		std::int32_t dispatch_count;
		std::uint32_t reserved;
		std::uint64_t logical_bytes;
		std::uint64_t allocated_bytes;
	};

	// Stable-slot, scratch, and break-latch costs for persistent constraints.
	struct ConstraintFeatureDiagnostics
	{
		std::int32_t declared_count;
		std::int32_t active_count;
		std::int32_t breakable_count;
		std::int32_t slot_capacity;
		std::int32_t body_capacity;
		std::int32_t break_capacity;
		FeatureResourceDiagnostics resources;
	};

	// Packed topology and optional GPU resource costs for reduced-coordinate articulations.
	struct ArticulationFeatureDiagnostics
	{
		std::int32_t articulation_count;
		std::int32_t link_count;
		std::int32_t dof_count;
		std::int32_t position_count;
		std::int32_t velocity_count;
		std::int32_t articulation_capacity;
		std::int32_t link_capacity;
		std::int32_t dof_capacity;
		std::int32_t position_capacity;
		std::int32_t velocity_capacity;
		FeatureResourceDiagnostics resources;
	};

	// Topology bounds and costs for articulation-coupled persistent constraints and contacts.
	struct CoupledFeatureDiagnostics
	{
		std::int32_t constraint_count;
		std::int32_t constraint_slot_capacity;
		std::int32_t target_capacity;
		std::int32_t island_capacity;
		std::int32_t island_block_capacity;
		std::int32_t contact_capacity;
		std::int32_t contact_target_capacity;
		std::int32_t contact_participant_capacity;
		std::int32_t contact_tree_capacity;
		std::uint32_t reserved;
		FeatureResourceDiagnostics resources;
	};

	// Packed frame output, submission-boundary counts, and sole-readback accounting.
	struct FrameOutputFeatureDiagnostics
	{
		std::int32_t body_count;
		std::int32_t event_capacity;
		std::int32_t articulation_count;
		std::int32_t constraint_break_count;
		std::int32_t coupled_failure_count;
		std::int32_t dispatch_count;
		std::int32_t readback_count;
		std::uint32_t reserved1;
		std::uint64_t logical_bytes;
		std::uint64_t allocated_bytes;
		std::uint64_t readback_bytes;
	};

	// First bounded failure from the most recently completed or rejected frame.
	struct StepFailureDiagnostics
	{
		std::int32_t reason;
		std::int32_t substep_index;
		std::int32_t item_index;
		std::int32_t status;
		std::int32_t iteration_count;
		std::uint32_t reserved;
		std::uint64_t identity;
		float residual;
		std::uint32_t reserved1;
	};

	struct Diagnostics
	{
		StructHeader header;
		StepProfile profile;
		std::uint64_t submitted_step;
		std::uint64_t completed_step;
		std::uint64_t state_checksum;
		std::int32_t body_count;
		std::int32_t shape_count;
		std::int32_t pair_count;
		std::int32_t contact_count;
		std::int32_t max_pairs;
		std::int32_t max_contacts;
		std::int32_t collision_event_count;
		std::int32_t collision_event_capacity;
		std::int32_t collision_event_overflow_substep;
		std::int32_t step_pending;
		std::int32_t device_removed_reason;
		std::int32_t articulation_count;
		std::int32_t constraint_count;
		ConstraintFeatureDiagnostics constraints;
		ArticulationFeatureDiagnostics articulations;
		CoupledFeatureDiagnostics coupled;
		FrameOutputFeatureDiagnostics frame_output;
		StepFailureDiagnostics failure;
	};

	static_assert(std::is_standard_layout_v<Config>);
	static_assert(std::is_standard_layout_v<BodyState>);
	static_assert(sizeof(Vector4) == 16);
	static_assert(sizeof(Matrix4) == 64);
	static_assert(sizeof(SpatialVector) == 32);
	static_assert(sizeof(InertiaProperties) == 48);
} // namespace pr::physics

extern "C"
{
	// DLL context lifecycle. Calls are reference counted and must be paired.
	PHYSICS_API pr::physics::DllHandle __stdcall Physics_Initialise(pr::physics::ReportErrorCB global_error_cb);
	PHYSICS_API void __stdcall Physics_Shutdown(pr::physics::DllHandle context);

	// Cache runtime-compiled GPU kernels in 'directory' for engines created afterwards, and for their atmospheres; null disables caching.
	// Each engine keeps the cache that was current when it was created. The cache is shared by every context token.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ShaderCacheDirectorySet(pr::physics::DllHandle context, wchar_t const* directory);

	// ABI discovery and error reporting.
	PHYSICS_API std::uint32_t __stdcall Physics_ApiVersion();
	PHYSICS_API pr::physics::EStatus __stdcall Physics_StructSize(pr::physics::EStructId struct_id, std::uint32_t* size);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_LastError(char* buffer, std::uint32_t capacity, std::uint32_t* required);

	// Engine lifecycle and device ownership.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_EngineCreate(pr::physics::DllHandle context, pr::physics::Config const* config, void* external_d3d12_device, pr::physics::EngineHandle* engine);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_EngineDestroy(pr::physics::EngineHandle engine);
	PHYSICS_API void __stdcall Physics_EngineAbandon(pr::physics::EngineHandle engine);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_EngineDeviceLeaseAcquire(pr::physics::EngineHandle engine, void** d3d12_device);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_EngineConfigGet(pr::physics::EngineHandle engine, pr::physics::Config* config);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_EngineConfigSet(pr::physics::EngineHandle engine, pr::physics::Config const* config);

	// Replace terrain on the owner thread between frames, or disable it with null. Native checkpoints reject terrain-equipped engines.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_EngineTerrainSet(pr::physics::EngineHandle engine, pr::physics::TerrainDesc const* terrain);

	// Replace or disable the independent infinite-height cylindrical boundary between completed frames; null removes it.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_EngineCylindricalBoundarySet(pr::physics::EngineHandle engine, pr::physics::CylindricalBoundaryDesc const* boundary);
	// Replace or disable the water environment between completed frames; null removes it. Water does not change native checkpoint content.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_EngineWaterSet(pr::physics::EngineHandle engine, pr::physics::WaterDesc const* water);

	// Replace the terrain heights used to correct water waves for depth; null means deep water everywhere. Applies to the current water and to
	// later Physics_EngineWaterSet calls. Only valid between completed frames.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_EngineWaterBathymetrySet(pr::physics::EngineHandle engine, pr::physics::WaterBathymetryDesc const* bathymetry);

	// Wind-driven wave spectrum with the fixed components described by 'spectrum'. 'count' must equal the component count, bands * directions.
	// Wind speed is m/s and fetch is metres. Amplitudes are for component directions relative to downwind; see Physics_WaveSpectrumElements.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_WaveSpectrumTargets(pr::physics::WaveSpectrumDesc const* spectrum, float wind_speed, float fetch, float* amplitudes, std::int32_t count);

	// Move each amplitude towards its target over 'dt' seconds with exponential time constant 'time_constant' seconds.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_WaveSpectrumRelax(float* amplitudes, float const* targets, std::int32_t count, float dt, float time_constant);

	// Return the crest sharpness of wind-driven waves for 'wind_speed' (m/s), for use with Physics_WaveSpectrumElements. Zero up to 4 m/s, rising
	// towards 0.7 in storms.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_WaveSpectrumCrestSharpness(float wind_speed, float* sharpness);

	// Write Gerstner elements for the components with a positive amplitude and a wavelength of at least 'min_wavelength', in component order.
	// 'heading' is the direction the wind blows towards, in radians anticlockwise from +X; component directions are rotated by it.
	// 'sharpness' in [0, 0.9] narrows wave crests and flattens troughs (see shared::WaterFieldWaveProfile) and is stored in each element's timing.x.
	// Steepness is zero. Fails when more than 'capacity' elements are needed.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_WaveSpectrumElements(pr::physics::WaveSpectrumDesc const* spectrum, float const* amplitudes, std::int32_t count, float heading, float sharpness, float min_wavelength, pr::physics::WaterElement* elements, std::int32_t capacity, std::int32_t* element_count);

	// Material properties.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_MaterialGet(pr::physics::EngineHandle engine, std::int32_t material_id, pr::physics::MaterialProperties* material);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_MaterialSet(pr::physics::EngineHandle engine, pr::physics::MaterialProperties const* material);

	// Shape creation and lifetime. Shapes belong to one engine and use generation-aware handles.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ShapeCreateSphere(pr::physics::EngineHandle engine, pr::physics::SphereShape const* desc, pr::physics::ShapeHandle* shape);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ShapeCreateBox(pr::physics::EngineHandle engine, pr::physics::BoxShape const* desc, pr::physics::ShapeHandle* shape);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ShapeCreateLine(pr::physics::EngineHandle engine, pr::physics::LineShape const* desc, pr::physics::ShapeHandle* shape);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ShapeCreateTriangle(pr::physics::EngineHandle engine, pr::physics::TriangleShape const* desc, pr::physics::ShapeHandle* shape);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ShapeCreatePolytope(pr::physics::EngineHandle engine, pr::physics::ShapeCommon const* common, pr::physics::Vector4 const* points, std::uint32_t point_count, pr::physics::ShapeHandle* shape);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ShapeCreateCompound(pr::physics::EngineHandle engine, pr::physics::ShapeCommon const* common, pr::physics::ShapeHandle const* children, std::uint32_t child_count, pr::physics::ShapeHandle* shape);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ShapeDestroy(pr::physics::EngineHandle engine, pr::physics::ShapeHandle shape);

	// Rigid-body creation, lifetime, and state.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_BodyCreate(pr::physics::EngineHandle engine, pr::physics::BodyDesc const* desc, pr::physics::BodyHandle* body);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_BodyDestroy(pr::physics::EngineHandle engine, pr::physics::BodyHandle body);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_BodyStateGet(pr::physics::EngineHandle engine, pr::physics::BodyHandle body, pr::physics::BodyState* state);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_BodyStateSet(pr::physics::EngineHandle engine, pr::physics::BodyHandle body, pr::physics::BodyState const* state);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_CommandsApply(pr::physics::EngineHandle engine, pr::physics::BodyCommand const* commands, std::uint32_t command_count);

	// Reduced-coordinate articulation creation, lifetime, state, and external fields.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ArticulationCreate(pr::physics::EngineHandle engine, pr::physics::ArticulationDesc const* desc, pr::physics::ArticulationLinkProperties const* links, pr::physics::ArticulationJointProperties const* joints, pr::physics::ArticulationHandle* articulation);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ArticulationDestroy(pr::physics::EngineHandle engine, pr::physics::ArticulationHandle articulation);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ArticulationStateGet(pr::physics::EngineHandle engine, pr::physics::ArticulationHandle articulation, pr::physics::ArticulationState* state, float* positions, float* velocities, float* accelerations, float* forces, std::uint32_t scalar_capacity, std::uint32_t* scalar_required);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ArticulationStateSet(pr::physics::EngineHandle engine, pr::physics::ArticulationHandle articulation, pr::physics::ArticulationState const* state, float const* positions, float const* velocities, float const* forces, std::uint32_t scalar_count);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ArticulationLinksCopy(pr::physics::EngineHandle engine, pr::physics::ArticulationHandle articulation, pr::physics::ArticulationLinkState* links, std::uint32_t capacity, std::uint32_t* required);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ArticulationLinkForceSet(pr::physics::EngineHandle engine, pr::physics::ArticulationHandle articulation, std::uint32_t link_index, pr::physics::SpatialVector const* force);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ArticulationLinkForceApply(pr::physics::EngineHandle engine, pr::physics::ArticulationHandle articulation, std::uint32_t link_index, pr::physics::SpatialVector const* force);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ArticulationLinkGravitySet(pr::physics::EngineHandle engine, pr::physics::ArticulationHandle articulation, std::uint32_t link_index, pr::physics::Vector4 const* gravity);

	// Engine-owned persistent D6 constraints with explicit overload repair.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ConstraintCreateD6(pr::physics::EngineHandle engine, pr::physics::D6ConstraintProperties const* desc, pr::physics::PersistentConstraintHandle* constraint);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ConstraintGetD6(pr::physics::EngineHandle engine, pr::physics::PersistentConstraintHandle constraint, pr::physics::D6ConstraintProperties* desc, std::int32_t* broken);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ConstraintUpdateD6(pr::physics::EngineHandle engine, pr::physics::PersistentConstraintHandle constraint, pr::physics::D6ConstraintProperties const* desc);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ConstraintSetEnabled(pr::physics::EngineHandle engine, pr::physics::PersistentConstraintHandle constraint, std::int32_t enabled);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ConstraintRepair(pr::physics::EngineHandle engine, pr::physics::PersistentConstraintHandle constraint);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_ConstraintDestroy(pr::physics::EngineHandle engine, pr::physics::PersistentConstraintHandle constraint);

	// Split and synchronous stepping. Commands are applied before submission.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_BeginStep(pr::physics::EngineHandle engine, float elapsed_seconds, double absolute_time_seconds, pr::physics::BodyCommand const* commands, std::uint32_t command_count);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_BeginStepEx(pr::physics::EngineHandle engine, float elapsed_seconds, double absolute_time_seconds, std::uint32_t substep_count, pr::physics::BodyCommand const* commands, std::uint32_t command_count);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_CompleteStep(pr::physics::EngineHandle engine);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_Step(pr::physics::EngineHandle engine, float elapsed_seconds, double absolute_time_seconds, pr::physics::BodyCommand const* commands, std::uint32_t command_count);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_StepEx(pr::physics::EngineHandle engine, float elapsed_seconds, double absolute_time_seconds, std::uint32_t substep_count, pr::physics::BodyCommand const* commands, std::uint32_t command_count);

	// Completed immutable state, buffered events, and diagnostics.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_SnapshotCopy(pr::physics::EngineHandle engine, pr::physics::BodySnapshot* snapshots, std::uint32_t capacity, std::uint32_t* required);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_EventsCopy(pr::physics::EngineHandle engine, pr::physics::Event* events, std::uint32_t capacity, std::uint32_t* required);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_DiagnosticsGet(pr::physics::EngineHandle engine, pr::physics::Diagnostics* diagnostics);

	// Opaque versioned restart checkpoints.
	// Size and write drain a pending step on the owner thread first, so a checkpoint always describes a
	// completed simulation state. Read requires an idle and empty engine.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_CheckpointSize(pr::physics::EngineHandle engine, std::uint64_t* required);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_CheckpointWrite(pr::physics::EngineHandle engine, void* buffer, std::uint64_t capacity, std::uint64_t* written);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_CheckpointRead(pr::physics::EngineHandle engine, void const* buffer, std::uint64_t size);

	// Engine-owned atmosphere solvers. Each atmosphere runs on the engine's device with its own compute queue, steps independently of the engine
	// step, and is destroyed with its engine. Atmospheres are not part of native checkpoints. All calls except Physics_AtmosphereTracersCopy
	// are owner-thread only. Creation compiles the solver kernels and blocks until the air is at rest on the reference profile.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_AtmosphereCreate(pr::physics::EngineHandle engine, pr::physics::AtmosphereDesc const* desc, pr::physics::AtmosphereHandle* atmosphere);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_AtmosphereDestroy(pr::physics::EngineHandle engine, pr::physics::AtmosphereHandle atmosphere);

	// Submit one step, and tracer advection when tracers exist, without waiting for the GPU. Fails with StepPending while a step is in flight.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_AtmosphereBeginStep(pr::physics::EngineHandle engine, pr::physics::AtmosphereHandle atmosphere, pr::physics::AtmosphereStepDesc const* step);

	// Finish the step in flight if the GPU has completed it, without waiting. 'idle' is set to 1 when no step is in flight after the call.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_AtmospherePollStep(pr::physics::EngineHandle engine, pr::physics::AtmosphereHandle atmosphere, std::int32_t* idle);

	// Wait for and finish the step in flight. Fails with NoStepPending when there is none.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_AtmosphereCompleteStep(pr::physics::EngineHandle engine, pr::physics::AtmosphereHandle atmosphere);

	// Change the column floor heights (one per column, as in AtmosphereDesc) and remap the air in changed columns. Blocks; requires no step in flight.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_AtmosphereFloorsSet(pr::physics::EngineHandle engine, pr::physics::AtmosphereHandle atmosphere, float const* floor_heights, std::int32_t count);

	// Set the floor temperature under each column (K, one per column in the floor-height order). The lowest layer relaxes toward it at the
	// descriptor's 'floor_exchange_rate'. Initially the reference temperature at each column's floor. The values are copied and used from the next step.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_AtmosphereFloorTemperaturesSet(pr::physics::EngineHandle engine, pr::physics::AtmosphereHandle atmosphere, float const* floor_temperatures, std::int32_t count);

	// Set the air outside the open faces (one entry per column in row-major floor-height order). Initially calm air at the reference temperature.
	// The values are copied and used from the next step.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_AtmosphereOutsideAirSet(pr::physics::EngineHandle engine, pr::physics::AtmosphereHandle atmosphere, pr::physics::AtmosphereOutsideAir const* outside_air, std::int32_t count);

	// Copy the tracer particles from the last finished step, or from creation. 'required' is the particle count.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_AtmosphereTracersCopy(pr::physics::EngineHandle engine, pr::physics::AtmosphereHandle atmosphere, pr::physics::AtmosphereTracerParticle* particles, std::uint32_t capacity, std::uint32_t* required);

	// Read the whole field back from the GPU and copy the cell-centre states, packed by layer, then row, then column (x fastest).
	// 'required' is the cell count and 'stats' is optional. Blocks; requires no step in flight.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_AtmosphereCellStatesCopy(pr::physics::EngineHandle engine, pr::physics::AtmosphereHandle atmosphere, pr::physics::AtmosphereCellState* cells, std::uint32_t capacity, std::uint32_t* required, pr::physics::AtmosphereStats* stats);
}
