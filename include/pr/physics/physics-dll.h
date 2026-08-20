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
	inline constexpr std::uint32_t PHYSICS_API_VERSION = 0x00010000U;
	inline constexpr std::uint32_t PHYSICS_STRUCT_VERSION = 1U;
	inline constexpr std::uint32_t PHYSICS_CHECKPOINT_VERSION = 2U;

	// Reported in place of a compound child index when the shape involved has no child identity.
	inline constexpr std::uint32_t PHYSICS_NO_CHILD = 0xFFFFFFFFU;

	// Maximum convex leaves addressable by one compound shape.
	inline constexpr std::uint32_t PHYSICS_MAX_COMPOUND_CHILDREN = 1024U;

	using EngineHandle = std::uint64_t;
	using ShapeHandle = std::uint64_t;
	using BodyHandle = std::uint64_t;

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
	};

	enum class EBodyFlags : std::uint32_t
	{
		None = 0,
		Enabled = 1U << 0,
		Sleeping = 1U << 1,
		NeverSleep = 1U << 2,
	};

	struct StructHeader
	{
		std::uint32_t size;
		std::uint32_t version;
	};

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
		std::int32_t step_pending;
		std::int32_t device_removed_reason;
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

	// Split and synchronous stepping. Commands are applied before submission.
	PHYSICS_API pr::physics::EStatus __stdcall Physics_BeginStep(pr::physics::EngineHandle engine, float elapsed_seconds, double absolute_time_seconds, pr::physics::BodyCommand const* commands, std::uint32_t command_count);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_CompleteStep(pr::physics::EngineHandle engine);
	PHYSICS_API pr::physics::EStatus __stdcall Physics_Step(pr::physics::EngineHandle engine, float elapsed_seconds, double absolute_time_seconds, pr::physics::BodyCommand const* commands, std::uint32_t command_count);

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
}
