//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_PHYSICS_GPU_PHYSICS_TYPES_HLSLI
#define PR_PHYSICS_GPU_PHYSICS_TYPES_HLSLI
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/bounding_box.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

static const int IntegrateThreadCount = 64;
static const int SleepThreadCount = 64;
static const int SweepThreadCount = 64;
static const int CollideThreadCount = 32;
static const int ResolveThreadCount = 64;
static const int ConstraintThreadCount = 64;
static const int ArticulationThreadCount = 64;
static const int SelectiveRefreshThreadCount = 64;
static const int FrameOutputThreadCount = 64;
static const int MaxColours = 32;
static const int GpuContactMaxPoints = 4;
static const int GpuConstraintRowsPerBlock = 6;

// GPU articulation enum values mirror their public CPU counterparts without exposing C++ enum types to HLSL.
static const int GpuArticulationRootType_Fixed = 0;
static const int GpuArticulationRootType_Floating = 1;
static const int GpuArticulationAxisType_Revolute = 0;
static const int GpuArticulationAxisType_Prismatic = 1;
static const int GpuArticulationIntegrationStatus_Success = 0;
static const int GpuArticulationIntegrationStatus_Singular = 1;
static const int GpuArticulationIntegrationStatus_NonFinite = 2;
static const int GpuArticulationIntegrationStatus_NonConverged = 3;

// GPU constraint endpoint flags:
static const uint GpuConstraintEndpointFlags_None = 0;
static const uint GpuConstraintEndpointFlags_Enabled = 1 << 0;
static const uint GpuConstraintEndpointFlags_CollideConnected = 1 << 1;
static const uint GpuConstraintEndpointFlags_ResetWarmStart = 1 << 2;
static const uint GpuConstraintEndpointFlags_Coupled = 1 << 3;

// GPU runtime constraint-block flags shared by rigid and articulation-coupled lanes.
static const uint ConstraintBlockFlags_Active = 1u << 0;
static const uint ConstraintBlockFlags_ResetWarmStart = 1u << 1;
static const uint ConstraintBlockFlags_CoupledPreconditionerValid = 1u << 2;

// Coupled gather targets identify the state owner updated by one deterministic adjacency reduction.
static const int GpuCoupledConstraintTargetType_Rigid = 0;
static const int GpuCoupledConstraintTargetType_Link = 1;

// GPU constraint axis modes mirror EConstraintAxisMode without exposing the public enum to HLSL.
static const int GpuConstraintAxisMode_Free = 0;
static const int GpuConstraintAxisMode_Locked = 1;
static const int GpuConstraintAxisMode_Limited = 2;
static const int GpuConstraintAxisMode_Driven = 3;

// Rigid body state flags:
static const int ERigidBodyStateFlags_None = 0;
static const int ERigidBodyStateFlags_Static = 1 << 0;
static const int ERigidBodyStateFlags_Sleeping = 1 << 1;
static const int ERigidBodyStateFlags_NeverSleep = 1 << 2;
static const int ERigidBodyStateFlags_Collided = 1 << 3;

// Collision shape types:
static const int SHAPE_SPHERE   = 0;
static const int SHAPE_BOX      = 1;
static const int SHAPE_LINE     = 2;
static const int SHAPE_TRIANGLE = 3;
static const int SHAPE_POLYTOPE = 4;
static const int SHAPE_ARRAY    = 5;

// The largest number of convex leaf children a single compound shape may flatten to.
// Child indices are packed into 16 bits within warm-start keys, and the broadphase expands
// compound pairs as an N*M nested loop, so the bound keeps both cost and identity well defined.
static const int MaxCompoundChildren = 1024;

// Collision pair bins. These are unordered shape pairs canonicalised by shape type.
static const int COLLISION_BIN_SPHERE_VS_SPHERE     = 0;
static const int COLLISION_BIN_BOX_VS_SPHERE        = 1;
static const int COLLISION_BIN_BOX_VS_BOX           = 2;
static const int COLLISION_BIN_LINE_VS_SPHERE       = 3;
static const int COLLISION_BIN_LINE_VS_BOX          = 4;
static const int COLLISION_BIN_LINE_VS_LINE         = 5;
static const int COLLISION_BIN_TRIANGLE_VS_SPHERE   = 6;
static const int COLLISION_BIN_TRIANGLE_VS_BOX      = 7;
static const int COLLISION_BIN_TRIANGLE_VS_LINE     = 8;
static const int COLLISION_BIN_TRIANGLE_VS_TRIANGLE = 9;
static const int COLLISION_BIN_POLYTOPE_VS_SPHERE   = 10;
static const int COLLISION_BIN_POLYTOPE_VS_BOX      = 11;
static const int COLLISION_BIN_POLYTOPE_VS_LINE     = 12;
static const int COLLISION_BIN_POLYTOPE_VS_TRIANGLE = 13;
static const int COLLISION_BIN_POLYTOPE_VS_POLYTOPE = 14;
static const int COLLISION_BIN_COUNT                = 15;

// Collision feature types. The value is the manifold point count.
static const int FEATURE_NONE = 0;
static const int FEATURE_VERT = 1;
static const int FEATURE_EDGE = 2;
static const int FEATURE_TRI  = 3;
static const int FEATURE_QUAD = 4;

static const uint POLY_FACE_IGNORE_AXIS = 1 << 0;
static const uint POLY_EDGE_IGNORE_AXES = 1 << 0;

// Sleep island flags:
static const uint GpuSleepIslandFlags_None = 0;
static const uint GpuSleepIslandFlags_Valid = 1 << 0;
static const uint GpuSleepIslandFlags_Sleeping = 1 << 1;
static const uint GpuSleepIslandFlags_Disturbed = 1 << 2;

// Sleep island stats flags:
static const uint GpuSleepIslandStatsFlags_Valid = 1 << 0;
static const uint GpuSleepIslandStatsFlags_AllLow = 1 << 1;
static const uint GpuSleepIslandStatsFlags_AllReady = 1 << 2;
static const uint GpuSleepIslandStatsFlags_Wake = 1 << 3;

// ---- GPU data structures ----
struct GpuSleepData
{
	float timer_s;
	int island_id;
	uint generation;
	uint flags;
};
struct GpuRigidBody
{
	// Object-to-world transform. Each HLSL row = one C++ column vector.
	// Row 0-2 = rotation basis vectors, Row 3 = position (w=1).
	row_major float4x4 o2w;

	// World-space spatial momentum [angular, linear] at centre of mass
	float4 momentum_ang;
	float4 momentum_lin;

	// World-space force accumulator [angular, linear] at centre of mass
	float4 force_ang;
	float4 force_lin;

	// Each body has its own gravity vector to define local "down" for this object. This value is updated
	// via the Gravity methods and should be called each frame to apply the gravity force to the body (even static bodies).
	float4 ws_gravity;

	// Object-space inverse inertia (compact symmetric 3x3)
	float4 inertia_inv_diagonal;  // {Ixx_inv, Iyy_inv, Izz_inv, 0}
	float4 inertia_inv_products;  // {Ixy_inv, Ixz_inv, Iyz_inv, 0}

	// Object-space CoM offset from model origin, packed with inverse mass.
	// The CoM offset is needed to convert CoM velocity to model-origin position changes.
	float4 os_com_and_invmass;    // {com_x, com_y, com_z, inv_mass}
	
	// Object-space bounding box
	BBox os_bbox; // object-space AABB (centre + half-extents)

	// State flags (ERigidBodyStateFlags)
	int state_flags;

	// The id of the shape for this object
	int shape_id;

	// Scratch: bitmask of graph-colouring colours used by this body.
	// Written by CSComputeCollisionTimes, read by CSAssignColours.
	uint colour_used;

	// Explicit padding keeps the structured-buffer stride a 16-byte multiple to match C++ alignment.
	uint pad0;

	// Sleeping state for this body.
	GpuSleepData sleep;
};
struct GpuSleepIsland
{
	BBox bbox_ws;
	uint flags;
	uint body_count;
	uint generation;
	uint pad0;
};
struct GpuSleepIslandStats
{
	uint flags;
	uint body_count;
	int island_id;
	uint pad1;
};
struct GpuShape
{
	row_major float4x4 s2rb; // shape-to-rigidbody transform

	// Shape bounds expressed in rigid-body space. The broadphase uses this to cull child pairs
	// of compound shapes without having to know each shape type.
	BBox rb_bbox;

	int type;
	int vert_offset;
	int vert_count;
	int material_id;
	int face_offset;
	int face_count;
	int edge_offset;
	int edge_count;

	// Compound (SHAPE_ARRAY) shapes describe a contiguous run of convex leaf shapes that follows
	// this entry in the shape buffer. 'child_count == 0' identifies a convex primitive, which is
	// the single-shape fast path used by the majority of bodies.
	int child_offset;
	int child_count;

	// Shape flags (collision::Shape::EFlags), carried per leaf so child shapes keep their own behaviour.
	int flags;

	// Explicit padding keeps the structured-buffer stride a 16-byte multiple to match C++ alignment.
	int pad0;

	float4 data;            // type-specific: sphere(r), box(half_xyz), line(half_len,radius)
};
struct GpuPolytopeFace
{
	float4 plane; // xyz = local outward unit normal, w matches Plane3 convention
	int index0;
	int index1;
	int index2;
	uint flags;
};
struct GpuPolytopeEdge
{
	float4 direction; // local-space, normalised
	int v0;
	int v1;
	int face0;
	int face1;
	uint flags;
	int pad0;
	int pad1;
	int pad2;
};
struct GpuCollisionPair
{
	int body_idx_a;        // Rigid body index for A
	int body_idx_b;        // Rigid body index for B
	int shape_idx_a;       // Collision shape index for A (a convex leaf, never a compound root)
	int shape_idx_b;       // Collision shape index for B (a convex leaf, never a compound root)
	row_major float4x4 b2a; // transform B into A's space

	// Declaration-order index of the leaf within its owning body's compound shape. Zero for a
	// body with a single convex shape. This is the stable half of the contact identity; the other
	// half is the body index.
	int child_idx_a;
	int child_idx_b;
	int pad0;
	int pad1;
};
struct GpuContact
{
	float4 axis;                           // collision normal, pointing from shape A toward shape B
	float4 manifold[GpuContactMaxPoints]; // contact manifold points in the collision function's common space
	int feature;                          // FEATURE_*; also the number of valid manifold points
	float depth;                          // Penetration depth (positive = overlapping).
	int pad0;
	int pad1;
};
struct GpuResolveContact
{
	float4 axis;            // collision normal (in A's object space), pointing from body_idx_a toward body_idx_b
	float4 contact_point;   // centroid of the manifold, used by the resolver until it becomes manifold-aware
	float4 manifold[GpuContactMaxPoints]; // manifold points at estimated collision time (in A's space)
	row_major float4x4 b2a; // B-to-A transform
	int body_idx_a;         // index into GpuRigidBody buffer
	int body_idx_b;         // index into GpuRigidBody buffer
	int mat_id_a;           // Material IDs from each shape
	int mat_id_b;           // Material IDs from each shape
	float depth;            // Penetration depth (positive = overlapping).
	float collision_time;   // Estimated sub-step collision time. Written by CSComputeCollisionTimes.
	int feature;            // FEATURE_*; also the number of valid manifold points
	int child_idx_a;        // Leaf index within body A's compound shape (0 for a single-shape body)
	float4 warmstart_impulse; // Accumulated physical impulse from the velocity solver, in body A space.
	int child_idx_b;        // Leaf index within body B's compound shape (0 for a single-shape body)
	int pad1;
	int pad2;
	int pad3;
};
struct GpuWarmStartEntry
{
	uint key;
	int body_idx_a;
	int body_idx_b;
	uint child_key; // Packed (child_idx_a << 16) | child_idx_b, disambiguating leaves of the same body pair.
	float4 impulse; // Cached physical impulse in body A space.
};

// Compact endpoint-local constraint frame. Rotation is a normalised {x,y,z,w} quaternion and position is a point.
struct GpuConstraintFrame
{
	float4 rotation;
	float4 position;
};

// Persistent parameters for one canonical D6 axis.
struct GpuConstraintAxisDesc
{
	int mode;
	float lower_limit;
	float upper_limit;
	float target_position;
	float target_velocity;
	float stiffness;
	float damping;
	float max_force;
};

// Persistent D6 parameters occupy exactly 256 bytes independently of frame-local endpoint indices.
struct GpuD6ConstraintDesc
{
	GpuConstraintFrame frame_a;
	GpuConstraintFrame frame_b;
	GpuConstraintAxisDesc axes[GpuConstraintRowsPerBlock];
};

// Frame-local stable-slot metadata uploaded after remapping endpoint identities to packed body indices.
struct GpuConstraintEndpoint
{
	int body_idx_a;
	int body_idx_b;
	uint flags;
	uint generation;
	float break_force;
	float break_torque;
	uint pad0;
	uint pad1;
};

// Optional articulation ownership and compact mobility addressing for one stable constraint slot; negative indices identify non-link endpoints.
struct GpuCoupledConstraintEndpoint
{
	int articulation_idx_a;
	int link_idx_a;
	int mobility_idx_a;
	int root_link_idx_a;

	int articulation_idx_b;
	int link_idx_b;
	int mobility_idx_b;
	int root_link_idx_b;
};

// Stable-slot topology for one coupled block; negative target indices identify fixed endpoints.
struct GpuCoupledConstraintBlockTopology
{
	int island_idx;
	int target_idx_a;
	int target_idx_b;
	int pad0;
};

// One compact deterministic reduction target and its contiguous contribution-index range.
struct GpuCoupledConstraintTarget
{
	int target_type;
	int target_idx;
	int island_idx;
	int adjacency_offset;

	int adjacency_count;
	int pad0;
	int pad1;
	int pad2;
};

// One independent coupled island and its contiguous stable-block-index range.
struct GpuCoupledConstraintIsland
{
	int block_offset;
	int block_count;
	int pad0;
	int pad1;
};

// One canonical body pair in the open-addressed connected-body collision-exclusion table.
// Body indices are stored plus one so {0,0} remains the empty-slot sentinel.
struct GpuCollisionExclusion
{
	uint body_idx_a_plus_one;
	uint body_idx_b_plus_one;
};

// Runtime block state written by the GPU row compiler and retained for warm-start continuity.
struct GpuConstraintBlock
{
	int body_idx_a;
	int body_idx_b;
	uint velocity_mask;
	uint position_mask;
	uint colour;
	uint row_states;
	uint flags;
	uint pad0;
};

// Runtime scalar row. Rigid Jacobians are world-space wrenches; articulation Jacobians use link coordinates at the link origin.
struct GpuConstraintRow
{
	float4 jacobian_a_ang;
	float4 jacobian_a_lin;
	float4 jacobian_b_ang;
	float4 jacobian_b_lin;
	float4 solve;  // {position_error, target_velocity, bias, gamma}
	float4 bounds; // {lower_impulse, upper_impulse, physical_impulse, pseudo_impulse}
};

// Symmetric inverse of one coupled block's exact-self approximate response, packed as 21 upper-triangular values plus padding.
struct GpuCoupledConstraintPreconditioner
{
	float4 packed[6];
};

// Per-body pseudo twist accumulated by split correction without changing physical momentum.
struct GpuConstraintPseudoVelocity
{
	float4 angular_velocity;
	float4 linear_velocity;
};
struct GpuCollisionCounters
{
	int pair_count; // The number of potentially colliding objects
	int contact_count; // The number of contact points found
	int pad0;
	int pad1;
};

// Frame-constant spatial force restored before every internal GPU substep.
struct GpuFrameForce
{
	float4 force_ang;
	float4 force_lin;
};

// One reduced-coordinate tree and its contiguous ranges in the packed articulation buffers.
struct GpuArticulation
{
	uint identity_low;
	uint identity_high;
	int link_offset;
	int link_count;

	int position_offset;
	int position_count;
	int velocity_offset;
	int velocity_count;

	int dof_offset;
	int dof_count;
	int root_type;
	int max_depth;

	GpuConstraintFrame root_to_world;
};

// Immutable topology, joint frames, and compact physical inertia for one articulation link.
struct GpuArticulationLink
{
	int parent_link_index;
	int articulation_index;
	int position_offset;
	int velocity_offset;

	int dof_offset;
	int dof_count;
	int child_offset;
	int child_count;

	int proxy_body_index;
	int depth;
	int joint_matrix_offset;
	int pad1;

	GpuConstraintFrame joint_to_parent;
	GpuConstraintFrame joint_to_child;
	GpuConstraintFrame shape_to_link;

	float4 inertia_diagonal;
	float4 inertia_products;
	float4 inertia_com_and_mass;
};

// One ordered scalar screw axis; xyz is a unit direction and w stores the exact integer axis type.
struct GpuArticulationDof
{
	float4 axis_and_type;
};

// One breadth level in the shared outward schedule; reversing the levels gives the inward order.
struct GpuArticulationLevel
{
	int depth;
	int link_offset;
	int link_count;
	int pad0;
};

// One padded angular-then-linear spatial vector shared by ABA inputs, outputs, and scratch.
struct GpuArticulationSpatialVector
{
	float4 ang;
	float4 lin;
};

// One six-column spatial matrix; each padded column keeps StructuredBuffer layout identical in C++ and HLSL.
struct GpuArticulationSpatialMatrix
{
	GpuArticulationSpatialVector columns[6];
};

// Symmetric force-to-motion self-link mobility packed as its 21 upper-triangular values plus three padding scalars.
struct GpuArticulationSpatialMobility
{
	float4 packed[6];
};

// One participating articulation and its compact output range in the optional mobility stream.
struct GpuArticulationMobilityRange
{
	int articulation_index;
	int mobility_offset;
	int link_count;
	int pad0;
};

// One padded six-by-six joint matrix stored by columns for bounded zero-to-six-DOF solves.
struct GpuArticulationJointMatrix
{
	float4 columns_low[6];
	float4 columns_high[6];
};

// Per-generalized-DOF force-ABA factors retained from prepare/inward until outward recovery.
struct GpuArticulationAbaDofScratch
{
	GpuArticulationSpatialVector motion_subspace;
	GpuArticulationSpatialVector u_column;
};

// Compact per-link force-ABA state retained between deterministic level dispatches.
struct GpuArticulationAbaScratch
{
	GpuConstraintFrame child_to_parent;
	GpuArticulationSpatialMatrix articulated_inertia;

	// This field is articulated bias through every inward level, then solved link acceleration during root/outward traversal.
	GpuArticulationSpatialVector articulated_bias_or_acceleration;
	GpuArticulationSpatialVector link_velocity;
	GpuArticulationSpatialVector joint_bias;
	int solve_valid;
	int pad0;
	int pad1;
	int pad2;
};

// Per-articulation transactional midpoint state; failure status remains sticky across later substep dispatches.
struct GpuArticulationIntegrationState
{
	GpuConstraintFrame root_to_world_start;
	int status;
	int iteration_count;
	float residual;
	float pad;
};

// Compact final articulation state gathered once per frame without copying topology or midpoint rollback scratch.
struct GpuArticulationFrameOutput
{
	GpuConstraintFrame root_to_world;
	uint identity_low;
	uint identity_high;
	int status;
	int iteration_count;
	float residual;
	float pad0;
	int pad1;
	int pad2;
};

// Aggregate counters and bounded-event status copied back once after all internal substeps.
struct GpuFrameOutputHeader
{
	GpuCollisionCounters final_counters;
	int max_pair_count;
	int max_contact_count;
	int event_count;
	int event_capacity;
	int event_overflow;
	int pair_limit_substep;
	int contact_limit_substep;
	int event_overflow_substep;
	int substep_count;
	int pad0;
	int pad1;
	int pad2;
};

// Transient reservation shared by the serial event-queue setup and parallel event copy.
struct GpuSubstepOutputState
{
	int event_base;
	int event_count;
	int substep_index;
	int pad0;
};

// One resolved collision retained in deterministic substep and solver order.
struct GpuCollisionEvent
{
	GpuResolveContact contact;
	int substep_index;
	int pad0;
	int pad1;
	int pad2;
};
struct GpuSelectiveRefreshMetrics
{
	uint scored_contact_count;
	uint selected_contact_count;
	uint selected_pair_count;
	uint pad0;
};
struct GpuMaterial
{
	float friction_static;
	float elasticity_norm;
	float elasticity_tang;
	float elasticity_tors;
	float density;
	float pad0;
	float pad1;
	float pad2;
};

// D3D12_DISPATCH_ARGUMENTS
struct DispatchArguments
{
	uint ThreadGroupCountX;
	uint ThreadGroupCountY;
	uint ThreadGroupCountZ;
};

#ifdef __cplusplus
}
#endif
#endif
