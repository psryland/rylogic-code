//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_PHYSICS_GPU_PHYSICS_TYPES_HLSLI
#define PR_PHYSICS_GPU_PHYSICS_TYPES_HLSLI
#include "pr/hlsl/bounding_box.hlsli"

static const int IntegrateThreadCount = 64;
static const int SweepThreadCount = 64;
static const int CollideThreadCount = 32;
static const int ResolveThreadCount = 64;
static const int MaxColours = 32;

// ---- Shape type enum (matches C++ EShape) ----
static const int SHAPE_SPHERE   = 0;
static const int SHAPE_BOX      = 1;
static const int SHAPE_LINE     = 2;
static const int SHAPE_TRIANGLE = 3;
static const int SHAPE_POLYTOPE = 4;

// ---- GPU data structures (must match C++ layout exactly) ----
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

	// Object-space inverse inertia (compact symmetric 3x3)
	float4 inertia_inv_diagonal;  // {Ixx_inv, Iyy_inv, Izz_inv, 0}
	float4 inertia_inv_products;  // {Ixy_inv, Ixz_inv, Iyz_inv, 0}

	// Object-space CoM offset from model origin, packed with inverse mass.
	// The CoM offset is needed to convert CoM velocity to model-origin position changes.
	float4 os_com_and_invmass;    // {com_x, com_y, com_z, inv_mass}
	
	// Object-space bounding box
	BBox os_bbox; // object-space AABB (centre + half-extents)

	// The id of the shape for this object
	int shape_id;

	// Scratch: bitmask of graph-colouring colours used by this body.
	// Written by CSComputeCollisionTimes, read by CSAssignColours.
	uint colour_used;

	int pad0;
	int pad1;
};
struct GpuShape
{
	row_major float4x4 s2rb; // shape-to-rigidbody transform
	int type;
	int vert_offset;
	int vert_count;
	int material_id;
	float4 data;            // type-specific: sphere(r), box(half_xyz), line(half_len,thickness)
};
struct GpuCollisionPair
{
	int body_idx_a;        // Rigid body index for A
	int body_idx_b;        // Rigid body index for B
	int shape_idx_a;       // Collision shape index for A
	int shape_idx_b;       // Collision shape index for B
	row_major float4x4 b2a; // transform B into A's space
};
struct GpuResolveContact
{
	float4 axis;            // collision normal (in A's object space)
	float4 contact_point;   // contact point at estimated collision time (in A's space)
	row_major float4x4 b2a; // B-to-A transform
	int body_idx_a;         // index into GpuRigidBody buffer
	int body_idx_b;         // index into GpuRigidBody buffer
	int mat_id_a;           // Material IDs from each shape
	int mat_id_b;           // Material IDs from each shape
	float depth;            // Penetration depth (positive = overlapping).
	float collision_time;   // Estimated sub-step collision time. Written by CSComputeCollisionTimes.
	int pad0;
	int pad1;
};
struct GpuCollisionCounters
{
	int body_count; // The number of bodies/shapes to test
	int pair_count; // The number of potentially colliding objects
	int contact_count; // The number of contact points found
	int pad0;
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
struct GpuIntegrateDiag
{
	float ke_before;
	float ke_after;
	float pad0;
	float pad1;
};
struct GpuPairDiag
{
	int body_idx_a;  // Rigid body index for A
	int body_idx_b;  // Rigid body index for B
	int shape_type_a;
	int shape_type_b;
	int gjk_iters;
	int epa_iters;
	int hit;
	int pad0;
};
struct DispatchArguments // D3D12_DISPATCH_ARGUMENTS
{
	uint ThreadGroupCountX;
	uint ThreadGroupCountY;
	uint ThreadGroupCountZ;
};

#endif
