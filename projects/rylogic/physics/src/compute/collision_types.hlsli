//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_PHYSICS_COLLISION_TYPES_HLSLI
#define PR_PHYSICS_COLLISION_TYPES_HLSLI

static const int IntegrateThreadCount = 64;
static const int SweepThreadCount = 64;
static const int CollideThreadCount = 32;
static const int ResolveThreadCount = 64;

// ---- Shape type enum (matches C++ EShape) ----
static const int SHAPE_SPHERE   = 0;
static const int SHAPE_BOX      = 1;
static const int SHAPE_LINE     = 2;
static const int SHAPE_TRIANGLE = 3;
static const int SHAPE_POLYTOPE = 4;

// ---- GPU data structures (must match C++ layout exactly) ----
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
	int shape_idx_a;
	int shape_idx_b;
	int pair_index;
	int pad0;
	row_major float4x4 b2a; // transform B into A's space
};
struct GpuResolveContact
{
	float4 axis;            // collision normal (in A's object space)
	float4 contact_pt;      // contact point at estimated collision time (in A's space)
	row_major float4x4 b2a; // B-to-A transform
	int body_idx_a;         //
	int body_idx_b;         //
	float elasticity;       // combined material elasticity (normal)
	float friction;         // combined material static friction
	float depth;            // Penetration depth (positive = overlapping).
	int mat_id_a;           // Material IDs from each shape
	int mat_id_b;           // Material IDs from each shape
	int pad0;
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
	int pair_index;
	int shape_type_a;
	int shape_type_b;
	int gjk_iters;
	int epa_iters;
	int hit;
	int pad0;
	int pad1;
};
struct DispatchArguments // D3D12_DISPATCH_ARGUMENTS
{
	uint ThreadGroupCountX;
	uint ThreadGroupCountY;
	uint ThreadGroupCountZ;
};

#endif
