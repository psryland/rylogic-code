//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_PHYSICS_RIGID_BODY_DYNAMICS_HLSLI
#define PR_PHYSICS_RIGID_BODY_DYNAMICS_HLSLI
#include "pr/hlsl/bounding_box.hlsli"

// Must match the C++ RigidBodyDynamics struct exactly (208 bytes per element).
struct RigidBodyDynamics
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
	int pad0;
	int pad1;
	int pad2;
};

#endif
