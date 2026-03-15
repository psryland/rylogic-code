//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2016
//*********************************************
// POD struct mirroring the HLSL RigidBodyDynamics layout.
// Used as a transfer format between CPU RigidBody objects and the GPU
// compute shader. Each field is a float4 for GPU alignment.
#pragma once
#include "pr/physics-2/forward.h"
#include "pr/physics-2/rigid_body/rigid_body.h"

namespace pr::physics
{
	// A flat, GPU-friendly representation of the rigid body state needed for integration.
	// Layout matches the HLSL RigidBodyDynamics struct exactly (13 × float4 = 208 bytes).
	//
	// Memory convention: C++ m4x4 stores columns contiguously (x, y, z, w members).
	// HLSL 'row_major float4x4' stores rows contiguously, so each HLSL row maps to one
	// C++ column vector. Use mul(v, o2w) in HLSL for transforms (row-vector convention).
	struct alignas(16) RigidBodyDynamics
	{
		// Object-to-world transform. In C++ this is column-major (x/y/z = basis vectors,
		// w = position). In HLSL with 'row_major float4x4', each row = one C++ column.
		m4x4 o2w;

		// World-space spatial momentum [angular, linear], measured at the centre of mass.
		v4 momentum_ang;   // angular momentum (torque accumulation)
		v4 momentum_lin;   // linear momentum

		// World-space external force accumulator [angular, linear], measured at the centre of mass.
		// Zeroed after each integration step so persistent forces must be re-applied each frame.
		v4 force_ang;      // external torque
		v4 force_lin;      // external force

		// Object-space inverse inertia in compact symmetric form.
		// The full 3×3 inverse inertia matrix is reconstructed from diagonal + off-diagonal terms.
		v4 inertia_inv_diagonal;  // {Ixx_inv, Iyy_inv, Izz_inv, 0}
		v4 inertia_inv_products;  // {Ixy_inv, Ixz_inv, Iyz_inv, 0}

		// Object-space CoM offset from model origin, packed with inverse mass.
		// The CoM offset is needed to convert CoM velocity to model-origin position changes.
		v4 os_com_and_invmass;    // {com_x, com_y, com_z, inv_mass}

		// Object-space bounding box for AABB computation in the integrate shader.
		// These allow the GPU to compute world-space AABBs after evolving the transform.
		v4 os_bbox_centre;   // {cx, cy, cz, 0} — object-space AABB centre
		v4 os_bbox_radius;   // {rx, ry, rz, 0} — object-space AABB half-extents
	};
	static_assert(sizeof(RigidBodyDynamics) == 208, "RigidBodyDynamics must be 208 bytes for GPU alignment");
	static_assert(alignof(RigidBodyDynamics) == 16, "RigidBodyDynamics must be 16-byte aligned");

	// World-space AABB computed by the GPU integrate shader.
	struct alignas(16) IntegrateAABB
	{
		BBox ws_bbox; // world-space AABB
	};
	static_assert(sizeof(IntegrateAABB) == 32, "IntegrateAABB must be 32 bytes");

	// Output from the GPU integration step for debug validation.
	// One entry per body, written by the compute shader.
	struct alignas(16) IntegrateOutput
	{
		float ke_before; // kinetic energy before integration
		float ke_after;  // kinetic energy after integration
		float pad0;
		float pad1;
	};
	static_assert(sizeof(IntegrateOutput) == 16, "IntegrateOutput must be 16 bytes");

	// Pack/Unpack a RigidBody's dynamic state into the flat GPU buffer format.
	inline RigidBodyDynamics PackDynamics(RigidBody const& rb)
	{
		auto const& o2w = rb.O2W();
		auto momentum = rb.MomentumWS();
		auto force = rb.ForceWS();
		auto iinv = rb.InertiaInvOS();
		auto com = rb.CentreOfMassOS();
		return RigidBodyDynamics
		{
			.o2w = o2w,
			.momentum_ang = momentum.ang,
			.momentum_lin = momentum.lin,
			.force_ang = force.ang,
			.force_lin = force.lin,
			.inertia_inv_diagonal = iinv.m_diagonal,
			.inertia_inv_products = iinv.m_products,
			.os_com_and_invmass = v4{com.x, com.y, com.z, iinv.InvMass()},
			.os_bbox_centre = rb.Shape().m_bbox.m_centre,
			.os_bbox_radius = rb.Shape().m_bbox.m_radius,
		};
	}
	inline void UnpackDynamics(RigidBodyDynamics const& dyn, RigidBody& rb)
	{
		rb.O2W(dyn.o2w);

		// Update momentum (the integrator advanced it by the full step)
		rb.MomentumWS(v8force{ dyn.momentum_ang, dyn.momentum_lin });

		// Forces are zeroed by the integrator after the second half-kick
		rb.ZeroForces();
	}
}
