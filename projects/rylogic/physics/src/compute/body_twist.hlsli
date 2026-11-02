//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Velocity access for GPU rigid bodies.
// A kinematic body has infinite mass, and its momentum slots hold its world-space twist (angular velocity, and linear velocity of the centre of mass).
// Other bodies hold world-space momentum in those slots. Shaders must read velocity with 'BodyTwistWS' and change momentum with 'AddBodyMomentumWS',
// so both kinds of body are handled the same way everywhere.
#ifndef PR_PHYSICS_GPU_BODY_TWIST_HLSLI
#define PR_PHYSICS_GPU_BODY_TWIST_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// World-space velocity of a body's centre of mass.
struct BodyTwist
{
	float3 ang; // Angular velocity (rad/s)
	float3 lin; // Linear velocity of the centre of mass (m/s)
};

// True if the body's pose is driven by its stored twist instead of by its momentum.
inline bool KinematicBody(in_(GpuRigidBody) body)
{
	return AllSet(body.state_flags, ERigidBodyStateFlags_Kinematic);
}

// Return the world-space twist of a body. Bodies with zero inverse mass that are not kinematic have zero twist.
inline BodyTwist BodyTwistWS(in_(GpuRigidBody) body)
{
	// Kinematic bodies store the twist directly
	BodyTwist twist;
	if (KinematicBody(body))
	{
		twist.ang = body.momentum_ang.xyz;
		twist.lin = body.momentum_lin.xyz;
		return twist;
	}

	// Other bodies derive the twist from momentum. The stored inertia is normalised by mass, so it is scaled by the inverse mass.
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	float3x3 ws_iinv = rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
	twist.ang = mul(ws_iinv, body.momentum_ang.xyz);
	twist.lin = inv_mass * body.momentum_lin.xyz;
	return twist;
}

// Add a world-space impulse (angular, linear) to a body's momentum. Kinematic bodies have infinite mass, so their twist is not changed.
inline void AddBodyMomentumWS(inout_(GpuRigidBody) body, float3 ang, float3 lin)
{
	// The momentum slots of a kinematic body hold its twist
	if (KinematicBody(body))
		return;

	body.momentum_ang.xyz += ang;
	body.momentum_lin.xyz += lin;
}

#ifdef __cplusplus
}
#endif
#endif
