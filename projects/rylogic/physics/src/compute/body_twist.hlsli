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

// True if the body is kinematic and its twist is not zero. A kinematic body has no sleep state; it is asleep exactly when it is not moving.
inline bool MovingKinematicBody(in_(GpuRigidBody) body)
{
	// The momentum slots of a kinematic body hold its twist
	return KinematicBody(body) && (any(body.momentum_ang.xyz) || any(body.momentum_lin.xyz));
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

// Move a kinematic body by its twist for 'dt' seconds and clear its forces, which have no effect on infinite mass.
// The rotation is about the centre of mass and uses the exact exponential map, so several drifts with the same twist give the same pose as one drift over the total time.
inline void KinematicDrift(inout_(GpuRigidBody) body, float dt)
{
	// Rotate about the centre of mass, then rebuild the model origin from the moved centre of mass
	// Transform the centre of mass as a point rather than indexing the translation row. DXC reads 'o2w[3]' of a row_major member
	// in an inout struct parameter as the matrix column, which loses the translation.
	float3 os_com = body.os_com_and_invmass.xyz;
	float3x3 rot = (float3x3)body.o2w;
	float3 com_ws = mul(float4(os_com, 1), body.o2w).xyz;
	float3x3 new_rot = orthonorm3x3(mul(rot, rodrigues_rotation(body.momentum_ang.xyz * dt)));
	float3 new_pos = com_ws + body.momentum_lin.xyz * dt - mul(os_com, new_rot);
	body.o2w = float4x4(
		float4(new_rot[0], 0),
		float4(new_rot[1], 0),
		float4(new_rot[2], 0),
		float4(new_pos, 1));

	// Forces are re-applied each step, and infinite mass ignores them
	body.force_ang = float4(0, 0, 0, 0);
	body.force_lin = float4(0, 0, 0, 0);
}

#ifdef __cplusplus
}
#endif
#endif
