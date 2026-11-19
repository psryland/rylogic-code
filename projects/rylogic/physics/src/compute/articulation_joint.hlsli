// Reduced-coordinate articulation configuration integration.
#ifndef PR_PHYSICS_ARTICULATION_JOINT_HLSLI
#define PR_PHYSICS_ARTICULATION_JOINT_HLSLI
#include "pr/hlsl/quaternions.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Validate or commit a joint drift from supplied coordinates and child-joint generalized velocities.
// Start and destination may alias. The velocity offset belongs to the supplied velocity stream, not to the position stream.
odr bool IntegrateArticulationJoint(GpuArticulationLink link, in_(RWStructuredBuffer<float>) start, in_(RWStructuredBuffer<float>) velocity, int velocity_offset, float dt, inout_(RWStructuredBuffer<float>) destination, bool commit)
{
	// Keep a whole joint candidate local so quaternion normalization never observes partially overwritten coordinates.
	float candidate[6];
	int position_count = link.dof_count;
	switch (link.joint_type)
	{
		case GpuArticulationJointType_Axes:
		{
			for (int index = 0; index != position_count; ++index)
				candidate[index] = start[link.position_offset + index] + dt * velocity[velocity_offset + index];

			break;
		}
		case GpuArticulationJointType_Spherical:
		{
			// Right multiplication integrates angular velocity expressed in the moving child-joint frame.
			float4 orientation = float4(start[link.position_offset], start[link.position_offset + 1], start[link.position_offset + 2], start[link.position_offset + 3]);
			float3 half_displacement = (0.5f * dt) * float3(velocity[velocity_offset], velocity[velocity_offset + 1], velocity[velocity_offset + 2]);
			if (!isfinite(dot(half_displacement, half_displacement)))
				return false;

			float4 result = normalize(quat_mul(orientation, quat_exp(half_displacement)));
			position_count = 4;
			for (int index = 0; index != position_count; ++index)
				candidate[index] = result[index];

			break;
		}
		default: { return false; }
	}

	// The caller owns whole-tree transaction boundaries; invalid coordinates must never be published as a successful drift.
	for (int index = 0; index != position_count; ++index)
		if (!isfinite(candidate[index]))
			return false;

	if (commit)
	{
		for (int index = 0; index != position_count; ++index)
			destination[link.position_offset + index] = candidate[index];
	}
	return true;
}

#ifdef __cplusplus
}
#endif
#endif
