//*********************************************
// Physics Engine — Shared D6 Constraint Operations
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "physics/src/compute/constraint_algebra.hlsli"

#ifdef __cplusplus
namespace PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE {
using namespace constraint_algebra;
#endif

static const uint ConstraintLimitState_Inactive = 0u;
static const uint ConstraintLimitState_Lower = 1u;
static const uint ConstraintLimitState_Upper = 2u;
static const uint ConstraintLimitState_Bilateral = 3u;

// Return an explicitly initialized runtime row in both HLSL and C++ replay builds.
GpuConstraintRow EmptyConstraintRow()
{
	GpuConstraintRow row;
	row.jacobian_a_ang = float4(0, 0, 0, 0);
	row.jacobian_a_lin = float4(0, 0, 0, 0);
	row.jacobian_b_ang = float4(0, 0, 0, 0);
	row.jacobian_b_lin = float4(0, 0, 0, 0);
	row.solve = float4(0, 0, 0, 0);
	row.bounds = float4(0, 0, 0, 0);
	return row;
}

// Return an explicitly initialized runtime block in both HLSL and C++ replay builds.
GpuConstraintBlock EmptyConstraintBlock()
{
	GpuConstraintBlock block;
	block.body_idx_a = -1;
	block.body_idx_b = -1;
	block.velocity_mask = 0u;
	block.position_mask = 0u;
	block.colour = MaxColours;
	block.row_states = 0u;
	block.flags = 0u;
	block.pad0 = 0u;
	return block;
}

// Return the packed upper-triangular scalar index for one symmetric six-dimensional matrix component.
int ConstraintPackedSymmetricIndex(int row, int column)
{
	int low = min(row, column);
	int high = max(row, column);
	return low * 6 - low * (low - 1) / 2 + high - low;
}

// Return a zeroed packed coupled-block preconditioner.
GpuCoupledConstraintPreconditioner EmptyCoupledConstraintPreconditioner()
{
	GpuCoupledConstraintPreconditioner preconditioner;
	for (int index = 0; index != 6; ++index)
		preconditioner.packed[index] = float4(0.0f, 0.0f, 0.0f, 0.0f);
	return preconditioner;
}

// Store one symmetric component in a packed coupled-block preconditioner.
void SetCoupledPreconditionerComponent(inout_(GpuCoupledConstraintPreconditioner) preconditioner, int row, int column, float value)
{
	int packed_index = ConstraintPackedSymmetricIndex(row, column);
	preconditioner.packed[packed_index / 4][packed_index % 4] = value;
}

// Return one symmetric component from a packed coupled-block preconditioner.
float CoupledPreconditionerComponent(GpuCoupledConstraintPreconditioner preconditioner, int row, int column)
{
	int packed_index = ConstraintPackedSymmetricIndex(row, column);
	return preconditioner.packed[packed_index / 4][packed_index % 4];
}

// Return the world direction 'd' for which dot(d, w_b - w_a) is the exact rate of change of dot(rotation_vector, axis), where 'rotation_vector' is the
// world rotation from frame A to frame B (at most pi radians) and 'axis' is a unit axis of frame A. Frame A's axes are only a good approximation for small
// rotations; for large rotations, turning about one axis also changes the other rotation-vector components.
float3 ConstraintRotationVectorRateAxis(float3 rotation_vector, float3 axis)
{
	// The inverse Jacobian of the rotation vector is I - 1/2[r]x + c[r]x^2; its transpose maps the coordinate's axis onto angular velocity.
	float angle = length(rotation_vector);
	float c = angle < 1e-3f
		? 1.0f / 12.0f + angle * angle / 720.0f
		: 1.0f / (angle * angle) - 1.0f / (2.0f * angle * tan(0.5f * angle));
	float3 r_cross_axis = cross(rotation_vector, axis);
	return axis + 0.5f * r_cross_axis + c * cross(rotation_vector, r_cross_axis);
}

// Return a stable two-bit row state for warm-start identity and impulse-bound selection.
uint ConstraintRowState(GpuConstraintAxisDesc axis, float position, out_(float) position_error)
{
	position_error = 0.0f;
	switch (axis.mode)
	{
		case GpuConstraintAxisMode_Free:
		{
			return ConstraintLimitState_Inactive;
		}
		case GpuConstraintAxisMode_Locked:
		{
			position_error = position - axis.target_position;
			return ConstraintLimitState_Bilateral;
		}
		case GpuConstraintAxisMode_Limited:
		{
			if (axis.lower_limit == axis.upper_limit)
			{
				position_error = position - axis.lower_limit;
				return ConstraintLimitState_Bilateral;
			}
			if (position <= axis.lower_limit)
			{
				position_error = position - axis.lower_limit;
				return ConstraintLimitState_Lower;
			}
			if (position >= axis.upper_limit)
			{
				position_error = position - axis.upper_limit;
				return ConstraintLimitState_Upper;
			}
			return ConstraintLimitState_Inactive;
		}
		case GpuConstraintAxisMode_Driven:
		{
			position_error = position - axis.target_position;
			return ConstraintLimitState_Bilateral;
		}
		default:
		{
			return ConstraintLimitState_Inactive;
		}
	}
}

// Intersect a row's unilateral or bilateral state with its per-step force cap.
float2 ConstraintImpulseBounds(uint state, float max_impulse)
{
	switch (state)
	{
		case ConstraintLimitState_Inactive:
		{
			return float2(0.0f, 0.0f);
		}
		case ConstraintLimitState_Lower:
		{
			return float2(0.0f, max_impulse);
		}
		case ConstraintLimitState_Upper:
		{
			return float2(-max_impulse, 0.0f);
		}
		case ConstraintLimitState_Bilateral:
		{
			return float2(-max_impulse, max_impulse);
		}
		default:
		{
			return float2(0.0f, 0.0f);
		}
	}
}

#ifdef __cplusplus
}
using namespace PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE;
#endif
