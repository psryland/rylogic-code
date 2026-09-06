//*********************************************
// Physics Engine — Shared D6 Constraint Operations
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once

#ifdef __cplusplus
namespace PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE {
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

// Invert a dense matrix of at most six rows using fixed-order deterministic pivoting.
bool InvertConstraintMatrix(float matrix[36], int dimension, float pivot_tolerance, arrayout_(float, inverse, 36))
{
	float augmented[72];
	for (int idx = 0; idx != 72; ++idx)
		augmented[idx] = 0.0f;
	for (int row = 0; row != dimension; ++row)
	{
		for (int column = 0; column != dimension; ++column)
			augmented[row * 12 + column] = matrix[row * 6 + column];
		augmented[row * 12 + 6 + row] = 1.0f;
	}

	for (int pivot_column = 0; pivot_column != dimension; ++pivot_column)
	{
		int pivot_row = pivot_column;
		float pivot_size = abs(augmented[pivot_row * 12 + pivot_column]);
		for (int row = pivot_column + 1; row != dimension; ++row)
		{
			float candidate = abs(augmented[row * 12 + pivot_column]);
			if (candidate > pivot_size)
			{
				pivot_row = row;
				pivot_size = candidate;
			}
		}
		if (!(pivot_size > pivot_tolerance))
			return false;

		if (pivot_row != pivot_column)
		{
			for (int column = 0; column != 12; ++column)
			{
				float temporary = augmented[pivot_column * 12 + column];
				augmented[pivot_column * 12 + column] = augmented[pivot_row * 12 + column];
				augmented[pivot_row * 12 + column] = temporary;
			}
		}

		float pivot = augmented[pivot_column * 12 + pivot_column];
		for (int column = 0; column != 12; ++column)
			augmented[pivot_column * 12 + column] /= pivot;

		for (int row = 0; row != dimension; ++row)
		{
			if (row == pivot_column)
				continue;

			float factor = augmented[row * 12 + pivot_column];
			for (int column = 0; column != 12; ++column)
				augmented[row * 12 + column] -= factor * augmented[pivot_column * 12 + column];
		}
	}

	for (int row = 0; row != dimension; ++row)
		for (int column = 0; column != dimension; ++column)
			inverse[row * 6 + column] = augmented[row * 12 + 6 + column];
	return true;
}

#ifdef __cplusplus
}
using namespace PR_CONSTRAINT_SOLVER_OPS_CPP_NAMESPACE;
#endif
