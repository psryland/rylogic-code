//*********************************************
// Physics Engine — Shared Constraint Algebra
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once

#ifdef __cplusplus
namespace constraint_algebra
{
	using std::abs;
	using std::isfinite;
	using std::max;
	using std::min;
	using std::sqrt;

	// Keep parameter syntax local so shared scalar algebra does not expose shader annotation macros to C++ callers.
	#define PR_CONSTRAINT_INLINE inline
	#define PR_CONSTRAINT_CONST const
	#define PR_CONSTRAINT_OUT(type) type&
	#define PR_CONSTRAINT_ARRAY_OUT(type, name, size) type (&name)[size]
#else
	#define PR_CONSTRAINT_INLINE
	#define PR_CONSTRAINT_CONST
	#define PR_CONSTRAINT_OUT(type) out type
	#define PR_CONSTRAINT_ARRAY_OUT(type, name, size) out type name[size]
#endif

// Return finite implicit spring coefficients, distinguishing a hard row from unrepresentable soft arithmetic.
PR_CONSTRAINT_INLINE bool ConstraintSoftParameters(float timestep, float stiffness, float damping, float position_error, PR_CONSTRAINT_OUT(float) gamma, PR_CONSTRAINT_OUT(float) bias)
{
	gamma = 0.0f;
	bias = 0.0f;
	if (!(timestep > 0.0f) || !isfinite(timestep) || !isfinite(stiffness) || !isfinite(damping) || !isfinite(position_error) || stiffness < 0.0f || damping < 0.0f)
		return false;

	// Only an exactly absent spring and damper describes a hard row; no geometric tolerance belongs in this denominator.
	if (stiffness == 0.0f && damping == 0.0f)
		return true;

	float elastic = timestep * stiffness;
	float denominator = timestep * (damping + elastic);
	if (!(denominator > 0.0f) || !isfinite(denominator))
		return false;

	gamma = 1.0f / denominator;
	bias = position_error * (elastic / denominator);
	return isfinite(gamma) && isfinite(bias);
}

// Invert a positive-diagonal response with stride six; the supplied absolute pivot tolerance is normalized by its largest diagonal.
PR_CONSTRAINT_INLINE bool InvertConstraintMatrix(float PR_CONSTRAINT_CONST matrix[36], int dimension, float pivot_tolerance, PR_CONSTRAINT_ARRAY_OUT(float, inverse, 36))
{
	if (dimension < 1 || dimension > 6 || !isfinite(pivot_tolerance) || pivot_tolerance < 0.0f)
		return false;

	// Equilibrate each row and column so unrelated mass and compliance scales cannot manufacture a rank deficiency.
	float diagonal[6];
	float scale = 0.0f;
	for (int row = 0; row != dimension; ++row)
	{
		float value = matrix[row * 6 + row];
		if (!(value > 0.0f) || !isfinite(value))
			return false;

		diagonal[row] = sqrt(value);
		scale = max(scale, value);
	}
	float tolerance = pivot_tolerance / scale;
	float augmented[72];
	for (int index = 0; index != 72; ++index)
		augmented[index] = 0.0f;

	for (int row = 0; row != dimension; ++row)
	{
		for (int column = 0; column != dimension; ++column)
		{
			float value = matrix[row * 6 + column] / diagonal[row] / diagonal[column];
			if (!isfinite(value))
				return false;

			augmented[row * 12 + column] = value;
		}
		augmented[row * 12 + 6 + row] = 1.0f;
	}

	// Fixed-order partial pivoting operates on the dimensionless matrix, retaining explicit finite and rank failure.
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
		if (!(pivot_size > tolerance) || !isfinite(pivot_size))
			return false;

		if (pivot_row != pivot_column)
		{
			for (int column = 0; column != 12; ++column)
			{
				float previous = augmented[pivot_column * 12 + column];
				augmented[pivot_column * 12 + column] = augmented[pivot_row * 12 + column];
				augmented[pivot_row * 12 + column] = previous;
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

	// Undo both diagonal scalings to return the inverse in the caller's original row coordinates.
	for (int row = 0; row != dimension; ++row)
	for (int column = 0; column != dimension; ++column)
	{
		inverse[row * 6 + column] = augmented[row * 12 + 6 + column] / diagonal[row] / diagonal[column];
		if (!isfinite(inverse[row * 6 + column]))
			return false;
	}
	return true;
}

// Minimize response-metric distance to a candidate inside a box using its SPD inverse response, with row stride six.
// Dimension must be 1..6; infinite bounds are supported. False means invalid arithmetic or exhausted work, and the output must be discarded.
PR_CONSTRAINT_INLINE bool ProjectConstraintBox(int dimension, float PR_CONSTRAINT_CONST inverse[36], float PR_CONSTRAINT_CONST candidate[6], float PR_CONSTRAINT_CONST lower[6], float PR_CONSTRAINT_CONST upper[6], PR_CONSTRAINT_ARRAY_OUT(float, result, 6))
{
	if (dimension < 1 || dimension > 6)
		return false;

	// A feasible unconstrained solution is already optimal and needs no matrix inspection or factorization.
	int state[6];
	bool feasible = true;
	for (int row = 0; row != dimension; ++row)
	{
		if (!isfinite(candidate[row]) || !(lower[row] <= upper[row]) || (!isfinite(lower[row]) && lower[row] > 0.0f) || (!isfinite(upper[row]) && upper[row] < 0.0f))
			return false;

		result[row] = min(max(candidate[row], lower[row]), upper[row]);
		state[row] = lower[row] == upper[row] ? 2 : candidate[row] < lower[row] ? -1 : candidate[row] > upper[row] ? +1 : 0;
		feasible = feasible && result[row] == candidate[row];
	}
	if (feasible)
		return true;

	// The working set starts at a feasible clamped point, but every accepted answer solves the full response-metric problem.
	for (int row = 0; row != dimension; ++row)
	{
		if (!(inverse[row * 6 + row] > 0.0f) || !isfinite(inverse[row * 6 + row]))
			return false;

		for (int column = 0; column != dimension; ++column)
			if (!isfinite(inverse[row * 6 + column]))
				return false;
	}

	// Each transition adds a blocking bound or releases a wrong-sign multiplier; fixed scratch and a hard cap bound GPU work.
	for (int iteration = 0; iteration != 64; ++iteration)
	{
		int active[6];
		int active_count = 0;
		for (int row = 0; row != dimension; ++row)
			if (state[row] != 0)
				active[active_count++] = row;

		// Solve inverse_AA * multiplier = candidate_A - bound_A using diagonally scaled Cholesky.
		float factor[36];
		float diagonal[6];
		float multiplier[6];
		for (int row = 0; row != active_count; ++row)
		{
			int axis = active[row];
			diagonal[row] = sqrt(inverse[axis * 6 + axis]);
			float bound = state[axis] == +1 ? upper[axis] : lower[axis];
			multiplier[row] = (candidate[axis] - bound) / diagonal[row];
			if (!isfinite(multiplier[row]))
				return false;

			for (int column = 0; column <= row; ++column)
			{
				int other = active[column];
				float value = (0.5f * inverse[axis * 6 + other] + 0.5f * inverse[other * 6 + axis]) / diagonal[row] / diagonal[column];
				for (int inner = 0; inner != column; ++inner)
					value -= factor[row * 6 + inner] * factor[column * 6 + inner];

				if (row == column)
				{
					if (!(value > 1.0e-7f) || !isfinite(value))
						return false;

					factor[row * 6 + column] = sqrt(value);
				}
				else
				{
					factor[row * 6 + column] = value / factor[column * 6 + column];
				}
			}
		}
		for (int row = 0; row != active_count; ++row)
		{
			for (int column = 0; column != row; ++column)
				multiplier[row] -= factor[row * 6 + column] * multiplier[column];

			multiplier[row] /= factor[row * 6 + row];
		}
		for (int reverse = active_count; reverse != 0; --reverse)
		{
			int row = reverse - 1;
			for (int column = row + 1; column != active_count; ++column)
				multiplier[row] -= factor[column * 6 + row] * multiplier[column];

			multiplier[row] /= factor[row * 6 + row];
		}
		for (int row = 0; row != active_count; ++row)
		{
			multiplier[row] /= diagonal[row];
			if (!isfinite(multiplier[row]))
				return false;
		}

		// Recover the exact face minimizer; an active coordinate is snapped only to its already enforced equality.
		float face[6];
		for (int row = 0; row != dimension; ++row)
		{
			face[row] = candidate[row];
			for (int column = 0; column != active_count; ++column)
			{
				int axis = active[column];
				face[row] -= (0.5f * inverse[row * 6 + axis] + 0.5f * inverse[axis * 6 + row]) * multiplier[column];
			}
			if (!isfinite(face[row]))
				return false;

			if (state[row] != 0)
				face[row] = state[row] == +1 ? upper[row] : lower[row];
		}

		// Preserve feasibility by stopping at the first inactive bound reached along the minimizing segment.
		float step = 1.0f;
		int blocking_axis = -1;
		int blocking_state = 0;
		for (int row = 0; row != dimension; ++row)
		{
			if (state[row] != 0 || (face[row] >= lower[row] && face[row] <= upper[row]))
				continue;

			int next_state = face[row] < lower[row] ? -1 : +1;
			float bound = next_state == -1 ? lower[row] : upper[row];
			float direction = face[row] - result[row];
			if (!isfinite(direction))
				return false;

			float fraction = (bound - result[row]) / direction;
			if (!isfinite(fraction))
				return false;

			if (blocking_axis < 0 || fraction < step)
			{
				step = max(0.0f, min(1.0f, fraction));
				blocking_axis = row;
				blocking_state = next_state;
			}
		}
		if (blocking_axis >= 0)
		{
			for (int row = 0; row != dimension; ++row)
			{
				float next = (1.0f - step) * result[row] + step * face[row];
				if (!isfinite(next))
					return false;

				result[row] = min(max(next, lower[row]), upper[row]);
			}

			state[blocking_axis] = blocking_state;
			result[blocking_axis] = blocking_state == -1 ? lower[blocking_axis] : upper[blocking_axis];
			continue;
		}

		// A feasible face minimizer is optimal only when every inequality multiplier has the correct KKT sign.
		for (int row = 0; row != dimension; ++row)
			result[row] = face[row];

		int release_axis = -1;
		float violation = 0.0f;
		for (int row = 0; row != active_count; ++row)
		{
			int axis = active[row];
			float wrong_sign = state[axis] == -1 ? multiplier[row] : state[axis] == +1 ? -multiplier[row] : 0.0f;
			if (wrong_sign > violation)
			{
				violation = wrong_sign;
				release_axis = axis;
			}
		}
		if (release_axis < 0)
			return true;

		state[release_axis] = 0;
	}
	return false;
}

#undef PR_CONSTRAINT_INLINE
#undef PR_CONSTRAINT_CONST
#undef PR_CONSTRAINT_OUT
#undef PR_CONSTRAINT_ARRAY_OUT
#ifdef __cplusplus
}
#endif
