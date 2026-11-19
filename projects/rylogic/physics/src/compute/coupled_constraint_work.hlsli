//*********************************************
// Physics Engine - Complete Constraint Work
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_PHYSICS_COUPLED_CONSTRAINT_WORK_HLSLI
#define PR_PHYSICS_COUPLED_CONSTRAINT_WORK_HLSLI
#include "pr/hlsl/core.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#define PR_WORK_PRECISE volatile
#else
#define PR_WORK_PRECISE precise
#endif

// Actual free and numerical-initial world-COM momenta for one unique rigid target.
struct GpuCoupledConstraintMomentumReference
{
	GpuArticulationSpatialVector free;
	GpuArticulationSpatialVector initial;
};

// Actual free and numerical-initial values in one native generalized component.
struct GpuCoupledConstraintSpeedReference
{
	float free;
	float initial;
};

// Contiguous unique rigid-target and complete-tree indices for one island.
struct GpuCoupledConstraintWorkSpan
{
	uint rigid_offset;
	uint rigid_count;
	uint tree_offset;
	uint tree_count;
};

// Local expression (value +/- error)*2^exponent; the two unsigned words encode a signed 64-bit exponent, low word first.
struct CoupledMeritValue
{
	float value;
	float error;
	uint2 exponent;
};

// Independently scaled components prevent a large unchanged coordinate from erasing smaller work.
struct CoupledWorkVector
{
	CoupledMeritValue components[3];
};

// Local spatial expression and its component-wise evaluation uncertainty.
struct CoupledWorkMotion
{
	CoupledWorkVector angular;
	CoupledWorkVector translation;
};

// Evaluator-only parent-before-child scratch, overwritten for every work comparison.
struct GpuCoupledConstraintWorkMotion
{
	CoupledWorkMotion delta;
	CoupledWorkMotion midpoint;
};

// Return a finite raw identity without merging distinct subnormal inputs under FTZ.
inline bool CoupledWorkSame(float a, float b)
{
	uint aa = asuint(a);
	uint bb = asuint(b);
	return (aa & 0x7fffffffu) < 0x7f800000u && (bb & 0x7fffffffu) < 0x7f800000u &&
		(aa == bb || ((aa | bb) & 0x7fffffffu) == 0u);
}

// Return whether the physical components denote the identical finite vector.
inline bool CoupledWorkSame3(float3 a, float3 b)
{
	return CoupledWorkSame(a.x, b.x) && CoupledWorkSame(a.y, b.y) && CoupledWorkSame(a.z, b.z);
}

// Sign-extend a native integer without requiring shader 64-bit integer support.
inline uint2 CoupledWorkExponent(int value)
{
	return uint2((uint)value, value < 0 ? 0xffffffffu : 0u);
}

// Compare signed two-word exponents without a floating-point conversion.
inline bool CoupledWorkExponentLess(uint2 a, uint2 b)
{
	uint ah = a.y ^ 0x80000000u;
	uint bh = b.y ^ 0x80000000u;
	return ah < bh || (ah == bh && a.x < b.x);
}

// Add signed two-word exponents, reporting overflow rather than wrapping the represented expression.
inline bool CoupledWorkExponentAdd(uint2 a, uint2 b, out_(uint2) result)
{
	result.x = a.x + b.x;
	result.y = a.y + b.y + (result.x < a.x ? 1u : 0u);
	return ((a.y ^ result.y) & (b.y ^ result.y) & 0x80000000u) == 0u;
}

// Subtract signed two-word exponents, including the low-word borrow.
inline bool CoupledWorkExponentSubtract(uint2 a, uint2 b, out_(uint2) result)
{
	result.x = a.x - b.x;
	result.y = a.y - b.y - (a.x < b.x ? 1u : 0u);
	return ((a.y ^ b.y) & (a.y ^ result.y) & 0x80000000u) == 0u;
}

// Mark a consumed expression unavailable while keeping the point placeholder finite for downstream validity checks.
inline CoupledMeritValue CoupledMeritUnavailable()
{
	CoupledMeritValue result;
	result.value = 0.0f;
	result.error = asfloat(0x7f800000u);
	result.exponent = uint2(0u, 0u);
	return result;
}

// Check the whole consumed enclosure before identities or zero multiplication can hide its failure.
inline bool CoupledMeritFinite(CoupledMeritValue value)
{
	return isfinite(value.value) && isfinite(value.error) && value.error >= 0.0f;
}

// Locate the leading bit of a nonzero raw significand with five bounded integer steps.
inline int CoupledWorkHighBit(uint value)
{
	int bit = 0;
	if (value >= 0x10000u) { value >>= 16; bit += 16; }
	if (value >= 0x100u) { value >>= 8; bit += 8; }
	if (value >= 0x10u) { value >>= 4; bit += 4; }
	if (value >= 0x4u) { value >>= 2; bit += 2; }
	if (value >= 0x2u) { bit += 1; }
	return bit;
}

// Shift a finite binary32 value by an exact dyadic power; any subnormal output is explicitly reported as lost.
inline float CoupledWorkScaleMantissa(float value, int shift, out_(bool) lost)
{
	uint bits = asuint(value);
	uint magnitude = bits & 0x7fffffffu;
	lost = false;
	if (magnitude == 0u)
		return 0.0f;

	int exponent = (int)(magnitude >> 23);
	uint fraction = magnitude & 0x007fffffu;
	if (exponent == 0)
	{
		// Normalize stored subnormals using integers before the GPU can flush an arithmetic operand.
		int bit = CoupledWorkHighBit(fraction);
		fraction = (fraction << (23 - bit)) & 0x007fffffu;
		exponent = bit - 22;
	}
	exponent += shift;
	if (exponent <= 0)
	{
		lost = true;
		return 0.0f;
	}
	if (exponent >= 255)
		return asfloat((bits & 0x80000000u) | 0x7f800000u);

	return asfloat((bits & 0x80000000u) | ((uint)exponent << 23) | fraction);
}

// Round a nonnegative mantissa bound upward, preserving exact zero and enclosing bound underflow.
inline float CoupledMeritRoundUp(float value)
{
	uint bits = asuint(value);
	if (bits == 0u)
		return 0.0f;

	if (bits >= 0x7f800000u)
		return asfloat(0x7f800000u);

	return asfloat(max(0x00800000u, bits + 1u));
}

// Add nonnegative bounds without rounding their sum downward.
inline float CoupledMeritErrorAdd(float a, float b)
{
	if (a == 0.0f)
		return b;
	if (b == 0.0f)
		return a;

	PR_WORK_PRECISE float sum = a + b;
	return CoupledMeritRoundUp(sum);
}

// Multiply nonnegative bounds without discarding a nonzero underflowed product.
inline float CoupledMeritErrorMul(float a, float b)
{
	if (a == 0.0f || b == 0.0f)
		return 0.0f;

	PR_WORK_PRECISE float product = a * b;
	return product == 0.0f ? asfloat(0x00800000u) : CoupledMeritRoundUp(product);
}

// Bound one scalar operation by an output ULP, with explicit FTZ coverage.
inline float CoupledMeritRoundError(float value)
{
	uint exponent = (asuint(value) & 0x7fffffffu) >> 23;
	if (exponent == 255u)
		return asfloat(0x7f800000u);

	return asfloat((max(exponent, 24u) - 23u) << 23);
}

// Normalize a local enclosure; range loss is charged in its current mantissa units, never as a fixed physical-unit allowance.
inline CoupledMeritValue CoupledMeritNormalize(CoupledMeritValue value)
{
	if (!CoupledMeritFinite(value))
		return CoupledMeritUnavailable();

	for (;;)
	{
		uint magnitude = max(asuint(value.value) & 0x7fffffffu, asuint(value.error));
		if (magnitude == 0u)
		{
			value.exponent = uint2(0u, 0u);
			return value;
		}
		int shift = (magnitude >> 23) != 0u ? (int)(magnitude >> 23) - 126 : CoupledWorkHighBit(magnitude) - 148;
		uint2 exponent;
		if (!CoupledWorkExponentAdd(value.exponent, CoupledWorkExponent(shift), exponent))
			return CoupledMeritUnavailable();

		value.exponent = exponent;
		bool point_lost, error_lost;
		value.value = CoupledWorkScaleMantissa(value.value, -shift, point_lost);
		value.error = CoupledWorkScaleMantissa(value.error, -shift, error_lost);
		value.error = error_lost ? asfloat(0x00800000u) : CoupledMeritRoundUp(value.error);
		if (point_lost)
			value.error = CoupledMeritErrorAdd(value.error, asfloat(0x00800000u));

		if (max(abs(value.value), value.error) < 1.0f)
			return value;

		// Upward error rounding can reach one; a second normalization then puts it strictly below one.
	}
}

// Decode any finite stored input exactly, including subnormals, without float arithmetic on the original value.
inline CoupledMeritValue CoupledMeritInput(float value)
{
	uint magnitude = asuint(value) & 0x7fffffffu;
	if (magnitude >= 0x7f800000u)
		return CoupledMeritUnavailable();

	CoupledMeritValue result;
	result.error = 0.0f;
	int shift = magnitude == 0u ? 0 : (magnitude >> 23) != 0u ? (int)(magnitude >> 23) - 126 : CoupledWorkHighBit(magnitude) - 148;
	bool lost;
	result.value = CoupledWorkScaleMantissa(value, -shift, lost);
	result.exponent = CoupledWorkExponent(shift);
	return result;
}

// Align a valid normalized operand to a greater exponent, enclosing the entire discarded operand at wide precision gaps.
inline CoupledMeritValue CoupledMeritAlign(CoupledMeritValue value, uint2 exponent)
{
	uint2 difference;
	bool representable = CoupledWorkExponentSubtract(exponent, value.exponent, difference);
	value.exponent = exponent;
	if (!representable || difference.y != 0u || difference.x >= 26u)
	{
		// Both normalized mantissas are below one, so their aligned sum of magnitudes is less than 2^-25.
		value.value = 0.0f;
		value.error = asfloat(103u << 23);
		return value;
	}
	bool point_lost, error_lost;
	value.value = CoupledWorkScaleMantissa(value.value, -(int)difference.x, point_lost);
	value.error = CoupledWorkScaleMantissa(value.error, -(int)difference.x, error_lost);
	value.error = error_lost ? asfloat(0x00800000u) : CoupledMeritRoundUp(value.error);
	if (point_lost)
		value.error = CoupledMeritErrorAdd(value.error, asfloat(0x00800000u));

	return value;
}

// Negate an expression exactly without changing its absolute uncertainty.
inline CoupledMeritValue CoupledWorkNegate(CoupledMeritValue value)
{
	if (!CoupledMeritFinite(value))
		return CoupledMeritUnavailable();

	value.value = -value.value;
	return CoupledMeritNormalize(value);
}

// Evaluate one ordered scalar addition and its local error recurrence.
inline CoupledMeritValue CoupledMeritAdd(CoupledMeritValue a, CoupledMeritValue b)
{
	if (!CoupledMeritFinite(a) || !CoupledMeritFinite(b))
		return CoupledMeritUnavailable();

	if (a.value == 0.0f && a.error == 0.0f)
		return b;
	if (b.value == 0.0f && b.error == 0.0f)
		return a;

	uint2 exponent = CoupledWorkExponentLess(a.exponent, b.exponent) ? b.exponent : a.exponent;
	a = CoupledMeritAlign(a, exponent);
	b = CoupledMeritAlign(b, exponent);
	PR_WORK_PRECISE float estimate = a.value + b.value;
	CoupledMeritValue result;
	result.value = estimate;
	result.error = CoupledMeritErrorAdd(CoupledMeritErrorAdd(a.error, b.error), CoupledMeritRoundError(estimate));
	result.exponent = exponent;
	return CoupledMeritNormalize(result);
}

// Evaluate one ordered scalar product and all input/operation uncertainty terms.
inline CoupledMeritValue CoupledMeritMul(CoupledMeritValue a, CoupledMeritValue b)
{
	if (!CoupledMeritFinite(a) || !CoupledMeritFinite(b))
		return CoupledMeritUnavailable();

	if ((a.value == 0.0f && a.error == 0.0f) || (b.value == 0.0f && b.error == 0.0f))
		return CoupledMeritInput(0.0f);

	CoupledMeritValue result;
	if (!CoupledWorkExponentAdd(a.exponent, b.exponent, result.exponent))
		return CoupledMeritUnavailable();

	PR_WORK_PRECISE float estimate = a.value * b.value;
	result.value = estimate;
	float input_error = CoupledMeritErrorAdd(
		CoupledMeritErrorAdd(CoupledMeritErrorMul(abs(a.value), b.error), CoupledMeritErrorMul(abs(b.value), a.error)),
		CoupledMeritErrorMul(a.error, b.error));
	result.error = CoupledMeritErrorAdd(input_error, CoupledMeritRoundError(estimate));
	return CoupledMeritNormalize(result);
}

// Convert a decided certificate to best-effort physical-unit diagnostic output; zero also denotes unrepresentable/unavailable work.
inline float CoupledMeritDiagnostic(CoupledMeritValue value)
{
	if (!CoupledMeritFinite(value) || value.value == 0.0f)
		return 0.0f;

	uint magnitude = asuint(value.value) & 0x7fffffffu;
	int point_exponent = (int)(magnitude >> 23);
	uint2 exponent;
	if (!CoupledWorkExponentAdd(value.exponent, CoupledWorkExponent(point_exponent), exponent) ||
		CoupledWorkExponentLess(exponent, CoupledWorkExponent(-23)) || !CoupledWorkExponentLess(exponent, CoupledWorkExponent(255)))
		return 0.0f;

	int encoded = (int)exponent.x;
	uint sign = asuint(value.value) & 0x80000000u;
	uint fraction = magnitude & 0x007fffffu;
	if (encoded > 0)
		return asfloat(sign | ((uint)encoded << 23) | fraction);

	// Construct subnormal diagnostic bits with nearest-even rounding, independently of GPU FTZ arithmetic.
	uint significand = fraction | 0x00800000u;
	uint shift = (uint)(1 - encoded);
	uint rounded = significand >> shift;
	uint remainder = significand & ((1u << shift) - 1u);
	uint halfway = 1u << (shift - 1u);
	if (remainder > halfway || (remainder == halfway && (rounded & 1u) != 0u))
		++rounded;

	return asfloat(sign | rounded);
}

// Preserve an exact raw no-op before any subtraction can flush or overflow.
inline CoupledMeritValue CoupledWorkDelta(float a, float b)
{
	if (CoupledWorkSame(a, b))
		return CoupledMeritInput(0.0f);

	return CoupledMeritAdd(CoupledMeritInput(b), CoupledMeritInput(-a));
}

// Halve before adding so finite endpoints never require an overflowing unhalved sum.
inline CoupledMeritValue CoupledWorkMidpoint(float a, float b)
{
	return CoupledMeritAdd(
		CoupledMeritMul(CoupledMeritInput(0.5f), CoupledMeritInput(a)),
		CoupledMeritMul(CoupledMeritInput(0.5f), CoupledMeritInput(b)));
}

// Load one vector component without reconstructing its uncertainty.
inline CoupledMeritValue CoupledWorkComponent(CoupledWorkVector vector, int component)
{
	return vector.components[component];
}

// Store one vector component and its evaluation uncertainty together.
inline void CoupledWorkSetComponent(inout_(CoupledWorkVector) vector, int component, CoupledMeritValue value)
{
	vector.components[component] = value;
}

// Interpret a three-component stored input through the same scalar FTZ boundary.
inline CoupledWorkVector CoupledWorkInput3(float3 value)
{
	CoupledWorkVector result;
	for (int component = 0; component != 3; ++component)
		CoupledWorkSetComponent(result, component, CoupledMeritInput(value[component]));

	return result;
}

// Form an actual primary difference without losing exact unchanged components.
inline CoupledWorkVector CoupledWorkDelta3(float3 a, float3 b)
{
	CoupledWorkVector result;
	for (int component = 0; component != 3; ++component)
		CoupledWorkSetComponent(result, component, CoupledWorkDelta(a[component], b[component]));

	return result;
}

// Form a bounded primary midpoint without adding the unhalved endpoints.
inline CoupledWorkVector CoupledWorkMidpoint3(float3 a, float3 b)
{
	CoupledWorkVector result;
	for (int component = 0; component != 3; ++component)
		CoupledWorkSetComponent(result, component, CoupledWorkMidpoint(a[component], b[component]));

	return result;
}

// Add spatial-expression components in canonical component order.
inline CoupledWorkVector CoupledWorkAdd3(CoupledWorkVector a, CoupledWorkVector b)
{
	CoupledWorkVector result;
	for (int component = 0; component != 3; ++component)
		CoupledWorkSetComponent(result, component, CoupledMeritAdd(CoupledWorkComponent(a, component), CoupledWorkComponent(b, component)));

	return result;
}

// Scale a vector expression without discarding its input uncertainty.
inline CoupledWorkVector CoupledWorkScale3(CoupledWorkVector vector, CoupledMeritValue scale)
{
	for (int component = 0; component != 3; ++component)
		CoupledWorkSetComponent(vector, component, CoupledMeritMul(CoupledWorkComponent(vector, component), scale));

	return vector;
}

// Reduce a contraction in x/y/z order, preserving uncancelled error contributions.
inline CoupledMeritValue CoupledWorkDot3(CoupledWorkVector a, CoupledWorkVector b)
{
	CoupledMeritValue sum = CoupledMeritInput(0.0f);
	for (int component = 0; component != 3; ++component)
		sum = CoupledMeritAdd(sum, CoupledMeritMul(CoupledWorkComponent(a, component), CoupledWorkComponent(b, component)));

	return sum;
}

// Evaluate a right-handed cross product with six products and three ordered differences.
inline CoupledWorkVector CoupledWorkCross3(CoupledWorkVector a, CoupledWorkVector b)
{
	CoupledWorkVector result;
	for (int component = 0; component != 3; ++component)
	{
		int j = (component + 1) % 3;
		int k = (component + 2) % 3;
		CoupledWorkSetComponent(result, component, CoupledMeritAdd(
			CoupledMeritMul(CoupledWorkComponent(a, j), CoupledWorkComponent(b, k)),
			CoupledWorkNegate(CoupledMeritMul(CoupledWorkComponent(a, k), CoupledWorkComponent(b, j)))));
	}
	return result;
}

// Multiply a stored three-by-three matrix in row order, bounding its complete contraction.
inline CoupledWorkVector CoupledWorkMatrix3(float3x3 matrix, CoupledWorkVector vector)
{
	CoupledWorkVector result;
	for (int component = 0; component != 3; ++component)
		CoupledWorkSetComponent(result, component, CoupledWorkDot3(CoupledWorkInput3(matrix[component]), vector));

	return result;
}

// Rotate a motion expression using the same fixed quaternion map as the articulation cache.
inline CoupledWorkVector CoupledWorkRotate(float4 q, CoupledWorkVector vector)
{
	CoupledWorkVector xyz = CoupledWorkInput3(q.xyz);
	CoupledWorkVector t = CoupledWorkScale3(CoupledWorkCross3(xyz, vector), CoupledMeritInput(2.0f));
	return CoupledWorkAdd3(CoupledWorkAdd3(vector, CoupledWorkScale3(t, CoupledMeritInput(q.w))), CoupledWorkCross3(xyz, t));
}

// Select the angular or linear part of a bounded motion.
inline CoupledWorkVector CoupledWorkPart(CoupledWorkMotion motion, bool angular)
{
	if (angular)
		return motion.angular;

	return motion.translation;
}

// Pack both independently scaled halves without padding or a shared vector exponent.
inline CoupledWorkMotion CoupledWorkPack(CoupledWorkVector angular, CoupledWorkVector translation)
{
	CoupledWorkMotion result;
	result.angular = angular;
	result.translation = translation;
	return result;
}

// Return an exactly zero bounded motion.
inline CoupledWorkMotion CoupledWorkZeroMotion()
{
	return CoupledWorkPack(CoupledWorkInput3(float3(0, 0, 0)), CoupledWorkInput3(float3(0, 0, 0)));
}

// Transport parent motion to the child with inversion and quaternion arithmetic inside the local bound.
inline CoupledWorkMotion CoupledWorkChildMotion(GpuConstraintFrame child_to_parent, CoupledWorkMotion parent)
{
	float4 inverse_q = float4(-child_to_parent.rotation.xyz, child_to_parent.rotation.w);
	CoupledWorkVector inverse_position = CoupledWorkScale3(CoupledWorkRotate(inverse_q, CoupledWorkInput3(child_to_parent.position.xyz)), CoupledMeritInput(-1.0f));
	CoupledWorkVector angular = CoupledWorkRotate(inverse_q, CoupledWorkPart(parent, true));
	CoupledWorkVector translation = CoupledWorkAdd3(CoupledWorkRotate(inverse_q, CoupledWorkPart(parent, false)), CoupledWorkCross3(inverse_position, angular));
	return CoupledWorkPack(angular, translation);
}

// Add one retained motion screw times an actual generalized expression.
inline CoupledWorkMotion CoupledWorkAddScrew(CoupledWorkMotion motion, GpuArticulationSpatialVector screw, CoupledMeritValue speed)
{
	return CoupledWorkPack(
		CoupledWorkAdd3(CoupledWorkPart(motion, true), CoupledWorkScale3(CoupledWorkInput3(screw.ang.xyz), speed)),
		CoupledWorkAdd3(CoupledWorkPart(motion, false), CoupledWorkScale3(CoupledWorkInput3(screw.lin.xyz), speed)));
}

// Evaluate physical link work from bounded primary-motion difference/midpoint, not rounded momentum/cache pairs.
inline CoupledMeritValue CoupledWorkLink(GpuArticulationLink link, CoupledWorkMotion delta, CoupledWorkMotion midpoint)
{
	CoupledWorkVector angular_delta = CoupledWorkPart(delta, true);
	CoupledWorkVector angular_midpoint = CoupledWorkPart(midpoint, true);
	CoupledWorkVector com = CoupledWorkInput3(link.inertia_com_and_mass.xyz);
	CoupledWorkVector linear_delta = CoupledWorkAdd3(CoupledWorkPart(delta, false), CoupledWorkCross3(angular_delta, com));
	CoupledWorkVector linear_midpoint = CoupledWorkAdd3(CoupledWorkPart(midpoint, false), CoupledWorkCross3(angular_midpoint, com));
	float3x3 inertia = float3x3(
		float3(link.inertia_diagonal.x, link.inertia_products.x, link.inertia_products.y),
		float3(link.inertia_products.x, link.inertia_diagonal.y, link.inertia_products.z),
		float3(link.inertia_products.y, link.inertia_products.z, link.inertia_diagonal.z));
	CoupledMeritValue work = CoupledMeritAdd(
		CoupledWorkDot3(angular_delta, CoupledWorkMatrix3(inertia, angular_midpoint)),
		CoupledWorkDot3(linear_delta, linear_midpoint));
	return CoupledMeritMul(work, CoupledMeritInput(link.inertia_com_and_mass.w));
}

// Evaluate the actual rigid-primary work through one fixed symmetric inverse metric.
inline CoupledMeritValue CoupledWorkRigid(GpuRigidBody body, GpuArticulationSpatialVector a, GpuArticulationSpatialVector b)
{
	CoupledMeritValue work = CoupledMeritInput(0.0f);
	if (!CoupledWorkSame3(a.ang.xyz, b.ang.xyz))
	{
		float3x3 inertia = float3x3(
			float3(body.inertia_inv_diagonal.x, body.inertia_inv_products.x, body.inertia_inv_products.y),
			float3(body.inertia_inv_products.x, body.inertia_inv_diagonal.y, body.inertia_inv_products.z),
			float3(body.inertia_inv_products.y, body.inertia_inv_products.z, body.inertia_inv_diagonal.z));
		float3x3 rotation = (float3x3)body.o2w;
		CoupledWorkVector delta = CoupledWorkMatrix3(rotation, CoupledWorkDelta3(a.ang.xyz, b.ang.xyz));
		CoupledWorkVector midpoint = CoupledWorkMatrix3(rotation, CoupledWorkMidpoint3(a.ang.xyz, b.ang.xyz));
		work = CoupledWorkDot3(delta, CoupledWorkMatrix3(inertia, midpoint));
	}
	if (!CoupledWorkSame3(a.lin.xyz, b.lin.xyz))
		work = CoupledMeritAdd(work, CoupledWorkDot3(CoupledWorkDelta3(a.lin.xyz, b.lin.xyz), CoupledWorkMidpoint3(a.lin.xyz, b.lin.xyz)));

	return CoupledMeritMul(work, CoupledMeritInput(body.os_com_and_invmass.w));
}

// Return prescribed rigid motion with its constitutive arithmetic included where needed.
inline CoupledWorkMotion CoupledWorkRigidTwist(GpuRigidBody body)
{
	if (AllSet(body.state_flags, ERigidBodyStateFlags_Kinematic))
		return CoupledWorkPack(CoupledWorkInput3(body.momentum_ang.xyz), CoupledWorkInput3(body.momentum_lin.xyz));

	// A non-kinematic immovable endpoint has exactly zero prescribed twist, regardless of its finite stored momentum.
	if ((asuint(body.os_com_and_invmass.w) & 0x7fffffffu) == 0u)
		return CoupledWorkZeroMotion();

	float3x3 inertia = float3x3(
		float3(body.inertia_inv_diagonal.x, body.inertia_inv_products.x, body.inertia_inv_products.y),
		float3(body.inertia_inv_products.x, body.inertia_inv_diagonal.y, body.inertia_inv_products.z),
		float3(body.inertia_inv_products.y, body.inertia_inv_products.z, body.inertia_inv_diagonal.z));
	float3x3 rotation = (float3x3)body.o2w;
	CoupledWorkVector angular = CoupledWorkMatrix3(transpose(rotation), CoupledWorkMatrix3(inertia, CoupledWorkMatrix3(rotation, CoupledWorkInput3(body.momentum_ang.xyz))));
	CoupledMeritValue inverse_mass = CoupledMeritInput(body.os_com_and_invmass.w);
	return CoupledWorkPack(CoupledWorkScale3(angular, inverse_mass), CoupledWorkScale3(CoupledWorkInput3(body.momentum_lin.xyz), inverse_mass));
}

// Evaluate the source/softness part of one current row; equal finite ledgers perform no polarization.
inline CoupledMeritValue CoupledWorkRow(GpuConstraintRow row, float a, float b, CoupledMeritValue prescribed)
{
	if (!CoupledMeritFinite(prescribed))
		return CoupledMeritUnavailable();

	if (CoupledWorkSame(a, b))
		return CoupledMeritInput(0.0f);

	CoupledMeritValue delta = CoupledWorkDelta(a, b);
	CoupledMeritValue source = CoupledMeritAdd(CoupledMeritAdd(CoupledMeritInput(row.solve.y), CoupledMeritInput(-row.solve.z)), CoupledWorkNegate(prescribed));
	CoupledMeritValue work = CoupledWorkNegate(CoupledMeritMul(source, delta));
	if ((asuint(row.solve.w) & 0x7fffffffu) != 0u)
		work = CoupledMeritAdd(work, CoupledMeritMul(CoupledMeritInput(row.solve.w), CoupledMeritMul(delta, CoupledWorkMidpoint(a, b))));

	return work;
}

#undef PR_WORK_PRECISE
#ifdef __cplusplus
static_assert(sizeof(GpuCoupledConstraintMomentumReference) == 64);
static_assert(sizeof(GpuCoupledConstraintSpeedReference) == 8);
static_assert(sizeof(GpuCoupledConstraintWorkSpan) == 16);
static_assert(sizeof(CoupledMeritValue) == 16);
static_assert(sizeof(CoupledWorkVector) == 48);
static_assert(sizeof(CoupledWorkMotion) == 96);
static_assert(sizeof(GpuCoupledConstraintWorkMotion) == 192);
}
#endif
#endif
