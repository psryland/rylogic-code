//*********************************************
// Physics Engine — Pure-Tree Articulation Force ABA
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Pass order:
//   1. CSArticulationPrepare, once per breadth level from roots to leaves.
//   2. CSArticulationInwardDynamics, once per breadth level from leaves to roots.
//   3. CSArticulationRootDynamics.
//   4. CSArticulationOutwardDynamics, once per non-root breadth level from roots to leaves.
//
// Every dispatched lane owns one scheduled link. During the inward pass that link owns the
// deterministic reduction of all direct children, so siblings never race on parent scratch.
// Persistent scratch is 336L + 64D + 4*sum(d_j^2) bytes. The complete traversal is
// O(L+D+sum(d_j^2)) work and memory, with every local matrix bounded to six dimensions.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#ifndef PR_ARTICULATION_ABA_CPP_NAMESPACE
#define PR_ARTICULATION_ABA_CPP_NAMESPACE articulation_force_aba_detail
#define PR_ARTICULATION_ABA_CPP_NAMESPACE_OWNED
#endif
namespace PR_ARTICULATION_ABA_CPP_NAMESPACE {
#endif

#ifndef PR_ARTICULATION_ABA_CUSTOM_RESOURCES
// Per-dispatch schedule range and packed-buffer bounds.
struct cbArticulationForceAba
{
	int level_offset;
	int level_count;
	int articulation_count;
	int link_count;
};

ConstantBuffer<cbArticulationForceAba> resource(g_aba, b0);
StructuredBuffer<GpuArticulation> resource(g_aba_articulations, t0);
StructuredBuffer<GpuArticulationLink> resource(g_aba_links, t1);
StructuredBuffer<GpuArticulationDof> resource(g_aba_dofs, t2);
StructuredBuffer<float> resource(g_aba_positions, t3);
StructuredBuffer<float> resource(g_aba_velocities, t4);
StructuredBuffer<float> resource(g_aba_forces, t5);
StructuredBuffer<GpuFrameForce> resource(g_aba_external_forces, t6);
StructuredBuffer<uint> resource(g_aba_children, t7);
StructuredBuffer<uint> resource(g_aba_level_links, t8);
RWStructuredBuffer<float> resource(g_aba_accelerations, u0);
RWStructuredBuffer<GpuArticulationAbaScratch> resource(g_aba_scratch, u1);
RWStructuredBuffer<GpuArticulationAbaDofScratch> resource(g_aba_dof_scratch, u2);
RWStructuredBuffer<float> resource(g_aba_inverse_joint_inertia, u3);
#endif

// Return an explicitly initialized spatial vector.
GpuArticulationSpatialVector AbaSpatialVector(float3 ang, float3 lin)
{
	GpuArticulationSpatialVector value;
	value.ang = float4(ang, 0.0f);
	value.lin = float4(lin, 0.0f);
	return value;
}

// Return a zero spatial vector.
GpuArticulationSpatialVector AbaZeroSpatialVector()
{
	return AbaSpatialVector(float3(0.0f, 0.0f, 0.0f), float3(0.0f, 0.0f, 0.0f));
}

// Return one angular-then-linear component.
float AbaSpatialComponent(GpuArticulationSpatialVector value, int index)
{
	return index < 3 ? value.ang[index] : value.lin[index - 3];
}

// Replace one angular-then-linear component.
void AbaSetSpatialComponent(inout_(GpuArticulationSpatialVector) value, int index, float component)
{
	if (index < 3)
		value.ang[index] = component;
	else
		value.lin[index - 3] = component;
}

// Return one unit spatial motion basis column.
GpuArticulationSpatialVector AbaSpatialBasis(int index)
{
	GpuArticulationSpatialVector value = AbaZeroSpatialVector();
	AbaSetSpatialComponent(value, index, 1.0f);
	return value;
}

// Add two spatial vectors.
GpuArticulationSpatialVector AbaAddSpatial(GpuArticulationSpatialVector lhs, GpuArticulationSpatialVector rhs)
{
	return AbaSpatialVector(lhs.ang.xyz + rhs.ang.xyz, lhs.lin.xyz + rhs.lin.xyz);
}

// Subtract two spatial vectors.
GpuArticulationSpatialVector AbaSubtractSpatial(GpuArticulationSpatialVector lhs, GpuArticulationSpatialVector rhs)
{
	return AbaSpatialVector(lhs.ang.xyz - rhs.ang.xyz, lhs.lin.xyz - rhs.lin.xyz);
}

// Scale one spatial vector.
GpuArticulationSpatialVector AbaScaleSpatial(GpuArticulationSpatialVector value, float scale)
{
	return AbaSpatialVector(scale * value.ang.xyz, scale * value.lin.xyz);
}

// Return the dual-space dot product of angular-then-linear vectors.
float AbaSpatialDot(GpuArticulationSpatialVector lhs, GpuArticulationSpatialVector rhs)
{
	return dot(lhs.ang.xyz, rhs.ang.xyz) + dot(lhs.lin.xyz, rhs.lin.xyz);
}

// Return the motion-space spatial cross product.
GpuArticulationSpatialVector AbaCrossMotion(GpuArticulationSpatialVector lhs, GpuArticulationSpatialVector rhs)
{
	return AbaSpatialVector(
		cross(lhs.ang.xyz, rhs.ang.xyz),
		cross(lhs.ang.xyz, rhs.lin.xyz) + cross(lhs.lin.xyz, rhs.ang.xyz));
}

// Return the dual force-space spatial cross product.
GpuArticulationSpatialVector AbaCrossForce(GpuArticulationSpatialVector motion, GpuArticulationSpatialVector force)
{
	return AbaSpatialVector(
		cross(motion.ang.xyz, force.ang.xyz) + cross(motion.lin.xyz, force.lin.xyz),
		cross(motion.ang.xyz, force.lin.xyz));
}

// Return an identity rigid transform in the shared quaternion-and-position representation.
GpuConstraintFrame AbaIdentityTransform()
{
	GpuConstraintFrame transform;
	transform.rotation = float4(0.0f, 0.0f, 0.0f, 1.0f);
	transform.position = float4(0.0f, 0.0f, 0.0f, 1.0f);
	return transform;
}

// Compose affine transforms with the same lhs-times-rhs semantics as the CPU articulation code.
GpuConstraintFrame AbaMultiplyTransform(GpuConstraintFrame lhs, GpuConstraintFrame rhs)
{
	GpuConstraintFrame transform;
	transform.rotation = quat_mul(lhs.rotation, rhs.rotation);
	transform.position = float4(quat_rotate(lhs.rotation, rhs.position.xyz) + lhs.position.xyz, 1.0f);
	return transform;
}

// Return the rigid inverse of a shared quaternion-and-position transform.
GpuConstraintFrame AbaInvertTransform(GpuConstraintFrame transform)
{
	GpuConstraintFrame inverse;
	inverse.rotation = quat_conjugate(transform.rotation);
	inverse.position = float4(-quat_rotate(inverse.rotation, transform.position.xyz), 1.0f);
	return inverse;
}

// Transform a spatial motion from the source frame into the target frame.
GpuArticulationSpatialVector AbaTransformMotion(GpuConstraintFrame source_to_target, GpuArticulationSpatialVector motion)
{
	float3 ang = quat_rotate(source_to_target.rotation, motion.ang.xyz);
	float3 lin = quat_rotate(source_to_target.rotation, motion.lin.xyz) + cross(source_to_target.position.xyz, ang);
	return AbaSpatialVector(ang, lin);
}

// Transform a spatial force from the source frame into the target frame.
GpuArticulationSpatialVector AbaTransformForce(GpuConstraintFrame source_to_target, GpuArticulationSpatialVector force)
{
	float3 lin = quat_rotate(source_to_target.rotation, force.lin.xyz);
	float3 ang = quat_rotate(source_to_target.rotation, force.ang.xyz) + cross(source_to_target.position.xyz, lin);
	return AbaSpatialVector(ang, lin);
}

// Return an explicitly zeroed six-column spatial matrix.
GpuArticulationSpatialMatrix AbaZeroSpatialMatrix()
{
	GpuArticulationSpatialMatrix matrix;
	for (int column = 0; column != 6; ++column)
		matrix.columns[column] = AbaZeroSpatialVector();
	return matrix;
}

// Multiply a six-column spatial matrix by one angular-then-linear vector.
GpuArticulationSpatialVector AbaMultiplySpatialMatrix(GpuArticulationSpatialMatrix matrix, GpuArticulationSpatialVector vector)
{
	GpuArticulationSpatialVector result = AbaZeroSpatialVector();
	for (int column = 0; column != 6; ++column)
		result = AbaAddSpatial(result, AbaScaleSpatial(matrix.columns[column], AbaSpatialComponent(vector, column)));
	return result;
}

// Return one component from a padded joint matrix stored by columns.
float AbaJointMatrixComponent(GpuArticulationJointMatrix matrix, int row, int column)
{
	return row < 4 ? matrix.columns_low[column][row] : matrix.columns_high[column][row - 4];
}

// Replace one component in a padded joint matrix stored by columns.
void AbaSetJointMatrixComponent(inout_(GpuArticulationJointMatrix) matrix, int row, int column, float value)
{
	if (row < 4)
		matrix.columns_low[column][row] = value;
	else
		matrix.columns_high[column][row - 4] = value;
}

// Return an explicitly zeroed padded joint matrix.
GpuArticulationJointMatrix AbaZeroJointMatrix()
{
	GpuArticulationJointMatrix matrix;
	for (int column = 0; column != 6; ++column)
	{
		matrix.columns_low[column] = float4(0.0f, 0.0f, 0.0f, 0.0f);
		matrix.columns_high[column] = float4(0.0f, 0.0f, 0.0f, 0.0f);
	}
	return matrix;
}

// Return one component from a padded six-scalar joint vector.
float AbaJointVectorComponent(float4 low, float4 high, int index)
{
	return index < 4 ? low[index] : high[index - 4];
}

// Replace one component in a padded six-scalar joint vector.
void AbaSetJointVectorComponent(inout_(float4) low, inout_(float4) high, int index, float value)
{
	if (index < 4)
		low[index] = value;
	else
		high[index - 4] = value;
}

// Multiply a bounded joint matrix by a padded joint vector.
void AbaMultiplyJointMatrix(GpuArticulationJointMatrix matrix, float4 vector_low, float4 vector_high, int count, out_(float4) result_low, out_(float4) result_high)
{
	result_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
	result_high = float4(0.0f, 0.0f, 0.0f, 0.0f);
	for (int row = 0; row != count; ++row)
	{
		float value = 0.0f;
		for (int column = 0; column != count; ++column)
			value += AbaJointMatrixComponent(matrix, row, column) * AbaJointVectorComponent(vector_low, vector_high, column);
		AbaSetJointVectorComponent(result_low, result_high, row, value);
	}
}

// Invert a symmetric positive-definite zero-to-six-dimensional joint matrix through scale-aware Cholesky solves.
void AbaInvertJointMatrix(GpuArticulationJointMatrix input_matrix, int count, out_(GpuArticulationJointMatrix) inverse, out_(bool) valid)
{
	GpuArticulationJointMatrix matrix = input_matrix;
	GpuArticulationJointMatrix lower = AbaZeroJointMatrix();
	inverse = AbaZeroJointMatrix();
	valid = true;
	float scale = 1.0f;

	// Symmetrize accumulated round-off and derive the same relative pivot scale used by the CPU solver.
	for (int row = 0; row != count; ++row)
	for (int column = 0; column != count; ++column)
		scale = max(scale, abs(AbaJointMatrixComponent(matrix, row, column)));
	for (int row = 0; row != count; ++row)
	for (int column = row + 1; column != count; ++column)
	{
		float value = 0.5f * (
			AbaJointMatrixComponent(matrix, row, column) +
			AbaJointMatrixComponent(matrix, column, row));
		AbaSetJointMatrixComponent(matrix, row, column, value);
		AbaSetJointMatrixComponent(matrix, column, row, value);
	}

	// Factor the active leading block while retaining finite fallback pivots for diagnostic output.
	float pivot_tolerance = 64.0f * 1.192092896e-7f * scale * (float)count;
	for (int row = 0; row != count; ++row)
	{
		for (int column = 0; column != row + 1; ++column)
		{
			float value = AbaJointMatrixComponent(matrix, row, column);
			for (int inner = 0; inner != column; ++inner)
				value -= AbaJointMatrixComponent(lower, row, inner) * AbaJointMatrixComponent(lower, column, inner);

			if (row == column)
			{
				valid = valid && isfinite(value) && value > pivot_tolerance;
				AbaSetJointMatrixComponent(lower, row, column, sqrt(max(value, pivot_tolerance)));
			}
			else
			{
				float diagonal = AbaJointMatrixComponent(lower, column, column);
				AbaSetJointMatrixComponent(lower, row, column, value / diagonal);
			}
		}
	}

	// Solve one unit right-hand side per column so the stored inverse is directly reusable during the outward pass.
	for (int inverse_column = 0; inverse_column != count; ++inverse_column)
	{
		float4 intermediate_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
		float4 intermediate_high = float4(0.0f, 0.0f, 0.0f, 0.0f);
		for (int row = 0; row != count; ++row)
		{
			float value = row == inverse_column ? 1.0f : 0.0f;
			for (int inner = 0; inner != row; ++inner)
				value -= AbaJointMatrixComponent(lower, row, inner) * AbaJointVectorComponent(intermediate_low, intermediate_high, inner);
			value /= AbaJointMatrixComponent(lower, row, row);
			AbaSetJointVectorComponent(intermediate_low, intermediate_high, row, value);
		}

		for (int reverse_row = 0; reverse_row != count; ++reverse_row)
		{
			int row = count - 1 - reverse_row;
			float value = AbaJointVectorComponent(intermediate_low, intermediate_high, row);
			for (int inner = row + 1; inner != count; ++inner)
				value -= AbaJointMatrixComponent(lower, inner, row) * AbaJointMatrixComponent(inverse, inner, inverse_column);
			value /= AbaJointMatrixComponent(lower, row, row);
			AbaSetJointMatrixComponent(inverse, row, inverse_column, value);
		}
	}
}

// Persist only the active d-by-d inverse block assigned to one packed link.
void AbaStoreInverseJointMatrix(GpuArticulationLink link, GpuArticulationJointMatrix inverse)
{
	for (int column = 0; column != link.dof_count; ++column)
	for (int row = 0; row != link.dof_count; ++row)
		g_aba_inverse_joint_inertia[link.joint_matrix_offset + column * link.dof_count + row] = AbaJointMatrixComponent(inverse, row, column);
}

// Materialize one tightly packed active inverse block into bounded lane-local storage.
GpuArticulationJointMatrix AbaLoadInverseJointMatrix(GpuArticulationLink link)
{
	GpuArticulationJointMatrix inverse = AbaZeroJointMatrix();
	for (int column = 0; column != link.dof_count; ++column)
	for (int row = 0; row != link.dof_count; ++row)
	{
		float value = g_aba_inverse_joint_inertia[link.joint_matrix_offset + column * link.dof_count + row];
		AbaSetJointMatrixComponent(inverse, row, column, value);
	}
	return inverse;
}

// Multiply compact physical link inertia by one link-frame spatial motion.
GpuArticulationSpatialVector AbaMultiplyPhysicalInertia(GpuArticulationLink link, GpuArticulationSpatialVector motion)
{
	float mass = link.inertia_com_and_mass.w;
	float3 centre_of_mass = link.inertia_com_and_mass.xyz;
	float3x3 central_unit_inertia = build_symmetric_3x3(link.inertia_diagonal.xyz, link.inertia_products.xyz);
	float3 centre_velocity = motion.lin.xyz + cross(motion.ang.xyz, centre_of_mass);
	float3 linear_momentum = mass * centre_velocity;
	float3 angular_momentum = mass * mul(central_unit_inertia, motion.ang.xyz) + cross(centre_of_mass, linear_momentum);
	return AbaSpatialVector(angular_momentum, linear_momentum);
}

// Materialize the physical link inertia as six spatial columns.
GpuArticulationSpatialMatrix AbaPhysicalInertia(GpuArticulationLink link)
{
	GpuArticulationSpatialMatrix inertia = AbaZeroSpatialMatrix();
	for (int column = 0; column != 6; ++column)
		inertia.columns[column] = AbaMultiplyPhysicalInertia(link, AbaSpatialBasis(column));
	return inertia;
}

// Return the transform generated by one scalar ordered joint coordinate.
GpuConstraintFrame AbaAxisTransform(GpuArticulationDof dof, float position)
{
	GpuConstraintFrame transform = AbaIdentityTransform();
	int axis_type = (int)dof.axis_and_type.w;
	switch (axis_type)
	{
		case GpuArticulationAxisType_Revolute:
		{
			float half_angle = 0.5f * position;
			transform.rotation = float4(sin(half_angle) * dof.axis_and_type.xyz, cos(half_angle));
			break;
		}
		case GpuArticulationAxisType_Prismatic:
		{
			transform.position = float4(position * dof.axis_and_type.xyz, 1.0f);
			break;
		}
		default:
		{
			break;
		}
	}
	return transform;
}

// Return the constant unit motion subspace of one scalar axis in its own frame.
GpuArticulationSpatialVector AbaAxisMotion(GpuArticulationDof dof)
{
	int axis_type = (int)dof.axis_and_type.w;
	switch (axis_type)
	{
		case GpuArticulationAxisType_Revolute:
		{
			return AbaSpatialVector(dof.axis_and_type.xyz, float3(0.0f, 0.0f, 0.0f));
		}
		case GpuArticulationAxisType_Prismatic:
		{
			return AbaSpatialVector(float3(0.0f, 0.0f, 0.0f), dof.axis_and_type.xyz);
		}
		default:
		{
			return AbaZeroSpatialVector();
		}
	}
}

// Load six contiguous generalized velocity values as one spatial vector.
GpuArticulationSpatialVector AbaLoadGeneralizedVelocity(int offset)
{
	return AbaSpatialVector(
		float3(g_aba_velocities[offset + 0], g_aba_velocities[offset + 1], g_aba_velocities[offset + 2]),
		float3(g_aba_velocities[offset + 3], g_aba_velocities[offset + 4], g_aba_velocities[offset + 5]));
}

// Load six contiguous generalized force values as one spatial vector.
GpuArticulationSpatialVector AbaLoadGeneralizedForce(int offset)
{
	return AbaSpatialVector(
		float3(g_aba_forces[offset + 0], g_aba_forces[offset + 1], g_aba_forces[offset + 2]),
		float3(g_aba_forces[offset + 3], g_aba_forces[offset + 4], g_aba_forces[offset + 5]));
}

// Store one spatial vector as six contiguous generalized accelerations.
void AbaStoreGeneralizedAcceleration(int offset, GpuArticulationSpatialVector spatial)
{
	g_aba_accelerations[offset + 0] = spatial.ang.x;
	g_aba_accelerations[offset + 1] = spatial.ang.y;
	g_aba_accelerations[offset + 2] = spatial.ang.z;
	g_aba_accelerations[offset + 3] = spatial.lin.x;
	g_aba_accelerations[offset + 4] = spatial.lin.y;
	g_aba_accelerations[offset + 5] = spatial.lin.z;
}

// Reuse the no-longer-needed articulated-bias field for solved acceleration after every inward level completes.
void AbaStoreLinkAcceleration(inout_(GpuArticulationAbaScratch) scratch, GpuArticulationSpatialVector acceleration)
{
	scratch.articulated_bias_or_acceleration = acceleration;
}

// Evaluate one serial ordered joint, retaining only its transform, bias, and per-DOF motion-subspace columns.
void AbaEvaluateJoint(GpuArticulationLink link, inout_(GpuArticulationAbaScratch) scratch, out_(GpuArticulationSpatialVector) joint_velocity)
{
	GpuConstraintFrame motion_to_parent_joint = AbaIdentityTransform();
	joint_velocity = AbaZeroSpatialVector();
	GpuArticulationSpatialVector joint_bias = AbaZeroSpatialVector();

	// Propagate zero parent motion through each moving scalar frame to recover aggregate joint motion.
	for (int axis_index = 0; axis_index != link.dof_count; ++axis_index)
	{
		GpuArticulationDof dof = g_aba_dofs[link.dof_offset + axis_index];
		GpuConstraintFrame axis_to_parent = AbaAxisTransform(dof, g_aba_positions[link.position_offset + axis_index]);
		GpuConstraintFrame parent_to_axis = AbaInvertTransform(axis_to_parent);
		GpuArticulationSpatialVector axis_velocity = AbaScaleSpatial(
			AbaAxisMotion(dof),
			g_aba_velocities[link.velocity_offset + axis_index]);
		motion_to_parent_joint = AbaMultiplyTransform(motion_to_parent_joint, axis_to_parent);
		joint_velocity = AbaAddSpatial(AbaTransformMotion(parent_to_axis, joint_velocity), axis_velocity);
		joint_bias = AbaAddSpatial(AbaTransformMotion(parent_to_axis, joint_bias), AbaCrossMotion(joint_velocity, axis_velocity));
	}

	// Move terminal joint quantities into the physical child-link frame.
	scratch.child_to_parent = AbaMultiplyTransform(
		AbaMultiplyTransform(link.joint_to_parent, motion_to_parent_joint),
		AbaInvertTransform(link.joint_to_child));
	joint_velocity = AbaTransformMotion(link.joint_to_child, joint_velocity);
	scratch.joint_bias = AbaTransformMotion(link.joint_to_child, joint_bias);

	// Probe every unit generalized speed and write each compact S column directly to its generalized-DOF slot.
	for (int column = 0; column != link.dof_count; ++column)
	{
		GpuArticulationSpatialVector unit_velocity = AbaZeroSpatialVector();
		for (int axis_index = 0; axis_index != link.dof_count; ++axis_index)
		{
			GpuArticulationDof dof = g_aba_dofs[link.dof_offset + axis_index];
			GpuConstraintFrame parent_to_axis = AbaInvertTransform(AbaAxisTransform(dof, g_aba_positions[link.position_offset + axis_index]));
			unit_velocity = AbaTransformMotion(parent_to_axis, unit_velocity);
			if (axis_index == column)
				unit_velocity = AbaAddSpatial(unit_velocity, AbaAxisMotion(dof));

		}
		GpuArticulationAbaDofScratch dof_scratch = g_aba_dof_scratch[link.dof_offset + column];
		dof_scratch.motion_subspace = AbaTransformMotion(link.joint_to_child, unit_velocity);
		g_aba_dof_scratch[link.dof_offset + column] = dof_scratch;
	}
}

// Return U*D^-1*U-transpose as a full spatial inertia.
GpuArticulationSpatialMatrix AbaJointInertiaReduction(GpuArticulationLink link, GpuArticulationJointMatrix inverse)
{
	GpuArticulationSpatialMatrix reduction = AbaZeroSpatialMatrix();
	for (int spatial_column = 0; spatial_column != 6; ++spatial_column)
	{
		float4 projection_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
		float4 projection_high = float4(0.0f, 0.0f, 0.0f, 0.0f);
		for (int joint_row = 0; joint_row != link.dof_count; ++joint_row)
			AbaSetJointVectorComponent(
				projection_low,
				projection_high,
				joint_row,
				AbaSpatialComponent(g_aba_dof_scratch[link.dof_offset + joint_row].u_column, spatial_column));

		float4 coefficients_low;
		float4 coefficients_high;
		AbaMultiplyJointMatrix(
			inverse,
			projection_low,
			projection_high,
			link.dof_count,
			coefficients_low,
			coefficients_high);
		for (int joint_column = 0; joint_column != link.dof_count; ++joint_column)
		{
			reduction.columns[spatial_column] = AbaAddSpatial(
				reduction.columns[spatial_column],
				AbaScaleSpatial(
					g_aba_dof_scratch[link.dof_offset + joint_column].u_column,
					AbaJointVectorComponent(coefficients_low, coefficients_high, joint_column)));
		}
	}
	return reduction;
}

// Transform a motion-to-force matrix through a rigid coordinate change.
GpuArticulationSpatialMatrix AbaTransformInertia(GpuConstraintFrame parent_to_child, GpuConstraintFrame child_to_parent, GpuArticulationSpatialMatrix child_inertia)
{
	GpuArticulationSpatialMatrix parent_inertia = AbaZeroSpatialMatrix();
	for (int column = 0; column != 6; ++column)
	{
		GpuArticulationSpatialVector child_motion = AbaTransformMotion(parent_to_child, AbaSpatialBasis(column));
		GpuArticulationSpatialVector child_force = AbaMultiplySpatialMatrix(child_inertia, child_motion);
		parent_inertia.columns[column] = AbaTransformForce(child_to_parent, child_force);
	}
	return parent_inertia;
}

// Add one matrix into another without relying on cross-language matrix operators.
void AbaAddSpatialMatrix(inout_(GpuArticulationSpatialMatrix) destination, GpuArticulationSpatialMatrix source)
{
	for (int column = 0; column != 6; ++column)
		destination.columns[column] = AbaAddSpatial(destination.columns[column], source.columns[column]);
}

// Prepare one link's kinematics, physical articulated inertia, and force bias.
void AbaPrepareLink(int link_index)
{
	GpuArticulationLink link = g_aba_links[link_index];
	GpuArticulationAbaScratch scratch = g_aba_scratch[link_index];
	scratch.articulated_inertia = AbaPhysicalInertia(link);
	scratch.solve_valid = 1;

	// Roots take their velocity directly from generalized state; every other link derives it from its prepared parent.
	if (link.parent_link_index < 0)
	{
		GpuArticulation articulation = g_aba_articulations[link.articulation_index];
		scratch.child_to_parent = AbaIdentityTransform();
		scratch.joint_bias = AbaZeroSpatialVector();
		scratch.link_velocity = AbaZeroSpatialVector();
		if (articulation.root_type == GpuArticulationRootType_Floating)
			scratch.link_velocity = AbaLoadGeneralizedVelocity(articulation.velocity_offset);
	}
	else
	{
		GpuArticulationSpatialVector joint_velocity;
		AbaEvaluateJoint(link, scratch, joint_velocity);
		GpuArticulationAbaScratch parent = g_aba_scratch[link.parent_link_index];
		GpuConstraintFrame parent_to_child = AbaInvertTransform(scratch.child_to_parent);
		scratch.link_velocity = AbaAddSpatial(
			AbaTransformMotion(parent_to_child, parent.link_velocity),
			joint_velocity);
		scratch.joint_bias = AbaAddSpatial(
			scratch.joint_bias,
			AbaCrossMotion(scratch.link_velocity, joint_velocity));
	}

	// Full force ABA retains gyroscopic bias and subtracts the link-frame external wrench.
	GpuArticulationSpatialVector momentum = AbaMultiplySpatialMatrix(scratch.articulated_inertia, scratch.link_velocity);
	GpuFrameForce external_force = g_aba_external_forces[link_index];
	scratch.articulated_bias_or_acceleration = AbaSubtractSpatial(
		AbaCrossForce(scratch.link_velocity, momentum),
		AbaSpatialVector(external_force.force_ang.xyz, external_force.force_lin.xyz));
	g_aba_scratch[link_index] = scratch;
}

// Eliminate every direct child joint and reduce its articulated operator into the selected parent.
void AbaInwardLink(int parent_index)
{
	GpuArticulationLink parent_link = g_aba_links[parent_index];
	GpuArticulationAbaScratch parent = g_aba_scratch[parent_index];

	// A parent lane visits each adjacency entry exactly once, preserving deterministic sibling reduction order.
	for (int child_offset = 0; child_offset != parent_link.child_count; ++child_offset)
	{
		int child_index = (int)g_aba_children[parent_link.child_offset + child_offset];
		GpuArticulationLink child_link = g_aba_links[child_index];
		GpuArticulationAbaScratch child = g_aba_scratch[child_index];
		GpuArticulationJointMatrix joint_inertia = AbaZeroJointMatrix();
		float4 reduced_force_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
		float4 reduced_force_high = float4(0.0f, 0.0f, 0.0f, 0.0f);

		// Form U, D, and u in ordered scalar-joint coordinates, reusing generalized acceleration storage for u.
		for (int row = 0; row != child_link.dof_count; ++row)
		{
			int dof_index = child_link.dof_offset + row;
			GpuArticulationAbaDofScratch dof_scratch = g_aba_dof_scratch[dof_index];
			dof_scratch.u_column = AbaMultiplySpatialMatrix(child.articulated_inertia, dof_scratch.motion_subspace);
			g_aba_dof_scratch[dof_index] = dof_scratch;
			float reduced_force =
				g_aba_forces[child_link.velocity_offset + row] -
				AbaSpatialDot(dof_scratch.motion_subspace, child.articulated_bias_or_acceleration);
			AbaSetJointVectorComponent(reduced_force_low, reduced_force_high, row, reduced_force);
			g_aba_accelerations[child_link.velocity_offset + row] = reduced_force;
		}
		for (int row = 0; row != child_link.dof_count; ++row)
		for (int column = 0; column != child_link.dof_count; ++column)
			AbaSetJointMatrixComponent(
				joint_inertia,
				row,
				column,
				AbaSpatialDot(
					g_aba_dof_scratch[child_link.dof_offset + row].motion_subspace,
					g_aba_dof_scratch[child_link.dof_offset + column].u_column));

		GpuArticulationJointMatrix inverse_joint_inertia;
		bool joint_valid;
		AbaInvertJointMatrix(joint_inertia, child_link.dof_count, inverse_joint_inertia, joint_valid);
		AbaStoreInverseJointMatrix(child_link, inverse_joint_inertia);
		child.solve_valid = child.solve_valid != 0 && joint_valid ? 1 : 0;
		parent.solve_valid = parent.solve_valid != 0 && child.solve_valid != 0 ? 1 : 0;

		// Remove joint-space response while retaining generalized-force and complete velocity-bias contributions.
		GpuArticulationSpatialMatrix reduction = AbaJointInertiaReduction(child_link, inverse_joint_inertia);
		GpuArticulationSpatialMatrix reduced_inertia = AbaZeroSpatialMatrix();
		for (int column = 0; column != 6; ++column)
			reduced_inertia.columns[column] = AbaSubtractSpatial(child.articulated_inertia.columns[column], reduction.columns[column]);

		float4 reduced_coefficients_low;
		float4 reduced_coefficients_high;
		AbaMultiplyJointMatrix(
			inverse_joint_inertia,
			reduced_force_low,
			reduced_force_high,
			child_link.dof_count,
			reduced_coefficients_low,
			reduced_coefficients_high);
		GpuArticulationSpatialVector reduced_bias = child.articulated_bias_or_acceleration;
		for (int column = 0; column != child_link.dof_count; ++column)
		{
			reduced_bias = AbaAddSpatial(
				reduced_bias,
				AbaScaleSpatial(
					g_aba_dof_scratch[child_link.dof_offset + column].u_column,
					AbaJointVectorComponent(reduced_coefficients_low, reduced_coefficients_high, column)));
		}
		reduced_bias = AbaAddSpatial(reduced_bias, AbaMultiplySpatialMatrix(reduced_inertia, child.joint_bias));

		// Transform the reduced child operator back to its parent and commit factors for the outward pass.
		GpuConstraintFrame parent_to_child = AbaInvertTransform(child.child_to_parent);
		AbaAddSpatialMatrix(
			parent.articulated_inertia,
			AbaTransformInertia(parent_to_child, child.child_to_parent, reduced_inertia));
		parent.articulated_bias_or_acceleration = AbaAddSpatial(
			parent.articulated_bias_or_acceleration,
			AbaTransformForce(child.child_to_parent, reduced_bias));
		g_aba_scratch[child_index] = child;
	}

	g_aba_scratch[parent_index] = parent;
}

// Solve one floating root's complete articulated inertia or impose zero acceleration on a fixed root.
void AbaRootDynamics(int articulation_index)
{
	GpuArticulation articulation = g_aba_articulations[articulation_index];
	int root_index = articulation.link_offset;
	GpuArticulationAbaScratch root = g_aba_scratch[root_index];
	GpuArticulationSpatialVector root_acceleration = AbaZeroSpatialVector();

	switch (articulation.root_type)
	{
		case GpuArticulationRootType_Fixed:
		{
			break;
		}
		case GpuArticulationRootType_Floating:
		{
			// Reuse the bounded SPD solve for the root's full six-dimensional articulated inertia.
			GpuArticulationJointMatrix root_inertia = AbaZeroJointMatrix();
			for (int row = 0; row != 6; ++row)
			for (int column = 0; column != 6; ++column)
				AbaSetJointMatrixComponent(
					root_inertia,
					row,
					column,
					AbaSpatialComponent(root.articulated_inertia.columns[column], row));

			GpuArticulationJointMatrix root_inverse;
			bool root_valid;
			AbaInvertJointMatrix(root_inertia, 6, root_inverse, root_valid);
			root.solve_valid = root.solve_valid != 0 && root_valid ? 1 : 0;

			GpuArticulationSpatialVector root_force = AbaLoadGeneralizedForce(articulation.velocity_offset);
			GpuArticulationSpatialVector root_rhs = AbaSubtractSpatial(root_force, root.articulated_bias_or_acceleration);
			float4 rhs_low = float4(root_rhs.ang.xyz, root_rhs.lin.x);
			float4 rhs_high = float4(root_rhs.lin.y, root_rhs.lin.z, 0.0f, 0.0f);
			float4 acceleration_low;
			float4 acceleration_high;
			AbaMultiplyJointMatrix(root_inverse, rhs_low, rhs_high, 6, acceleration_low, acceleration_high);
			root_acceleration = AbaSpatialVector(
				acceleration_low.xyz,
				float3(acceleration_low.w, acceleration_high.x, acceleration_high.y));
			AbaStoreGeneralizedAcceleration(articulation.velocity_offset, root_acceleration);
			break;
		}
		default:
		{
			root.solve_valid = 0;
			break;
		}
	}

	// All inward consumers have completed, so articulated bias now becomes the root's solved acceleration.
	AbaStoreLinkAcceleration(root, root_acceleration);
	g_aba_scratch[root_index] = root;
}

// Recover one non-root link and its ordered generalized accelerations from its solved parent.
void AbaOutwardLink(int link_index)
{
	GpuArticulationLink link = g_aba_links[link_index];
	GpuArticulationAbaScratch scratch = g_aba_scratch[link_index];
	GpuArticulationSpatialVector parent_acceleration = g_aba_scratch[link.parent_link_index].articulated_bias_or_acceleration;
	GpuArticulationSpatialVector link_acceleration = AbaAddSpatial(
		AbaTransformMotion(AbaInvertTransform(scratch.child_to_parent), parent_acceleration),
		scratch.joint_bias);

	// Load the phase-reused generalized u values before overwriting them with qdd.
	float4 acceleration_rhs_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 acceleration_rhs_high = float4(0.0f, 0.0f, 0.0f, 0.0f);
	for (int row = 0; row != link.dof_count; ++row)
	{
		AbaSetJointVectorComponent(
			acceleration_rhs_low,
			acceleration_rhs_high,
			row,
			g_aba_accelerations[link.velocity_offset + row]);
	}

	// Solve qdd=D^-1*(u-U-transpose*a) and add S*qdd to the propagated link acceleration.
	for (int row = 0; row != link.dof_count; ++row)
	{
		float value =
			AbaJointVectorComponent(acceleration_rhs_low, acceleration_rhs_high, row) -
			AbaSpatialDot(link_acceleration, g_aba_dof_scratch[link.dof_offset + row].u_column);
		AbaSetJointVectorComponent(acceleration_rhs_low, acceleration_rhs_high, row, value);
	}

	float4 joint_acceleration_low;
	float4 joint_acceleration_high;
	GpuArticulationJointMatrix inverse_joint_inertia = AbaLoadInverseJointMatrix(link);
	AbaMultiplyJointMatrix(
		inverse_joint_inertia,
		acceleration_rhs_low,
		acceleration_rhs_high,
		link.dof_count,
		joint_acceleration_low,
		joint_acceleration_high);
	for (int row = 0; row != link.dof_count; ++row)
	{
		float joint_acceleration = AbaJointVectorComponent(joint_acceleration_low, joint_acceleration_high, row);
		link_acceleration = AbaAddSpatial(
			link_acceleration,
			AbaScaleSpatial(g_aba_dof_scratch[link.dof_offset + row].motion_subspace, joint_acceleration));
		g_aba_accelerations[link.velocity_offset + row] = joint_acceleration;
	}

	// This link's bias is dead once its acceleration is solved; child levels consume the replacement value.
	AbaStoreLinkAcceleration(scratch, link_acceleration);
	g_aba_scratch[link_index] = scratch;
}

#ifndef PR_ARTICULATION_ABA_NO_ENTRYPOINTS
// Prepare one scheduled link through the canonical resource operation.
numthreads(CSArticulationPrepare, ArticulationThreadCount, 1, 1)
void CSArticulationPrepare(int3 DTID(dtid))
{
	if (dtid.x >= g_aba.level_count)
		return;

	int link_index = (int)g_aba_level_links[g_aba.level_offset + dtid.x];
	if (link_index < 0 || link_index >= g_aba.link_count)
		return;

	AbaPrepareLink(link_index);
}

// Reduce one scheduled parent through the canonical resource operation.
numthreads(CSArticulationInwardDynamics, ArticulationThreadCount, 1, 1)
void CSArticulationInwardDynamics(int3 DTID(dtid))
{
	if (dtid.x >= g_aba.level_count)
		return;

	int parent_index = (int)g_aba_level_links[g_aba.level_offset + dtid.x];
	if (parent_index < 0 || parent_index >= g_aba.link_count)
		return;

	AbaInwardLink(parent_index);
}

// Solve one scheduled root through the canonical resource operation.
numthreads(CSArticulationRootDynamics, ArticulationThreadCount, 1, 1)
void CSArticulationRootDynamics(int3 DTID(dtid))
{
	if (dtid.x >= g_aba.articulation_count)
		return;

	AbaRootDynamics(dtid.x);
}

// Recover one scheduled non-root link through the canonical resource operation.
numthreads(CSArticulationOutwardDynamics, ArticulationThreadCount, 1, 1)
void CSArticulationOutwardDynamics(int3 DTID(dtid))
{
	if (dtid.x >= g_aba.level_count)
		return;

	int link_index = (int)g_aba_level_links[g_aba.level_offset + dtid.x];
	if (link_index < 0 || link_index >= g_aba.link_count)
		return;

	AbaOutwardLink(link_index);
}
#endif

#ifdef __cplusplus
}
#ifdef PR_ARTICULATION_ABA_CPP_NAMESPACE_OWNED
#undef PR_ARTICULATION_ABA_CPP_NAMESPACE_OWNED
#undef PR_ARTICULATION_ABA_CPP_NAMESPACE
#endif
}
#endif
