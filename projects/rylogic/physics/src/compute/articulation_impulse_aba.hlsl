//*********************************************
// Physics Engine — Articulation Impulse ABA
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// One invocation owns one participating articulation and applies all gathered link-frame
// impulses through exactly one inward/outward ABA response at fixed configuration.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Compact participation and packed-buffer bounds for one impulse-ABA dispatch.
struct cbArticulationImpulseAba
{
	int participating_articulation_count;
	int articulation_count;
	int link_count;
	int mobility_count;
};

ConstantBuffer<cbArticulationImpulseAba> resource(g_impulse, b0);
StructuredBuffer<GpuArticulationMobilityRange> resource(g_mobility_ranges, t0);
StructuredBuffer<GpuArticulation> resource(g_aba_articulations, t1);
StructuredBuffer<GpuArticulationLink> resource(g_aba_links, t2);
StructuredBuffer<GpuArticulationDof> resource(g_aba_dofs, t3);
StructuredBuffer<float> resource(g_aba_positions, t4);
StructuredBuffer<float> resource(g_aba_forces, t5);
StructuredBuffer<GpuFrameForce> resource(g_aba_external_forces, t6);
StructuredBuffer<uint> resource(g_aba_children, t7);
StructuredBuffer<GpuArticulationSpatialMobility> resource(g_link_mobilities, t8);
StructuredBuffer<GpuArticulationSpatialVector> resource(g_link_impulses, t9);
RWStructuredBuffer<float> resource(g_aba_velocities, u0);
RWStructuredBuffer<float> resource(g_aba_accelerations, u1);
RWStructuredBuffer<GpuArticulationAbaScratch> resource(g_aba_scratch, u2);
RWStructuredBuffer<GpuArticulationAbaDofScratch> resource(g_aba_dof_scratch, u3);
RWStructuredBuffer<float> resource(g_aba_inverse_joint_inertia, u4);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_impulse_work, u5);

#ifdef __cplusplus
}
#endif

// Reuse the canonical spatial transforms and retained ABA factor accessors.
#define PR_ARTICULATION_ABA_CUSTOM_RESOURCES
#define PR_ARTICULATION_ABA_NO_ENTRYPOINTS
#define PR_ARTICULATION_ABA_CPP_NAMESPACE articulation_impulse_aba_detail
#include "physics/src/compute/articulation_force_aba.hlsl"
#undef PR_ARTICULATION_ABA_CPP_NAMESPACE
#undef PR_ARTICULATION_ABA_NO_ENTRYPOINTS
#undef PR_ARTICULATION_ABA_CUSTOM_RESOURCES

#ifdef __cplusplus
namespace pr::physics {
using namespace articulation_impulse_aba_detail;
#endif

#define PR_ARTICULATION_MOBILITY_OPS_CPP_NAMESPACE articulation_impulse_mobility_ops_detail
#include "physics/src/compute/articulation_mobility_ops.hlsli"
#undef PR_ARTICULATION_MOBILITY_OPS_CPP_NAMESPACE

// Return true when both halves of one spatial vector are finite.
bool ImpulseSpatialFinite(GpuArticulationSpatialVector value)
{
	return
		isfinite(value.ang.x) && isfinite(value.ang.y) && isfinite(value.ang.z) &&
		isfinite(value.lin.x) && isfinite(value.lin.y) && isfinite(value.lin.z);
}

// Mark one tree invalid without committing a partial generalized or cached-link response.
void ImpulseInvalidateTree(int root_link_index)
{
	GpuArticulationAbaScratch root_scratch = g_aba_scratch[root_link_index];
	root_scratch.solve_valid = 0;
	g_aba_scratch[root_link_index] = root_scratch;
}

// Reduce one child impulse through its retained joint factors and add it to the parent accumulator.
void ImpulseReduceChild(GpuArticulationMobilityRange range, GpuArticulation articulation, int local_link_index)
{
	int link_index = articulation.link_offset + local_link_index;
	int work_index = range.mobility_offset + local_link_index;
	GpuArticulationLink link = g_aba_links[link_index];
	GpuArticulationSpatialVector child_bias = g_impulse_work[work_index];
	float4 reduced_force_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 reduced_force_high = float4(0.0f, 0.0f, 0.0f, 0.0f);

	// The impulse solve has zero generalized input, so u is minus the joint projection of the accumulated bias impulse.
	for (int row = 0; row != link.dof_count; ++row)
	{
		float reduced_force = -AbaSpatialDot(g_aba_dof_scratch[link.dof_offset + row].motion_subspace, child_bias);
		AbaSetJointVectorComponent(reduced_force_low, reduced_force_high, row, reduced_force);
		g_aba_accelerations[link.velocity_offset + row] = reduced_force;
	}

	float4 coefficients_low;
	float4 coefficients_high;
	AbaMultiplyJointMatrix(
		AbaLoadInverseJointMatrix(link),
		reduced_force_low,
		reduced_force_high,
		link.dof_count,
		coefficients_low,
		coefficients_high);
	GpuArticulationSpatialVector reduced_bias = child_bias;
	for (int column = 0; column != link.dof_count; ++column)
	{
		reduced_bias = AbaAddSpatial(
			reduced_bias,
			AbaScaleSpatial(
				g_aba_dof_scratch[link.dof_offset + column].u_column,
				AbaJointVectorComponent(coefficients_low, coefficients_high, column)));
	}

	int parent_work_index = range.mobility_offset + link.parent_link_index - articulation.link_offset;
	g_impulse_work[parent_work_index] = AbaAddSpatial(
		g_impulse_work[parent_work_index],
		AbaTransformForce(g_aba_scratch[link_index].child_to_parent, reduced_bias));
}

// Recover one child link and its generalized velocity delta from its solved parent.
void ImpulseRecoverChild(GpuArticulationMobilityRange range, GpuArticulation articulation, int local_link_index)
{
	int link_index = articulation.link_offset + local_link_index;
	int work_index = range.mobility_offset + local_link_index;
	GpuArticulationLink link = g_aba_links[link_index];
	int parent_work_index = range.mobility_offset + link.parent_link_index - articulation.link_offset;
	GpuArticulationSpatialVector link_delta = AbaTransformMotion(
		AbaInvertTransform(g_aba_scratch[link_index].child_to_parent),
		g_impulse_work[parent_work_index]);
	float4 rhs_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 rhs_high = float4(0.0f, 0.0f, 0.0f, 0.0f);

	// Recover q-delta = D^-1(u - U-transpose*parent-delta) from the mobility factors.
	for (int row = 0; row != link.dof_count; ++row)
	{
		float value =
			g_aba_accelerations[link.velocity_offset + row] -
			AbaSpatialDot(link_delta, g_aba_dof_scratch[link.dof_offset + row].u_column);
		AbaSetJointVectorComponent(rhs_low, rhs_high, row, value);
	}

	float4 joint_delta_low;
	float4 joint_delta_high;
	AbaMultiplyJointMatrix(
		AbaLoadInverseJointMatrix(link),
		rhs_low,
		rhs_high,
		link.dof_count,
		joint_delta_low,
		joint_delta_high);
	for (int row = 0; row != link.dof_count; ++row)
	{
		float joint_delta = AbaJointVectorComponent(joint_delta_low, joint_delta_high, row);
		g_aba_accelerations[link.velocity_offset + row] = joint_delta;
		link_delta = AbaAddSpatial(
			link_delta,
			AbaScaleSpatial(g_aba_dof_scratch[link.dof_offset + row].motion_subspace, joint_delta));
	}
	g_impulse_work[work_index] = link_delta;
}

// Apply one complete simultaneous impulse response without rebuilding the fixed-configuration factors.
numthreads(CSArticulationApplyImpulses, ArticulationThreadCount, 1, 1)
void CSArticulationApplyImpulses(int3 DTID(dtid))
{
	if (dtid.x >= g_impulse.participating_articulation_count)
		return;

	GpuArticulationMobilityRange range = g_mobility_ranges[dtid.x];
	if (
		range.articulation_index < 0 || range.articulation_index >= g_impulse.articulation_count ||
		range.mobility_offset < 0 || range.link_count < 1 ||
		range.mobility_offset + range.link_count > g_impulse.mobility_count)
		return;

	GpuArticulation articulation = g_aba_articulations[range.articulation_index];
	if (
		articulation.link_count != range.link_count ||
		articulation.link_offset < 0 ||
		articulation.link_offset + articulation.link_count > g_impulse.link_count ||
		g_aba_scratch[articulation.link_offset].solve_valid == 0)
		return;

	// Reject the complete tree before mutation if any gathered impulse is non-finite.
	for (int local_link_index = 0; local_link_index != articulation.link_count; ++local_link_index)
	{
		int work_index = range.mobility_offset + local_link_index;
		int link_index = articulation.link_offset + local_link_index;
		if (!ImpulseSpatialFinite(g_link_impulses[work_index]) || g_aba_scratch[link_index].solve_valid == 0)
		{
			ImpulseInvalidateTree(articulation.link_offset);
			return;
		}
	}

	// Convert the compact participating-link impulses into ABA bias sign convention.
	for (int local_link_index = 0; local_link_index != articulation.link_count; ++local_link_index)
	{
		int work_index = range.mobility_offset + local_link_index;
		g_impulse_work[work_index] = AbaScaleSpatial(g_link_impulses[work_index], -1.0f);
	}

	// Eliminate child joint response in reverse topological order without changing the retained articulated inertias.
	for (int local_link_index = articulation.link_count; local_link_index-- != 1;)
		ImpulseReduceChild(range, articulation, local_link_index);

	// The root self mobility is the inverse articulated inertia for floating roots and exactly zero for fixed roots.
	GpuArticulationSpatialVector root_delta = MobilityApply(
		g_link_mobilities[range.mobility_offset],
		AbaScaleSpatial(g_impulse_work[range.mobility_offset], -1.0f));
	g_impulse_work[range.mobility_offset] = root_delta;

	// Recover every child response in parent-before-child order.
	for (int local_link_index = 1; local_link_index != articulation.link_count; ++local_link_index)
		ImpulseRecoverChild(range, articulation, local_link_index);

	// Validate every prospective complete-tree update before committing any persistent velocity.
	for (int local_link_index = 0; local_link_index != articulation.link_count; ++local_link_index)
	{
		int link_index = articulation.link_offset + local_link_index;
		GpuArticulationSpatialVector link_delta = g_impulse_work[range.mobility_offset + local_link_index];
		if (!ImpulseSpatialFinite(link_delta) || !ImpulseSpatialFinite(AbaAddSpatial(g_aba_scratch[link_index].link_velocity, link_delta)))
		{
			ImpulseInvalidateTree(articulation.link_offset);
			return;
		}

		GpuArticulationLink link = g_aba_links[link_index];
		for (int row = 0; row != link.dof_count; ++row)
		{
			float joint_delta = g_aba_accelerations[link.velocity_offset + row];
			if (!isfinite(joint_delta) || !isfinite(g_aba_velocities[link.velocity_offset + row] + joint_delta))
			{
				ImpulseInvalidateTree(articulation.link_offset);
				return;
			}
		}
	}
	if (articulation.root_type == GpuArticulationRootType_Floating)
	{
		for (int component = 0; component != 6; ++component)
		{
			if (!isfinite(g_aba_velocities[articulation.velocity_offset + component] + AbaSpatialComponent(root_delta, component)))
			{
				ImpulseInvalidateTree(articulation.link_offset);
				return;
			}
		}
	}

	// Commit generalized and cached link velocities together so later coupled residuals observe one complete-tree update.
	if (articulation.root_type == GpuArticulationRootType_Floating)
	{
		for (int component = 0; component != 6; ++component)
			g_aba_velocities[articulation.velocity_offset + component] += AbaSpatialComponent(root_delta, component);
	}
	for (int local_link_index = 1; local_link_index != articulation.link_count; ++local_link_index)
	{
		GpuArticulationLink link = g_aba_links[articulation.link_offset + local_link_index];
		for (int row = 0; row != link.dof_count; ++row)
			g_aba_velocities[link.velocity_offset + row] += g_aba_accelerations[link.velocity_offset + row];
	}
	for (int local_link_index = 0; local_link_index != articulation.link_count; ++local_link_index)
	{
		int link_index = articulation.link_offset + local_link_index;
		GpuArticulationAbaScratch scratch = g_aba_scratch[link_index];
		scratch.link_velocity = AbaAddSpatial(scratch.link_velocity, g_impulse_work[range.mobility_offset + local_link_index]);
		g_aba_scratch[link_index] = scratch;
	}
}

#ifdef __cplusplus
}
#endif
