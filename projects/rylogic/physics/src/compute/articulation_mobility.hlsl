//*********************************************
// Physics Engine — Articulation Self-Link Mobility
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// One serial lane per participating articulation rebuilds the final-configuration ABA factors,
// then propagates exact self-link impulse mobility from the root to every link in linear tree order.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Compact participating ranges and packed-buffer bounds for one optional mobility dispatch.
struct cbArticulationMobility
{
	int participating_articulation_count;
	int articulation_count;
	int link_count;
	int mobility_count;
};

ConstantBuffer<cbArticulationMobility> resource(g_mobility, b0);
StructuredBuffer<GpuArticulationMobilityRange> resource(g_mobility_ranges, t0);
StructuredBuffer<GpuArticulation> resource(g_aba_articulations, t1);
StructuredBuffer<GpuArticulationLink> resource(g_aba_links, t2);
StructuredBuffer<GpuArticulationDof> resource(g_aba_dofs, t3);
StructuredBuffer<float> resource(g_aba_positions, t4);
StructuredBuffer<float> resource(g_aba_velocities, t5);
StructuredBuffer<float> resource(g_aba_forces, t6);
StructuredBuffer<GpuFrameForce> resource(g_aba_external_forces, t7);
StructuredBuffer<uint> resource(g_aba_children, t8);
RWStructuredBuffer<float> resource(g_aba_accelerations, u0);
RWStructuredBuffer<GpuArticulationAbaScratch> resource(g_aba_scratch, u1);
RWStructuredBuffer<GpuArticulationAbaDofScratch> resource(g_aba_dof_scratch, u2);
RWStructuredBuffer<float> resource(g_aba_inverse_joint_inertia, u3);
RWStructuredBuffer<GpuArticulationSpatialMobility> resource(g_link_mobilities, u4);

#ifdef __cplusplus
}
#endif

// Reuse the canonical transform, factorization, and bounded joint-matrix operations.
#define PR_ARTICULATION_ABA_CUSTOM_RESOURCES
#define PR_ARTICULATION_ABA_NO_ENTRYPOINTS
#define PR_ARTICULATION_ABA_CPP_NAMESPACE articulation_mobility_aba_detail
#include "physics/src/compute/articulation_force_aba.hlsl"
#undef PR_ARTICULATION_ABA_CPP_NAMESPACE
#undef PR_ARTICULATION_ABA_NO_ENTRYPOINTS
#undef PR_ARTICULATION_ABA_CUSTOM_RESOURCES

#ifdef __cplusplus
namespace pr::physics {
using namespace articulation_mobility_aba_detail;
#endif

#define PR_ARTICULATION_MOBILITY_OPS_CPP_NAMESPACE articulation_mobility_ops_detail
#include "physics/src/compute/articulation_mobility_ops.hlsli"
#undef PR_ARTICULATION_MOBILITY_OPS_CPP_NAMESPACE

// Return D^-1 times one joint-space vector.
void MobilityJointSolve(GpuArticulationLink link, float4 rhs_low, float4 rhs_high, out_(float4) result_low, out_(float4) result_high)
{
	AbaMultiplyJointMatrix(
		AbaLoadInverseJointMatrix(link),
		rhs_low,
		rhs_high,
		link.dof_count,
		result_low,
		result_high);
}

// Return S*D^-1*S-transpose times one link-frame wrench.
GpuArticulationSpatialVector MobilityJointResponse(GpuArticulationLink link, GpuArticulationSpatialVector force)
{
	float4 rhs_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 rhs_high = float4(0.0f, 0.0f, 0.0f, 0.0f);
	for (int row = 0; row != link.dof_count; ++row)
	{
		AbaSetJointVectorComponent(
			rhs_low,
			rhs_high,
			row,
			AbaSpatialDot(g_aba_dof_scratch[link.dof_offset + row].motion_subspace, force));
	}

	float4 coefficients_low;
	float4 coefficients_high;
	MobilityJointSolve(link, rhs_low, rhs_high, coefficients_low, coefficients_high);
	GpuArticulationSpatialVector response = AbaZeroSpatialVector();
	for (int column = 0; column != link.dof_count; ++column)
	{
		response = AbaAddSpatial(
			response,
			AbaScaleSpatial(
				g_aba_dof_scratch[link.dof_offset + column].motion_subspace,
				AbaJointVectorComponent(coefficients_low, coefficients_high, column)));
	}
	return response;
}

// Project one child-frame wrench through an eliminated joint before transferring it to the parent.
GpuArticulationSpatialVector MobilityProjectForce(GpuArticulationLink link, GpuArticulationSpatialVector force)
{
	float4 rhs_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 rhs_high = float4(0.0f, 0.0f, 0.0f, 0.0f);
	for (int row = 0; row != link.dof_count; ++row)
	{
		AbaSetJointVectorComponent(
			rhs_low,
			rhs_high,
			row,
			AbaSpatialDot(g_aba_dof_scratch[link.dof_offset + row].motion_subspace, force));
	}

	float4 coefficients_low;
	float4 coefficients_high;
	MobilityJointSolve(link, rhs_low, rhs_high, coefficients_low, coefficients_high);
	GpuArticulationSpatialVector projected = force;
	for (int column = 0; column != link.dof_count; ++column)
	{
		projected = AbaSubtractSpatial(
			projected,
			AbaScaleSpatial(
				g_aba_dof_scratch[link.dof_offset + column].u_column,
				AbaJointVectorComponent(coefficients_low, coefficients_high, column)));
	}
	return projected;
}

// Project parent-derived child motion into the joint's zero-generalized-force subspace.
GpuArticulationSpatialVector MobilityProjectMotion(GpuArticulationLink link, GpuArticulationSpatialVector motion)
{
	float4 rhs_low = float4(0.0f, 0.0f, 0.0f, 0.0f);
	float4 rhs_high = float4(0.0f, 0.0f, 0.0f, 0.0f);
	for (int row = 0; row != link.dof_count; ++row)
	{
		AbaSetJointVectorComponent(
			rhs_low,
			rhs_high,
			row,
			AbaSpatialDot(motion, g_aba_dof_scratch[link.dof_offset + row].u_column));
	}

	float4 coefficients_low;
	float4 coefficients_high;
	MobilityJointSolve(link, rhs_low, rhs_high, coefficients_low, coefficients_high);
	GpuArticulationSpatialVector projected = motion;
	for (int column = 0; column != link.dof_count; ++column)
	{
		projected = AbaSubtractSpatial(
			projected,
			AbaScaleSpatial(
				g_aba_dof_scratch[link.dof_offset + column].motion_subspace,
				AbaJointVectorComponent(coefficients_low, coefficients_high, column)));
	}
	return projected;
}

// Pack one root's fixed or floating self-mobility and retain explicit singular status.
GpuArticulationSpatialMobility MobilityRoot(GpuArticulation articulation)
{
	GpuArticulationSpatialMobility mobility = MobilityZero();
	int root_index = articulation.link_offset;
	GpuArticulationAbaScratch root = g_aba_scratch[root_index];
	switch (articulation.root_type)
	{
		case GpuArticulationRootType_Fixed:
		{
			break;
		}
		case GpuArticulationRootType_Floating:
		{
			GpuArticulationJointMatrix root_inertia = AbaZeroJointMatrix();
			for (int row = 0; row != 6; ++row)
			for (int column = 0; column != 6; ++column)
				AbaSetJointMatrixComponent(root_inertia, row, column, AbaSpatialComponent(root.articulated_inertia.columns[column], row));

			GpuArticulationJointMatrix root_inverse;
			bool root_valid;
			AbaInvertJointMatrix(root_inertia, 6, root_inverse, root_valid);
			root.solve_valid = root.solve_valid != 0 && root_valid ? 1 : 0;
			for (int row = 0; row != 6; ++row)
			for (int column = row; column != 6; ++column)
				MobilitySetComponent(mobility, row, column, AbaJointMatrixComponent(root_inverse, row, column));
			break;
		}
		default:
		{
			root.solve_valid = 0;
			break;
		}
	}
	g_aba_scratch[root_index] = root;
	return mobility;
}

// Compute exact self-link mobilities for one compact participating tree.
void MobilityPrepareArticulation(GpuArticulationMobilityRange range)
{
	GpuArticulation articulation = g_aba_articulations[range.articulation_index];

	// Rebuild factors at committed end-of-substep coordinates rather than reusing midpoint-configuration factors.
	for (int local_link_index = 0; local_link_index != articulation.link_count; ++local_link_index)
		AbaPrepareLink(articulation.link_offset + local_link_index);
	for (int reverse_link_index = articulation.link_count; reverse_link_index-- != 0;)
		AbaInwardLink(articulation.link_offset + reverse_link_index);

	g_link_mobilities[range.mobility_offset] = MobilityRoot(articulation);
	for (int local_link_index = 1; local_link_index != articulation.link_count; ++local_link_index)
	{
		int link_index = articulation.link_offset + local_link_index;
		GpuArticulationLink link = g_aba_links[link_index];
		GpuArticulationAbaScratch scratch = g_aba_scratch[link_index];
		GpuArticulationSpatialMobility parent_mobility =
			g_link_mobilities[range.mobility_offset + link.parent_link_index - articulation.link_offset];
		GpuArticulationSpatialMobility mobility = MobilityZero();
		GpuConstraintFrame parent_to_child = AbaInvertTransform(scratch.child_to_parent);

		for (int spatial_column = 0; spatial_column != 6; ++spatial_column)
		{
			GpuArticulationSpatialVector force = AbaSpatialBasis(spatial_column);
			GpuArticulationSpatialVector parent_force =
				AbaTransformForce(scratch.child_to_parent, MobilityProjectForce(link, force));
			GpuArticulationSpatialVector parent_motion = MobilityApply(parent_mobility, parent_force);
			GpuArticulationSpatialVector propagated_motion =
				MobilityProjectMotion(link, AbaTransformMotion(parent_to_child, parent_motion));
			GpuArticulationSpatialVector response =
				AbaAddSpatial(propagated_motion, MobilityJointResponse(link, force));

			for (int spatial_row = 0; spatial_row != spatial_column + 1; ++spatial_row)
				MobilitySetComponent(mobility, spatial_row, spatial_column, AbaSpatialComponent(response, spatial_row));
		}
		g_link_mobilities[range.mobility_offset + local_link_index] = mobility;
	}
}

// Rebuild and propagate one participating articulation without touching unrelated trees.
numthreads(CSArticulationPrepareMobility, ArticulationThreadCount, 1, 1)
void CSArticulationPrepareMobility(int3 DTID(dtid))
{
	if (dtid.x >= g_mobility.participating_articulation_count)
		return;

	GpuArticulationMobilityRange range = g_mobility_ranges[dtid.x];
	if (
		range.articulation_index < 0 || range.articulation_index >= g_mobility.articulation_count ||
		range.mobility_offset < 0 || range.link_count < 1 ||
		range.mobility_offset + range.link_count > g_mobility.mobility_count)
		return;

	GpuArticulation articulation = g_aba_articulations[range.articulation_index];
	if (articulation.link_count != range.link_count)
		return;

	MobilityPrepareArticulation(range);
}

#ifdef __cplusplus
}
#endif
