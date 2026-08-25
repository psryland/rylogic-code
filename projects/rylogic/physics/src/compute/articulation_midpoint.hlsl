//*********************************************
// Physics Engine — Fused Pure-Tree Articulation Midpoint Integration
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// One invocation owns one contiguous articulation and streams the canonical force-ABA phases through
// global dimension-scaled scratch. One dispatch advances one internal substep.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Per-substep integration bounds shared by every articulation lane.
struct cbArticulationMidpoint
{
	float dt;
	int articulation_count;
	int link_count;
	int velocity_count;
};

ConstantBuffer<cbArticulationMidpoint> resource(g_midpoint, b0);
StructuredBuffer<GpuArticulationLink> resource(g_aba_links, t0);
StructuredBuffer<GpuArticulationDof> resource(g_aba_dofs, t1);
StructuredBuffer<float> resource(g_aba_forces, t2);
StructuredBuffer<GpuFrameForce> resource(g_aba_external_forces, t3);
StructuredBuffer<uint> resource(g_aba_children, t4);
RWStructuredBuffer<GpuArticulation> resource(g_aba_articulations, u0);
RWStructuredBuffer<float> resource(g_aba_positions, u1);
RWStructuredBuffer<float> resource(g_aba_velocities, u2);
RWStructuredBuffer<float> resource(g_aba_accelerations, u3);
RWStructuredBuffer<GpuArticulationAbaScratch> resource(g_aba_scratch, u4);
RWStructuredBuffer<GpuArticulationAbaDofScratch> resource(g_aba_dof_scratch, u5);
RWStructuredBuffer<float> resource(g_aba_inverse_joint_inertia, u6);
RWStructuredBuffer<float> resource(g_midpoint_position_start, u7);
RWStructuredBuffer<float> resource(g_midpoint_velocity_start, u8);
RWStructuredBuffer<float> resource(g_midpoint_velocity, u9);
RWStructuredBuffer<GpuArticulationIntegrationState> resource(g_midpoint_state, u10);

#ifdef __cplusplus
}
#endif

// Reuse one canonical implementation of every resource-dependent ABA link operation.
#define PR_ARTICULATION_ABA_CUSTOM_RESOURCES
#define PR_ARTICULATION_ABA_NO_ENTRYPOINTS
#define PR_ARTICULATION_ABA_CPP_NAMESPACE articulation_midpoint_aba_detail
#include "physics/src/compute/articulation_force_aba.hlsl"
#undef PR_ARTICULATION_ABA_CPP_NAMESPACE
#undef PR_ARTICULATION_ABA_NO_ENTRYPOINTS
#undef PR_ARTICULATION_ABA_CUSTOM_RESOURCES

#ifdef __cplusplus
namespace pr::physics {
using namespace articulation_midpoint_aba_detail;
#endif

static const int ArticulationMidpointIterationLimit = 12;
static const float ArticulationMidpointRelativeTolerance = 2.0e-6f;
static const float ArticulationMidpointMaximumAngularDisplacement = 1.0e4f;

// Return whether one scalar can participate in a committed integration result.
bool MidpointFinite(float value)
{
	return isfinite(value);
}

// Return whether every meaningful component of one spatial vector is finite.
bool MidpointFiniteSpatial(GpuArticulationSpatialVector value)
{
	return
		MidpointFinite(value.ang.x) && MidpointFinite(value.ang.y) && MidpointFinite(value.ang.z) &&
		MidpointFinite(value.lin.x) && MidpointFinite(value.lin.y) && MidpointFinite(value.lin.z);
}

// Return whether one root frame contains a finite unit-or-renormalisable quaternion and finite position.
bool MidpointFiniteFrame(GpuConstraintFrame frame)
{
	return
		MidpointFinite(frame.rotation.x) && MidpointFinite(frame.rotation.y) &&
		MidpointFinite(frame.rotation.z) && MidpointFinite(frame.rotation.w) &&
		MidpointFinite(frame.position.x) && MidpointFinite(frame.position.y) && MidpointFinite(frame.position.z);
}

// Integrate a body-frame floating-root twist with midpoint translation and exponential-map rotation.
GpuConstraintFrame MidpointIntegrateRoot(GpuConstraintFrame root_to_world, int velocity_offset, float dt, out_(bool) valid)
{
	float3 angular_velocity = float3(
		g_midpoint_velocity[velocity_offset + 0],
		g_midpoint_velocity[velocity_offset + 1],
		g_midpoint_velocity[velocity_offset + 2]);
	float3 linear_velocity = float3(
		g_midpoint_velocity[velocity_offset + 3],
		g_midpoint_velocity[velocity_offset + 4],
		g_midpoint_velocity[velocity_offset + 5]);
	float3 angular_displacement = dt * angular_velocity;

	// Match the CPU guard before trigonometric evaluation can turn an extreme displacement into plausible finite output.
	valid =
		MidpointFinite(angular_displacement.x) && MidpointFinite(angular_displacement.y) && MidpointFinite(angular_displacement.z) &&
		MidpointFinite(linear_velocity.x) && MidpointFinite(linear_velocity.y) && MidpointFinite(linear_velocity.z) &&
		max(abs(angular_displacement.x), max(abs(angular_displacement.y), abs(angular_displacement.z))) <= ArticulationMidpointMaximumAngularDisplacement;

	GpuConstraintFrame integrated = root_to_world;
	if (!valid)
		return integrated;

	float4 half_rotation = quat_exp(0.25f * angular_displacement);
	float4 full_rotation = quat_exp(0.5f * angular_displacement);
	float4 midpoint_rotation = quat_mul(root_to_world.rotation, half_rotation);
	integrated.position = float4(root_to_world.position.xyz + quat_rotate(midpoint_rotation, dt * linear_velocity), 1.0f);
	integrated.rotation = normalize(quat_mul(root_to_world.rotation, full_rotation));
	valid = MidpointFiniteFrame(integrated);
	return integrated;
}

// Execute one complete force ABA in contiguous topological order using the shared link operations.
void MidpointSolveForceAba(int articulation_index)
{
	GpuArticulation articulation = g_aba_articulations[articulation_index];

	// Parent-before-child preparation makes every link velocity dependency available in the same invocation.
	for (int link_offset = 0; link_offset != articulation.link_count; ++link_offset)
		AbaPrepareLink(articulation.link_offset + link_offset);

	// Reverse topological parent reductions observe completed descendants and preserve packed child order.
	for (int reverse_offset = articulation.link_count; reverse_offset-- != 0;)
		AbaInwardLink(articulation.link_offset + reverse_offset);

	AbaRootDynamics(articulation_index);

	// Parent-before-child recovery overwrites reduced forces with qdd and link bias with solved acceleration.
	for (int link_offset = 1; link_offset != articulation.link_count; ++link_offset)
		AbaOutwardLink(articulation.link_offset + link_offset);
}

// Classify the canonical ABA output without allowing regularized diagnostic values to look successful.
int MidpointAbaStatus(GpuArticulation articulation)
{
	for (int link_offset = 0; link_offset != articulation.link_count; ++link_offset)
	{
		GpuArticulationAbaScratch scratch = g_aba_scratch[articulation.link_offset + link_offset];
		if (scratch.solve_valid == 0)
			return GpuArticulationIntegrationStatus_Singular;
		if (!MidpointFiniteSpatial(scratch.articulated_bias_or_acceleration))
			return GpuArticulationIntegrationStatus_NonFinite;
	}

	for (int velocity_offset = 0; velocity_offset != articulation.velocity_count; ++velocity_offset)
	{
		if (!MidpointFinite(g_aba_accelerations[articulation.velocity_offset + velocity_offset]))
			return GpuArticulationIntegrationStatus_NonFinite;
	}
	return GpuArticulationIntegrationStatus_Success;
}

// Restore the transaction's accepted generalized state and floating-root pose after any failed trial.
void MidpointRestoreAccepted(int articulation_index, GpuArticulation articulation, GpuArticulationIntegrationState state)
{
	for (int offset = 0; offset != articulation.position_count; ++offset)
		g_aba_positions[articulation.position_offset + offset] = g_midpoint_position_start[articulation.position_offset + offset];
	for (int offset = 0; offset != articulation.velocity_count; ++offset)
		g_aba_velocities[articulation.velocity_offset + offset] = g_midpoint_velocity_start[articulation.velocity_offset + offset];

	articulation.root_to_world = state.root_to_world_start;
	g_aba_articulations[articulation_index] = articulation;
}

// Record one failed transaction after restoring its accepted primary state.
void MidpointFail(int articulation_index, GpuArticulation articulation, inout_(GpuArticulationIntegrationState) state, int status)
{
	MidpointRestoreAccepted(articulation_index, articulation, state);
	state.status = status;
	g_midpoint_state[articulation_index] = state;
}

// Advance one articulation by one bounded implicit-midpoint substep.
numthreads(CSArticulationMidpoint, ArticulationThreadCount, 1, 1)
void CSArticulationMidpoint(int3 DTID(dtid))
{
	int articulation_index = dtid.x;
	if (articulation_index >= g_midpoint.articulation_count)
		return;

	GpuArticulationIntegrationState state = g_midpoint_state[articulation_index];
	if (state.status != GpuArticulationIntegrationStatus_Success)
		return;

	state.iteration_count = 0;
	state.residual = 0.0f;
	if (g_midpoint.dt == 0.0f)
	{
		g_midpoint_state[articulation_index] = state;
		return;
	}

	GpuArticulation articulation = g_aba_articulations[articulation_index];
	state.root_to_world_start = articulation.root_to_world;

	// Snapshot the accepted generalized state once so all trial mutations can be rolled back transactionally.
	for (int offset = 0; offset != articulation.position_count; ++offset)
		g_midpoint_position_start[articulation.position_offset + offset] = g_aba_positions[articulation.position_offset + offset];
	for (int offset = 0; offset != articulation.velocity_count; ++offset)
		g_midpoint_velocity_start[articulation.velocity_offset + offset] = g_aba_velocities[articulation.velocity_offset + offset];
	if (!MidpointFiniteFrame(state.root_to_world_start))
	{
		MidpointFail(articulation_index, articulation, state, GpuArticulationIntegrationStatus_NonFinite);
		return;
	}

	// Seed midpoint velocity from force ABA evaluated at the accepted state.
	MidpointSolveForceAba(articulation_index);
	int status = MidpointAbaStatus(articulation);
	if (status != GpuArticulationIntegrationStatus_Success)
	{
		MidpointFail(articulation_index, articulation, state, status);
		return;
	}
	for (int offset = 0; offset != articulation.velocity_count; ++offset)
		g_midpoint_velocity[articulation.velocity_offset + offset] =
			g_midpoint_velocity_start[articulation.velocity_offset + offset] +
			0.5f * g_midpoint.dt * g_aba_accelerations[articulation.velocity_offset + offset];

	bool converged = false;
	int root_velocity_count = 0;
	switch (articulation.root_type)
	{
		case GpuArticulationRootType_Fixed:
		{
			break;
		}
		case GpuArticulationRootType_Floating:
		{
			root_velocity_count = 6;
			break;
		}
		default:
		{
			MidpointFail(articulation_index, articulation, state, GpuArticulationIntegrationStatus_NonFinite);
			return;
		}
	}
	for (int iteration = 0; iteration != ArticulationMidpointIterationLimit; ++iteration)
	{
		// Materialize the current midpoint candidate in the shared state consumed by canonical ABA.
		for (int offset = 0; offset != articulation.velocity_count; ++offset)
		{
			float candidate_velocity = g_midpoint_velocity[articulation.velocity_offset + offset];
			if (!MidpointFinite(candidate_velocity))
			{
				MidpointFail(articulation_index, articulation, state, GpuArticulationIntegrationStatus_NonFinite);
				return;
			}
			g_aba_velocities[articulation.velocity_offset + offset] = candidate_velocity;
		}
		for (int offset = 0; offset != articulation.position_count; ++offset)
		{
			g_aba_positions[articulation.position_offset + offset] =
				g_midpoint_position_start[articulation.position_offset + offset] +
				0.5f * g_midpoint.dt * g_midpoint_velocity[articulation.velocity_offset + root_velocity_count + offset];
		}
		switch (articulation.root_type)
		{
			case GpuArticulationRootType_Fixed:
			{
				break;
			}
			case GpuArticulationRootType_Floating:
			{
				bool root_valid;
				articulation.root_to_world = MidpointIntegrateRoot(state.root_to_world_start, articulation.velocity_offset, 0.5f * g_midpoint.dt, root_valid);
				if (!root_valid)
				{
					MidpointFail(articulation_index, articulation, state, GpuArticulationIntegrationStatus_NonFinite);
					return;
				}
				g_aba_articulations[articulation_index] = articulation;
				break;
			}
			default:
			{
				MidpointFail(articulation_index, articulation, state, GpuArticulationIntegrationStatus_NonFinite);
				return;
			}
		}

		// Fixed-point residual uses the newly seeded midpoint velocity and the CPU relative tolerance.
		MidpointSolveForceAba(articulation_index);
		status = MidpointAbaStatus(articulation);
		if (status != GpuArticulationIntegrationStatus_Success)
		{
			MidpointFail(articulation_index, articulation, state, status);
			return;
		}

		float residual = 0.0f;
		float maximum_velocity = 0.0f;
		for (int offset = 0; offset != articulation.velocity_count; ++offset)
		{
			int velocity_index = articulation.velocity_offset + offset;
			float next_velocity =
				g_midpoint_velocity_start[velocity_index] +
				0.5f * g_midpoint.dt * g_aba_accelerations[velocity_index];
			if (!MidpointFinite(next_velocity))
			{
				MidpointFail(articulation_index, articulation, state, GpuArticulationIntegrationStatus_NonFinite);
				return;
			}
			residual = max(residual, abs(next_velocity - g_midpoint_velocity[velocity_index]));
			maximum_velocity = max(maximum_velocity, abs(next_velocity));
			g_midpoint_velocity[velocity_index] = next_velocity;
		}

		state.iteration_count = iteration + 1;
		state.residual = residual;
		converged = residual <= ArticulationMidpointRelativeTolerance * (1.0f + maximum_velocity);
		if (converged)
			break;
	}

	if (!converged)
	{
		MidpointFail(articulation_index, articulation, state, GpuArticulationIntegrationStatus_NonConverged);
		return;
	}

	// Commit only after finite convergence; midpoint ABA outputs remain available as the accepted qdd/link acceleration.
	for (int offset = 0; offset != articulation.position_count; ++offset)
	{
		g_aba_positions[articulation.position_offset + offset] =
			g_midpoint_position_start[articulation.position_offset + offset] +
			g_midpoint.dt * g_midpoint_velocity[articulation.velocity_offset + root_velocity_count + offset];
	}
	for (int offset = 0; offset != articulation.velocity_count; ++offset)
	{
		g_aba_velocities[articulation.velocity_offset + offset] =
			g_midpoint_velocity_start[articulation.velocity_offset + offset] +
			g_midpoint.dt * g_aba_accelerations[articulation.velocity_offset + offset];
	}
	switch (articulation.root_type)
	{
		case GpuArticulationRootType_Fixed:
		{
			break;
		}
		case GpuArticulationRootType_Floating:
		{
			bool root_valid;
			articulation.root_to_world = MidpointIntegrateRoot(state.root_to_world_start, articulation.velocity_offset, g_midpoint.dt, root_valid);
			if (!root_valid)
			{
				MidpointFail(articulation_index, articulation, state, GpuArticulationIntegrationStatus_NonFinite);
				return;
			}
			g_aba_articulations[articulation_index] = articulation;
			break;
		}
		default:
		{
			MidpointFail(articulation_index, articulation, state, GpuArticulationIntegrationStatus_NonFinite);
			return;
		}
	}
	g_midpoint_state[articulation_index] = state;
}

#ifdef __cplusplus
}
#endif
