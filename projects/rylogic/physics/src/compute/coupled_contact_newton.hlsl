//*********************************************
// Physics Engine - Coupled ordinary-contact Newton update
// Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/hlsl/core.hlsli"
#include "physics/src/compute/physics_types.hlsli"
#include "physics/src/compute/contact_coulomb_linearisation.hlsli"

ConstantBuffer<GpuCoupledContactParams> resource(g_coupled_contact, b0);
StructuredBuffer<GpuCollisionCounters> resource(g_coupled_contact_counters, t0);
StructuredBuffer<GpuArticulationLink> resource(g_aba_links, t1);
StructuredBuffer<GpuConstraintFrame> resource(g_coupled_contact_link_to_world, t2);
StructuredBuffer<GpuArticulationSpatialMobility> resource(g_link_mobilities, t3);
StructuredBuffer<GpuArticulationMobilityRange> resource(g_mobility_ranges, t4);
StructuredBuffer<GpuArticulation> resource(g_aba_articulations, t5);
StructuredBuffer<GpuCoupledContactBlock> resource(g_coupled_contact_blocks, t6);
StructuredBuffer<GpuResolveContact> resource(g_coupled_contact_contacts, t7);
StructuredBuffer<GpuRigidBody> resource(g_coupled_contact_bodies, t8);
RWStructuredBuffer<GpuCoupledContactScratch> resource(g_coupled_contact_scratch, u0);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_contact_contributions, u1);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_contact_target_impulses, u2);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_link_impulses, u3);
RWStructuredBuffer<GpuCoupledContactState> resource(g_coupled_contact_state, u4);
RWStructuredBuffer<uint2> resource(g_contact_nodes, u5);
RWStructuredBuffer<uint2> resource(g_contact_order, u6);
RWStructuredBuffer<GpuCoupledContactKrylov> resource(g_contact_krylov, u7);
RWStructuredBuffer<GpuArticulationAbaScratch> resource(g_aba_scratch, u8);
RWStructuredBuffer<GpuArticulationAbaDofScratch> resource(g_aba_dof_scratch, u9);
RWStructuredBuffer<float> resource(g_aba_inverse_joint_inertia, u10);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_impulse_work, u11);
RWStructuredBuffer<float> resource(g_impulse_velocity_deltas, u12);

// These declarations belong to unused shared helpers, not this shader's live resource interface.
StructuredBuffer<GpuArticulationDof> resource(g_aba_dofs, t20);
StructuredBuffer<float> resource(g_aba_positions, t21);
StructuredBuffer<float> resource(g_aba_forces, t22);
StructuredBuffer<GpuFrameForce> resource(g_aba_external_forces, t23);
StructuredBuffer<uint> resource(g_aba_children, t24);
RWStructuredBuffer<float> resource(g_aba_velocities, u20);
RWStructuredBuffer<float> resource(g_aba_accelerations, u21);
RWStructuredBuffer<uint> resource(g_coupled_contact_endpoint_order, u22);
RWStructuredBuffer<GpuConstraintPseudoVelocity> resource(g_coupled_contact_rigid_pseudo, u23);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_contact_link_pseudo, u24);

#define g_coupled_contact_links g_aba_links
#define g_coupled_contact_mobilities g_link_mobilities
#define g_coupled_contact_aba_scratch g_aba_scratch
#define g_coupled_contact_articulations g_aba_articulations
#define PR_COUPLED_CONTACT_CUSTOM_RESOURCES
#define PR_COUPLED_CONTACT_NO_ENTRYPOINTS
#include "physics/src/compute/coupled_contact.hlsl"
#undef PR_COUPLED_CONTACT_NO_ENTRYPOINTS
#undef PR_COUPLED_CONTACT_CUSTOM_RESOURCES

#define PR_ARTICULATION_IMPULSE_ABA_CUSTOM_RESOURCES
#define PR_ARTICULATION_IMPULSE_ABA_NO_ENTRYPOINTS
#include "physics/src/compute/articulation_impulse_aba.hlsl"
#undef PR_ARTICULATION_IMPULSE_ABA_NO_ENTRYPOINTS
#undef PR_ARTICULATION_IMPULSE_ABA_CUSTOM_RESOURCES

// Find and compress a participant's component. Component construction has one writer.
uint ContactComponent(uint node)
{
	// Parent indices decrease, so following this forest cannot cycle.
	uint root = node;
	while (g_contact_nodes[root].x != root)
		root = g_contact_nodes[root].x;

	// Retain direct representatives for subsequent row grouping.
	while (node != root)
	{
		uint parent = g_contact_nodes[node].x;
		g_contact_nodes[node].x = root;
		node = parent;
	}
	return root;
}

// Order rows by component and immutable contact geometry, independently of append order.
bool ContactRowPrecedes(uint2 lhs, uint2 rhs)
{
	// Inactive entries sort behind every participating component without reading invalid contact indices.
	if (lhs.x != rhs.x)
		return lhs.x < rhs.x;

	if (lhs.x == CoupledContactInvalidKey)
		return false;

	return CoupledContactEndpointPrecedes(2u * lhs.y, 2u * rhs.y);
}

// Restore a row max-heap whose child subtrees are already heaps.
void ContactSiftRow(int root, int count)
{
	// This heap is over component/row pairs rather than the separate target/endpoint ordering.
	uint2 value = g_contact_order[root];
	while (root < count / 2)
	{
		int child = 2 * root + 1;
		if (child + 1 < count)
		{
			// HLSL must not speculatively read beyond the heap.
			if (ContactRowPrecedes(g_contact_order[child], g_contact_order[child + 1]))
				++child;
		}

		if (!ContactRowPrecedes(value, g_contact_order[child]))
			break;

		g_contact_order[root] = g_contact_order[child];
		root = child;
	}
	g_contact_order[root] = value;
}

// Build uncapped connected participant groups once per prepared contact set, without a CPU contact-count readback.
numthreads(CSBuildCoupledContactComponents, 1, 1, 1)
void CSBuildCoupledContactComponents(int3 DTID(dtid))
{
	// One topology writer avoids atomics and partially joined components; later Newton work is independent between groups.
	int node_count = g_coupled_contact.rigid_body_count + g_coupled_contact.articulation_count;
	int count = CoupledContactCount();
	for (int node = 0; node != node_count; ++node)
		g_contact_nodes[node] = uint2(node, 0);

	// Both links of a self contact name one articulation participant, never two independent bodies.
	for (int row = 0; row != count; ++row)
	{
		if (g_coupled_contact_scratch[row].candidate_impulse.w == 0.0f)
			continue;

		GpuCoupledContactBlock block = g_coupled_contact_blocks[row];
		if (block.participant_a >= 0 && block.participant_b >= 0)
		{
			uint a = ContactComponent(block.participant_a);
			uint b = ContactComponent(block.participant_b);
			g_contact_nodes[max(a, b)].x = min(a, b);
		}
	}

	// Retain a distinct ordering for physical coupling; position and warm-start target reductions keep their existing order.
	for (int row = 0; row != count; ++row)
	{
		uint key = CoupledContactInvalidKey;
		if (g_coupled_contact_scratch[row].candidate_impulse.w != 0.0f)
		{
			GpuCoupledContactBlock block = g_coupled_contact_blocks[row];
			key = ContactComponent(max(block.participant_a, block.participant_b));
		}
		g_contact_order[row] = uint2(key, row);
	}
	for (int root = count / 2; root != 0;)
	{
		--root;
		ContactSiftRow(root, count);
	}
	for (int end = count; end > 1;)
	{
		--end;
		uint2 first = g_contact_order[0];
		g_contact_order[0] = g_contact_order[end];
		g_contact_order[end] = first;
		ContactSiftRow(0, end);
	}
}

// Clear a group's detached mechanical operand; no physical speed or accumulated impulse is changed.
void ContactClearResponse(int start, int end)
{
	// Reset inexpensive visit marks before clearing each distinct mechanical participant once.
	for (int pos = start; pos != end; ++pos)
	{
		GpuCoupledContactBlock block = g_coupled_contact_blocks[g_contact_order[pos].y];
		for (int side = 0; side != 2; ++side)
		{
			int participant = side == 0 ? block.participant_a : block.participant_b;
			if (participant >= 0)
				g_contact_nodes[participant].y = 0;
		}
	}

	// Mark one means cleared; the response evaluation changes it to two after visiting a complete tree.
	for (int pos = start; pos != end; ++pos)
	{
		GpuCoupledContactBlock block = g_coupled_contact_blocks[g_contact_order[pos].y];
		for (int side = 0; side != 2; ++side)
		{
			int participant = side == 0 ? block.participant_a : block.participant_b;
			if (participant < 0)
				continue;

			if (g_contact_nodes[participant].y != 0u)
				continue;

			g_contact_nodes[participant].y = 1;
			if (participant < g_coupled_contact.rigid_body_count)
			{
				g_coupled_contact_target_impulses[participant] = CoupledContactZeroSpatial();
			}
			else
			{
				GpuArticulation tree = g_aba_articulations[participant - g_coupled_contact.rigid_body_count];
				for (int link = 0; link != tree.link_count; ++link)
					g_link_impulses[tree.link_offset + link] = CoupledContactZeroSpatial();
			}
		}
	}
}

// Add one endpoint wrench to its complete-tree or ordinary rigid response operand.
void ContactAccumulate(int body, float3 point_ws, float3 impulse)
{
	// Immovable endpoints have no incremental response, including prescribed kinematic motion.
	int participant = CoupledContactParticipant(body);
	if (participant < 0)
		return;

	GpuArticulationSpatialVector wrench = CoupledContactEndpointImpulse(body, point_ws, impulse);
	int link = CoupledContactProxyLink(body);
	if (link >= 0)
	{
		g_link_impulses[link] = AbaAddSpatial(g_link_impulses[link], wrench);
	}
	else
	{
		g_coupled_contact_target_impulses[body] = AbaAddSpatial(g_coupled_contact_target_impulses[body], wrench);
	}
}

// Read a detached whole-participant response at one world-space contact point.
float3 ContactResponseAt(int body_idx, float3 point_ws)
{
	// Articulation responses contain both endpoints' wrenches before the single complete-tree evaluation.
	int link = CoupledContactProxyLink(body_idx);
	if (link >= 0)
	{
		GpuConstraintFrame frame = g_coupled_contact_link_to_world[link];
		GpuArticulationSpatialVector twist = g_impulse_work[link];
		float3 local_point = quat_rotate(quat_conjugate(frame.rotation), point_ws - frame.position.xyz);
		return quat_rotate(frame.rotation, twist.lin.xyz + cross(twist.ang.xyz, local_point));
	}
	if (!CoupledContactRigidDynamic(body_idx))
		return float3(0, 0, 0);

	// Ordinary rigid increments use exactly the same momentum-to-point response as the owning contact transaction.
	GpuRigidBody body = g_coupled_contact_bodies[body_idx];
	GpuArticulationSpatialVector wrench = g_coupled_contact_target_impulses[body_idx];
	float3 com = body.o2w[3].xyz + mul(body.os_com_and_invmass.xyz, (float3x3)body.o2w);
	float3 angular = mul(CoupledContactRigidInverseInertia(body_idx), wrench.ang.xyz);
	return body.os_com_and_invmass.w * wrench.lin.xyz + cross(angular, point_ws - com);
}

// Apply the full relative W = J M^-1 J^T to a group's stored operand, including same-tree cross terms.
bool ContactApplyResponse(int start, int end)
{
	// Gather in canonical contact order before evaluating any participant.
	ContactClearResponse(start, end);
	for (int pos = start; pos != end; ++pos)
	{
		uint row = g_contact_order[pos].y;
		GpuResolveContact contact = g_coupled_contact_contacts[row];
		GpuRigidBody body_a = g_coupled_contact_bodies[contact.body_idx_a];
		float3 point_ws = mul(float4(contact.contact_point.xyz, 1), body_a.o2w).xyz;
		float3 impulse = mul(g_contact_krylov[row].operand.xyz, (float3x3)body_a.o2w);
		ContactAccumulate(contact.body_idx_a, point_ws, -impulse);
		ContactAccumulate(contact.body_idx_b, point_ws, +impulse);
	}

	// Each participating tree is evaluated once per mechanical product, with unchanged retained factors.
	for (int pos = start; pos != end; ++pos)
	{
		GpuCoupledContactBlock block = g_coupled_contact_blocks[g_contact_order[pos].y];
		for (int side = 0; side != 2; ++side)
		{
			int participant = side == 0 ? block.participant_a : block.participant_b;
			if (participant < g_coupled_contact.rigid_body_count)
				continue;

			if (g_contact_nodes[participant].y != 1u)
				continue;

			g_contact_nodes[participant].y = 2;
			int tree_idx = participant - g_coupled_contact.rigid_body_count;
			GpuArticulationMobilityRange range = g_mobility_ranges[tree_idx];
			GpuArticulation tree = g_aba_articulations[tree_idx];
			if (!ImpulseEvaluateResponseTree(range, tree, false))
				return false;
		}
	}

	// Both endpoint responses contribute to B-minus-A velocity in the original generating A frame.
	for (int pos = start; pos != end; ++pos)
	{
		uint row = g_contact_order[pos].y;
		GpuResolveContact contact = g_coupled_contact_contacts[row];
		GpuRigidBody body_a = g_coupled_contact_bodies[contact.body_idx_a];
		float3 point_ws = mul(float4(contact.contact_point.xyz, 1), body_a.o2w).xyz;
		float3 response = mul((float3x3)body_a.o2w, ContactResponseAt(contact.body_idx_b, point_ws) - ContactResponseAt(contact.body_idx_a, point_ws));
		if (!CoulombFinite(response))
			return false;

		g_contact_krylov[row].response = float4(response, 0);
	}
	return true;
}

// Scale a contact variable without inverting a possibly singular tangent response block.
float3 ContactScale(uint row, float3 value)
{
	// The positive scalar weights change units only; W still supplies every mechanical response.
	float3 axis = g_coupled_contact_contacts[row].axis.xyz;
	float2 weights = g_contact_krylov[row].inverse_weights.xy;
	return ScaleCoulombCoordinates(value, axis, weights.x, weights.y);
}

// Apply J*S or its true transpose S*(W*A^T+B^T), where J=A*W+B is generally nonsymmetric.
bool ContactApplyJacobian(int start, int end, bool transposed)
{
	// The transpose first applies the residual's velocity derivative, not J itself or merely the symmetric mechanical W.
	for (int pos = start; pos != end; ++pos)
	{
		uint row = g_contact_order[pos].y;
		GpuCoupledContactKrylov work = g_contact_krylov[row];
		float3x3 a = float3x3(work.velocity_derivative[0].xyz, work.velocity_derivative[1].xyz, work.velocity_derivative[2].xyz);
		float3 operand = transposed ? mul(transpose(a), work.residual.xyz) : ContactScale(row, work.direction.xyz);
		g_contact_krylov[row].operand = float4(operand, 0);
	}
	if (!ContactApplyResponse(start, end))
		return false;

	// B includes the friction-disc radius derivative with respect to normal support.
	for (int pos = start; pos != end; ++pos)
	{
		uint row = g_contact_order[pos].y;
		GpuCoupledContactKrylov work = g_contact_krylov[row];
		float3x3 a = float3x3(work.velocity_derivative[0].xyz, work.velocity_derivative[1].xyz, work.velocity_derivative[2].xyz);
		float3x3 b = float3x3(work.impulse_derivative[0].xyz, work.impulse_derivative[1].xyz, work.impulse_derivative[2].xyz);
		float3 value = transposed
			? ContactScale(row, work.response.xyz + mul(transpose(b), work.residual.xyz))
			: mul(a, work.response.xyz) + mul(b, work.operand.xyz);
		if (!CoulombFinite(value))
			return false;

		if (transposed)
			g_contact_krylov[row].gradient = float4(value, 0);
		else
			g_contact_krylov[row].jacobian_product = float4(value, 0);
	}
	return true;
}

// Restore the accepted impulse and its exact zero incremental response when no trial is selected.
void ContactResetCandidate(int start, int end)
{
	// A transpose product or rejected trial must not survive as selected-delta diagnostic data.
	for (int pos = start; pos != end; ++pos)
	{
		uint row = g_contact_order[pos].y;
		g_coupled_contact_scratch[row].candidate_impulse.xyz = g_coupled_contact_contacts[row].warmstart_impulse.xyz;
		g_contact_krylov[row].operand = float4(0, 0, 0, 0);
		g_contact_krylov[row].response = float4(0, 0, 0, 0);
	}
}

// Search one projected raw direction. False means invalid arithmetic; an exhausted finite direction returns true without acceptance.
bool ContactSearchArc(int start, int end, float nonlinear_squared, float tolerance_squared, bool gradient_direction, out_(bool) accepted, inout_(uint) response_products)
{
	// Shorten before projection: the resulting curve cannot reuse a scaled response of the full projected endpoint.
	accepted = false;
	float fraction = 1.0f;
	for (;;)
	{
		// Form represented deltas for the entire component before any complete mechanical response.
		bool changed = false;
		for (int pos = start; pos != end; ++pos)
		{
			uint row = g_contact_order[pos].y;
			GpuResolveContact contact = g_coupled_contact_contacts[row];
			GpuCoupledContactKrylov work = g_contact_krylov[row];
			float3 direction = gradient_direction ? work.gradient.xyz : work.step.xyz;
			float3 delta = g_coupled_contact.relaxation * ContactScale(row, direction);
			float3 unprojected = contact.warmstart_impulse.xyz + fraction * delta;
			if (!CoulombFinite(delta) || !CoulombFinite(unprojected))
				return false;

			// Guard before projection: rounded no-change must not depend on bitwise projector idempotence, even within a changing component.
			float3 candidate = contact.warmstart_impulse.xyz;
			if (any(unprojected != candidate))
			{
				if (!ProjectCoulombMetricImpulse(unprojected, contact.axis.xyz, g_coupled_contact_blocks[row].friction, work.inverse_weights.xy, candidate))
					return false;
			}
			float3 represented_delta = candidate - contact.warmstart_impulse.xyz;
			if (!CoulombFinite(candidate) || !CoulombFinite(represented_delta))
				return false;

			g_coupled_contact_scratch[row].candidate_impulse.xyz = candidate;
			g_contact_krylov[row].operand = float4(represented_delta, 0);
			g_contact_krylov[row].response = float4(0, 0, 0, 0);
			changed = changed || any(candidate != contact.warmstart_impulse.xyz);
		}
		if (!changed)
		{
			// Exhaustion is unresolved finite state, not a law or infeasibility certificate.
			ContactResetCandidate(start, end);
			return true;
		}
		if (!ContactApplyResponse(start, end))
			return false;

		++response_products;

		// Acceptance measures the ordinary law at this exact represented candidate and its fresh complete W response.
		float trial_squared = 0;
		for (int pos = start; pos != end; ++pos)
		{
			uint row = g_contact_order[pos].y;
			GpuResolveContact contact = g_coupled_contact_contacts[row];
			GpuCoupledContactBlock block = g_coupled_contact_blocks[row];
			GpuRigidBody body_a = g_coupled_contact_bodies[contact.body_idx_a];
			float3 point_ws = mul(float4(contact.contact_point.xyz, 1), body_a.o2w).xyz;
			float3 velocity = CoupledContactRelativeVelocity(contact, point_ws) - block.target_normal_speed * contact.axis.xyz + g_contact_krylov[row].response.xyz;
			CoulombLinearisation law;
			if (!LineariseCoulombLaw(float3x3(block.response_0.xyz, block.response_1.xyz, block.response_2.xyz), velocity, g_coupled_contact_scratch[row].candidate_impulse.xyz, contact.axis.xyz, block.friction, law))
				return false;

			trial_squared += dot(law.residual, law.residual);
		}
		if (!isfinite(trial_squared))
			return false;

		if (trial_squared < nonlinear_squared || trial_squared <= tolerance_squared)
		{
			accepted = true;
			return true;
		}

		// Finite represented no-change, not an empirical search cap, terminates a direction that cannot make progress.
		fraction *= 0.5f;
	}
}

// Select at most one candidate, using the same projected arc for the frozen Newton step and the original-law negative gradient.
bool ContactSelectCandidate(int start, int end, float nonlinear_squared, float tolerance_squared)
{
	// Counts describe candidate W products and the fallback transpose, excluding CGLS and final publication.
	uint response_products = 0;
	uint gradient_products = 0;
	bool accepted = false;
	ContactResetCandidate(start, end);
	if (nonlinear_squared > tolerance_squared)
	{
		if (!ContactSearchArc(start, end, nonlinear_squared, tolerance_squared, false, accepted, response_products))
			return false;

		if (!accepted)
		{
			// residual is original -F, so gradient is -S*J^T*F: add it, do not negate it again.
			if (!ContactApplyJacobian(start, end, true))
				return false;

			++gradient_products;
			if (!ContactSearchArc(start, end, nonlinear_squared, tolerance_squared, true, accepted, response_products))
				return false;
		}
	}
	for (int pos = start; pos != end; ++pos)
	{
		// Every row names its component's variable search cost; accepted operand/response remain the actual represented delta and W*delta.
		uint row = g_contact_order[pos].y;
		g_contact_krylov[row].inverse_weights.zw = float2(response_products, gradient_products);
	}
	return true;
}

// Compute one safeguarded contact update per caller sweep; Krylov products solve only its frozen linear equation.
numthreads(CSBuildCoupledContactCandidates, ConstraintThreadCount, 1, 1)
void CSBuildCoupledContactCandidates(int3 DTID(dtid))
{
	// One invocation owns a complete connected component; separate components share no mutable mechanical target.
	int start = CoupledContactThreadIndex(dtid);
	int count = CoupledContactCount();
	if (start >= count)
		return;

	uint key = g_contact_order[start].x;
	if (key == CoupledContactInvalidKey)
		return;

	if (start != 0)
	{
		// Guard the prefix boundary before reading the preceding row.
		if (g_contact_order[start - 1].x == key)
			return;
	}

	int end = start + 1;
	while (end < count)
	{
		// Stop at the first distinct participant component.
		if (g_contact_order[end].x != key)
			break;

		++end;
	}

	// Linearise at actual accepted state: r(lambda+delta)=r_actual+W*delta, so accumulated impulses are not applied twice.
	float residual_squared = 0;
	for (int pos = start; pos != end; ++pos)
	{
		uint row = g_contact_order[pos].y;
		GpuResolveContact contact = g_coupled_contact_contacts[row];
		GpuCoupledContactBlock block = g_coupled_contact_blocks[row];
		GpuRigidBody body_a = g_coupled_contact_bodies[contact.body_idx_a];
		float3 point_ws = mul(float4(contact.contact_point.xyz, 1), body_a.o2w).xyz;
		float3 velocity = CoupledContactRelativeVelocity(contact, point_ws) - block.target_normal_speed * contact.axis.xyz;
		CoulombLinearisation law;
		if (!LineariseCoulombLaw(float3x3(block.response_0.xyz, block.response_1.xyz, block.response_2.xyz), velocity, contact.warmstart_impulse.xyz, contact.axis.xyz, block.friction, law))
		{
			CoupledContactFail(CoupledContactFailureNonFinite);
			return;
		}
		GpuCoupledContactKrylov work = (GpuCoupledContactKrylov)0;
		for (int axis = 0; axis != 3; ++axis)
		{
			work.velocity_derivative[axis] = float4(law.velocity_derivative[axis], 0);
			work.impulse_derivative[axis] = float4(law.impulse_derivative[axis], 0);
		}
		work.inverse_weights = float4(1.0f / law.normal_weight, 1.0f / max(law.normal_weight, law.tangent_weight), 0, 0);
		work.residual = float4(-law.residual, 0);
		g_contact_krylov[row] = work;
		residual_squared += dot(law.residual, law.residual);
	}

	// CGLS needs no inverse or regularisation of a rank-deficient Jacobian. Its 3m product cap is not a convergence certificate.
	float nonlinear_squared = residual_squared;
	float tolerance_squared = g_coupled_contact.velocity_tolerance * g_coupled_contact.velocity_tolerance;
	if (!isfinite(residual_squared))
	{
		CoupledContactFail(CoupledContactFailureNonFinite);
		return;
	}
	if (residual_squared > tolerance_squared)
	{
		if (!ContactApplyJacobian(start, end, true))
		{
			CoupledContactFail(CoupledContactFailureNonFinite);
			return;
		}
		float gamma = 0;
		for (int pos = start; pos != end; ++pos)
		{
			uint row = g_contact_order[pos].y;
			float3 gradient = g_contact_krylov[row].gradient.xyz;
			g_contact_krylov[row].direction = float4(gradient, 0);
			gamma += dot(gradient, gradient);
		}
		for (uint iteration = 0; iteration != 3u * (uint)(end - start); ++iteration)
		{
			// A zero gradient is a singular least-squares stationary point, not proof that the nonlinear contact law is satisfied.
			if (!isfinite(gamma))
			{
				CoupledContactFail(CoupledContactFailureNonFinite);
				return;
			}
			if (gamma == 0.0f)
				break;

			if (!ContactApplyJacobian(start, end, false))
			{
				CoupledContactFail(CoupledContactFailureNonFinite);
				return;
			}
			float denominator = 0;
			for (int pos = start; pos != end; ++pos)
			{
				float3 value = g_contact_krylov[g_contact_order[pos].y].jacobian_product.xyz;
				denominator += dot(value, value);
			}
			if (!isfinite(denominator))
			{
				CoupledContactFail(CoupledContactFailureNonFinite);
				return;
			}
			if (denominator == 0.0f)
				break;

			// Advance the frozen linear solve, not the physical state or nonlinear active set.
			float alpha = gamma / denominator;
			residual_squared = 0;
			for (int pos = start; pos != end; ++pos)
			{
				uint row = g_contact_order[pos].y;
				GpuCoupledContactKrylov work = g_contact_krylov[row];
				work.step.xyz += alpha * work.direction.xyz;
				work.residual.xyz -= alpha * work.jacobian_product.xyz;
				residual_squared += dot(work.residual.xyz, work.residual.xyz);
				g_contact_krylov[row] = work;
			}
			if (!isfinite(residual_squared))
			{
				CoupledContactFail(CoupledContactFailureNonFinite);
				return;
			}
			// Reserve the existing relaxation's share of the caller's tolerance for linear-solve error.
			if (residual_squared <= tolerance_squared * g_coupled_contact.relaxation * g_coupled_contact.relaxation)
				break;

			if (!ContactApplyJacobian(start, end, true))
			{
				CoupledContactFail(CoupledContactFailureNonFinite);
				return;
			}
			float next_gamma = 0;
			for (int pos = start; pos != end; ++pos)
			{
				float3 gradient = g_contact_krylov[g_contact_order[pos].y].gradient.xyz;
				next_gamma += dot(gradient, gradient);
			}
			float beta = next_gamma / gamma;
			for (int pos = start; pos != end; ++pos)
			{
				uint row = g_contact_order[pos].y;
				g_contact_krylov[row].direction.xyz = g_contact_krylov[row].gradient.xyz + beta * g_contact_krylov[row].direction.xyz;
			}
			gamma = next_gamma;
		}
	}

	// Recover original -F after CGLS. The retained residual is now nonlinear, not the last frozen-linear residual.
	for (int pos = start; pos != end; ++pos)
	{
		uint row = g_contact_order[pos].y;
		GpuResolveContact contact = g_coupled_contact_contacts[row];
		GpuCoupledContactBlock block = g_coupled_contact_blocks[row];
		GpuRigidBody body_a = g_coupled_contact_bodies[contact.body_idx_a];
		float3 point_ws = mul(float4(contact.contact_point.xyz, 1), body_a.o2w).xyz;
		float3 velocity = CoupledContactRelativeVelocity(contact, point_ws) - block.target_normal_speed * contact.axis.xyz;
		CoulombLinearisation law;
		if (!LineariseCoulombLaw(float3x3(block.response_0.xyz, block.response_1.xyz, block.response_2.xyz), velocity, contact.warmstart_impulse.xyz, contact.axis.xyz, block.friction, law))
		{
			CoupledContactFail(CoupledContactFailureNonFinite);
			return;
		}
		g_contact_krylov[row].residual = float4(-law.residual, 0);
	}

	// A non-descending full endpoint cannot veto shorter points on the projected raw-Newton arc.
	if (!ContactSelectCandidate(start, end, nonlinear_squared, tolerance_squared))
	{
		// Invalid arithmetic preserves the existing whole-transaction failure behavior, rather than becoming an exhausted direction.
		CoupledContactFail(CoupledContactFailureNonFinite);
		return;
	}

	// Publish only the selected update. A rounded no-change retains unresolved law error for the next caller sweep.
	ContactClearResponse(start, end);
	for (int pos = start; pos != end; ++pos)
	{
		// Physical response and accepted impulses use the same represented candidate; later observation still measures actual primary state.
		uint row = g_contact_order[pos].y;
		GpuResolveContact contact = g_coupled_contact_contacts[row];
		float3 candidate = g_coupled_contact_scratch[row].candidate_impulse.xyz;
		if (!CoupledContactPublishContributions(row, contact, candidate - contact.warmstart_impulse.xyz))
			CoupledContactFail(CoupledContactFailureNonFinite);
	}
}
