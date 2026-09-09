//*********************************************
// Physics Engine — Transient Coupled Contacts
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Solves GPU-generated contacts that touch articulation proxies without expanding a tree into
// independent rigid bodies. Contact blocks use exact self-link mobility, deterministic endpoint
// reduction, and one complete-tree impulse ABA response per outer Jacobi sweep.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Fixed bounds and solve controls shared by every transient-contact phase.
struct cbCoupledContact
{
	int max_contacts;
	int body_count;
	int rigid_body_count;
	int articulation_count;

	int link_count;
	int target_count;
	int work_count;
	int phase;

	int velocity_delta_count;
	float relaxation;
	float restitution_scale;
	float dt;

	float position_slop;
	float position_beta;
	float max_position_speed;
	int position_iteration_index;
};

ConstantBuffer<cbCoupledContact> resource(g_coupled_contact, b0);

StructuredBuffer<GpuCollisionCounters> resource(g_coupled_contact_counters, t0);
StructuredBuffer<GpuMaterial> resource(g_coupled_contact_materials, t1);
StructuredBuffer<GpuArticulationLink> resource(g_coupled_contact_links, t2);
StructuredBuffer<GpuConstraintFrame> resource(g_coupled_contact_link_to_world, t3);
StructuredBuffer<GpuArticulationSpatialMobility> resource(g_coupled_contact_mobilities, t4);
StructuredBuffer<GpuArticulationAbaScratch> resource(g_coupled_contact_aba_scratch, t5);
StructuredBuffer<GpuArticulationMobilityRange> resource(g_coupled_contact_ranges, t6);
RWStructuredBuffer<GpuRigidBody> resource(g_coupled_contact_bodies, u0);
RWStructuredBuffer<GpuResolveContact> resource(g_coupled_contact_contacts, u1);
RWStructuredBuffer<GpuCoupledContactBlock> resource(g_coupled_contact_blocks, u2);
RWStructuredBuffer<GpuCoupledContactScratch> resource(g_coupled_contact_scratch, u3);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_contact_contributions, u4);
RWStructuredBuffer<uint> resource(g_coupled_contact_endpoint_keys, u5);
RWStructuredBuffer<uint> resource(g_coupled_contact_endpoint_order, u6);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_contact_target_impulses, u7);
RWStructuredBuffer<uint> resource(g_coupled_contact_participant_degrees, u8);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_contact_link_impulses, u9);
RWStructuredBuffer<uint> resource(g_coupled_contact_tree_selection, u10);
RWStructuredBuffer<uint> resource(g_coupled_contact_tree_results, u11);
RWStructuredBuffer<GpuCoupledContactState> resource(g_coupled_contact_state, u12);
RWStructuredBuffer<GpuConstraintPseudoVelocity> resource(g_coupled_contact_rigid_pseudo, u13);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_contact_link_pseudo, u14);
RWStructuredBuffer<float> resource(g_coupled_contact_generalized_pseudo, u15);
RWStructuredBuffer<GpuArticulationSpatialVector> resource(g_coupled_contact_articulation_work, u16);
RWStructuredBuffer<float> resource(g_coupled_contact_velocity_deltas, u17);
RWStructuredBuffer<GpuArticulation> resource(g_coupled_contact_articulations, u18);
RWStructuredBuffer<float> resource(g_coupled_contact_positions, u19);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
namespace pr::physics {
#endif

static const uint CoupledContactInvalidKey = 0xFFFFFFFFu;
static const int CoupledContactPhaseWarmStart = 0;
static const int CoupledContactPhaseVelocity = 1;
static const int CoupledContactPhasePosition = 2;
static const uint CoupledContactFailureTopology = 1u << 0;
static const uint CoupledContactFailureNonFinite = 1u << 1;
static const float CoupledContactMaximumAngularDisplacement = 1.0e4f;

// Return a fully initialized zero spatial value for detached reduction buffers.
GpuArticulationSpatialVector CoupledContactZeroSpatial()
{
	GpuArticulationSpatialVector value;
	value.ang = float4(0.0f, 0.0f, 0.0f, 0.0f);
	value.lin = float4(0.0f, 0.0f, 0.0f, 0.0f);
	return value;
}

// Return one angular-then-linear component.
float CoupledContactSpatialComponent(GpuArticulationSpatialVector value, int index)
{
	return index < 3 ? value.ang[index] : value.lin[index - 3];
}

// Replace one angular-then-linear component.
void CoupledContactSetSpatialComponent(inout_(GpuArticulationSpatialVector) value, int index, float component)
{
	if (index < 3)
		value.ang[index] = component;
	else
		value.lin[index - 3] = component;
}

// Return one component from the packed symmetric 6x6 mobility matrix.
float CoupledContactMobilityComponent(GpuArticulationSpatialMobility mobility, int row, int column)
{
	int low = min(row, column);
	int high = max(row, column);
	int packed_index = low * 6 - low * (low - 1) / 2 + high - low;
	return mobility.packed[packed_index / 4][packed_index % 4];
}

// Apply one packed force-to-motion mobility without depending on the full ABA shader resource layout.
GpuArticulationSpatialVector CoupledContactApplyMobility(GpuArticulationSpatialMobility mobility, GpuArticulationSpatialVector force)
{
	GpuArticulationSpatialVector motion = CoupledContactZeroSpatial();
	for (int column = 0; column != 6; ++column)
	{
		float force_component = CoupledContactSpatialComponent(force, column);
		for (int row = 0; row != 6; ++row)
		{
			CoupledContactSetSpatialComponent(
				motion,
				row,
				CoupledContactSpatialComponent(motion, row) + CoupledContactMobilityComponent(mobility, row, column) * force_component);
		}
	}
	return motion;
}

// Return an inactive block whose matrix and endpoint metadata cannot participate accidentally.
GpuCoupledContactBlock CoupledContactInactiveBlock()
{
	GpuCoupledContactBlock block;
	block.response_0 = float4(0.0f, 0.0f, 0.0f, 0.0f);
	block.response_1 = float4(0.0f, 0.0f, 0.0f, 0.0f);
	block.response_2 = float4(0.0f, 0.0f, 0.0f, 0.0f);
	block.target_normal_speed = 0.0f;
	block.friction = 0.0f;
	block.participant_a = -1;
	block.participant_b = -1;
	return block;
}

// Return true when all physical components of a vector are finite.
bool CoupledContactVectorFinite(float3 value)
{
	return isfinite(value.x) && isfinite(value.y) && isfinite(value.z);
}

// Return true when both halves of a spatial vector are finite.
bool CoupledContactSpatialFinite(GpuArticulationSpatialVector value)
{
	return CoupledContactVectorFinite(value.ang.xyz) && CoupledContactVectorFinite(value.lin.xyz);
}

// Return whether an accumulated pseudo angular velocity can be integrated safely this substep.
bool CoupledContactAngularDisplacementValid(float3 angular_velocity)
{
	float3 displacement = g_coupled_contact.dt * angular_velocity;
	return
		CoupledContactVectorFinite(displacement) &&
		max(abs(displacement.x), max(abs(displacement.y), abs(displacement.z))) <= CoupledContactMaximumAngularDisplacement;
}

// Return one spatial sum without relying on shader-only operator overloads.
GpuArticulationSpatialVector CoupledContactAddSpatial(GpuArticulationSpatialVector lhs, GpuArticulationSpatialVector rhs)
{
	lhs.ang += rhs.ang;
	lhs.lin += rhs.lin;
	return lhs;
}

// Record a sticky transaction failure without relying on floating-point atomics.
void CoupledContactFail(uint failure)
{
#ifdef __cplusplus
	g_coupled_contact_state[0].valid = 0u;
	g_coupled_contact_state[0].failure_flags |= failure;
#else
	uint ignored;
	InterlockedAnd(g_coupled_contact_state[0].valid, 0u, ignored);
	InterlockedOr(g_coupled_contact_state[0].failure_flags, failure, ignored);
#endif
}

// Bound unsigned attempted-count bits before signed indexing, matching the endpoint sort without changing the raw counter.
uint CoupledContactBoundedCount(uint capacity)
{
	return min((uint)g_coupled_contact_counters[0].contact_count, capacity);
}

// Return the bounded number of contacts produced by narrowphase.
int CoupledContactCount()
{
	return (int)CoupledContactBoundedCount((uint)g_coupled_contact.max_contacts);
}

// Flatten large dispatches; saturated padding indices are rejected by every signed logical bound before resource access.
int CoupledContactThreadIndex(int3 dtid)
{
	uint index = (uint)dtid.x + (uint)dtid.y * 65535u * (uint)ConstraintThreadCount;
	return (int)min(index, 0x7FFFFFFFu);
}

// Return true when a packed body index denotes a hidden articulation-link proxy.
bool CoupledContactProxyBody(int body_idx)
{
	return body_idx >= g_coupled_contact.rigid_body_count && body_idx < g_coupled_contact.body_count;
}

// Resolve a proxy body to its globally packed link and verify the contiguous suffix invariant.
int CoupledContactProxyLink(int body_idx)
{
	if (!CoupledContactProxyBody(body_idx))
		return -1;

	int link_idx = body_idx - g_coupled_contact.rigid_body_count;
	if (
		link_idx < 0 ||
		link_idx >= g_coupled_contact.link_count ||
		g_coupled_contact_links[link_idx].proxy_body_index != body_idx)
		return -1;
	return link_idx;
}

// Return the root proxy that owns sleep state for a validated packed link.
int CoupledContactRootProxy(int link_idx)
{
	int articulation_idx = g_coupled_contact_links[link_idx].articulation_index;
	return g_coupled_contact.rigid_body_count + g_coupled_contact_articulations[articulation_idx].link_offset;
}

// Return whether a contact endpoint belongs to a sleeping articulation.
bool CoupledContactSleepingTree(int body_idx)
{
	int link_idx = CoupledContactProxyLink(body_idx);
	if (link_idx < 0)
		return false;

	return AllSet(g_coupled_contact_bodies[CoupledContactRootProxy(link_idx)].state_flags, ERigidBodyStateFlags_Sleeping);
}

// Return true when one ordinary rigid endpoint can accept momentum.
bool CoupledContactRigidDynamic(int body_idx)
{
	return
		body_idx >= 0 &&
		body_idx < g_coupled_contact.rigid_body_count &&
		g_coupled_contact_bodies[body_idx].os_com_and_invmass.w > 0.0f &&
		!AllSet(g_coupled_contact_bodies[body_idx].state_flags, ERigidBodyStateFlags_Static);
}

// Map a dynamic endpoint to the degree-damping participant representing its complete response unit.
int CoupledContactParticipant(int body_idx)
{
	if (CoupledContactRigidDynamic(body_idx))
		return body_idx;

	int link_idx = CoupledContactProxyLink(body_idx);
	if (link_idx < 0)
		return -1;

	int articulation_idx = g_coupled_contact_links[link_idx].articulation_index;
	if (articulation_idx < 0 || articulation_idx >= g_coupled_contact.articulation_count)
		return -1;
	return g_coupled_contact.rigid_body_count + articulation_idx;
}

// Map a dynamic endpoint to its deterministic impulse-reduction target.
uint CoupledContactTarget(int body_idx)
{
	if (CoupledContactRigidDynamic(body_idx))
		return (uint)body_idx;

	int link_idx = CoupledContactProxyLink(body_idx);
	if (link_idx < 0)
		return CoupledContactInvalidKey;
	return (uint)(g_coupled_contact.rigid_body_count + link_idx);
}

// Return one rigid body's world-space inverse inertia around its centre of mass.
float3x3 CoupledContactRigidInverseInertia(int body_idx)
{
	GpuRigidBody body = g_coupled_contact_bodies[body_idx];
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 os_iinv = inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
	return rotate_inertia_inv(os_iinv, (float3x3)body.o2w);
}

// Return one endpoint's current world-space point velocity from authoritative rigid or articulation state.
float3 CoupledContactPointVelocity(int body_idx, float3 point_ws)
{
	if (CoupledContactProxyBody(body_idx))
	{
		int link_idx = CoupledContactProxyLink(body_idx);
		if (link_idx < 0)
			return float3(0.0f, 0.0f, 0.0f);

		GpuConstraintFrame frame = g_coupled_contact_link_to_world[link_idx];
		GpuArticulationSpatialVector twist = g_coupled_contact_aba_scratch[link_idx].link_velocity;
		float3 point_link = quat_rotate(quat_conjugate(frame.rotation), point_ws - frame.position.xyz);
		float3 velocity_link = twist.lin.xyz + cross(twist.ang.xyz, point_link);
		return quat_rotate(frame.rotation, velocity_link);
	}

	if (!CoupledContactRigidDynamic(body_idx))
		return float3(0.0f, 0.0f, 0.0f);

	GpuRigidBody body = g_coupled_contact_bodies[body_idx];
	float3 com_ws = body.o2w[3].xyz + mul(body.os_com_and_invmass.xyz, (float3x3)body.o2w);
	float3 angular_velocity = mul(CoupledContactRigidInverseInertia(body_idx), body.momentum_ang.xyz);
	float3 linear_velocity = body.os_com_and_invmass.w * body.momentum_lin.xyz;
	return linear_velocity + cross(angular_velocity, point_ws - com_ws);
}

// Return the endpoint's world-space point-velocity response to a world-space impulse at that point.
float3 CoupledContactPointResponse(int body_idx, float3 point_ws, float3 impulse_ws)
{
	if (CoupledContactProxyBody(body_idx))
	{
		int link_idx = CoupledContactProxyLink(body_idx);
		if (link_idx < 0 || link_idx >= g_coupled_contact.link_count)
			return float3(0.0f, 0.0f, 0.0f);

		GpuConstraintFrame frame = g_coupled_contact_link_to_world[link_idx];
		float3 point_link = quat_rotate(quat_conjugate(frame.rotation), point_ws - frame.position.xyz);
		float3 impulse_link = quat_rotate(quat_conjugate(frame.rotation), impulse_ws);
		GpuArticulationSpatialVector wrench;
		wrench.ang = float4(cross(point_link, impulse_link), 0.0f);
		wrench.lin = float4(impulse_link, 0.0f);
		GpuArticulationSpatialVector response = CoupledContactApplyMobility(g_coupled_contact_mobilities[link_idx], wrench);
		return quat_rotate(frame.rotation, response.lin.xyz + cross(response.ang.xyz, point_link));
	}

	if (!CoupledContactRigidDynamic(body_idx))
		return float3(0.0f, 0.0f, 0.0f);

	GpuRigidBody body = g_coupled_contact_bodies[body_idx];
	float3 com_ws = body.o2w[3].xyz + mul(body.os_com_and_invmass.xyz, (float3x3)body.o2w);
	float3 lever = point_ws - com_ws;
	float3 angular_response = mul(CoupledContactRigidInverseInertia(body_idx), cross(lever, impulse_ws));
	return body.os_com_and_invmass.w * impulse_ws + cross(angular_response, lever);
}

// Build the symmetric 3x3 relative point-response matrix in body A space.
float3x3 CoupledContactResponseMatrix(GpuResolveContact contact, float3 point_ws)
{
	float3x3 rot_a = (float3x3)g_coupled_contact_bodies[contact.body_idx_a].o2w;
	float3 response_columns[3];
	for (int axis_idx = 0; axis_idx != 3; ++axis_idx)
	{
		float3 impulse_a = float3(0.0f, 0.0f, 0.0f);
		impulse_a[axis_idx] = 1.0f;
		float3 impulse_ws = mul(impulse_a, rot_a);
		float3 response_ws =
			CoupledContactPointResponse(contact.body_idx_a, point_ws, impulse_ws) +
			CoupledContactPointResponse(contact.body_idx_b, point_ws, impulse_ws);
		response_columns[axis_idx] = mul(rot_a, response_ws);
	}

	float3x3 response = float3x3(
		float3(response_columns[0].x, response_columns[1].x, response_columns[2].x),
		float3(response_columns[0].y, response_columns[1].y, response_columns[2].y),
		float3(response_columns[0].z, response_columns[1].z, response_columns[2].z));
	return 0.5f * (response + transpose(response));
}

// Return the current B-minus-A point velocity in body A space.
float3 CoupledContactRelativeVelocity(GpuResolveContact contact, float3 point_ws)
{
	float3 velocity_ws =
		CoupledContactPointVelocity(contact.body_idx_b, point_ws) -
		CoupledContactPointVelocity(contact.body_idx_a, point_ws);
	return mul((float3x3)g_coupled_contact_bodies[contact.body_idx_a].o2w, velocity_ws);
}

// Return one endpoint's detached position-only point velocity in world space.
float3 CoupledContactPseudoPointVelocity(int body_idx, float3 point_ws)
{
	int link_idx = CoupledContactProxyLink(body_idx);
	if (link_idx >= 0)
	{
		GpuConstraintFrame frame = g_coupled_contact_link_to_world[link_idx];
		GpuArticulationSpatialVector pseudo = g_coupled_contact_link_pseudo[link_idx];
		float3 point_link = quat_rotate(quat_conjugate(frame.rotation), point_ws - frame.position.xyz);
		return quat_rotate(frame.rotation, pseudo.lin.xyz + cross(pseudo.ang.xyz, point_link));
	}

	if (!CoupledContactRigidDynamic(body_idx))
		return float3(0.0f, 0.0f, 0.0f);

	GpuRigidBody body = g_coupled_contact_bodies[body_idx];
	float3 com_ws = body.o2w[3].xyz + mul(body.os_com_and_invmass.xyz, (float3x3)body.o2w);
	GpuConstraintPseudoVelocity pseudo = g_coupled_contact_rigid_pseudo[body_idx];
	return pseudo.linear_velocity.xyz + cross(pseudo.angular_velocity.xyz, point_ws - com_ws);
}

// Return the detached B-minus-A position-only point velocity in body A space.
float3 CoupledContactPseudoRelativeVelocity(GpuResolveContact contact, float3 point_ws)
{
	float3 velocity_ws =
		CoupledContactPseudoPointVelocity(contact.body_idx_b, point_ws) -
		CoupledContactPseudoPointVelocity(contact.body_idx_a, point_ws);
	return mul((float3x3)g_coupled_contact_bodies[contact.body_idx_a].o2w, velocity_ws);
}

// Make a cached impulse feasible without changing its positive normal component.
float3 CoupledContactProjectImpulse(float3 impulse, float3 axis, float friction)
{
	float normal = max(dot(impulse, axis), 0.0f);
	float3 tangent = impulse - dot(impulse, axis) * axis;
	float tangent_length = length(tangent);
	float tangent_limit = max(friction, 0.0f) * normal;
	if (tangent_length > tangent_limit && tangent_length > 1.0e-12f)
		tangent *= tangent_limit / tangent_length;
	return normal * axis + tangent;
}

// Return a local Coulomb impulse that corrects closing motion and opposes slip within the available normal support.
float3 CoupledContactSolveImpulse(GpuCoupledContactBlock block, float3 old_impulse, float3 residual, float3 axis)
{
	// A frictionless contact has only one active direction, regardless of the full point-response matrix.
	float old_normal = dot(old_impulse, axis);
	if (block.friction == 0.0f)
		return max(old_normal - dot(residual, axis) * block.response_0.w, 0.0f) * axis;

	// Local sweeps account for normal/tangent coupling without another articulation-response dispatch.
	float3x3 response = float3x3(block.response_0.xyz, block.response_1.xyz, block.response_2.xyz);
	float3 candidate = old_impulse;
	for (int iteration = 0; iteration != 4; ++iteration)
	{
		// Normal complementarity is independent of slip; a joint cone projection would manufacture normal support.
		float3 velocity = residual + mul(response, candidate - old_impulse);
		float normal = dot(candidate, axis);
		float corrected_normal = max(normal - dot(velocity, axis) * block.response_0.w, 0.0f);
		candidate += (corrected_normal - normal) * axis;

		// The tangent trace bounds its response, so this scalar step permits an ordinary friction-disc projection.
		velocity = residual + mul(response, candidate - old_impulse);
		float3 slip = velocity - dot(velocity, axis) * axis;
		float3 tangent = candidate - corrected_normal * axis - block.response_1.w * slip;
		float tangent_length = length(tangent);
		float tangent_limit = block.friction * corrected_normal;
		if (tangent_length > tangent_limit && tangent_length > 0.0f)
			tangent *= tangent_limit / tangent_length;

		candidate = corrected_normal * axis + tangent;
	}
	return candidate;
}

// Convert one endpoint's signed point impulse into its target-local spatial impulse.
GpuArticulationSpatialVector CoupledContactEndpointImpulse(int body_idx, float3 point_ws, float3 impulse_ws)
{
	GpuArticulationSpatialVector impulse = CoupledContactZeroSpatial();
	int link_idx = CoupledContactProxyLink(body_idx);
	if (link_idx >= 0)
	{
		GpuConstraintFrame frame = g_coupled_contact_link_to_world[link_idx];
		float3 point_link = quat_rotate(quat_conjugate(frame.rotation), point_ws - frame.position.xyz);
		float3 force_link = quat_rotate(quat_conjugate(frame.rotation), impulse_ws);
		impulse.ang.xyz = cross(point_link, force_link);
		impulse.lin.xyz = force_link;
		return impulse;
	}

	if (CoupledContactRigidDynamic(body_idx))
	{
		GpuRigidBody body = g_coupled_contact_bodies[body_idx];
		float3 com_ws = body.o2w[3].xyz + mul(body.os_com_and_invmass.xyz, (float3x3)body.o2w);
		impulse.ang.xyz = cross(point_ws - com_ws, impulse_ws);
		impulse.lin.xyz = impulse_ws;
	}
	return impulse;
}

// Publish two endpoint contributions for one body-A-space impulse delta.
bool CoupledContactPublishContributions(int contact_idx, GpuResolveContact contact, float3 impulse_delta_a)
{
	GpuRigidBody body_a = g_coupled_contact_bodies[contact.body_idx_a];
	float3 point_ws = mul(float4(contact.contact_point.xyz, 1.0f), body_a.o2w).xyz;
	float3 impulse_ws = mul(impulse_delta_a, (float3x3)body_a.o2w);
	GpuArticulationSpatialVector impulse_a = CoupledContactEndpointImpulse(contact.body_idx_a, point_ws, -impulse_ws);
	GpuArticulationSpatialVector impulse_b = CoupledContactEndpointImpulse(contact.body_idx_b, point_ws, +impulse_ws);
	if (!CoupledContactSpatialFinite(impulse_a) || !CoupledContactSpatialFinite(impulse_b))
		return false;

	g_coupled_contact_contributions[2 * contact_idx + 0] = impulse_a;
	g_coupled_contact_contributions[2 * contact_idx + 1] = impulse_b;
	return true;
}

// Wake one ordinary rigid endpoint after a committed coupled-contact impulse.
void CoupledContactWakeRigid(inout_(GpuRigidBody) body)
{
	if (body.os_com_and_invmass.w <= 0.0f || AllSet(body.state_flags, ERigidBodyStateFlags_Static))
		return;

	body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Collided, true);
	if (!AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping) && body.sleep.island_id < 0)
		return;

	body.sleep.generation++;
	body.state_flags = SetFlag(body.state_flags, ERigidBodyStateFlags_Sleeping, false);
	body.sleep.timer_s = 0.0f;
	body.sleep.island_id = -1;
	body.sleep.flags = 0;
}

// Clear topology, degree, and transaction buffers before preparing a new transient contact set.
numthreads(CSClearCoupledContacts, ConstraintThreadCount, 1, 1)
void CSClearCoupledContacts(int3 DTID(dtid))
{
	int idx = CoupledContactThreadIndex(dtid);
	if (idx >= g_coupled_contact.work_count)
		return;

	// A fixed-capacity radix sort requires an invalid suffix, even when narrowphase produced fewer contacts this frame.
	if (idx < g_coupled_contact.max_contacts)
	{
		g_coupled_contact_blocks[idx] = CoupledContactInactiveBlock();
		GpuCoupledContactScratch scratch;
		scratch.candidate_impulse = float4(0.0f, 0.0f, 0.0f, 0.0f);
		scratch.position_impulse = float4(0.0f, 0.0f, 0.0f, 0.0f);
		scratch.candidate_position_impulse = float4(0.0f, 0.0f, 0.0f, 0.0f);
		g_coupled_contact_scratch[idx] = scratch;
		g_coupled_contact_contributions[2 * idx + 0] = CoupledContactZeroSpatial();
		g_coupled_contact_contributions[2 * idx + 1] = CoupledContactZeroSpatial();
		g_coupled_contact_endpoint_keys[2 * idx + 0] = CoupledContactInvalidKey;
		g_coupled_contact_endpoint_keys[2 * idx + 1] = CoupledContactInvalidKey;
		g_coupled_contact_endpoint_order[2 * idx + 0] = 2 * idx + 0;
		g_coupled_contact_endpoint_order[2 * idx + 1] = 2 * idx + 1;
	}
	if (idx < g_coupled_contact.target_count)
		g_coupled_contact_target_impulses[idx] = CoupledContactZeroSpatial();
	if (idx < g_coupled_contact.rigid_body_count + g_coupled_contact.articulation_count)
		g_coupled_contact_participant_degrees[idx] = 0u;
	if (idx < g_coupled_contact.link_count)
		g_coupled_contact_link_impulses[idx] = CoupledContactZeroSpatial();
	if (idx < g_coupled_contact.articulation_count)
	{
		g_coupled_contact_tree_selection[idx] = 0u;
		g_coupled_contact_tree_results[idx] = 1u;
	}
	if (idx == 0)
	{
		GpuCoupledContactState state;
		state.valid = 1u;
		state.failure_flags = 0u;
		state.pad0 = 0u;
		state.pad1 = 0u;
		g_coupled_contact_state[0] = state;
	}
}

// Prepare exact-self contact blocks and deterministic endpoint keys for every proxy-touching contact.
numthreads(CSPrepareCoupledContacts, ConstraintThreadCount, 1, 1)
void CSPrepareCoupledContacts(int3 DTID(dtid))
{
	int contact_idx = CoupledContactThreadIndex(dtid);
	if (contact_idx >= CoupledContactCount())
		return;

	GpuResolveContact contact = g_coupled_contact_contacts[contact_idx];
	if (
		contact.body_idx_a < 0 || contact.body_idx_a >= g_coupled_contact.body_count ||
		contact.body_idx_b < 0 || contact.body_idx_b >= g_coupled_contact.body_count ||
		(!CoupledContactProxyBody(contact.body_idx_a) && !CoupledContactProxyBody(contact.body_idx_b)))
		return;

	// Reject malformed proxy suffix mappings before any articulation resource is indexed.
	if (
		(CoupledContactProxyBody(contact.body_idx_a) && CoupledContactProxyLink(contact.body_idx_a) < 0) ||
		(CoupledContactProxyBody(contact.body_idx_b) && CoupledContactProxyLink(contact.body_idx_b) < 0))
	{
		CoupledContactFail(CoupledContactFailureTopology);
		return;
	}

	int participant_a = CoupledContactParticipant(contact.body_idx_a);
	int participant_b = CoupledContactParticipant(contact.body_idx_b);
	uint target_a = CoupledContactTarget(contact.body_idx_a);
	uint target_b = CoupledContactTarget(contact.body_idx_b);
	if (participant_a >= 0)
		InterlockedAdd(g_coupled_contact_participant_degrees[participant_a], 1u);
	if (participant_b >= 0)
		InterlockedAdd(g_coupled_contact_participant_degrees[participant_b], 1u);
	g_coupled_contact_endpoint_keys[2 * contact_idx + 0] = target_a;
	g_coupled_contact_endpoint_keys[2 * contact_idx + 1] = target_b;

	// A contact with no movable endpoint has no velocity response and remains intentionally inactive.
	if (participant_a < 0 && participant_b < 0)
		return;

	GpuRigidBody body_a = g_coupled_contact_bodies[contact.body_idx_a];
	float3 point_ws = mul(float4(contact.contact_point.xyz, 1.0f), body_a.o2w).xyz;
	float3x3 response = CoupledContactResponseMatrix(contact, point_ws);
	if (
		!CoupledContactVectorFinite(response[0]) ||
		!CoupledContactVectorFinite(response[1]) ||
		!CoupledContactVectorFinite(response[2]))
		return;

	// Only normal mobility is required; locked tangent directions must not invalidate a solvable contact.
	float normal_response = dot(contact.axis.xyz, mul(response, contact.axis.xyz));
	if (!(normal_response > 0.0f))
		return;

	// Use the response's actual scale rather than an absolute mass or inertia threshold.
	float tangent_trace = max(response[0][0] + response[1][1] + response[2][2] - normal_response, 0.0f);
	float inverse_normal_response = 1.0f / normal_response;
	float inverse_tangent_trace = tangent_trace > 0.0f ? 1.0f / tangent_trace : 0.0f;
	if (!isfinite(inverse_normal_response) || !isfinite(inverse_tangent_trace))
		return;

	// Restitution is captured before warm starting so repeated outer sweeps converge to one impact target instead of rebounding repeatedly.
	float3 relative_velocity = CoupledContactRelativeVelocity(contact, point_ws);
	float closing_speed = dot(relative_velocity, contact.axis.xyz);
	GpuMaterial material_a = g_coupled_contact_materials[contact.mat_id_a];
	GpuMaterial material_b = g_coupled_contact_materials[contact.mat_id_b];
	float rest_factor = saturate(abs(closing_speed) * 10.0f);
	float elasticity = g_coupled_contact.restitution_scale * rest_factor * 0.5f * (material_a.elasticity_norm + material_b.elasticity_norm);
	float friction_ratio = min(sqrt(material_a.friction_static * material_b.friction_static), 0.9999f);

	GpuCoupledContactBlock block;
	block.response_0 = float4(response[0], inverse_normal_response);
	block.response_1 = float4(response[1], inverse_tangent_trace);
	block.response_2 = float4(response[2], 0.0f);
	block.target_normal_speed = closing_speed < 0.0f ? -elasticity * closing_speed : 0.0f;
	block.friction = friction_ratio / (1.000001f - friction_ratio);
	block.participant_a = participant_a;
	block.participant_b = participant_b;
	g_coupled_contact_blocks[contact_idx] = block;

	GpuCoupledContactScratch scratch;
	scratch.candidate_impulse = float4(0.0f, 0.0f, 0.0f, 1.0f);
	scratch.position_impulse = float4(0.0f, 0.0f, 0.0f, 0.0f);
	scratch.candidate_position_impulse = float4(0.0f, 0.0f, 0.0f, 0.0f);
	g_coupled_contact_scratch[contact_idx] = scratch;
}

// Clear detached position-only accumulators once before the fixed-configuration push-out sweeps.
numthreads(CSPrepareCoupledContactPosition, ConstraintThreadCount, 1, 1)
void CSPrepareCoupledContactPosition(int3 DTID(dtid))
{
	int idx = CoupledContactThreadIndex(dtid);
	if (idx >= g_coupled_contact.work_count)
		return;

	if (idx < CoupledContactCount())
	{
		GpuCoupledContactScratch scratch = g_coupled_contact_scratch[idx];
		scratch.position_impulse = float4(0.0f, 0.0f, 0.0f, 0.0f);
		scratch.candidate_position_impulse = float4(0.0f, 0.0f, 0.0f, 0.0f);
		g_coupled_contact_scratch[idx] = scratch;
	}
	if (idx < g_coupled_contact.rigid_body_count)
	{
		GpuConstraintPseudoVelocity pseudo;
		pseudo.angular_velocity = float4(0.0f, 0.0f, 0.0f, 0.0f);
		pseudo.linear_velocity = float4(0.0f, 0.0f, 0.0f, 0.0f);
		g_coupled_contact_rigid_pseudo[idx] = pseudo;
	}
	if (idx < g_coupled_contact.link_count)
		g_coupled_contact_link_pseudo[idx] = CoupledContactZeroSpatial();
	if (idx < g_coupled_contact.velocity_delta_count)
		g_coupled_contact_generalized_pseudo[idx] = 0.0f;
}

// Reset detached response streams while retaining prepared topology, degrees, and contact blocks.
numthreads(CSBeginCoupledContactTransaction, ConstraintThreadCount, 1, 1)
void CSBeginCoupledContactTransaction(int3 DTID(dtid))
{
	int idx = CoupledContactThreadIndex(dtid);
	if (idx >= g_coupled_contact.work_count)
		return;

	if (idx < g_coupled_contact.target_count)
		g_coupled_contact_target_impulses[idx] = CoupledContactZeroSpatial();
	if (idx < g_coupled_contact.link_count)
		g_coupled_contact_link_impulses[idx] = CoupledContactZeroSpatial();
	if (idx < g_coupled_contact.articulation_count)
	{
		uint participates = g_coupled_contact_participant_degrees[g_coupled_contact.rigid_body_count + idx] != 0u ? 1u : 0u;
		g_coupled_contact_tree_selection[idx] = participates;
		g_coupled_contact_tree_results[idx] = 1u;
	}
	if (idx == 0)
	{
		GpuCoupledContactState state;
		state.valid = 1u;
		state.failure_flags = 0u;
		state.pad0 = 0u;
		state.pad1 = 0u;
		g_coupled_contact_state[0] = state;
	}
}

// Build detached contributions from the shared previous-frame contact cache.
numthreads(CSBuildCoupledContactWarmStart, ConstraintThreadCount, 1, 1)
void CSBuildCoupledContactWarmStart(int3 DTID(dtid))
{
	int contact_idx = CoupledContactThreadIndex(dtid);
	if (contact_idx >= CoupledContactCount())
		return;

	GpuCoupledContactScratch scratch = g_coupled_contact_scratch[contact_idx];
	if (scratch.candidate_impulse.w == 0.0f)
		return;

	GpuResolveContact contact = g_coupled_contact_contacts[contact_idx];

	// A cache seed must not inject motion into a sleeper before a fresh contact impulse decides whether to wake it.
	float3 impulse = float3(0.0f, 0.0f, 0.0f);
	if (!CoupledContactSleepingTree(contact.body_idx_a) && !CoupledContactSleepingTree(contact.body_idx_b))
		impulse = CoupledContactProjectImpulse(contact.warmstart_impulse.xyz, contact.axis.xyz, g_coupled_contact_blocks[contact_idx].friction);

	scratch.candidate_impulse.xyz = impulse;
	g_coupled_contact_scratch[contact_idx] = scratch;
	if (!CoupledContactPublishContributions(contact_idx, contact, impulse))
		CoupledContactFail(CoupledContactFailureNonFinite);
}

// Build one degree-damped projected block-Jacobi impulse candidate.
numthreads(CSBuildCoupledContactCandidates, ConstraintThreadCount, 1, 1)
void CSBuildCoupledContactCandidates(int3 DTID(dtid))
{
	int contact_idx = CoupledContactThreadIndex(dtid);
	if (contact_idx >= CoupledContactCount())
		return;

	GpuCoupledContactScratch scratch = g_coupled_contact_scratch[contact_idx];
	if (scratch.candidate_impulse.w == 0.0f)
		return;

	GpuResolveContact contact = g_coupled_contact_contacts[contact_idx];
	GpuCoupledContactBlock block = g_coupled_contact_blocks[contact_idx];
	GpuRigidBody body_a = g_coupled_contact_bodies[contact.body_idx_a];
	float3 point_ws = mul(float4(contact.contact_point.xyz, 1.0f), body_a.o2w).xyz;
	float3 relative_velocity = CoupledContactRelativeVelocity(contact, point_ws);
	float3 target_velocity = block.target_normal_speed * contact.axis.xyz;
	float3 residual = relative_velocity - target_velocity;
	// Partition simultaneous physical impulses by maximum endpoint degree to bound additive responses through shared bodies and trees.
	uint degree_a = block.participant_a >= 0 ? g_coupled_contact_participant_degrees[block.participant_a] : 0u;
	uint degree_b = block.participant_b >= 0 ? g_coupled_contact_participant_degrees[block.participant_b] : 0u;
	float damping = g_coupled_contact.relaxation / max(1.0f, (float)max(degree_a, degree_b));
	float3 old_impulse = contact.warmstart_impulse.xyz;
	float3 candidate = old_impulse + damping * (CoupledContactSolveImpulse(block, old_impulse, residual, contact.axis.xyz) - old_impulse);
	float3 impulse_delta = candidate - old_impulse;
	if (!CoupledContactVectorFinite(relative_velocity) || !CoupledContactVectorFinite(candidate) || !CoupledContactVectorFinite(impulse_delta))
	{
		CoupledContactFail(CoupledContactFailureNonFinite);
		return;
	}

	scratch.candidate_impulse.xyz = candidate;
	g_coupled_contact_scratch[contact_idx] = scratch;
	if (!CoupledContactPublishContributions(contact_idx, contact, impulse_delta))
		CoupledContactFail(CoupledContactFailureNonFinite);
}

// Build one detached position-level candidate without modifying physical momenta or generalized velocities.
numthreads(CSBuildCoupledContactPositionCandidates, ConstraintThreadCount, 1, 1)
void CSBuildCoupledContactPositionCandidates(int3 DTID(dtid))
{
	int contact_idx = CoupledContactThreadIndex(dtid);
	if (contact_idx >= CoupledContactCount())
		return;

	GpuCoupledContactScratch scratch = g_coupled_contact_scratch[contact_idx];
	if (scratch.candidate_impulse.w == 0.0f)
		return;

	GpuResolveContact contact = g_coupled_contact_contacts[contact_idx];
	GpuCoupledContactBlock block = g_coupled_contact_blocks[contact_idx];
	GpuRigidBody body_a = g_coupled_contact_bodies[contact.body_idx_a];
	float3 point_ws = mul(float4(contact.contact_point.xyz, 1.0f), body_a.o2w).xyz;
	float3 relative_velocity = CoupledContactPseudoRelativeVelocity(contact, point_ws);
	float correction_depth = max(contact.depth - g_coupled_contact.position_slop, 0.0f);
	float target_speed = min(
		g_coupled_contact.max_position_speed,
		g_coupled_contact.position_beta * correction_depth / g_coupled_contact.dt);
	float3 old_impulse = scratch.position_impulse.xyz;
	uint degree_a = block.participant_a >= 0 ? g_coupled_contact_participant_degrees[block.participant_a] : 0u;
	uint degree_b = block.participant_b >= 0 ? g_coupled_contact_participant_degrees[block.participant_b] : 0u;
	float participant_degree = max(1.0f, (float)max(degree_a, degree_b));

	// Accelerate only the zero-pseudo-velocity first sweep; later contacts need conservative damping even when their accumulated impulse is still zero.
	float damping = g_coupled_contact.position_iteration_index == 0
		? g_coupled_contact.relaxation * min(1.0f, 1.0f / max(g_coupled_contact.position_beta * participant_degree, 1.0f))
		: g_coupled_contact.relaxation / participant_degree;
	float normal_residual = dot(relative_velocity, contact.axis.xyz) - target_speed;
	float3 candidate = max(dot(old_impulse, contact.axis.xyz) - damping * block.response_0.w * normal_residual, 0.0f) * contact.axis.xyz;

	// Retain accepted normal separation within the fixed collision manifold.
	candidate = max(dot(candidate, contact.axis.xyz), dot(old_impulse, contact.axis.xyz)) * contact.axis.xyz;
	float3 impulse_delta = candidate - old_impulse;
	if (
		!CoupledContactVectorFinite(relative_velocity) ||
		!isfinite(target_speed) ||
		!CoupledContactVectorFinite(candidate) ||
		!CoupledContactVectorFinite(impulse_delta))
	{
		CoupledContactFail(CoupledContactFailureNonFinite);
		return;
	}

	scratch.candidate_position_impulse = float4(candidate, 0.0f);
	g_coupled_contact_scratch[contact_idx] = scratch;
	if (!CoupledContactPublishContributions(contact_idx, contact, impulse_delta))
		CoupledContactFail(CoupledContactFailureNonFinite);
}

// Deterministically reduce sorted endpoint contributions and populate rigid targets plus the compact link impulse stream.
numthreads(CSGatherCoupledContactTargets, ConstraintThreadCount, 1, 1)
void CSGatherCoupledContactTargets(int3 DTID(dtid))
{
	int contact_idx = CoupledContactThreadIndex(dtid);
	int contact_count = CoupledContactCount();
	if (contact_idx >= contact_count)
		return;

	// The inactive suffix may retain an older ordering and must never enter this reduction.
	int base_position = 2 * contact_idx;
	int endpoint_count = 2 * contact_count;
	for (int offset = 0; offset != 2; ++offset)
	{
		int position = base_position + offset;
		if (position >= endpoint_count)
			continue;

		uint key = g_coupled_contact_endpoint_keys[position];
		if (key == CoupledContactInvalidKey || key >= (uint)g_coupled_contact.target_count)
			continue;
		if (position != 0 && g_coupled_contact_endpoint_keys[position - 1] == key)
			continue;

		// One segment leader sums equal keys in radix order, making every target reduction independent of thread scheduling.
		GpuArticulationSpatialVector impulse = CoupledContactZeroSpatial();
		for (int segment_position = position; segment_position != endpoint_count; ++segment_position)
		{
			if (g_coupled_contact_endpoint_keys[segment_position] != key)
				break;

			uint contribution_idx = g_coupled_contact_endpoint_order[segment_position];
			if (contribution_idx >= (uint)endpoint_count)
			{
				CoupledContactFail(CoupledContactFailureTopology);
				return;
			}
			GpuArticulationSpatialVector contribution = g_coupled_contact_contributions[contribution_idx];
			impulse.ang += contribution.ang;
			impulse.lin += contribution.lin;
		}
		if (!CoupledContactSpatialFinite(impulse))
		{
			CoupledContactFail(CoupledContactFailureNonFinite);
			return;
		}

		g_coupled_contact_target_impulses[key] = impulse;
		if (key < (uint)g_coupled_contact.rigid_body_count)
		{
			GpuRigidBody body = g_coupled_contact_bodies[key];
			if (g_coupled_contact.phase == CoupledContactPhasePosition)
			{
				GpuConstraintPseudoVelocity current = g_coupled_contact_rigid_pseudo[key];
				float3 angular_velocity = current.angular_velocity.xyz + mul(CoupledContactRigidInverseInertia(key), impulse.ang.xyz);
				float3 linear_velocity = current.linear_velocity.xyz + body.os_com_and_invmass.w * impulse.lin.xyz;
				if (!CoupledContactAngularDisplacementValid(angular_velocity) || !CoupledContactVectorFinite(linear_velocity))
					CoupledContactFail(CoupledContactFailureNonFinite);
			}
			else if (
				!CoupledContactVectorFinite(body.momentum_ang.xyz + impulse.ang.xyz) ||
				!CoupledContactVectorFinite(body.momentum_lin.xyz + impulse.lin.xyz))
			{
				CoupledContactFail(CoupledContactFailureNonFinite);
			}
		}
		else
		{
			uint link_idx = key - (uint)g_coupled_contact.rigid_body_count;
			if (link_idx >= (uint)g_coupled_contact.link_count)
			{
				CoupledContactFail(CoupledContactFailureTopology);
				return;
			}
			g_coupled_contact_link_impulses[link_idx] = impulse;
		}
	}
}

// Validate prospective complete-tree pseudo state before any rigid or articulation position state is committed.
numthreads(CSValidateCoupledContactPositionTrees, ConstraintThreadCount, 1, 1)
void CSValidateCoupledContactPositionTrees(int3 DTID(dtid))
{
	int articulation_idx = CoupledContactThreadIndex(dtid);
	if (articulation_idx >= g_coupled_contact.articulation_count)
		return;
	if (g_coupled_contact_tree_selection[articulation_idx] == 0u)
		return;
	if (g_coupled_contact_tree_results[articulation_idx] == 0u)
	{
		CoupledContactFail(CoupledContactFailureNonFinite);
		return;
	}

	GpuArticulationMobilityRange range = g_coupled_contact_ranges[articulation_idx];
	int velocity_end = articulation_idx + 1 < g_coupled_contact.articulation_count
		? g_coupled_contact_ranges[articulation_idx + 1].velocity_delta_offset
		: g_coupled_contact.velocity_delta_count;
	if (
		range.articulation_index < 0 ||
		range.articulation_index >= g_coupled_contact.articulation_count ||
		range.mobility_offset < 0 ||
		range.link_count < 0 ||
		range.mobility_offset + range.link_count > g_coupled_contact.link_count ||
		range.velocity_delta_offset < 0 ||
		velocity_end < range.velocity_delta_offset ||
		velocity_end > g_coupled_contact.velocity_delta_count)
	{
		CoupledContactFail(CoupledContactFailureTopology);
		return;
	}

	for (int local_link_idx = 0; local_link_idx != range.link_count; ++local_link_idx)
	{
		int link_idx = range.mobility_offset + local_link_idx;
		GpuArticulationSpatialVector prospective = CoupledContactAddSpatial(
			g_coupled_contact_link_pseudo[link_idx],
			g_coupled_contact_articulation_work[link_idx]);
		if (!CoupledContactSpatialFinite(prospective) || !CoupledContactAngularDisplacementValid(prospective.ang.xyz))
		{
			CoupledContactFail(CoupledContactFailureNonFinite);
			return;
		}
	}
	for (int velocity_idx = range.velocity_delta_offset; velocity_idx != velocity_end; ++velocity_idx)
	{
		if (!isfinite(g_coupled_contact_generalized_pseudo[velocity_idx] + g_coupled_contact_velocity_deltas[velocity_idx]))
		{
			CoupledContactFail(CoupledContactFailureNonFinite);
			return;
		}
	}

	GpuArticulation articulation = g_coupled_contact_articulations[range.articulation_index];
	if (articulation.root_type == GpuArticulationRootType_Floating)
	{
		if (velocity_end - range.velocity_delta_offset < 6)
		{
			CoupledContactFail(CoupledContactFailureTopology);
			return;
		}

		float3 root_angular_velocity = float3(
			g_coupled_contact_generalized_pseudo[range.velocity_delta_offset + 0] + g_coupled_contact_velocity_deltas[range.velocity_delta_offset + 0],
			g_coupled_contact_generalized_pseudo[range.velocity_delta_offset + 1] + g_coupled_contact_velocity_deltas[range.velocity_delta_offset + 1],
			g_coupled_contact_generalized_pseudo[range.velocity_delta_offset + 2] + g_coupled_contact_velocity_deltas[range.velocity_delta_offset + 2]);
		if (!CoupledContactAngularDisplacementValid(root_angular_velocity))
			CoupledContactFail(CoupledContactFailureNonFinite);
	}
}

// Fold every selected impulse-ABA validity result into the global contact transaction.
numthreads(CSValidateCoupledContactTrees, ConstraintThreadCount, 1, 1)
void CSValidateCoupledContactTrees(int3 DTID(dtid))
{
	int articulation_idx = CoupledContactThreadIndex(dtid);
	if (articulation_idx >= g_coupled_contact.articulation_count)
		return;
	if (g_coupled_contact_tree_selection[articulation_idx] != 0u && g_coupled_contact_tree_results[articulation_idx] == 0u)
		CoupledContactFail(CoupledContactFailureNonFinite);
}

// Select either every participating tree or none so rigid and articulation state commit atomically.
numthreads(CSSelectCoupledContactTrees, ConstraintThreadCount, 1, 1)
void CSSelectCoupledContactTrees(int3 DTID(dtid))
{
	int articulation_idx = CoupledContactThreadIndex(dtid);
	if (articulation_idx >= g_coupled_contact.articulation_count)
		return;
	g_coupled_contact_tree_selection[articulation_idx] &=
		g_coupled_contact_state[0].valid != 0u ? 1u : 0u;
}

// Commit a complete finite transaction to rigid physical or detached pseudo state and contact accumulators.
numthreads(CSCommitCoupledContacts, ConstraintThreadCount, 1, 1)
void CSCommitCoupledContacts(int3 DTID(dtid))
{
	int idx = CoupledContactThreadIndex(dtid);
	if (idx >= g_coupled_contact.work_count)
		return;

	bool valid = g_coupled_contact_state[0].valid != 0u;
	if (idx < g_coupled_contact.rigid_body_count && valid)
	{
		GpuArticulationSpatialVector impulse = g_coupled_contact_target_impulses[idx];
		bool has_finite_impulse =
			CoupledContactSpatialFinite(impulse) &&
			(any(impulse.ang.xyz != float3(0.0f, 0.0f, 0.0f)) || any(impulse.lin.xyz != float3(0.0f, 0.0f, 0.0f)));
		if (g_coupled_contact.phase == CoupledContactPhasePosition)
		{
			if (has_finite_impulse)
			{
				GpuRigidBody body = g_coupled_contact_bodies[idx];
				GpuConstraintPseudoVelocity pseudo = g_coupled_contact_rigid_pseudo[idx];
				pseudo.angular_velocity.xyz += mul(CoupledContactRigidInverseInertia(idx), impulse.ang.xyz);
				pseudo.linear_velocity.xyz += body.os_com_and_invmass.w * impulse.lin.xyz;
				g_coupled_contact_rigid_pseudo[idx] = pseudo;
			}
		}
		else if (g_coupled_contact_participant_degrees[idx] != 0u)
		{
			// The rigid target thread is the sole body writer, preventing contact-index flag updates from racing the momentum commit.
			GpuRigidBody body = g_coupled_contact_bodies[idx];
			if (has_finite_impulse)
			{
				body.momentum_ang.xyz += impulse.ang.xyz;
				body.momentum_lin.xyz += impulse.lin.xyz;
			}
			CoupledContactWakeRigid(body);
			g_coupled_contact_bodies[idx] = body;
		}
	}

	// Link targets may share a root; atomic flag updates do not race ordinary-body momentum writes.
	if (valid && idx >= g_coupled_contact.rigid_body_count && idx < g_coupled_contact.target_count)
	{
		GpuArticulationSpatialVector impulse = g_coupled_contact_target_impulses[idx];
		if (CoupledContactSpatialFinite(impulse) &&
			(any(impulse.ang.xyz != float3(0.0f, 0.0f, 0.0f)) || any(impulse.lin.xyz != float3(0.0f, 0.0f, 0.0f))))
		{
			int root_body_idx = CoupledContactRootProxy(idx - g_coupled_contact.rigid_body_count);
			InterlockedAnd(g_coupled_contact_bodies[root_body_idx].state_flags, ~ERigidBodyStateFlags_Sleeping);
		}
	}

	if (idx < CoupledContactCount())
	{
		GpuCoupledContactScratch scratch = g_coupled_contact_scratch[idx];
		if (scratch.candidate_impulse.w == 0.0f)
			return;

		GpuResolveContact contact = g_coupled_contact_contacts[idx];
		if (!valid)
		{
			if (g_coupled_contact.phase == CoupledContactPhaseWarmStart)
				contact.warmstart_impulse = float4(0.0f, 0.0f, 0.0f, 0.0f);
		}
		else if (g_coupled_contact.phase == CoupledContactPhaseVelocity || g_coupled_contact.phase == CoupledContactPhaseWarmStart)
		{
			contact.warmstart_impulse = float4(scratch.candidate_impulse.xyz, 0.0f);
		}
		else if (g_coupled_contact.phase == CoupledContactPhasePosition)
		{
			scratch.position_impulse = scratch.candidate_position_impulse;
			g_coupled_contact_scratch[idx] = scratch;
		}

		g_coupled_contact_contacts[idx] = contact;
	}

}

// Accumulate accepted complete-tree link and generalized deltas into contact-owned detached pseudo state.
numthreads(CSCommitCoupledContactPositionArticulations, ConstraintThreadCount, 1, 1)
void CSCommitCoupledContactPositionArticulations(int3 DTID(dtid))
{
	int articulation_idx = CoupledContactThreadIndex(dtid);
	if (
		articulation_idx >= g_coupled_contact.articulation_count ||
		g_coupled_contact_state[0].valid == 0u ||
		g_coupled_contact_tree_selection[articulation_idx] == 0u ||
		g_coupled_contact_tree_results[articulation_idx] == 0u)
		return;

	GpuArticulationMobilityRange range = g_coupled_contact_ranges[articulation_idx];
	int velocity_end = articulation_idx + 1 < g_coupled_contact.articulation_count
		? g_coupled_contact_ranges[articulation_idx + 1].velocity_delta_offset
		: g_coupled_contact.velocity_delta_count;
	for (int local_link_idx = 0; local_link_idx != range.link_count; ++local_link_idx)
	{
		int link_idx = range.mobility_offset + local_link_idx;
		g_coupled_contact_link_pseudo[link_idx] = CoupledContactAddSpatial(
			g_coupled_contact_link_pseudo[link_idx],
			g_coupled_contact_articulation_work[link_idx]);
	}
	for (int velocity_idx = range.velocity_delta_offset; velocity_idx != velocity_end; ++velocity_idx)
		g_coupled_contact_generalized_pseudo[velocity_idx] += g_coupled_contact_velocity_deltas[velocity_idx];
}

// Integrate a validated body-frame floating-root pseudo twist with midpoint translation.
GpuConstraintFrame CoupledContactIntegrateRoot(GpuConstraintFrame root_to_world, int velocity_offset)
{
	float3 angular_velocity = float3(
		g_coupled_contact_generalized_pseudo[velocity_offset + 0],
		g_coupled_contact_generalized_pseudo[velocity_offset + 1],
		g_coupled_contact_generalized_pseudo[velocity_offset + 2]);
	float3 linear_velocity = float3(
		g_coupled_contact_generalized_pseudo[velocity_offset + 3],
		g_coupled_contact_generalized_pseudo[velocity_offset + 4],
		g_coupled_contact_generalized_pseudo[velocity_offset + 5]);
	float3 angular_displacement = g_coupled_contact.dt * angular_velocity;
	float4 half_rotation = quat_exp(0.25f * angular_displacement);
	float4 full_rotation = quat_exp(0.5f * angular_displacement);
	float4 midpoint_rotation = quat_mul(root_to_world.rotation, half_rotation);

	GpuConstraintFrame integrated = root_to_world;
	integrated.position = float4(
		root_to_world.position.xyz + quat_rotate(midpoint_rotation, g_coupled_contact.dt * linear_velocity),
		1.0f);
	integrated.rotation = normalize(quat_mul(root_to_world.rotation, full_rotation));
	return integrated;
}

// Integrate converged contact pseudo state exactly once into rigid transforms and articulation coordinates.
numthreads(CSApplyCoupledContactPosition, ConstraintThreadCount, 1, 1)
void CSApplyCoupledContactPosition(int3 DTID(dtid))
{
	int idx = CoupledContactThreadIndex(dtid);
	if (idx < g_coupled_contact.rigid_body_count && CoupledContactRigidDynamic(idx))
	{
		GpuConstraintPseudoVelocity pseudo = g_coupled_contact_rigid_pseudo[idx];
		if (
			(any(pseudo.angular_velocity.xyz != float3(0.0f, 0.0f, 0.0f)) || any(pseudo.linear_velocity.xyz != float3(0.0f, 0.0f, 0.0f))) &&
			CoupledContactAngularDisplacementValid(pseudo.angular_velocity.xyz) &&
			CoupledContactVectorFinite(pseudo.linear_velocity.xyz))
		{
			GpuRigidBody body = g_coupled_contact_bodies[idx];
			float3 com_ws = body.o2w[3].xyz + mul(body.os_com_and_invmass.xyz, (float3x3)body.o2w);
			float3x3 rotation = (float3x3)body.o2w;
			float3x3 new_rotation = orthonorm3x3(mul(rotation, rodrigues_rotation(g_coupled_contact.dt * pseudo.angular_velocity.xyz)));
			float3 new_com_ws = com_ws + g_coupled_contact.dt * pseudo.linear_velocity.xyz;
			float3 new_position = new_com_ws - mul(body.os_com_and_invmass.xyz, new_rotation);
			body.o2w = float4x4(
				float4(new_rotation[0], 0.0f),
				float4(new_rotation[1], 0.0f),
				float4(new_rotation[2], 0.0f),
				float4(new_position, 1.0f));
			g_coupled_contact_bodies[idx] = body;
		}
	}

	if (idx >= g_coupled_contact.articulation_count)
		return;

	GpuArticulationMobilityRange range = g_coupled_contact_ranges[idx];
	GpuArticulation articulation = g_coupled_contact_articulations[range.articulation_index];
	int velocity_end = idx + 1 < g_coupled_contact.articulation_count
		? g_coupled_contact_ranges[idx + 1].velocity_delta_offset
		: g_coupled_contact.velocity_delta_count;
	bool has_correction = false;
	for (int velocity_idx = range.velocity_delta_offset; velocity_idx != velocity_end; ++velocity_idx)
		has_correction = has_correction || g_coupled_contact_generalized_pseudo[velocity_idx] != 0.0f;
	if (!has_correction)
		return;

	GpuConstraintFrame root_to_world = articulation.root_to_world;
	if (articulation.root_type == GpuArticulationRootType_Floating)
	{
		if (
			velocity_end - range.velocity_delta_offset < 6 ||
			!CoupledContactAngularDisplacementValid(float3(
				g_coupled_contact_generalized_pseudo[range.velocity_delta_offset + 0],
				g_coupled_contact_generalized_pseudo[range.velocity_delta_offset + 1],
				g_coupled_contact_generalized_pseudo[range.velocity_delta_offset + 2])))
			return;

		root_to_world = CoupledContactIntegrateRoot(root_to_world, range.velocity_delta_offset);
		if (
			!isfinite(root_to_world.rotation.x) || !isfinite(root_to_world.rotation.y) ||
			!isfinite(root_to_world.rotation.z) || !isfinite(root_to_world.rotation.w) ||
			!CoupledContactVectorFinite(root_to_world.position.xyz))
			return;
	}

	// Validate every reduced coordinate before writing any part of this articulation.
	for (int local_link_idx = 1; local_link_idx != articulation.link_count; ++local_link_idx)
	{
		GpuArticulationLink link = g_coupled_contact_links[articulation.link_offset + local_link_idx];
		for (int row = 0; row != link.dof_count; ++row)
		{
			int local_velocity_idx = link.velocity_offset - articulation.velocity_offset + row;
			float position =
				g_coupled_contact_positions[link.position_offset + row] +
				g_coupled_contact.dt * g_coupled_contact_generalized_pseudo[range.velocity_delta_offset + local_velocity_idx];
			if (!isfinite(position))
				return;
		}
	}

	for (int local_link_idx = 1; local_link_idx != articulation.link_count; ++local_link_idx)
	{
		GpuArticulationLink link = g_coupled_contact_links[articulation.link_offset + local_link_idx];
		for (int row = 0; row != link.dof_count; ++row)
		{
			int local_velocity_idx = link.velocity_offset - articulation.velocity_offset + row;
			g_coupled_contact_positions[link.position_offset + row] +=
				g_coupled_contact.dt * g_coupled_contact_generalized_pseudo[range.velocity_delta_offset + local_velocity_idx];
		}
	}
	articulation.root_to_world = root_to_world;
	g_coupled_contact_articulations[range.articulation_index] = articulation;
}

#ifdef __cplusplus
}
#endif
