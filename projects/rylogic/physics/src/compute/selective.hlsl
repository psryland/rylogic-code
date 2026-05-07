//*********************************************
// Physics Engine — Selective Contact Refresh
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Scores resolved contacts, then rebuilds a compact collision-pair work set for
// contacts that still look unstable. The follow-up narrowphase pass refreshes the
// manifolds from current body transforms before the resolver applies more impulses.
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "pr/hlsl/bounding_box.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

struct cbSelectiveRefresh
{
	int max_pairs;
	int max_contacts;
	int body_count;
	int sleeping_enabled;
	int full_max_pairs;
	int pad_i0;
	int pad_i1;
	int pad_i2;

	float depth_slop;
	float support_depth_slop;
	float closing_speed_slop;
	float support_alignment;

	float aabb_margin;
	float pad0;
	float pad1;
	float pad2;
};

ConstantBuffer<cbSelectiveRefresh> resource(g, b0);
RWStructuredBuffer<GpuCollisionCounters> resource(g_dst_counters, u0);
RWStructuredBuffer<GpuCollisionPair> resource(g_dst_pairs, u1);
RWStructuredBuffer<DispatchArguments> resource(g_dst_dispatch_args, u2);
RWStructuredBuffer<uint> resource(g_problem_bodies, u3);
RWStructuredBuffer<GpuSelectiveRefreshMetrics> resource(g_metrics, u4);
RWStructuredBuffer<GpuRigidBody> resource(g_bodies, u5);
StructuredBuffer<GpuCollisionCounters> resource(g_source_counters, t0);
StructuredBuffer<GpuResolveContact> resource(g_source_contacts, t1);
StructuredBuffer<GpuCollisionCounters> resource(g_full_counters, t2);
StructuredBuffer<GpuCollisionPair> resource(g_full_pairs, t3);

bool DynamicBody(in_(GpuRigidBody) body)
{
	return body.os_com_and_invmass.w > 0.0f &&
		!AllSet(body.state_flags, ERigidBodyStateFlags_Static);
}

bool SleepingBody(in_(GpuRigidBody) body)
{
	return g.sleeping_enabled != 0 &&
		AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping);
}

float3x3 OsInverseInertia(in_(GpuRigidBody) body)
{
	float inv_mass = body.os_com_and_invmass.w;
	return inv_mass * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz);
}

float3 BodyVelocityAtPoint(in_(GpuRigidBody) body, float3 pt_in_a, float3 com_in_a, float3x3 rot_a)
{
	float inv_mass = body.os_com_and_invmass.w;
	float3x3 ws_iinv = rotate_inertia_inv(OsInverseInertia(body), (float3x3)body.o2w);
	float3 omega_in_a = mul(rot_a, mul(ws_iinv, body.momentum_ang.xyz));
	float3 v_com_in_a = mul(rot_a, inv_mass * body.momentum_lin.xyz);
	return v_com_in_a + cross(omega_in_a, pt_in_a - com_in_a);
}

float ClosingSpeed(in_(GpuResolveContact) contact, in_(GpuRigidBody) body_a, in_(GpuRigidBody) body_b, float4x4 b2a)
{
	float3x3 rot_a = (float3x3)body_a.o2w;
	float3x3 b2a_rot = (float3x3)b2a;
	float3 com_a_in_a = body_a.os_com_and_invmass.xyz;
	float3 com_b_in_a = b2a[3].xyz + mul(body_b.os_com_and_invmass.xyz, b2a_rot);
	float3 pt = contact.contact_point.xyz;
	float3 v_a = BodyVelocityAtPoint(body_a, pt, com_a_in_a, rot_a);
	float3 v_b = BodyVelocityAtPoint(body_b, pt, com_b_in_a, rot_a);
	return dot(v_b - v_a, contact.axis.xyz);
}

bool SupportContact(in_(GpuResolveContact) contact, in_(GpuRigidBody) body_a)
{
	float3 gravity = NormaliseOrZero(body_a.ws_gravity.xyz);
	if (dot(gravity, gravity) == 0.0f)
		return false;

	float3 axis_ws = mul(contact.axis.xyz, (float3x3)body_a.o2w);
	return abs(dot(axis_ws, gravity)) >= g.support_alignment;
}

BBox InflatedWorldBBox(in_(GpuRigidBody) body)
{
	BBox bbox = BBox_Transform(body.os_bbox, body.o2w);
	bbox.radius.xyz += g.aabb_margin;
	return bbox;
}

numthreads(CSPrepareSelectiveRefresh, SelectiveRefreshThreadCount, 1, 1)
void CSPrepareSelectiveRefresh(int3 DTID(dtid))
{
	int idx = dtid.x;
	if (idx < g.body_count)
		g_problem_bodies[idx] = 0;

	if (idx == 0)
	{
		g_dst_counters[0].pair_count = 0;
		g_dst_counters[0].contact_count = 0;
		g_dst_dispatch_args[0].ThreadGroupCountX = 0;
		g_dst_dispatch_args[0].ThreadGroupCountY = 1;
		g_dst_dispatch_args[0].ThreadGroupCountZ = 1;
		g_metrics[0].scored_contact_count = 0;
		g_metrics[0].selected_contact_count = 0;
		g_metrics[0].selected_pair_count = 0;
		g_metrics[0].pad0 = 0;
	}
}

numthreads(CSScoreSelectiveContacts, SelectiveRefreshThreadCount, 1, 1)
void CSScoreSelectiveContacts(int3 DTID(dtid))
{
	int contact_idx = dtid.x;
	int contact_count = min(g_source_counters[0].contact_count, g.max_contacts);
	if (contact_idx >= contact_count)
		return;

	InterlockedAdd(g_metrics[0].scored_contact_count, 1);

	GpuResolveContact contact = g_source_contacts[contact_idx];
	GpuRigidBody body_a = g_bodies[contact.body_idx_a];
	GpuRigidBody body_b = g_bodies[contact.body_idx_b];
	if (!DynamicBody(body_a) && !DynamicBody(body_b))
		return;
	if (SleepingBody(body_a) && SleepingBody(body_b))
		return;

	float4x4 b2a = mul(body_b.o2w, InvertOrthonormal(body_a.o2w));
	float closing_speed = ClosingSpeed(contact, body_a, body_b, b2a);
	bool support_contact = SupportContact(contact, body_a);
	float depth_slop = support_contact ? g.support_depth_slop : g.depth_slop;
	bool select =
		contact.depth > depth_slop ||
		closing_speed < -g.closing_speed_slop;

	if (!select)
		return;

	InterlockedOr(g_problem_bodies[contact.body_idx_a], 1);
	InterlockedOr(g_problem_bodies[contact.body_idx_b], 1);
	InterlockedAdd(g_metrics[0].selected_contact_count, 1);
}

numthreads(CSCompactSelectivePairs, SelectiveRefreshThreadCount, 1, 1)
void CSCompactSelectivePairs(int3 DTID(dtid))
{
	int pair_idx = dtid.x;
	int pair_count = min(g_full_counters[0].pair_count, g.full_max_pairs);
	if (pair_idx >= pair_count)
		return;

	GpuCollisionPair src_pair = g_full_pairs[pair_idx];
	if (g_problem_bodies[src_pair.body_idx_a] == 0 && g_problem_bodies[src_pair.body_idx_b] == 0)
		return;

	GpuRigidBody body_a = g_bodies[src_pair.body_idx_a];
	GpuRigidBody body_b = g_bodies[src_pair.body_idx_b];
	if (!DynamicBody(body_a) && !DynamicBody(body_b))
		return;
	if (SleepingBody(body_a) && SleepingBody(body_b))
		return;
	if (!BBox_IsIntersection(InflatedWorldBBox(body_a), InflatedWorldBBox(body_b)))
		return;

	uint slot;
	InterlockedAdd(g_dst_counters[0].pair_count, 1, slot);
	if (slot >= g.max_pairs)
		return;

	GpuCollisionPair dst_pair;
	dst_pair.body_idx_a = src_pair.body_idx_a;
	dst_pair.body_idx_b = src_pair.body_idx_b;
	dst_pair.shape_idx_a = body_a.shape_id;
	dst_pair.shape_idx_b = body_b.shape_id;
	dst_pair.b2a = mul(body_b.o2w, InvertOrthonormal(body_a.o2w));
	g_dst_pairs[slot] = dst_pair;
	InterlockedAdd(g_metrics[0].selected_pair_count, 1);
}

numthreads(CSBuildSelectiveDispatch, 1, 1, 1)
void CSBuildSelectiveDispatch(int3 DTID(dtid))
{
	uint pair_count = min(g_dst_counters[0].pair_count, g.max_pairs);
	g_dst_dispatch_args[0].ThreadGroupCountX = (pair_count + CollideThreadCount - 1) / CollideThreadCount;
	g_dst_dispatch_args[0].ThreadGroupCountY = 1;
	g_dst_dispatch_args[0].ThreadGroupCountZ = 1;
}

#ifdef __cplusplus
}
#endif
