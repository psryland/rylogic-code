//*********************************************
// Physics Engine — Articulation Link Proxies
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/quaternions.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "physics/src/compute/physics_types.hlsli"

#ifdef __cplusplus
namespace pr::physics {
#endif

// Active forest bounds and broadphase settings shared by proxy force and refresh passes.
struct cbArticulationLinkProxies
{
	int articulation_count;
	int link_count;
	int broadphase_sort_axis;
	float broadphase_aabb_margin;
};

ConstantBuffer<cbArticulationLinkProxies> resource(g_proxy, b0);
StructuredBuffer<GpuArticulationLink> resource(g_aba_links, t0);
StructuredBuffer<GpuArticulationDof> resource(g_aba_dofs, t1);
StructuredBuffer<GpuFrameForce> resource(g_aba_external_forces, t2);
StructuredBuffer<float> resource(g_aba_forces, t3);
StructuredBuffer<uint> resource(g_aba_children, t4);
RWStructuredBuffer<GpuArticulation> resource(g_aba_articulations, u0);
RWStructuredBuffer<float> resource(g_aba_positions, u1);
RWStructuredBuffer<float> resource(g_aba_velocities, u2);
RWStructuredBuffer<GpuArticulationAbaScratch> resource(g_aba_scratch, u3);
RWStructuredBuffer<GpuArticulationAbaDofScratch> resource(g_aba_dof_scratch, u4);
RWStructuredBuffer<GpuRigidBody> resource(g_proxy_bodies, u5);
RWStructuredBuffer<int> resource(g_proxy_aabb_idx, u6);
RWStructuredBuffer<float> resource(g_proxy_aabb_sort, u7);
RWStructuredBuffer<BBox> resource(g_proxy_aabb_box, u8);
RWStructuredBuffer<GpuFrameForce> resource(g_proxy_external_forces, u9);
RWStructuredBuffer<float> resource(g_aba_accelerations, u10);
RWStructuredBuffer<float> resource(g_aba_inverse_joint_inertia, u11);
RWStructuredBuffer<GpuConstraintFrame> resource(g_link_to_world, u12);

#ifdef __cplusplus
}
#endif

// Reuse the canonical articulation transform, joint, inertia, and spatial-vector operations.
#define PR_ARTICULATION_ABA_CUSTOM_RESOURCES
#define PR_ARTICULATION_ABA_NO_ENTRYPOINTS
#define PR_ARTICULATION_ABA_CPP_NAMESPACE articulation_link_proxy_aba_detail
#include "physics/src/compute/articulation_force_aba.hlsl"
#undef PR_ARTICULATION_ABA_CPP_NAMESPACE
#undef PR_ARTICULATION_ABA_NO_ENTRYPOINTS
#undef PR_ARTICULATION_ABA_CUSTOM_RESOURCES

#ifdef __cplusplus
namespace pr::physics {
using namespace articulation_link_proxy_aba_detail;
#endif

// Convert a proxy's world-space force and centre-of-mass torque into a link-frame wrench at the link origin.
GpuArticulationSpatialVector ProxyLinkWrench(GpuArticulationLink link, GpuRigidBody body)
{
	float4 proxy_to_world_rotation = quat_from_float3x3((float3x3)body.o2w);
	float4 world_to_proxy_rotation = quat_conjugate(proxy_to_world_rotation);
	float3 force_ws = body.force_lin.xyz;
	float3 com_offset_ws = mul(float4(body.os_com_and_invmass.xyz, 0.0f), body.o2w).xyz;
	float3 torque_at_proxy_origin_ws = body.force_ang.xyz + cross(com_offset_ws, force_ws);
	GpuArticulationSpatialVector proxy_wrench = AbaSpatialVector(
		quat_rotate(world_to_proxy_rotation, torque_at_proxy_origin_ws),
		quat_rotate(world_to_proxy_rotation, force_ws));
	return AbaTransformForce(link.shape_to_link, proxy_wrench);
}

// Refresh one link's joint transform and velocity without evaluating force or acceleration state.
void ProxyPrepareKinematics(int link_index)
{
	GpuArticulationLink link = g_aba_links[link_index];
	GpuArticulationAbaScratch scratch = g_aba_scratch[link_index];
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
		scratch.link_velocity = AbaAddSpatial(
			AbaTransformMotion(AbaInvertTransform(scratch.child_to_parent), parent.link_velocity),
			joint_velocity);
		scratch.joint_bias = AbaAddSpatial(
			scratch.joint_bias,
			AbaCrossMotion(scratch.link_velocity, joint_velocity));
	}
	g_aba_scratch[link_index] = scratch;
}

// Write one link's proxy body state in the rigid-module convention while retaining articulation ownership.
void ProxyWriteBody(GpuArticulationLink link, GpuConstraintFrame link_to_world, GpuArticulationSpatialVector link_velocity)
{
	int body_index = link.proxy_body_index;
	GpuRigidBody body = g_proxy_bodies[body_index];
	GpuConstraintFrame proxy_to_world = AbaMultiplyTransform(link_to_world, link.shape_to_link);
	GpuArticulationSpatialVector link_momentum = AbaMultiplySpatialMatrix(AbaPhysicalInertia(link), link_velocity);
	float3 momentum_lin_ws = quat_rotate(link_to_world.rotation, link_momentum.lin.xyz);
	float3 com_offset_ws = quat_rotate(link_to_world.rotation, link.inertia_com_and_mass.xyz);
	float3 momentum_ang_ws =
		quat_rotate(link_to_world.rotation, link_momentum.ang.xyz) -
		cross(com_offset_ws, momentum_lin_ws);

	body.o2w = quat_to_float4x4(proxy_to_world.rotation, proxy_to_world.position.xyz);
	body.momentum_ang = float4(momentum_ang_ws, 0.0f);
	body.momentum_lin = float4(momentum_lin_ws, 0.0f);
	g_proxy_bodies[body_index] = body;

	// Shape-less force proxies retain a degenerate bound but never enter broadphase until coupled collision support enables them.
	BBox ws_bbox;
	if (body.shape_id >= 0)
	{
		ws_bbox = body.os_bbox.Transform(body.o2w);
	}
	else
	{
		ws_bbox.centre = float4(0.0f, 0.0f, 0.0f, 1.0f);
		ws_bbox.radius = float4(0.0f, 0.0f, 0.0f, 0.0f);
	}
	float3 centre = ws_bbox.centre.xyz;
	float3 radius = ws_bbox.radius.xyz;
	float sort_centre = centre.x;
	float sort_radius = radius.x;
	switch (g_proxy.broadphase_sort_axis)
	{
		case 1:
		{
			sort_centre = centre.y;
			sort_radius = radius.y;
			break;
		}
		case 2:
		{
			sort_centre = centre.z;
			sort_radius = radius.z;
			break;
		}
		default:
		{
			break;
		}
	}

	float margin = max(g_proxy.broadphase_aabb_margin, 0.0f);
	g_proxy_aabb_box[body_index] = ws_bbox;
	g_proxy_aabb_sort[2 * body_index + 0] = sort_centre - sort_radius - margin;
	g_proxy_aabb_sort[2 * body_index + 1] = sort_centre + sort_radius + margin;
	g_proxy_aabb_idx[2 * body_index + 0] = (body_index << 1) | 0;
	g_proxy_aabb_idx[2 * body_index + 1] = (body_index << 1) | 1;
}

// Gather one independent proxy accumulator into the matching per-substep ABA external wrench.
numthreads(CSArticulation, ArticulationThreadCount, 1, 1)
void CSArticulationGatherProxyForces(int3 DTID(dtid))
{
	int link_index = dtid.x;
	if (link_index >= g_proxy.link_count)
		return;

	GpuArticulationLink link = g_aba_links[link_index];
	GpuFrameForce baseline = g_aba_external_forces[link_index];
	GpuArticulationSpatialVector proxy_wrench = ProxyLinkWrench(link, g_proxy_bodies[link.proxy_body_index]);
	GpuFrameForce external_force;
	external_force.force_ang = baseline.force_ang + proxy_wrench.ang;
	external_force.force_lin = baseline.force_lin + proxy_wrench.lin;
	g_proxy_external_forces[link_index] = external_force;
}

// Reconstruct final link kinematics in one serial lane per tree and update every hidden proxy.
numthreads(CSArticulation, ArticulationThreadCount, 1, 1)
void CSArticulationRefreshProxies(int3 DTID(dtid))
{
	int articulation_index = dtid.x;
	if (articulation_index >= g_proxy.articulation_count)
		return;

	GpuArticulation articulation = g_aba_articulations[articulation_index];
	for (int local_link_index = 0; local_link_index != articulation.link_count; ++local_link_index)
	{
		int link_index = articulation.link_offset + local_link_index;
		GpuArticulationLink link = g_aba_links[link_index];
		ProxyPrepareKinematics(link_index);

		GpuConstraintFrame link_to_world;
		if (link.parent_link_index < 0)
			link_to_world = articulation.root_to_world;
		else
			link_to_world = AbaMultiplyTransform(g_link_to_world[link.parent_link_index], g_aba_scratch[link_index].child_to_parent);

		g_link_to_world[link_index] = link_to_world;
		ProxyWriteBody(link, link_to_world, g_aba_scratch[link_index].link_velocity);
	}
}

#ifdef __cplusplus
}
#endif
