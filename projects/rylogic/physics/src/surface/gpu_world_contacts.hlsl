//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/vector.hlsli"
#include "pr/hlsl/spatial_algebra.hlsli"
#include "src/compute/physics_types.hlsli"
#include "pr/physics/surface/surface_sampling.hlsli"
#include "pr/physics/terrain/landscape/baseline_surface.hlsli"

// Patch span and total sample count for one packed primitive; matches GpuWorldContacts::Range.
struct WorldRange
{
	uint begin, patch_count, sample_count, pad;
};

// Packed body, shape, and child identity; matches GpuWorldContacts::Instance.
struct WorldInstance
{
	uint body, shape, child, pad;
};

// A sampled world-space position and its surface normal and nonnegative contact depth.
struct WorldCandidate
{
	float4 position;
	float4 normal_depth;
	uint valid;
};

// Dispatch constants and conservative terrain height bound; matches the host Constants layout.
cbuffer Params : register(b0)
{
	uint substep, endpoint, max_contacts;
	float height_upper;
	int sleeping_enabled, island_count;
	uint mode, boundary_material;
	double centre_x, centre_y, radius;
	float spacing, max_motion;
	float max_penetration, dt;
	uint plan_offset, phase;
};
StructuredBuffer<GpuRigidBody> bodies : register(t0);
StructuredBuffer<GpuShape> shapes : register(t1);
StructuredBuffer<BaselineRecipe> recipes : register(t2);
StructuredBuffer<WorldRange> plans : register(t3);
StructuredBuffer<SurfacePatch> patches : register(t4);
StructuredBuffer<WorldInstance> instances : register(t5);
StructuredBuffer<GpuSleepIsland> sleep_islands : register(t6);
StructuredBuffer<GpuRigidBody> previous_bodies : register(t7);
RWStructuredBuffer<GpuCollisionCounters> counters : register(u0);
RWStructuredBuffer<GpuResolveContact> contacts : register(u1);
RWStructuredBuffer<uint> status : register(u2);

groupshared WorldCandidate batch[64];
groupshared WorldCandidate selected[32];
groupshared float3 normals[8];
groupshared uint cluster_count;

// Refine a float square-root seed with double arithmetic so metre-scale radii retain sub-millimetre radial depth.
double RadialDistance(double2 radial)
{
	double square = radial.x * radial.x + radial.y * radial.y;
	double seed = (double)sqrt((float)square);
	return seed > 0 ? 0.5 * (seed + square / seed) : seed;
}

// Append selected surface contacts for one primitive per group; report invalid queries or excess slope groups through status.
numthreads(CSWorldContacts, 64, 1, 1)
void CSWorldContacts(uint3 group : SV_GroupID, uint lane : SV_GroupIndex)
{
	// Validate the entire primitive's query domain before using the height bound for a conservative rejection.
	WorldInstance instance = instances[group.x];
	GpuRigidBody body = bodies[instance.body];
	GpuShape shape = shapes[instance.shape];
	float4x4 s2w = mul(shape.s2rb, body.o2w);
	BBox bounds = shape.rb_bbox.Transform(body.o2w);
	BaselineRecipe recipe = recipes[0];
	if (!all(isfinite(bounds.centre)) || !all(isfinite(bounds.radius)) || (mode == 0 && (
		abs((double)bounds.centre.x) + bounds.radius.x > recipe.m_supported_coordinate_abs_m ||
		abs((double)bounds.centre.y) + bounds.radius.y > recipe.m_supported_coordinate_abs_m)))
	{
		if (lane == 0) InterlockedOr(status[0], 1);
		return;
	}
	if (mode == 0 && bounds.centre.z - bounds.radius.z > height_upper)
		return;

	// Bound each leaf's horizontal motion, including child offsets and rotation, before rejecting distant boundary queries.
	if (mode != 0)
	{
		GpuRigidBody previous = previous_bodies[instance.body];
		float3 com = mul(float4(body.os_com_and_invmass.xyz, 1), body.o2w).xyz;
		float3x3 inverse_inertia = rotate_inertia_inv(body.os_com_and_invmass.w * build_symmetric_3x3(body.inertia_inv_diagonal.xyz, body.inertia_inv_products.xyz), (float3x3)body.o2w);
		float3 omega = mul(body.momentum_ang.xyz, inverse_inertia);
		float lever = length(bounds.centre.xyz - com) + length(bounds.radius.xyz) + spacing;
		float speed_bound = length(body.momentum_lin.xy * body.os_com_and_invmass.w) + length(omega) * lever;
		float2 old_centre = mul(shape.rb_bbox.centre, previous.o2w).xy;
		float4x4 rotation_delta = body.o2w - previous.o2w;
		float2 dx = mul(float4(1, 0, 0, 0), rotation_delta).xy;
		float2 dy = mul(float4(0, 1, 0, 0), rotation_delta).xy;
		float2 dz = mul(float4(0, 0, 1, 0), rotation_delta).xy;
		float displacement = length(bounds.centre.xy - old_centre) + sqrt(dot(dx, dx) + dot(dy, dy) + dot(dz, dz)) * length(shape.rb_bbox.radius.xyz);
		if (!isfinite(speed_bound) || !isfinite(displacement) || speed_bound * dt > max_motion || displacement > max_motion)
		{
			InterlockedOr(status[0], 8);
			uint claimed;
			InterlockedCompareExchange(status[1], 0, 1, claimed);
			if (claimed == 0)
			{
				status[2] = instance.body;
				status[3] = substep;
				status[4] = phase;
				status[5] = asuint(body.momentum_lin.x * body.os_com_and_invmass.w);
				status[6] = asuint(body.momentum_lin.y * body.os_com_and_invmass.w);
				status[7] = asuint(body.momentum_lin.z * body.os_com_and_invmass.w);
				status[8] = asuint(omega.x);
				status[9] = asuint(omega.y);
				status[10] = asuint(omega.z);
				status[11] = asuint(lever);
				status[12] = asuint(speed_bound * dt);
				status[13] = asuint(displacement);
				status[14] = asuint(length(bounds.centre.xy - old_centre));
				status[15] = asuint(displacement - length(bounds.centre.xy - old_centre));
				status[16] = asuint(dt);
				status[17] = asuint(max_motion);
			}
		}

		// The enclosing XY rectangle proves absence of boundary contact without enumerating the surface.
		double2 furthest = abs((double2)bounds.centre.xy - double2(centre_x, centre_y)) + (double2)bounds.radius.xy;
		double inner_radius = radius - spacing - 0.001;
		if (furthest.x * furthest.x + furthest.y * furthest.y < inner_radius * inner_radius)
			return;
	}

	// Match ordinary broadphase eligibility: unchanged sleeping support must not apply bias impulses, but disturbed islands still need world support.
	if (mode != 2 && sleeping_enabled != 0 && AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping))
	{
		int island_id = body.sleep.island_id;
		if (island_id < 0 || island_id >= island_count ||
			!AnySet(sleep_islands[island_id].flags, GpuSleepIslandFlags_Disturbed))
			return;
	}

	// Stream fixed-size batches while retaining at most eight slope groups with four spatial slots each.
	if (lane < 32) selected[lane] = (WorldCandidate)0;
	if (lane == 0) cluster_count = 0;
	GroupMemoryBarrierWithGroupSync();
	WorldRange plan = plans[instance.shape + plan_offset];
	for (uint base = 0; base < plan.sample_count; base += 64)
	{
		WorldCandidate candidate = (WorldCandidate)0;
		uint ordinal = base + lane;
		if (ordinal < plan.sample_count)
		{
			// Locate the patch owning this ordinal without expanding the compact plan into a point buffer.
			uint lo = 0, hi = plan.patch_count;
			while (lo < hi)
			{
				uint mid = lo + (hi - lo) / 2;
				if (ordinal < patches[plan.begin + mid].m_sample_end) hi = mid;
				else lo = mid + 1;
			}

			// Evaluate the selected world surface at the fully transformed shape sample.
			uint first = lo == 0 ? 0 : patches[plan.begin + lo - 1].m_sample_end;
			SurfaceSample sample = EmitSurfaceSample(patches[plan.begin + lo], ordinal - first);
			float4 position = mul(sample.m_pos_local, s2w);
			float3 normal = float3(0, 0, 1);
			float depth = 0;
			if (mode != 0)
			{
				// XY radial distance gives the exact tangent-plane depth; neither contact nor domain has a height cutoff.
				double2 radial = (double2)position.xy - double2(centre_x, centre_y);
				double distance = RadialDistance(radial);
				depth = (float)(distance - radius);
				if (distance > 0)
					normal = float3(-(float2)(radial / distance), 0);

				// A spacing-sized cover margin bounds unsampled surface extent; reject unsupported states rather than clipping solver impulses.
				if (!all(isfinite(position)) || !isfinite(depth) || depth + spacing > max_penetration)
					InterlockedOr(status[0], 4);
			}
			else
			{
				// Terrain keeps its canonical field and normal-distance convention.
				BaselineResult field = BaselineEvaluate(recipe, (double2)position.xy);
				if (field.m_status != 0)
					InterlockedOr(status[0], 1);

				// Project the vertical gap onto the local terrain normal; retain a 1 mm normal-distance contact margin.
				normal = normalize(float3(-(float)field.m_dx, -(float)field.m_dy, 1));
				depth = (float)(field.m_height - (double)position.z) * normal.z;
			}
			if (!all(isfinite(normal)) || dot(normal, normal) < 0.5f || !isfinite(depth))
				InterlockedOr(status[0], 1);
			else if (mode != 2 && depth >= -0.001f)
			{
				candidate.position = position;
				candidate.normal_depth = float4(normal, max(0, depth));
				candidate.valid = 1;
			}
		}

		// Validation has no cross-lane reduction or solver output.
		if (mode == 2)
			continue;

		// Make every lane's result visible before selecting contacts in primitive ordinal order.
		batch[lane] = candidate;
		GroupMemoryBarrierWithGroupSync();
		if (lane == 0)
		{
			for (uint i = 0; i != 64; ++i)
			{
				WorldCandidate c = batch[i];
				if (!c.valid) continue;

				// Match the first group normal within ten degrees without averaging normals across distinct slopes.
				uint cluster = 0;
				for (; cluster != cluster_count; ++cluster)
					if (dot(normals[cluster], c.normal_depth.xyz) >= 0.98480775f) break;
				if (cluster == cluster_count)
				{
					if (cluster_count == 8)
					{
						InterlockedOr(status[0], 2);
						continue;
					}
					normals[cluster_count++] = c.normal_depth.xyz;
				}

				// Preserve contact spread in the surface tangent plane, preferring depth and then distance from the bounds centre.
				float2 offset = c.position.xy - bounds.centre.xy;
				if (mode == 1)
					offset = float2(dot(c.position.xy - bounds.centre.xy, float2(-c.normal_depth.y, c.normal_depth.x)), c.position.z - bounds.centre.z);
				uint quadrant = (offset.x >= 0 ? 1 : 0) + (offset.y >= 0 ? 2 : 0);
				uint slot = 4 * cluster + quadrant;
				WorldCandidate old = selected[slot];
				float2 old_offset = old.position.xy - bounds.centre.xy;
				if (mode == 1)
					old_offset = float2(dot(old.position.xy - bounds.centre.xy, float2(-c.normal_depth.y, c.normal_depth.x)), old.position.z - bounds.centre.z);
				bool replace = old.valid == 0 || c.normal_depth.w > old.normal_depth.w + 0.0001f;
				if (abs(c.normal_depth.w - old.normal_depth.w) <= 0.0001f)
					replace = replace || dot(offset, offset) > dot(old_offset, old_offset);
				if (replace) selected[slot] = c;
			}
		}
		GroupMemoryBarrierWithGroupSync();
	}

	// Validation never accesses the unbound solver streams.
	if (mode == 2)
		return;

	// Append body-local solver contacts against the shapeless world endpoint; excess contact counts remain visible to overflow checks.
	if (lane == 0)
	{
		float4x4 w2a = InvertOrthonormal(body.o2w);
		for (uint i = 0; i != 32; ++i)
		{
			WorldCandidate candidate = selected[i];
			if (!candidate.valid) continue;
			uint slot;
			InterlockedAdd(counters[0].contact_count, 1, slot);
			if (slot >= max_contacts) continue;
			GpuResolveContact contact = (GpuResolveContact)0;
			float4 normal = float4(candidate.normal_depth.xyz, 0);
			float depth = candidate.normal_depth.w;
			contact.axis = mul(-normal, w2a);
			contact.contact_point = mul(candidate.position + normal * (0.5f * depth), w2a);
			contact.manifold[0] = contact.contact_point;
			contact.b2a = w2a;
			contact.body_idx_a = instance.body;
			contact.body_idx_b = endpoint;
			contact.mat_id_a = shape.material_id;
			contact.mat_id_b = mode == 0 ? recipe.m_material_id : boundary_material;
			contact.depth = depth;
			contact.feature = FEATURE_VERT;
			contact.child_idx_a = instance.child;
			contact.child_idx_b = 0;
			contacts[slot] = contact;
		}
	}
}
