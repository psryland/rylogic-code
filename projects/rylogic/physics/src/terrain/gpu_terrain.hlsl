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

// Patch span and total sample count for one packed primitive; matches GpuTerrain::Range.
struct TerrainRange
{
	uint begin, patch_count, sample_count, pad;
};

// Packed body, shape, and child identity; matches GpuTerrain::Instance.
struct TerrainInstance
{
	uint body, shape, child, pad;
};

// A sampled world-space position and its terrain normal and nonnegative contact depth.
struct TerrainCandidate
{
	float4 position;
	float4 normal_depth;
	uint valid;
};

// Dispatch constants and conservative terrain height bound; matches the host Constants layout.
cbuffer Params : register(b0)
{
	uint instance_count, endpoint, max_contacts;
	float height_upper;
};
StructuredBuffer<GpuRigidBody> bodies : register(t0);
StructuredBuffer<GpuShape> shapes : register(t1);
StructuredBuffer<BaselineRecipe> recipes : register(t2);
StructuredBuffer<TerrainRange> plans : register(t3);
StructuredBuffer<SurfacePatch> patches : register(t4);
StructuredBuffer<TerrainInstance> instances : register(t5);
RWStructuredBuffer<GpuCollisionCounters> counters : register(u0);
RWStructuredBuffer<GpuResolveContact> contacts : register(u1);
RWStructuredBuffer<uint> status : register(u2);

groupshared TerrainCandidate batch[64];
groupshared TerrainCandidate selected[32];
groupshared float3 normals[8];
groupshared uint cluster_count;

// Append selected surface contacts for one primitive per group; report invalid queries or excess slope groups through status.
numthreads(CSTerrain, 64, 1, 1)
void CSTerrain(uint3 group : SV_GroupID, uint lane : SV_GroupIndex)
{
	// Validate the entire primitive's query domain before using the height bound for a conservative rejection.
	TerrainInstance instance = instances[group.x];
	GpuRigidBody body = bodies[instance.body];
	GpuShape shape = shapes[instance.shape];
	float4x4 s2w = mul(shape.s2rb, body.o2w);
	BBox bounds = shape.rb_bbox.Transform(body.o2w);
	BaselineRecipe recipe = recipes[0];
	if (!all(isfinite(bounds.centre)) || !all(isfinite(bounds.radius)) ||
		abs((double)bounds.centre.x) + bounds.radius.x > recipe.m_supported_coordinate_abs_m ||
		abs((double)bounds.centre.y) + bounds.radius.y > recipe.m_supported_coordinate_abs_m)
	{
		if (lane == 0) InterlockedOr(status[0], 1);
		return;
	}
	if (bounds.centre.z - bounds.radius.z > height_upper)
		return;

	// Stream fixed-size batches while retaining at most eight slope groups with four spatial slots each.
	if (lane < 32) selected[lane] = (TerrainCandidate)0;
	if (lane == 0) cluster_count = 0;
	GroupMemoryBarrierWithGroupSync();
	TerrainRange plan = plans[instance.shape];
	for (uint base = 0; base < plan.sample_count; base += 64)
	{
		TerrainCandidate candidate = (TerrainCandidate)0;
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

			// Evaluate the canonical terrain field at the fully transformed shape sample.
			uint first = lo == 0 ? 0 : patches[plan.begin + lo - 1].m_sample_end;
			SurfaceSample sample = EmitSurfaceSample(patches[plan.begin + lo], ordinal - first);
			float4 position = mul(sample.m_pos_local, s2w);
			BaselineResult field = BaselineEvaluate(recipe, (double2)position.xy);
			if (field.m_status != 0)
			{
				InterlockedOr(status[0], 1);
			}
			else
			{
				// Project the vertical gap onto the local terrain normal; retain a 1 mm normal-distance contact margin.
				float3 normal = normalize(float3(-(float)field.m_dx, -(float)field.m_dy, 1));
				float depth = (float)(field.m_height - (double)position.z) * normal.z;
				if (!all(isfinite(normal)) || dot(normal, normal) < 0.5f || !isfinite(depth))
					InterlockedOr(status[0], 1);
				else if (depth >= -0.001f)
				{
					candidate.position = position;
					candidate.normal_depth = float4(normal, max(0, depth));
					candidate.valid = 1;
				}
			}
		}

		// Make every lane's result visible before selecting contacts in primitive ordinal order.
		batch[lane] = candidate;
		GroupMemoryBarrierWithGroupSync();
		if (lane == 0)
		{
			for (uint i = 0; i != 64; ++i)
			{
				TerrainCandidate c = batch[i];
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

				// Prefer depth within each world-XY quadrant, breaking near-equal depths by horizontal distance from the bounds centre.
				float2 offset = c.position.xy - bounds.centre.xy;
				uint quadrant = (offset.x >= 0 ? 1 : 0) + (offset.y >= 0 ? 2 : 0);
				uint slot = 4 * cluster + quadrant;
				TerrainCandidate old = selected[slot];
				float2 old_offset = old.position.xy - bounds.centre.xy;
				bool replace = old.valid == 0 || c.normal_depth.w > old.normal_depth.w + 0.0001f;
				if (abs(c.normal_depth.w - old.normal_depth.w) <= 0.0001f)
					replace = replace || dot(offset, offset) > dot(old_offset, old_offset);
				if (replace) selected[slot] = c;
			}
		}
		GroupMemoryBarrierWithGroupSync();
	}

	// Append body-local solver contacts against the shapeless world endpoint; excess contact counts remain visible to overflow checks.
	if (lane == 0)
	{
		float4x4 w2a = InvertOrthonormal(body.o2w);
		for (uint i = 0; i != 32; ++i)
		{
			TerrainCandidate candidate = selected[i];
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
			contact.mat_id_b = recipe.m_material_id;
			contact.depth = depth;
			contact.feature = FEATURE_VERT;
			contact.child_idx_a = instance.child;
			contact.child_idx_b = 0;
			contacts[slot] = contact;
		}
	}
}
