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

// Generate contacts between physical shapes and two optional, shapeless world surfaces:
//   mode 0: the procedural heightfield z = height(x,y);
//   mode 1: the inside of a vertical cylinder, centred at (centre_x, centre_y), with no top, bottom, or outer face.
// Ordinary shape-versus-shape collision is handled elsewhere. Both world surfaces feed the ordinary rigid-body solver;
// this shader does not integrate motion, apply impulses, move a body back inside, or clamp its velocity.
//
// GpuWorldContacts dispatches one 64-lane group per primitive leaf. A compound body has one instance per leaf, and articulation
// proxies use the same path. Cached shape-local surface plans supply points; lanes evaluate those points in batches of 64.
// A contact-producing group retains at most eight normal directions and four representative points per direction.
//
// Invariant: valid finite shape geometry and a rigid current pose determine contact geometry, independently of speed or step size.
// Positive depth means a sample is on the forbidden side. Any representable depth is submitted to the ordinary solver.
// Collision is discrete at the predicted pose, not a continuous time-of-impact search: contacts that appear and disappear between
// poses are not detected. No velocity, timestep, previous pose, or assumed maximum penetration enters this calculation.
// Distances are metres. The caller owns shape/configuration validity; integration and resolution own the body pose.
//
// status[0] accumulates failure flags; status[1..5] identifies the first failing source and its body, shape, sample and detail.
// These are invariant/domain/resource diagnostics, never a normal response to fast motion or a large overlap.
// Engine::CompleteStep checks this after GPU completion and rejects the frame before publishing caller-visible body state.

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

// Dispatch constants; layout must match GpuWorldContacts::Constants.
// height_upper bounds the entire terrain recipe, not just the current patch. spacing is the cylinder's sample coverage margin;
// terrain and cylinder can select different cached plans through plan_offset.
cbuffer Params : register(b0)
{
	uint endpoint, max_contacts, plan_offset;
	float height_upper;
	int sleeping_enabled, island_count;
	uint mode, boundary_material;
	double centre_x, centre_y, radius;
	float spacing, pad;
};
StructuredBuffer<GpuRigidBody> bodies : register(t0);
StructuredBuffer<GpuShape> shapes : register(t1);
StructuredBuffer<BaselineRecipe> recipes : register(t2);
StructuredBuffer<WorldRange> plans : register(t3);
StructuredBuffer<SurfacePatch> patches : register(t4);
StructuredBuffer<WorldInstance> instances : register(t5);
StructuredBuffer<GpuSleepIsland> sleep_islands : register(t6);
RWStructuredBuffer<GpuCollisionCounters> counters : register(u0);
RWStructuredBuffer<GpuResolveContact> contacts : register(u1);
RWStructuredBuffer<uint> status : register(u2);

groupshared WorldCandidate batch[64];
groupshared WorldCandidate selected[32];
groupshared float3 normals[8];
groupshared uint cluster_count;

// Retain the origin of a failed invariant/query or exhausted storage; never substitute a fabricated contact.
// Causes: 1 predicted leaf bounds, 2 terrain domain, 3 terrain evaluation, 4 sample arithmetic, 5 normal-group capacity.
void ReportFailure(uint cause, WorldInstance instance, uint sample, uint detail)
{
	InterlockedOr(status[0], 1u << (cause - 1));
	uint previous;
	InterlockedCompareExchange(status[1], 0, cause, previous);
	if (previous == 0)
	{
		status[2] = instance.body;
		status[3] = instance.shape;
		status[4] = sample;
		status[5] = detail;
	}
}

// Scale before squaring so a finite position does not overflow the float square-root seed.
// One double refinement retains accurate radial depth while HLSL provides only a float square root.
double RadialDistance(double2 radial)
{
	double scale = max(abs(radial.x), abs(radial.y));
	if (scale == 0)
		return 0;

	// The scaled squared length lies in [1,2], independent of world distance.
	double2 unit = radial / scale;
	double square = unit.x * unit.x + unit.y * unit.y;
	double seed = (double)sqrt((float)square);
	return scale * (0.5 * (seed + square / seed));
}

// Append selected surface contacts for one primitive per group; report invalid queries or excess slope groups through status.
numthreads(CSWorldContacts, 64, 1, 1)
void CSWorldContacts(uint3 group : SV_GroupID, uint lane : SV_GroupIndex)
{
	// shape.s2rb maps the leaf into body space; body.o2w then places it in world space. rb_bbox already includes the leaf placement.
	// Reject nonfinite bounds and terrain-domain overflow before the height early-out, so distant invalid queries cannot look successful.
	WorldInstance instance = instances[group.x];
	GpuRigidBody body = bodies[instance.body];
	GpuShape shape = shapes[instance.shape];
	float4x4 s2w = mul(shape.s2rb, body.o2w);
	BBox bounds = shape.rb_bbox.Transform(body.o2w);
	BaselineRecipe recipe = recipes[0];
	if (!all(isfinite(bounds.centre)) || !all(isfinite(bounds.radius)))
	{
		// The pose/shape producer has violated its finite-geometry invariant; identify it before using its bounds.
		if (lane == 0) ReportFailure(1, instance, 0, 0);
		return;
	}
	if (mode == 0 && (
		abs((double)bounds.centre.x) + bounds.radius.x > recipe.m_supported_coordinate_abs_m ||
		abs((double)bounds.centre.y) + bounds.radius.y > recipe.m_supported_coordinate_abs_m))
	{
		// The procedural evaluator, unlike the cylinder, has a configured finite query domain.
		if (lane == 0) ReportFailure(2, instance, 0, 0);
		return;
	}
	if (mode == 0 && bounds.centre.z - bounds.radius.z > height_upper)
		return;

	// The cylinder depends only on current XY geometry. A fully interior bounding rectangle cannot contain a wall contact.
	// Its margin covers the sample spacing and the 1 mm contact tolerance; this is a geometric absence test, not a motion limit.
	if (mode != 0)
	{
		double2 furthest = abs((double2)bounds.centre.xy - double2(centre_x, centre_y)) + (double2)bounds.radius.xy;
		double inner_radius = radius - spacing - 0.001;
		if (inner_radius > 0 && furthest.x * furthest.x + furthest.y * furthest.y < inner_radius * inner_radius)
			return;
	}

	// Skip contact generation for an undisturbed sleeper, matching ordinary broadphase eligibility. Re-solving unchanged support
	// could inject bias impulses and wake it. Changing a body pose or the world surface wakes affected bodies at the owning boundary.
	if (sleeping_enabled != 0 && AllSet(body.state_flags, ERigidBodyStateFlags_Sleeping))
	{
		int island_id = body.sleep.island_id;
		if (island_id < 0 || island_id >= island_count ||
			!AnySet(sleep_islands[island_id].flags, GpuSleepIslandFlags_Disturbed))
			return;
	}

	// Stream the cached plan without materialising every point in GPU storage. 'selected' persists across batches; 'batch' is scratch.
	// The fixed 8 x 4 capacity bounds solver output per leaf. Exceeding the normal-group capacity is an error, not silent truncation.
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
			// Each compact patch describes a consecutive range of sample ordinals. Binary-search its exclusive end, then ask
			// EmitSurfaceSample to reconstruct just this lane's shape-local point.
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
				// Cylinder interior is the permitted side. A point outside radius has positive depth; its normal points inward.
				// This is the radial depth of this sampled point, not an exact deepest point of the whole shape. Height is irrelevant.
				double2 radial = (double2)position.xy - double2(centre_x, centre_y);
				double distance = RadialDistance(radial);
				depth = (float)(distance - radius);
				if (distance > 0)
					normal = float3(-(float2)(radial / distance), 0);

			}
			else
			{
				// The terrain is the authoritative procedural field, not a rendered triangle mesh. Evaluate height and derivatives
				// at the sample's XY, then construct the upward normal to the local tangent plane.
				BaselineResult field = BaselineEvaluate(recipe, (double2)position.xy);
				if (field.m_status != 0)
					ReportFailure(3, instance, ordinal, field.m_status);

				// Project the vertical height gap onto that normal. This is a local tangent-plane approximation, not a global
				// closest-point search. The common test below keeps points within 1 mm of contact and gives them zero negative depth.
				double scale = max(1.0, max(abs(field.m_dx), abs(field.m_dy)));
				normal = normalize(float3((float)(-field.m_dx / scale), (float)(-field.m_dy / scale), (float)(1.0 / scale)));
				depth = (float)((field.m_height - (double)position.z) * (double)normal.z);
			}
			if (!all(isfinite(position)) || !all(isfinite(normal)) || !isfinite(depth))
				ReportFailure(4, instance, ordinal, mode);
			else if (depth >= -0.001f)
			{
				candidate.position = position;
				candidate.normal_depth = float4(normal, max(0, depth));
				candidate.valid = 1;
			}
		}

		// Make every lane's result visible before selecting contacts in primitive ordinal order.
		batch[lane] = candidate;
		GroupMemoryBarrierWithGroupSync();
		if (lane == 0)
		{
			for (uint i = 0; i != 64; ++i)
			{
				WorldCandidate c = batch[i];
				if (!c.valid) continue;

				// Group nearby normal directions (cos(10 degrees) = 0.98480775). Keep distinct slopes separate rather than averaging
				// them into a fictional surface. Selection order is deterministic for a given sample plan.
				uint cluster = 0;
				for (; cluster != cluster_count; ++cluster)
					if (dot(normals[cluster], c.normal_depth.xyz) >= 0.98480775f) break;
				if (cluster == cluster_count)
				{
					if (cluster_count == 8)
					{
						ReportFailure(5, instance, base + i, mode);
						continue;
					}
					normals[cluster_count++] = c.normal_depth.xyz;
				}

				// Keep one representative in each of four spatial quadrants so a face can resist rotation as well as translation.
				// Terrain uses XY; cylinder contacts use circumferential tangent/height coordinates. Prefer deeper points, then
				// farther points for nearly tied depth, to preserve useful leverage. These are reduced samples, not an exact manifold.
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

	// Convert selected contacts to body-A space, as the ordinary solver expects. World normals point toward the body, so negate
	// them for the solver's A-to-B axis. The world endpoint is static and shapeless; materials still come from both participants.
	// The shared atomic counter reserves output slots. Keep attempted counts even on overflow so the engine rejects lost contacts;
	// never write past capacity. The separate solver owns friction, restitution, impulses and position correction.
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
