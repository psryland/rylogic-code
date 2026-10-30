//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Per-view classification of population instances (see 'pr/view3d-12/shaders/population.hlsli').
//
// The renderer records these kernels in order for each (population, view) pair:
//  - Clear:   zero the list header.
//  - Count:   count the visible entries of each LOD.
//  - Prefix:  turn the counts into list offsets, and write the indirect draw arguments.
//  - Scatter: write the visible entries into the per-LOD lists.
// Count and Scatter make the same decision for each instance, so the counts match the entries written.
//
// View buffer layout (bytes):
//  [0, HEADER_SIZE)          the list header (see VIEW3D_POPULATION_HEADER_*)
//  [HEADER_SIZE, list_base)  indirect draw argument records, 'ArgStride' bytes each
//  [list_base, ...)          the visible list entries, 8 bytes each
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "pr/view3d-12/shaders/population.hlsli"

// The size of an indirect draw argument record (D3D12_DRAW_INDEXED_ARGUMENTS, and D3D12_DRAW_ARGUMENTS padded to the same size).
static const uint ArgStride = 20;

// Diagnostic counters in the header, at VIEW3D_POPULATION_HEADER_DIAG.
static const uint DiagTested = 0;    // Live instance slots tested.
static const uint DiagEmitted = 1;   // List entries written.
static const uint DiagLiveCount = 2; // Blocks whose live count exceeded their capacity.

// Constants of one classification. Mirrors 'CullConstants' in 'population.cpp'.
struct CullConstants
{
	float4 planes[4];                                // World-space side planes of the view frustum. Inside means 'dot(plane.xyz, p) + plane.w >= 0'.
	float4 eye;                                      // xyz = world-space projection origin, w = 1 for orthographic projections.
	float4 range_origin;                             // xyz = world-space point that the draw distance is measured from.
	float4 bound;                                    // xyz = instance-space centre of the prototype bounding sphere, w = radius.
	float4 lod_size[VIEW3D_POPULATION_MAX_LODS / 4]; // The minimum projected radius (pixels) of each LOD.
	float4 lod_fade[VIEW3D_POPULATION_MAX_LODS / 4]; // The transition width of each LOD, as a fraction of its minimum size.
	float size_scale;                                // Projected radius in pixels = radius * size_scale / distance (no division for orthographic).
	float draw_distance;                             // Instances further than this are culled. Zero means unlimited.
	uint lod_count;                                  // The number of LODs in the prototype.
	uint arg_count;                                  // The number of indirect argument records.
	uint list_base;                                  // Byte offset of the first list entry.
	uint pad0;
	uint pad1;
	uint pad2;
};

// A block that passed the CPU culling test.
struct VisibleBlock
{
	uint first;    // The first instance slot of the block.
	uint capacity; // The number of instance slots in the block.
	uint block;    // The caller's block index, which selects the live count.
	uint pad;
};

// The template of an indirect draw argument record.
struct ArgTemplate
{
	uint lod;     // The LOD that the draw renders. Selects the instance count.
	uint count;   // The index count (indexed) or vertex count (non-indexed) of the nugget.
	uint start;   // The first index (indexed) or vertex (non-indexed) of the nugget.
	uint pad;
};

ConstantBuffer<CullConstants> g_cull : register(b0);
StructuredBuffer<View3DPopulationInstance> g_instances : register(t0);
StructuredBuffer<VisibleBlock> g_blocks : register(t1);
StructuredBuffer<uint> g_live_counts : register(t2);
StructuredBuffer<ArgTemplate> g_args : register(t3);
RWByteAddressBuffer g_view : register(u0);

// The result of classifying one instance.
struct Classification
{
	uint lod0;    // The LOD of the primary entry, or ~0 if the instance is culled.
	uint dither0; // The dither word of the primary entry.
	uint lod1;    // The LOD of the complementary transition entry, or ~0 if there is none.
	uint dither1; // The dither word of the complementary entry.
};

// Return element 'i' of a packed float4 array.
float Packed(float4 arr[VIEW3D_POPULATION_MAX_LODS / 4], uint i)
{
	return arr[i / 4][i % 4];
}

// Hash an instance index to a dither seed, so the dither pattern of an instance does not depend on its list position.
uint DitherSeed(uint source)
{
	uint h = source * 0x9E3779B1u;
	h ^= h >> 15;
	h *= 0x85EBCA77u;
	h ^= h >> 13;
	return h & VIEW3D_POPULATION_DITHER_SEED_MASK;
}

// Decide which LOD lists 'source' appears in for this view.
Classification Classify(uint source)
{
	// Start with a culled result.
	Classification result;
	result.lod0 = ~0u;
	result.dither0 = 0;
	result.lod1 = ~0u;
	result.dither1 = 0;

	// Disabled instances are never drawn.
	View3DPopulationInstance inst = g_instances[source];
	if ((inst.info.y & VIEW3D_POPULATION_FLAG_ENABLED) == 0)
		return result;

	// Place the prototype bounding sphere. The radius scales by the longest instance axis, which is exact for rotation with axis scaling.
	float3 centre = View3DPopulationTransform(inst, float4(g_cull.bound.xyz, 1)).xyz;
	float3 ax = float3(inst.o2w[0].x, inst.o2w[1].x, inst.o2w[2].x);
	float3 ay = float3(inst.o2w[0].y, inst.o2w[1].y, inst.o2w[2].y);
	float3 az = float3(inst.o2w[0].z, inst.o2w[1].z, inst.o2w[2].z);
	float radius = g_cull.bound.w * sqrt(max(dot(ax, ax), max(dot(ay, ay), dot(az, az))));

	// Reject spheres outside the frustum side planes.
	[unroll] for (uint p = 0; p != 4; ++p)
	{
		if (dot(g_cull.planes[p].xyz, centre) + g_cull.planes[p].w < -radius)
			return result;
	}

	// Reject instances beyond the draw distance.
	if (g_cull.draw_distance > 0 && length(centre - g_cull.range_origin.xyz) - radius > g_cull.draw_distance)
		return result;

	// The projected radius in pixels. Orthographic projections do not depend on distance.
	float dist = length(centre - g_cull.eye.xyz);
	float size = g_cull.eye.w != 0 ? radius * g_cull.size_scale : radius * g_cull.size_scale / max(dist, 1e-6f);

	// Select the most detailed LOD whose minimum size the instance reaches. Below the last LOD's minimum size, the instance is not drawn.
	uint lod = 0;
	for (; lod != g_cull.lod_count && size < Packed(g_cull.lod_size, lod); ++lod) {}
	if (lod == g_cull.lod_count)
		return result;

	result.lod0 = lod;

	// Within the transition band above the minimum size, the LOD fades out with a screen-door dither and the next LOD fades in with the
	// complementary mask. The last LOD fades to nothing.
	float t = Packed(g_cull.lod_size, lod);
	float w = Packed(g_cull.lod_fade, lod);
	if (w > 0 && size < t * (1 + w))
	{
		float coverage = saturate((size - t) / (t * w));
		uint word = VIEW3D_POPULATION_DITHER_ACTIVE | (DitherSeed(source) << VIEW3D_POPULATION_DITHER_SEED_SHIFT) | uint(coverage * 65535.0f + 0.5f);
		result.dither0 = word;
		if (lod + 1 != g_cull.lod_count)
		{
			result.lod1 = lod + 1;
			result.dither1 = word | VIEW3D_POPULATION_DITHER_COMPLEMENT;
		}
	}
	return result;
}

// Return the instance slot for a thread, or false if the thread has no live slot. 'dtid.y' selects the visible block.
bool InstanceSlot(uint3 dtid, out uint source)
{
	// Threads beyond the block's live count have no work. The live count is clamped to the block capacity.
	VisibleBlock blk = g_blocks[dtid.y];
	uint live = min(g_live_counts[blk.block], blk.capacity);
	source = blk.first + dtid.x;
	return dtid.x < live;
}

// Zero the list header.
numthreads(CSPopClear, 32, 1, 1)
void CSPopClear(uint3 dtid : SV_DispatchThreadID)
{
	// One thread per header word.
	g_view.Store(dtid.x * 4, 0);
}

// Count the entries of each LOD list.
numthreads(CSPopCount, 64, 1, 1)
void CSPopCount(uint3 dtid : SV_DispatchThreadID)
{
	// Report blocks whose live count exceeds their capacity once, from their first thread.
	VisibleBlock blk = g_blocks[dtid.y];
	if (dtid.x == 0 && g_live_counts[blk.block] > blk.capacity)
		g_view.InterlockedAdd(VIEW3D_POPULATION_HEADER_DIAG + DiagLiveCount * 4, 1);

	uint source;
	if (!InstanceSlot(dtid, source))
		return;

	// Count the entries this instance adds to each list.
	Classification c = Classify(source);
	g_view.InterlockedAdd(VIEW3D_POPULATION_HEADER_DIAG + DiagTested * 4, 1);
	if (c.lod0 != ~0u)
		g_view.InterlockedAdd(VIEW3D_POPULATION_HEADER_COUNTS + c.lod0 * 4, 1);
	if (c.lod1 != ~0u)
		g_view.InterlockedAdd(VIEW3D_POPULATION_HEADER_COUNTS + c.lod1 * 4, 1);
}

// Convert the LOD counts to list offsets and write the indirect draw arguments.
numthreads(CSPopPrefix, 1, 1, 1)
void CSPopPrefix(uint3 dtid : SV_DispatchThreadID)
{
	// Lay out the LOD lists one after another.
	uint total = 0;
	for (uint lod = 0; lod != VIEW3D_POPULATION_MAX_LODS; ++lod)
	{
		g_view.Store(VIEW3D_POPULATION_HEADER_OFFSETS + lod * 4, total);
		total += g_view.Load(VIEW3D_POPULATION_HEADER_COUNTS + lod * 4);
	}
	g_view.Store(VIEW3D_POPULATION_HEADER_DIAG + DiagEmitted * 4, total);

	// Each draw renders its LOD's whole list. The start instance is zero because 'View3DPopulationFetch' adds the list offset.
	// Indexed records are {IndexCount, InstanceCount, StartIndex, BaseVertex, StartInstance}; non-indexed records use the first four fields.
	for (uint i = 0; i != g_cull.arg_count; ++i)
	{
		ArgTemplate a = g_args[i];
		uint instances = g_view.Load(VIEW3D_POPULATION_HEADER_COUNTS + a.lod * 4);
		uint ofs = VIEW3D_POPULATION_HEADER_SIZE + i * ArgStride;
		g_view.Store4(ofs, uint4(a.count, instances, a.start, 0));
		g_view.Store(ofs + 16, 0);
	}
}

// Write the visible entries to the LOD lists.
numthreads(CSPopScatter, 64, 1, 1)
void CSPopScatter(uint3 dtid : SV_DispatchThreadID)
{
	// Only live slots produce entries.
	uint source;
	if (!InstanceSlot(dtid, source))
		return;

	// Append each entry at its LOD's offset plus a cursor. The list order is not stable.
	Classification c = Classify(source);
	if (c.lod0 != ~0u)
	{
		uint first = g_view.Load(VIEW3D_POPULATION_HEADER_OFFSETS + c.lod0 * 4);
		uint index;
		g_view.InterlockedAdd(VIEW3D_POPULATION_HEADER_CURSORS + c.lod0 * 4, 1, index);
		g_view.Store2(g_cull.list_base + (first + index) * 8, uint2(source, c.dither0));
	}
	if (c.lod1 != ~0u)
	{
		uint first = g_view.Load(VIEW3D_POPULATION_HEADER_OFFSETS + c.lod1 * 4);
		uint index;
		g_view.InterlockedAdd(VIEW3D_POPULATION_HEADER_CURSORS + c.lod1 * 4, 1, index);
		g_view.Store2(g_cull.list_base + (first + index) * 8, uint2(source, c.dither1));
	}
}
