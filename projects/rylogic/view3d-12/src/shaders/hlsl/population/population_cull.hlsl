//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Per-view classification of population instances (see 'pr/view3d-12/shaders/population.hlsli').
//
// The renderer records these kernels in order for each (population, view) pair. All prototypes of the population are classified together:
//  - Clear:   zero the list header.
//  - Count:   count the visible entries of each (prototype, LOD) list.
//  - Prefix:  turn the counts into list offsets, and write the indirect draw arguments.
//  - Scatter: write the visible entries into the lists.
// Count and Scatter make the same decision for each instance, so the counts match the entries written.
//
// View buffer layout (bytes):
//  [0, header_size)          the list header (see VIEW3D_POPULATION_HEADER_*)
//  [header_size, list_base)  indirect draw argument records, 'ArgStride' bytes each
//  [list_base, ...)          the visible list entries, 8 bytes each
#include "view3d-12/src/shaders/hlsl/types.hlsli"
#include "pr/view3d-12/shaders/population.hlsli"

// The size of an indirect draw argument record (D3D12_DRAW_INDEXED_ARGUMENTS, and D3D12_DRAW_ARGUMENTS padded to the same size).
static const uint ArgStride = 20;

// Diagnostic counters in the header, at VIEW3D_POPULATION_HEADER_DIAG. After the draws, the renderer copies the header to a readback buffer
// without waiting for the GPU; 'Population::Stats' reads the copy once the GPU has completed the frame (see View3D_PopulationStats).
static const uint DiagTested = 0;    // Live instance slots tested.
static const uint DiagEmitted = 1;   // List entries written.
static const uint DiagLiveCount = 2; // Blocks whose live count exceeded their capacity.

// Constants of one classification. Mirrors 'CullConstants' in 'population.cpp'.
struct CullConstants
{
	float4 planes[4];     // World-space side planes of the view frustum. Inside means 'dot(plane.xyz, p) + plane.w >= 0'.
	float4 eye;           // xyz = world-space projection origin, w = 1 for orthographic projections.
	float4 range_origin;  // xyz = world-space point that the draw distance is measured from.
	float size_scale;     // Projected radius in pixels = radius * size_scale / distance (no division for orthographic).
	uint arg_count;       // The number of indirect argument records.
	uint list_base;       // Byte offset of the first list entry.
	uint list_count;      // The number of visible lists: one per (prototype, LOD).
	uint prototype_count; // The number of prototypes.
	uint header_size;     // The size of the list header in bytes.
	float coverage;       // The fraction of each entry's pixels that are drawn, in [0,1]. See ApplyCoverage.
	uint pad1;
};

// The classification constants of one prototype. Mirrors 'PrototypeConstants' in 'population.cpp'.
struct PrototypeConstants
{
	float4 bound;                                    // xyz = instance-space centre of the prototype bounding sphere, w = radius.
	float4 lod_size[VIEW3D_POPULATION_MAX_LODS / 4]; // The minimum projected radius (pixels) of each LOD.
	float4 lod_fade[VIEW3D_POPULATION_MAX_LODS / 4]; // The transition width of each LOD, as a fraction of its minimum size.
	float4 fade;                                     // Origin distance bands: x,y = fade-in start and width, z,w = fade-out start and width. See Classify.
	float draw_distance;                             // Instances whose bounding sphere is further than this are culled. Zero means unlimited.
	uint lod_count;                                  // The number of LODs in the prototype.
	uint list_first;                                 // The list of the prototype's first LOD. Its other LODs use the following lists.
	uint pad;
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
	uint list;    // The visible list that the draw renders. Selects the instance count.
	uint count;   // The index count (indexed) or vertex count (non-indexed) of the nugget.
	uint start;   // The first index (indexed) or vertex (non-indexed) of the nugget.
	uint pad;
};

ConstantBuffer<CullConstants> g_cull : register(b0);
StructuredBuffer<View3DPopulationInstance> g_instances : register(t0);
StructuredBuffer<VisibleBlock> g_blocks : register(t1);
StructuredBuffer<uint> g_live_counts : register(t2);
StructuredBuffer<ArgTemplate> g_args : register(t3);
StructuredBuffer<PrototypeConstants> g_prototypes : register(t4);
RWByteAddressBuffer g_view : register(u0);

// The result of classifying one instance.
struct Classification
{
	uint list0;   // The list of the primary entry, or ~0 if the instance is culled.
	uint dither0; // The dither word of the primary entry.
	uint list1;   // The list of the complementary transition entry, or ~0 if there is none.
	uint dither1; // The dither word of the complementary entry.
};

// Return element 'i' of a packed float4 array.
float Packed(float4 arr[VIEW3D_POPULATION_MAX_LODS / 4], uint i)
{
	return arr[i / 4][i % 4];
}

// Hash an instance origin to a dither seed. The pattern does not depend on the list position, and instances with the same origin in different
// populations get the same seed, so their distance fades can be exactly complementary.
uint DitherSeed(float3 origin)
{
	uint3 b = asuint(origin);
	uint h = b.x * 0x9E3779B1u ^ b.y * 0x85EBCA77u ^ b.z * 0xC2B2AE3Du;
	h ^= h >> 15;
	h *= 0x85EBCA77u;
	h ^= h >> 13;
	return h & VIEW3D_POPULATION_DITHER_SEED_MASK;
}

// Return a dither word with 'coverage' in [0,1].
uint DitherWord(uint seed, float coverage)
{
	return VIEW3D_POPULATION_DITHER_ACTIVE | (seed << VIEW3D_POPULATION_DITHER_SEED_SHIFT) | uint(saturate(coverage) * 65535.0f + 0.5f);
}

// Return 'word' reduced to the population coverage 'coverage' in (0,1]. 'seed' is the instance's dither seed.
uint ApplyCoverage(uint word, uint seed, float coverage)
{
	// Full coverage leaves the entry unchanged, and a fully drawn entry keeps 'coverage' of its pixels.
	if (coverage >= 1)
		return word;
	if ((word & VIEW3D_POPULATION_DITHER_ACTIVE) == 0)
		return DitherWord(seed, coverage);

	// An entry keeps the pixels whose hash is below its threshold 'k', or not below it when complementary. Scaling a plain entry's threshold to
	// 'k * coverage', and raising a complementary entry's threshold to '1 - (1 - k) * coverage', keeps 'coverage' of each entry's own pixels.
	// Both results are subsets of the original masks, so complementary entries still never cover a pixel twice.
	float k = (word & VIEW3D_POPULATION_DITHER_COVERAGE_MASK) / 65535.0f;
	if ((word & VIEW3D_POPULATION_DITHER_COMPLEMENT) != 0)
		return DitherWord(seed, 1 - (1 - k) * coverage) | VIEW3D_POPULATION_DITHER_COMPLEMENT;

	return DitherWord(seed, k * coverage);
}

// Decide which lists 'source' appears in for this view.
Classification Classify(uint source)
{
	// Start with a culled result. Populations with zero coverage draw nothing.
	Classification result;
	result.list0 = ~0u;
	result.dither0 = 0;
	result.list1 = ~0u;
	result.dither1 = 0;
	if (g_cull.coverage <= 0)
		return result;

	// Disabled instances, and instances naming no prototype, are never drawn. Root views are not range checked, so the index must be.
	View3DPopulationInstance inst = g_instances[source];
	if ((inst.info.y & VIEW3D_POPULATION_FLAG_ENABLED) == 0 || inst.info.w >= g_cull.prototype_count)
		return result;

	PrototypeConstants proto = g_prototypes[inst.info.w];

	// Place the prototype bounding sphere. The radius scales by the longest instance axis, which is exact for rotation with axis scaling.
	float3 centre = View3DPopulationTransform(inst, float4(proto.bound.xyz, 1)).xyz;
	float3 ax = float3(inst.o2w[0].x, inst.o2w[1].x, inst.o2w[2].x);
	float3 ay = float3(inst.o2w[0].y, inst.o2w[1].y, inst.o2w[2].y);
	float3 az = float3(inst.o2w[0].z, inst.o2w[1].z, inst.o2w[2].z);
	float radius = proto.bound.w * sqrt(max(dot(ax, ax), max(dot(ay, ay), dot(az, az))));

	// Reject spheres outside the frustum side planes.
	[unroll] for (uint p = 0; p != 4; ++p)
	{
		if (dot(g_cull.planes[p].xyz, centre) + g_cull.planes[p].w < -radius)
			return result;
	}

	// Reject instances beyond the draw distance, measured to the closest point of the bounding sphere.
	float nearest = length(centre - g_cull.range_origin.xyz) - radius;
	if (proto.draw_distance > 0 && nearest > proto.draw_distance)
		return result;

	// Reject instances outside the fade bands, measured to the instance origin. The origin is the same for every prototype that draws an
	// instance, so populations with matching bands make the same decision.
	float3 origin = float3(inst.o2w[0].w, inst.o2w[1].w, inst.o2w[2].w);
	float od = length(origin - g_cull.range_origin.xyz);
	float4 fade = proto.fade;
	if (od < fade.x || od >= fade.z + fade.w)
		return result;

	// Inside a band, both directions use the fade-out coverage computed from the same band values. A fade-in entry takes the complementary
	// mask, so a fade-in band and a fade-out band with equal values together cover each pixel of an instance exactly once.
	uint seed = DitherSeed(origin);
	uint fade_word = 0;
	if (od < fade.x + fade.y)
		fade_word = DitherWord(seed, 1 - (od - fade.x) / fade.y) | VIEW3D_POPULATION_DITHER_COMPLEMENT;
	else if (od > fade.z)
		fade_word = DitherWord(seed, 1 - (od - fade.z) / fade.w);

	// A faded instance uses its own coverage instead of the band fade. Instances that would keep no pixels are culled.
	if ((inst.info.y & VIEW3D_POPULATION_FLAG_FADED) != 0)
	{
		uint coverage = inst.info.y >> VIEW3D_POPULATION_FLAG_FADE_SHIFT;
		bool complement = (inst.info.y & VIEW3D_POPULATION_FLAG_FADE_COMPLEMENT) != 0;
		if (coverage == (complement ? 0xFFFFu : 0u))
			return result;

		fade_word = VIEW3D_POPULATION_DITHER_ACTIVE | (seed << VIEW3D_POPULATION_DITHER_SEED_SHIFT) | coverage | (complement ? VIEW3D_POPULATION_DITHER_COMPLEMENT : 0u);
	}

	// The projected radius in pixels. Orthographic projections do not depend on distance.
	float dist = length(centre - g_cull.eye.xyz);
	float size = g_cull.eye.w != 0 ? radius * g_cull.size_scale : radius * g_cull.size_scale / max(dist, 1e-6f);

	// Select the most detailed LOD whose minimum size the instance reaches. Below the last LOD's minimum size, the instance is not drawn.
	uint lod = 0;
	for (; lod != proto.lod_count && size < Packed(proto.lod_size, lod); ++lod) {}
	if (lod == proto.lod_count)
		return result;

	result.list0 = proto.list_first + lod;

	// A distance fade replaces the LOD transition, because an entry has one coverage. The LOD switch inside a fade band is abrupt.
	if (fade_word != 0)
	{
		result.dither0 = ApplyCoverage(fade_word, seed, g_cull.coverage);
		return result;
	}

	// Within the transition band above the minimum size, the LOD fades out with a screen-door dither and the next LOD fades in with the
	// complementary mask. The last LOD fades to nothing.
	float t = Packed(proto.lod_size, lod);
	float w = Packed(proto.lod_fade, lod);
	if (w > 0 && size < t * (1 + w))
	{
		uint word = DitherWord(seed, (size - t) / (t * w));
		result.dither0 = word;
		if (lod + 1 != proto.lod_count)
		{
			result.list1 = proto.list_first + lod + 1;
			result.dither1 = ApplyCoverage(word | VIEW3D_POPULATION_DITHER_COMPLEMENT, seed, g_cull.coverage);
		}
	}
	result.dither0 = ApplyCoverage(result.dither0, seed, g_cull.coverage);
	return result;
}

// Add the number of active lanes that name each list to that list's word in 'base', and return each lane's position among the lanes that
// named the same list, after the value the word held. Lanes passing ~0 add nothing and return 0. Must be called by every active lane.
uint WaveListAdd(uint base, uint list)
{
	// Many instances in a wave usually share a few lists, so one atomic per distinct list replaces one atomic per lane. Contended
	// atomics on the same few header words otherwise serialise the whole classification.
	uint index = 0;
	bool pending = list != ~0u;
	while (WaveActiveAnyTrue(pending))
	{
		// Serve every lane naming the smallest outstanding list together. The first lane reserves the range for all of them.
		uint key = WaveActiveMin(pending ? list : ~0u);
		bool mine = pending && list == key;
		uint count = WaveActiveCountBits(mine);
		uint first = 0;
		if (WaveIsFirstLane())
			g_view.InterlockedAdd(base + key * 4, count, first);

		first = WaveReadLaneFirst(first);
		if (mine)
		{
			index = first + WavePrefixCountBits(mine);
			pending = false;
		}
	}
	return index;
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
numthreads(CSPopClear, 64, 1, 1)
void CSPopClear(uint3 dtid : SV_DispatchThreadID)
{
	// One thread per header word. The last group may extend past the header.
	if (dtid.x * 4 < g_cull.header_size)
		g_view.Store(dtid.x * 4, 0);
}

// Count the entries of each list.
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

	// Count the entries this instance adds to each list, with one atomic per wave rather than per instance.
	Classification c = Classify(source);
	uint tested = WaveActiveCountBits(true);
	if (WaveIsFirstLane())
		g_view.InterlockedAdd(VIEW3D_POPULATION_HEADER_DIAG + DiagTested * 4, tested);

	WaveListAdd(VIEW3D_POPULATION_HEADER_COUNTS, c.list0);
	WaveListAdd(VIEW3D_POPULATION_HEADER_COUNTS, c.list1);
}

// The number of threads in the single prefix group. Lists and argument records are spread over the group, because a population can have
// hundreds of them and one thread working through them in turn is slow.
static const uint PrefixGroupSize = 256;

// The sum of each wave's counts in the current chunk of lists, and the total of the lists in earlier chunks.
groupshared uint gs_wave_sums[PrefixGroupSize];
groupshared uint gs_running;

// Convert the list counts to list offsets and write the indirect draw arguments. Must be dispatched as one group.
numthreads(CSPopPrefix, PrefixGroupSize, 1, 1)
void CSPopPrefix(uint3 gtid : SV_GroupThreadID)
{
	// Lay out the lists one after another, in chunks of one list per thread. The OFFSETS field follows the COUNTS field.
	uint offsets = VIEW3D_POPULATION_HEADER_COUNTS + g_cull.list_count * 4;
	uint lane_count = WaveGetLaneCount();
	uint wave = gtid.x / lane_count;
	uint wave_count = (PrefixGroupSize + lane_count - 1) / lane_count;
	if (gtid.x == 0)
		gs_running = 0;

	GroupMemoryBarrierWithGroupSync();
	for (uint chunk = 0; chunk < g_cull.list_count; chunk += PrefixGroupSize)
	{
		// Each list's offset is the total of the earlier chunks, plus the counts of earlier waves in this chunk, plus the earlier lanes of its wave.
		uint list = chunk + gtid.x;
		uint count = list < g_cull.list_count ? g_view.Load(VIEW3D_POPULATION_HEADER_COUNTS + list * 4) : 0;
		uint in_wave = WavePrefixSum(count);
		uint wave_sum = WaveActiveSum(count);
		if (WaveIsFirstLane())
			gs_wave_sums[wave] = wave_sum;

		GroupMemoryBarrierWithGroupSync();
		uint before = gs_running;
		for (uint w = 0; w != wave; ++w)
			before += gs_wave_sums[w];

		if (list < g_cull.list_count)
			g_view.Store(offsets + list * 4, before + in_wave);

		// Advance the running total once every thread has read it.
		GroupMemoryBarrierWithGroupSync();
		if (gtid.x == 0)
		{
			// Add the counts of every wave in the chunk.
			uint total = gs_running;
			for (uint w = 0; w != wave_count; ++w)
				total += gs_wave_sums[w];

			gs_running = total;
		}
		GroupMemoryBarrierWithGroupSync();
	}
	if (gtid.x == 0)
		g_view.Store(VIEW3D_POPULATION_HEADER_DIAG + DiagEmitted * 4, gs_running);

	// Each draw renders its list in full. The start instance is zero because 'View3DPopulationFetch' adds the list offset.
	// Indexed records are {IndexCount, InstanceCount, StartIndex, BaseVertex, StartInstance}; non-indexed records use the first four fields.
	for (uint i = gtid.x; i < g_cull.arg_count; i += PrefixGroupSize)
	{
		ArgTemplate a = g_args[i];
		uint instances = g_view.Load(VIEW3D_POPULATION_HEADER_COUNTS + a.list * 4);
		uint ofs = g_cull.header_size + i * ArgStride;
		g_view.Store4(ofs, uint4(a.count, instances, a.start, 0));
		g_view.Store(ofs + 16, 0);
	}
}

// Write the visible entries to the lists.
numthreads(CSPopScatter, 64, 1, 1)
void CSPopScatter(uint3 dtid : SV_DispatchThreadID)
{
	// Only live slots produce entries.
	uint source;
	if (!InstanceSlot(dtid, source))
		return;

	// Append each entry at its list's offset plus a cursor, reserving the cursors once per wave and list. The list order is not stable.
	// The OFFSETS and CURSORS fields follow the COUNTS field.
	uint offsets = VIEW3D_POPULATION_HEADER_COUNTS + g_cull.list_count * 4;
	uint cursors = VIEW3D_POPULATION_HEADER_COUNTS + g_cull.list_count * 8;
	Classification c = Classify(source);
	uint index0 = WaveListAdd(cursors, c.list0);
	uint index1 = WaveListAdd(cursors, c.list1);
	if (c.list0 != ~0u)
	{
		uint first = g_view.Load(offsets + c.list0 * 4);
		g_view.Store2(g_cull.list_base + (first + index0) * 8, uint2(source, c.dither0));
	}
	if (c.list1 != ~0u)
	{
		uint first = g_view.Load(offsets + c.list1 * 4);
		g_view.Store2(g_cull.list_base + (first + index1) * 8, uint2(source, c.dither1));
	}
}