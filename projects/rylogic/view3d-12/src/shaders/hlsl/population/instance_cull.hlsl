//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Per-view instance culling of nuggets drawn with GPU-written draw arguments (see 'pr/view3d-12/shaders/indirect_instances.hlsli').
//
// The renderer records these kernels in order for each (drawlist element, view) pair:
//  - Reset: copy the source draw arguments into the view buffer with an instance count of zero.
//  - Cull:  append the index of each source instance whose bounding sphere is inside the view, and count it.
//
// View buffer layout (bytes):
//  [0, 20)   the draw arguments: D3D12_DRAW_INDEXED_ARGUMENTS, or D3D12_DRAW_ARGUMENTS followed by an unused word
//  [20, 32)  unused, so the list starts on a 16-byte boundary
//  [32, ...) the visible list: one uint source instance index per drawn instance
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// The word of the draw arguments that holds the instance count. It is the same word in both argument layouts.
static const uint InstanceCountWord = 1;

// The byte offset of the visible list in the view buffer.
static const uint ListOffset = 32;

// Constants of one cull. Mirrors 'InstanceCullConstants' in 'instance_culler.cpp'.
struct InstanceCullConstants
{
	float4 planes[4];   // Object-space side planes of the view frustum. Inside means 'dot(plane.xyz, p) + plane.w >= 0'.
	uint arg_words;     // The number of words in the draw arguments: 5 for indexed draws, 4 otherwise.
	uint capacity;      // The number of instance bounds. Source instances beyond this are not drawn.
	float radius_scale; // Scales object-space radii to world-space radii, because the planes are normalised in world space.
	uint pad;
};

ConstantBuffer<InstanceCullConstants> g_cull : register(b0);
ByteAddressBuffer g_source : register(t0);       // The owner's draw arguments
StructuredBuffer<float4> g_bounds : register(t1); // The owner's instance bounds: object-space centre (xyz) and radius (w)
RWByteAddressBuffer g_view : register(u0);       // The view buffer

// Copy the source draw arguments with no instances.
numthreads(CSInstanceReset, 1, 1, 1)
void CSInstanceReset()
{
	// One thread copies the few argument words.
	for (uint i = 0; i != g_cull.arg_words; ++i)
		g_view.Store(i * 4, i == InstanceCountWord ? 0u : g_source.Load(i * 4));
}

// Append each visible source instance to the list.
numthreads(CSInstanceCull, 64, 1, 1)
void CSInstanceCull(uint3 dtid : SV_DispatchThreadID)
{
	// The source instance count is written by the GPU, so it is only known here. It must not exceed the capacity of the bounds.
	uint i = dtid.x;
	uint count = min(g_source.Load(InstanceCountWord * 4), g_cull.capacity);
	if (i >= count)
		return;

	// Keep instances whose sphere is not entirely outside any side plane.
	float4 bound = g_bounds[i];
	float radius = bound.w * g_cull.radius_scale;
	[unroll] for (uint p = 0; p != 4; ++p)
	{
		if (dot(g_cull.planes[p].xyz, bound.xyz) + g_cull.planes[p].w < -radius)
			return;
	}

	// Claim a list slot. The list order does not matter to the draws.
	uint slot;
	g_view.InterlockedAdd(InstanceCountWord * 4, 1, slot);
	g_view.Store(ListOffset + slot * 4, i);
}
