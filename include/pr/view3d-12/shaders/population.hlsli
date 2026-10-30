//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Public shader contract for population (GPU-driven instanced) rendering.
//
// A population draws many placements of one prototype. Each placement is a 'View3DPopulationInstance' record in a GPU buffer.
// For each consuming view (the main camera or a shadow view), the renderer selects the visible instances and their LODs on the GPU.
// It writes one list of 'View3DPopulationEntry' records per LOD, then draws each (LOD, nugget) pair with one indirect draw.
// The vertex shader of a population draw finds its entry with 'View3DPopulationFetch' and places the vertex with the instance transform.
//
// Instance transforms are affine (rotation, nonuniform scale, shear, mirroring, and translation). Normals use the cofactor of the linear part,
// so they stay perpendicular to surfaces under nonuniform scale. Projective transforms are not supported.
//
// Smooth LOD transitions use screen-door dithering, not alpha blending. During a transition, one instance appears in two adjacent LOD lists
// with complementary dither masks. Every pixel shader used by a population material must call 'View3DPopulationClipDither' (the stock forward
// and shadow pixel shaders do). The dither value is zero for ordinary (non-population) draws, which keeps every pixel.
#ifndef PR_VIEW3D_POPULATION_HLSLI
#define PR_VIEW3D_POPULATION_HLSLI
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"

// Contract version of the population layouts and bindings in this file.
#define VIEW3D_POPULATION_VERSION 1

// The maximum number of LODs in a population prototype.
#define VIEW3D_POPULATION_MAX_LODS 8

// Instance flags in 'View3DPopulationInstance.info.y'. Disabled instances are never drawn.
#define VIEW3D_POPULATION_FLAG_ENABLED 0x1

// Draw-time bindings, available to the vertex and pixel stages of population draws in the forward and shadow render steps.
//  - Instances: 'StructuredBuffer<View3DPopulationInstance>', the population's instance records.
//  - View: 'ByteAddressBuffer', the renderer's per-view visible lists. Read it only through 'View3DPopulationFetch'.
//  - Payload: 'ByteAddressBuffer', the caller's optional payload. Unbound when the population has no payload.
//  - Draw: 'ConstantBuffer<View3DPopulationDraw>', the LOD and list location of the current draw.
#define VIEW3D_POPULATION_INSTANCES_REGISTER t24
#define VIEW3D_POPULATION_VIEW_REGISTER t25
#define VIEW3D_POPULATION_PAYLOAD_REGISTER t26
#define VIEW3D_POPULATION_DRAW_REGISTER b8

// Producer bindings, used by compute shaders that generate population content on the GPU (see View3D_PopulationProduce).
//  - Constants: 'ConstantBuffer<T>', the caller's constants.
//  - Instances: 'RWStructuredBuffer<View3DPopulationInstance>', the population's instance records.
//  - LiveCounts: 'RWStructuredBuffer<uint>', one live instance count per block. A block draws instances [first, first + live count).
//  - Payload: 'RWByteAddressBuffer', the caller's optional payload. Unbound when the population has no payload.
// A producer must keep each live count within its block's capacity, and keep the live instances inside the block's published bounds.
// The renderer clamps live counts to capacity and reports the violation through the population diagnostics.
#define VIEW3D_POPULATION_PRODUCER_CONSTANTS_REGISTER b0
#define VIEW3D_POPULATION_PRODUCER_INSTANCES_REGISTER u0
#define VIEW3D_POPULATION_PRODUCER_LIVE_COUNTS_REGISTER u1
#define VIEW3D_POPULATION_PRODUCER_PAYLOAD_REGISTER u2

// Byte offsets of the per-view list header. The header holds 'VIEW3D_POPULATION_MAX_LODS' uints for each field.
#define VIEW3D_POPULATION_HEADER_COUNTS 0
#define VIEW3D_POPULATION_HEADER_OFFSETS 32
#define VIEW3D_POPULATION_HEADER_CURSORS 64
#define VIEW3D_POPULATION_HEADER_DIAG 96
#define VIEW3D_POPULATION_HEADER_SIZE 128

// Bit fields of the dither word in 'View3DPopulationEntry.dither'.
//  - Bits 0-15: coverage, as unorm16.
//  - Bits 16-29: a seed derived from the source instance index, so the mask does not depend on the list order.
//  - Bit 30: dithering is active. Zero means the entry is fully drawn.
//  - Bit 31: complementary. The entry keeps exactly the pixels that the non-complementary entry with the same coverage and seed discards.
#define VIEW3D_POPULATION_DITHER_COVERAGE_MASK 0x0000FFFFu
#define VIEW3D_POPULATION_DITHER_SEED_SHIFT 16
#define VIEW3D_POPULATION_DITHER_SEED_MASK 0x3FFFu
#define VIEW3D_POPULATION_DITHER_ACTIVE 0x40000000u
#define VIEW3D_POPULATION_DITHER_COMPLEMENT 0x80000000u

// One placement of a population prototype. 64 bytes.
struct View3DPopulationInstance
{
	// Rows of the affine instance-to-world transform: 'world[i] = dot(o2w[i], float4(p, 1))'.
	float4 o2w[3];

	// x = tint colour, packed as 0xAARRGGBB. Multiplies the vertex colour.
	// y = flags, see VIEW3D_POPULATION_FLAG_*.
	// z = caller payload index, interpreted only by caller shaders.
	// w = reserved, must be zero.
	uint4 info;
};

// One entry of a per-view visible list.
struct View3DPopulationEntry
{
	uint source; // The index of the instance record. The list order is not stable and carries no identity.
	uint dither; // The dither word for this entry. See VIEW3D_POPULATION_DITHER_*.
};

// Per-draw constants of a population draw.
struct View3DPopulationDraw
{
	uint lod;       // The LOD that this draw renders.
	uint list_base; // Byte offset of the first list entry in the view buffer.
	uint pad0;
	uint pad1;
};

#ifndef __cplusplus

// Read the list entry for 'instance_id' (SV_InstanceID) of the current population draw.
View3DPopulationEntry View3DPopulationFetch(ByteAddressBuffer view, View3DPopulationDraw draw, uint instance_id)
{
	// The LOD's first entry is written by the renderer's classification pass. SV_InstanceID does not include the draw's start instance.
	uint first = view.Load(VIEW3D_POPULATION_HEADER_OFFSETS + draw.lod * 4);
	uint2 raw = view.Load2(draw.list_base + (first + instance_id) * 8);

	View3DPopulationEntry entry;
	entry.source = raw.x;
	entry.dither = raw.y;
	return entry;
}

// Transform an instance-space point (w = 1) or direction (w = 0) to world space.
float4 View3DPopulationTransform(View3DPopulationInstance inst, float4 v)
{
	// The rows hold the translation in 'w', so directions ignore it.
	return float4(dot(inst.o2w[0], v), dot(inst.o2w[1], v), dot(inst.o2w[2], v), v.w);
}

// Transform an instance-space normal (w = 0) to a unit world-space normal. Zero normals stay zero.
float4 View3DPopulationTransformNormal(View3DPopulationInstance inst, float4 n)
{
	// The cofactor of the linear part is the inverse transpose scaled by the determinant. The sign of the determinant keeps normals of mirrored
	// instances facing out. The scale does not matter because the result is normalised.
	float3 r0 = inst.o2w[0].xyz;
	float3 r1 = inst.o2w[1].xyz;
	float3 r2 = inst.o2w[2].xyz;
	float3 c0 = cross(r1, r2);
	float3 c1 = cross(r2, r0);
	float3 c2 = cross(r0, r1);
	float det = dot(r0, c0);
	float3 ws = float3(dot(c0, n.xyz), dot(c1, n.xyz), dot(c2, n.xyz)) * (det < 0 ? -1.0f : 1.0f);

	// Normalise non-zero normals. Zero normals mean "face the light" to the lighting code.
	float len_sq = dot(ws, ws);
	if (len_sq > 0)
		ws *= rsqrt(len_sq);

	return float4(ws, 0);
}

// Return the instance tint as a linear float4.
float4 View3DPopulationTint(View3DPopulationInstance inst)
{
	// Unpack 0xAARRGGBB, matching the renderer's colour packing.
	uint c = inst.info.x;
	return float4((c >> 16) & 0xFF, (c >> 8) & 0xFF, c & 0xFF, (c >> 24) & 0xFF) / 255.0f;
}

// Return true if a pixel of a population entry with dither word 'dither' at screen position 'ss_xy' is drawn.
bool View3DPopulationDitherKeep(uint dither, float2 ss_xy)
{
	// Inactive dither words keep every pixel. This includes ordinary draws, whose dither value is zero.
	if ((dither & VIEW3D_POPULATION_DITHER_ACTIVE) == 0)
		return true;

	// Hash the pixel and seed to a uniform 16-bit value. Entries with the same seed get the same value at a pixel, so complementary entries
	// with the same coverage cover each pixel exactly once.
	uint2 px = uint2(ss_xy);
	uint seed = (dither >> VIEW3D_POPULATION_DITHER_SEED_SHIFT) & VIEW3D_POPULATION_DITHER_SEED_MASK;
	uint h = px.x * 0x8DA6B343u ^ px.y * 0xD8163841u ^ seed * 0xCB1AB31Fu;
	h ^= h >> 16;
	h *= 0x7FEB352Du;
	h ^= h >> 15;
	h *= 0x846CA68Bu;
	h ^= h >> 16;

	// Keep the pixel when the hash is below the coverage, or when it is not below it for complementary entries.
	uint coverage = dither & VIEW3D_POPULATION_DITHER_COVERAGE_MASK;
	bool below = (h & 0xFFFFu) < coverage;
	bool complement = (dither & VIEW3D_POPULATION_DITHER_COMPLEMENT) != 0;
	return below != complement;
}

// Discard the current pixel if the population dither mask excludes it. Call this from every pixel shader used by population materials.
void View3DPopulationClipDither(uint dither, float2 ss_xy)
{
	// 'clip' with a negative value discards the pixel.
	if (!View3DPopulationDitherKeep(dither, ss_xy))
		clip(-1);
}

// Place a procedural Forward vertex output, computed with the population's proxy element, at a population instance.
// 'v' comes from 'View3DProceduralForwardVertex' using the draw's element constants, whose object-to-world transform is the identity.
View3DForwardVertexOut View3DPopulationForwardVertex(View3DForwardVertexOut v, View3DPopulationInstance inst, View3DPopulationEntry entry, View3DForwardFrame frame)
{
	// Apply the instance placement and tint, then project with the frame camera.
	View3DForwardVertexOut output = v;
	output.ws_vert = View3DPopulationTransform(inst, v.ws_vert);
	output.ws_norm = View3DPopulationTransformNormal(inst, v.ws_norm);
	output.ss_vert = mul(output.ws_vert, frame.w2s);
	output.diff = v.diff * View3DPopulationTint(inst);
	output.dither = entry.dither;
	return output;
}

// Produce the ShadowMap pixel input for a population vertex from caller-generated model-space surface data.
// Population shadow draws render into one shadow view, given by 'draw.views[0][0]', so the instance index selects the list entry only.
View3DShadowVertexOut View3DPopulationShadowVertex(
	float4 ms_vert,
	float4 diff,
	float2 tex0,
	View3DShadowDrawViews draw,
	StructuredBuffer<View3DShadowView> views,
	View3DShadowElement nugget,
	View3DPopulationInstance inst,
	View3DPopulationEntry entry)
{
	// Place the vertex with the element's model transform and the instance transform, then project into the shadow view.
	View3DShadowVertexOut output = (View3DShadowVertexOut)0;
	uint view = draw.views[0][0];
	float4 ws_vert = View3DPopulationTransform(inst, View3DProceduralWorldPosition(ms_vert, nugget.m2o, nugget.o2w));
	output.ss_vert = mul(ws_vert, views[view].w2s);
	output.diff = diff * nugget.tint * View3DPopulationTint(inst);
	output.tex0 = mul(float4(tex0, 0, 1), nugget.tex2surf0).xy;
	output.dither = entry.dither;
	output.viewport = view - draw.info.x;
	return output;
}

#endif
#endif
