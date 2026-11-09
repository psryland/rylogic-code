//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Public shader contract for population (GPU-driven instanced) rendering.
//
// A population draws many placements of one or more prototypes. Each placement is a 'View3DPopulationInstance' record in a GPU buffer that names its prototype.
// For each consuming view (the main camera or a shadow view), the renderer selects the visible instances and their LODs on the GPU in one pass over all prototypes.
// It writes one list of 'View3DPopulationEntry' records per (prototype, LOD), then draws each (prototype, LOD, nugget) triple with one indirect draw.
// The vertex shader of a population draw finds its entry with 'View3DPopulationFetch' and places the vertex with the instance transform.
//
// Instance transforms are affine (rotation, nonuniform scale, shear, mirroring, and translation). Normals use the cofactor of the linear part,
// so they stay perpendicular to surfaces under nonuniform scale. Projective transforms are not supported.
//
// Smooth LOD transitions use screen-door dithering, not alpha blending. During a transition, one instance appears in two adjacent LOD lists
// with complementary dither masks. Prototype distance fade bands use the same dither, so a far prototype can take over from a near one.
// A population-wide coverage (see View3D_PopulationCoverageSet) further thins every entry with the same dither, to dissolve a population in or out.
// Every pixel shader used by a population material must call 'View3DPopulationClipDither' (the stock forward
// and shadow pixel shaders do). The dither value is zero for ordinary (non-population) draws, which keeps every pixel.
//
// A LOD can generate or deform its vertices with procedural vertex shaders (see procedural_vertex.hlsli). The LOD object must use the
// ProceduralVertexId vertex source, and its procedural ForwardRender and ShadowMap vertex shaders replace the stock population vertex shaders.
// Each wrapper builds its vertex with the procedural helpers, finds its instance with 'View3DPopulationFetch', and places the vertex with
// 'View3DPopulationForwardVertex' or 'View3DPopulationShadowVertex'. Use the same deformation code in both wrappers so shadows match the surface.
// The renderer cannot detect changes that only the shaders see (such as new procedural constants), so shadow views that contain such a
// population re-render every frame instead of using their cached content. LOD nuggets with the ShadowCastExclude nugget flag are not drawn
// into shadow views, so such a LOD needs no ShadowMap vertex shader.
#ifndef PR_VIEW3D_POPULATION_HLSLI
#define PR_VIEW3D_POPULATION_HLSLI
#include "pr/view3d-12/shaders/procedural_vertex.hlsli"

// Contract version of the population layouts and bindings in this file.
#define VIEW3D_POPULATION_VERSION 3

// The maximum number of LODs in a population prototype.
#define VIEW3D_POPULATION_MAX_LODS 8

// Instance flags in 'View3DPopulationInstance.info.y'. Disabled instances are never drawn.
// A FADED instance is dithered by its own coverage, held as unorm16 in bits 16-31 of the flags word, instead of by its prototype's fade bands.
// With FADE_COMPLEMENT it keeps exactly the pixels that a draw with the same coverage and origin discards, so an ordinary object with the
// dither word from 'View3D_PopulationDitherWord' can hand over to the instance without a pop. Like a fade band, this replaces the LOD transition dither.
// FADED instances that keep no pixels (coverage 0, or coverage 1 with FADE_COMPLEMENT) are culled.
#define VIEW3D_POPULATION_FLAG_ENABLED 0x1
#define VIEW3D_POPULATION_FLAG_FADED 0x2
#define VIEW3D_POPULATION_FLAG_FADE_COMPLEMENT 0x4
#define VIEW3D_POPULATION_FLAG_FADE_SHIFT 16

// Draw-time bindings, available to the vertex and pixel stages of population draws in the forward and shadow render steps.
//  - Instances: 'StructuredBuffer<View3DPopulationInstance>', the population's instance records.
//  - View: 'ByteAddressBuffer', the renderer's per-view visible lists. Read it only through 'View3DPopulationFetch'.
//  - Payload: 'ByteAddressBuffer', the caller's optional payload. Unbound when the population has no payload.
//  - Draw: 'ConstantBuffer<View3DPopulationDraw>', the list and list location of the current draw.
//  - Skin: 'StructuredBuffer<View3DPopulationSkinfluence>', the skin of the LOD model being drawn. Only meaningful for skinned LOD models.
#define VIEW3D_POPULATION_INSTANCES_REGISTER t24
#define VIEW3D_POPULATION_VIEW_REGISTER t25
#define VIEW3D_POPULATION_PAYLOAD_REGISTER t26
#define VIEW3D_POPULATION_SKIN_REGISTER t27
#define VIEW3D_POPULATION_DRAW_REGISTER b8

// Skinned LODs. When a LOD model has a skin, the stock population vertex shaders pose each instance from the payload (procedural LODs do not).
// 'info.z' of a skinned instance is the byte offset of a 'View3DPopulationSkinPose' record in the payload. The record names two bone palettes,
// also in the payload, and the vertex uses the blend of the two poses. A palette holds one row-major float4x4 per bone of the model's skin,
// 64 bytes each, mapping object-space rest positions to object-space posed positions (see 'View3D_ObjectSkinningPalette'). Offsets must be 4-byte aligned.

// Producer bindings, used by compute shaders that generate population content on the GPU (see View3D_PopulationProduce).
//  - Constants: 'ConstantBuffer<T>', the caller's constants.
//  - Instances: 'RWStructuredBuffer<View3DPopulationInstance>', the population's instance records.
//  - LiveCounts: 'RWStructuredBuffer<uint>', one live instance count per block. A block draws instances [first, first + live count).
//  - Payload: 'RWByteAddressBuffer', the caller's optional payload. A null view, which reads zero and discards writes, when the population has no payload.
// Producer views cover exactly the population buffers, so writes outside them are discarded.
// A producer must keep each live count within its block's capacity, and keep the live instances inside the block's published bounds.
// The renderer clamps live counts to capacity and reports the violation through the population diagnostics.
#define VIEW3D_POPULATION_PRODUCER_CONSTANTS_REGISTER b0
#define VIEW3D_POPULATION_PRODUCER_INSTANCES_REGISTER u0
#define VIEW3D_POPULATION_PRODUCER_LIVE_COUNTS_REGISTER u1
#define VIEW3D_POPULATION_PRODUCER_PAYLOAD_REGISTER u2

// Layout of the per-view list header. A population has one visible list per (prototype, LOD). The header starts with four DIAG words of renderer
// diagnostic counters, which callers read through View3D_PopulationStats rather than from shaders. Then it holds one uint per list for each of the
// COUNTS, OFFSETS, and CURSORS fields, in that order, and is padded to a multiple of 16 bytes. Shaders read list offsets only through
// 'View3DPopulationFetch', which takes the location of the OFFSETS field from the draw constants.
#define VIEW3D_POPULATION_HEADER_DIAG 0
#define VIEW3D_POPULATION_HEADER_COUNTS 16

// Bit fields of the dither word in 'View3DPopulationEntry.dither'.
//  - Bits 0-15: coverage, as unorm16.
//  - Bits 16-29: a seed derived from the instance origin, so the mask does not depend on the list order, and instances with the same origin in
//    different populations get the same mask.
//  - Bit 30: dithering is active. Zero means the entry is fully drawn.
//  - Bit 31: complementary. The entry keeps exactly the pixels that the non-complementary entry with the same coverage and seed discards.
#define VIEW3D_POPULATION_DITHER_COVERAGE_MASK 0x0000FFFFu
#define VIEW3D_POPULATION_DITHER_SEED_SHIFT 16
#define VIEW3D_POPULATION_DITHER_SEED_MASK 0x3FFFu
#define VIEW3D_POPULATION_DITHER_ACTIVE 0x40000000u
#define VIEW3D_POPULATION_DITHER_COMPLEMENT 0x80000000u

// One placement of one of a population's prototypes. 64 bytes.
struct View3DPopulationInstance
{
	// Rows of the affine instance-to-world transform: 'world[i] = dot(o2w[i], float4(p, 1))'.
	float4 o2w[3];

	// x = tint colour, packed as 0xAARRGGBB. Multiplies the vertex colour.
	// y = flags, see VIEW3D_POPULATION_FLAG_*.
	// z = caller payload index, interpreted only by caller shaders.
	// w = the index of the prototype that this instance places. Instances with an index outside the population's prototypes are not drawn.
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
	uint list;      // The visible list, one per (prototype, LOD), that this draw renders.
	uint list_base; // Byte offset of the first list entry in the view buffer.
	uint offsets;   // Byte offset of the list offsets field of the header in the view buffer.
	uint pad;
};

// The pose of a skinned instance, in the payload at the byte offset in 'info.z'. 16 bytes.
struct View3DPopulationSkinPose
{
	uint pose0;  // Byte offset of the first bone palette in the payload
	uint pose1;  // Byte offset of the second bone palette in the payload
	float blend; // The weight of 'pose1', in [0,1]
	uint pad;
};

// The bone influences of one skinned vertex. Eight 16-bit bone indices and eight 16-bit unorm weights, low halves first. Matches the model skin.
struct View3DPopulationSkinfluence
{
	uint4 bones;
	uint4 weights;
};

#ifndef __cplusplus

// Return the blend of bone 'bone' from the two palettes of 'pose', as rows of a row-major matrix.
float4x4 View3DPopulationSkinBone(ByteAddressBuffer payload, View3DPopulationSkinPose pose, uint bone)
{
	// Each palette matrix is four float4 rows, 64 bytes
	uint ofs = bone * 64;
	float4x4 m0 = float4x4(
		asfloat(payload.Load4(pose.pose0 + ofs + 0)),
		asfloat(payload.Load4(pose.pose0 + ofs + 16)),
		asfloat(payload.Load4(pose.pose0 + ofs + 32)),
		asfloat(payload.Load4(pose.pose0 + ofs + 48)));
	float4x4 m1 = float4x4(
		asfloat(payload.Load4(pose.pose1 + ofs + 0)),
		asfloat(payload.Load4(pose.pose1 + ofs + 16)),
		asfloat(payload.Load4(pose.pose1 + ofs + 32)),
		asfloat(payload.Load4(pose.pose1 + ofs + 48)));
	return lerp(m0, m1, pose.blend);
}

// Pose an object-space rest vertex and normal of a skinned population instance. 'skin_index' is the vertex's skin index ('idx0.x');
// negative indices mean the vertex has no skin and is left unchanged.
void View3DPopulationSkinVertex(
	ByteAddressBuffer payload,
	StructuredBuffer<View3DPopulationSkinfluence> skin,
	View3DPopulationInstance inst,
	int skin_index,
	inout float4 os_vert,
	inout float4 os_norm)
{
	// Unused vertex slots can hold a negative skin index
	if (skin_index < 0)
		return;

	// Read the instance pose record and the vertex influences
	uint4 raw = payload.Load4(inst.info.z);
	View3DPopulationSkinPose pose;
	pose.pose0 = raw.x;
	pose.pose1 = raw.y;
	pose.blend = asfloat(raw.z);
	pose.pad = raw.w;
	View3DPopulationSkinfluence influence = skin[skin_index];

	// Accumulate the weighted bone transforms. Zero weights are skipped, because most vertices use few of the eight influences.
	float3 vert = 0;
	float3 norm = 0;
	[unroll] for (uint i = 0; i != 8; ++i)
	{
		// Influence 'i' is in the low half of component 'i/2' for even 'i', and the high half for odd 'i'
		uint shift = (i & 1) * 16;
		float weight = ((influence.weights[i / 2] >> shift) & 0xFFFF) / 65535.0f;
		if (weight == 0)
			continue;

		float4x4 m = View3DPopulationSkinBone(payload, pose, (influence.bones[i / 2] >> shift) & 0xFFFF);
		vert += mul(os_vert, m).xyz * weight;
		norm += mul(os_norm, m).xyz * weight;
	}
	os_vert = float4(vert, 1);
	os_norm = float4(norm, 0);
}

// Read the list entry for 'instance_id' (SV_InstanceID) of the current population draw.
View3DPopulationEntry View3DPopulationFetch(ByteAddressBuffer view, View3DPopulationDraw draw, uint instance_id)
{
	// The list's first entry is written by the renderer's classification pass. SV_InstanceID does not include the draw's start instance.
	uint first = view.Load(draw.offsets + draw.list * 4);
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
	output.ss_vert = mul(output.ws_vert, frame.cam.w2s);
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
