//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2014
//***********************************************
// Renders scene depth into the shadow atlas. Each instance of a draw renders into one shadow view.
// The views of a draw are selected by root constants, and the atlas region of each view is selected by the viewport index.

#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/camera.hlsli"
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/lighting/lighting_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/shadow/shadow_map_cbuf.hlsli"
#include "pr/view3d-12/shaders/population.hlsli"

// Constant buffers
ConstantBuffer<CBufDrawViews> resource(g_draw, b0);
ConstantBuffer<CBufElement> resource(g_element, b1);

// The render step's per-element constants, selected by the draw's element index
StructuredBuffer<ElementConstants> resource(g_elements, t2);
static const ElementConstants g_nugget = g_elements[g_element.index];

// Texture2D /w sampler
Texture2D<float4> resource(m_texture0, t0);
SamplerState      resource(m_sampler0, s0);

// The frame's shadow views
StructuredBuffer<ShadowView> resource(g_shadow_views, t1);

// Must match 'View3DShadowVertexOut' in procedural_vertex.hlsli
struct PSIn_ShadowMap
{
	float4 ss_vert :SV_POSITION;
	float4 diff :COLOR0;
	float2 tex0 :TEXCOORD0;
	uint dither :DITHER0; // Population LOD dither word, zero for ordinary draws
	uint viewport :SV_ViewportArrayIndex;
};

// Default SMAP VS
PSIn_ShadowMap VSShadowMap(VSIn In, uint instance_id :SV_InstanceID)
{
	PSIn_ShadowMap Out = (PSIn_ShadowMap)0;

	// Each instance renders into the view given by the draw's view list
	uint view = g_draw.views[instance_id / 4][instance_id % 4];
	Out.viewport = view - g_draw.info.x;

	// Transform
	float4 os_vert = mul(In.vert, g_nugget.m2o);
	float4 ws_vert = mul(os_vert, g_nugget.o2w);
	Out.ss_vert = mul(ws_vert, g_shadow_views[view].w2s);

	// Tinting and per vertex colour
	Out.diff = In.diff * g_nugget.tint;

	// Texture2D (with transform)
	Out.tex0 = mul(float4(In.tex0, 0, 1), g_nugget.tex2surf0).xy;

	return Out;
}

// Population draw bindings (see 'pr/view3d-12/shaders/population.hlsli')
StructuredBuffer<View3DPopulationInstance> g_pop_instances : register(t24);
ByteAddressBuffer g_pop_view : register(t25);
ConstantBuffer<View3DPopulationDraw> g_pop_draw : register(b8);

// SMAP VS for population draws. A population draw renders into the single view given by 'g_draw.views[0][0]', so the instance index selects
// the population list entry, not a view.
PSIn_ShadowMap VSShadowMapPopulation(VSIn In, uint instance_id :SV_InstanceID)
{
	// Find the instance record for this draw instance, and the draw's view
	PSIn_ShadowMap Out = (PSIn_ShadowMap)0;
	View3DPopulationEntry entry = View3DPopulationFetch(g_pop_view, g_pop_draw, instance_id);
	View3DPopulationInstance inst = g_pop_instances[entry.source];
	uint view = g_draw.views[0][0];
	Out.viewport = view - g_draw.info.x;

	// Place the vertex with the element's model transform and the instance transform, then project into the shadow view
	float4 os_vert = mul(In.vert, g_nugget.m2o);
	float4 ws_vert = View3DPopulationTransform(inst, mul(os_vert, g_nugget.o2w));
	Out.ss_vert = mul(ws_vert, g_shadow_views[view].w2s);

	// Tinting, alpha cut-out texture coordinates, and the LOD transition dither
	Out.diff = In.diff * g_nugget.tint * View3DPopulationTint(inst);
	Out.tex0 = mul(float4(In.tex0, 0, 1), g_nugget.tex2surf0).xy;
	Out.dither = entry.dither;
	return Out;
}

// Default SMAP PS. Only used to discard transparent texels. Depth is written by the rasterizer.
void PSShadowMap(PSIn_ShadowMap In)
{
	// Population LOD transitions discard their dithered pixels first. Ordinary draws keep every pixel.
	View3DPopulationClipDither(In.dither, In.ss_vert.xy);

	// Texture2D (with transform)
	float4 diff = In.diff;
	if (HasTex0(g_nugget.flags))
		diff = m_texture0.Sample(m_sampler0, In.tex0) * diff;

	// Cut-out texels of opaque surfaces do not cast shadows. Alpha blended surfaces cast shadows over their whole area.
	if (!HasAlpha(g_nugget.flags))
		clip(diff.a - 0.5);
}
