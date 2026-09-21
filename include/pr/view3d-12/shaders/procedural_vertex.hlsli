//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#ifndef PR_VIEW3D_PROCEDURAL_VERTEX_HLSLI
#define PR_VIEW3D_PROCEDURAL_VERTEX_HLSLI
#include "pr/hlsl/core.hlsli"
#include "pr/hlsl/camera.hlsli"

// Contract version implemented by the public ShaderOptions descriptor.
#define VIEW3D_PROCEDURAL_VERTEX_VERSION 1

// Renderer-owned and caller-owned constants occupy these fixed stage-local registers.
#define VIEW3D_FORWARD_FRAME_REGISTER b0
#define VIEW3D_FORWARD_NUGGET_REGISTER b1
#define VIEW3D_PROCEDURAL_FORWARD_CONSTANTS_REGISTER b6
#define VIEW3D_RAYCAST_NUGGET_REGISTER b1
#define VIEW3D_PROCEDURAL_RAYCAST_CONSTANTS_REGISTER b2
#define VIEW3D_SHADOW_FRAME_REGISTER b0
#define VIEW3D_SHADOW_NUGGET_REGISTER b1
#define VIEW3D_PROCEDURAL_SHADOW_CONSTANTS_REGISTER b2

// Stock per-nugget constants shared by Forward and ShadowMap procedural vertex wrappers.
struct View3DForwardShadowNugget
{
	int4 flags;
	row_major float4x4 m2o;
	row_major float4x4 o2w;
	row_major float4x4 o2s;
	row_major float4x4 n2w;
	row_major float4x4 tex2surf0;
	float4 tint;
	float4 colour_blend;
	float env_reflectivity;
	float3 far_clip_fade;
};

// Stock Forward frame data needed to transform generated world-space positions.
struct View3DForwardFrame
{
	row_major float4x4 c2w;
	row_major float4x4 c2s;
	row_major float4x4 w2c;
	row_major float4x4 w2s;
};

// Stock Forward pixel-shader input emitted by a procedural vertex wrapper.
struct View3DForwardVertexOut
{
	float4 ss_vert : SV_POSITION;
	float4 ws_vert : POSITION1;
	float4 ws_norm : NORMAL0;
	float4 diff : COLOR0;
	float2 tex0 : TEXCOORD0;
	float2 idx0 : INDICES0;
};

// Stock per-nugget constants consumed by a procedural RayCast vertex wrapper.
struct View3DRayCastNugget
{
	int4 flags;
	row_major float4x4 m2o;
	row_major float4x4 o2w;
	row_major float4x4 n2w;
	uint2 inst_ptr;
};

// Stock face-geometry-shader input emitted by a procedural RayCast vertex wrapper.
struct View3DRayCastVertexOut
{
	float4 ws_vert : WSVertex;
};

// Stock shadow frame constants consumed by a procedural ShadowMap vertex wrapper.
struct View3DShadowFrame
{
	row_major float4x4 w2l;
	row_major float4x4 l2s;
};

// Stock ShadowMap pixel-shader input emitted by a procedural vertex wrapper.
struct View3DShadowVertexOut
{
	float4 ss_vert : SV_POSITION;
	float4 ws_vert : POSITION1;
	float4 diff : COLOR0;
	float2 tex0 : TEXCOORD0;
};

// Transform a generated model-space position through the stock object placement.
float4 View3DProceduralWorldPosition(float4 ms_vert, View3DForwardShadowNugget nugget)
{
	// Preserve the renderer's model-to-object and object-to-world transform order.
	return mul(mul(ms_vert, nugget.m2o), nugget.o2w);
}

// Produce the stock Forward output from caller-generated model-space surface data.
// Declare the generated capabilities in the nugget: Norm enables lighting of ms_norm (w=0), Colr describes diff, and Tex0 describes UV0.
// Those capabilities do not request physical vertex attributes; the caller owns the validity of each declared shader output.
View3DForwardVertexOut View3DProceduralForwardVertex(
	float4 ms_vert,
	float4 ms_norm,
	float4 diff,
	float2 tex0,
	float2 idx0,
	View3DForwardFrame frame,
	View3DForwardShadowNugget nugget)
{
	// Match the stock Forward vertex-to-pixel semantic and transform contract.
	View3DForwardVertexOut output = (View3DForwardVertexOut)0;
	output.ws_vert = View3DProceduralWorldPosition(ms_vert, nugget);
	output.ss_vert = mul(output.ws_vert, frame.w2s);
	output.ws_norm = mul(ms_norm, nugget.n2w);
	output.diff = diff * nugget.tint;
	output.tex0 = tex0;
	output.idx0 = idx0;
	return output;
}

// Produce the stock RayCast geometry-shader input from a generated model-space position.
View3DRayCastVertexOut View3DProceduralRayCastVertex(float4 ms_vert, View3DRayCastNugget nugget)
{
	// Match the stock RayCast model-to-world transform contract.
	View3DRayCastVertexOut output = (View3DRayCastVertexOut)0;
	output.ws_vert = mul(mul(ms_vert, nugget.m2o), nugget.o2w);
	return output;
}

// Produce the stock ShadowMap pixel input from caller-generated model-space surface data.
View3DShadowVertexOut View3DProceduralShadowVertex(float4 ms_vert, float4 diff, float2 tex0, View3DShadowFrame frame, View3DForwardShadowNugget nugget)
{
	// Match the stock shadow depth, tint, and texture-coordinate contract.
	View3DShadowVertexOut output = (View3DShadowVertexOut)0;
	output.ws_vert = View3DProceduralWorldPosition(ms_vert, nugget);
	float4 ls_vert = mul(output.ws_vert, frame.w2l);
	float2 clip_planes = ClipPlanes(frame.l2s);
	output.ws_vert.w = Frac(clip_planes.y, -ls_vert.z, clip_planes.x);
	output.ss_vert = mul(ls_vert, frame.l2s);
	output.diff = diff * nugget.tint;
	output.tex0 = mul(float4(tex0, 0, 1), nugget.tex2surf0).xy;
	return output;
}

// Return the canonical invalid floating-point sentinel for a violated caller contract.
float View3DProceduralInvalidFloat()
{
	// Preserve one stable quiet-NaN bit pattern across all stage wrappers.
	return asfloat(0x7FC00000);
}

// Return an unmistakable invalid position for a failed procedural evaluation.
float4 View3DProceduralInvalidPosition()
{
	// Invalid generated geometry must not collapse onto a plausible fallback surface.
	float value = View3DProceduralInvalidFloat();
	return float4(value, value, value, value);
}

// Return the canonical diagnostic colour for a failed procedural evaluation.
float4 View3DProceduralInvalidColour()
{
	// Forward diagnostics use an opaque colour that cannot resemble ordinary terrain output.
	return float4(1, 0, 1, 1);
}

#endif
