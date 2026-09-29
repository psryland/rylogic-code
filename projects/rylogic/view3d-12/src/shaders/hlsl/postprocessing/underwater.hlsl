//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Underwater post-processing pass. See post_processing.h for the effect settings.
#include "view3d-12/src/shaders/hlsl/postprocessing/underwater_cbuf.hlsli"

static const float TAU = 6.28318530718f;

ConstantBuffer<CBufUnderwater> resource(g_underwater, b0);
Texture2D<float4> resource(g_scene_colour, t0);
Texture2D<float> resource(g_scene_depth, t1);
SamplerState resource(g_linear_clamp, s0);

struct PSIn_PostEffect
{
	float4 ss_vert :SV_Position;
};

// Generate a triangle that covers the whole render target.
PSIn_PostEffect VSPostEffect(uint vid :SV_VertexID)
{
	// Vertices (-1,+1), (+3,+1), (-1,-3) cover NDC; the scissor rect limits the output to the scene viewport.
	PSIn_PostEffect Out = (PSIn_PostEffect)0;
	float2 uv = float2((vid << 1) & 2, vid & 2);
	Out.ss_vert = float4(uv * float2(2, -2) + float2(-1, 1), 0, 1);
	return Out;
}

// Offset of the distortion, in viewport-normalised coordinates.
float2 Distortion(float2 uv, float aspect)
{
	// Measure positions in viewport heights so ripples stay round for any aspect ratio.
	float2 q = float2(uv.x * aspect, uv.y) * (g_underwater.distortion_frequency * TAU);
	float t = g_underwater.phase;

	// Sum waves travelling in different directions so the pattern does not look like a single ripple.
	// Time multipliers are integers so the wrapped phase never causes a visible jump.
	float2 offset;
	offset.x = sin(q.y + t) + 0.5f * sin(0.7f * (q.x + q.y) - 2.0f * t);
	offset.y = cos(q.x + t) + 0.5f * sin(0.8f * (q.x - q.y) + 2.0f * t);

	// Scale so the largest offset equals the amplitude, then convert viewport heights to normalised x.
	offset *= g_underwater.distortion_amplitude / 1.5f;
	offset.x /= aspect;
	return offset;
}

// Distance from the camera to the surface at 'pixel', or infinity where no geometry was drawn.
float SurfaceDistance(int2 pixel, float2 uv)
{
	// Without scene depth, treat every pixel as distant.
	if (g_underwater.has_depth == 0)
		return asfloat(0x7F800000);

	// Pixels where no geometry was drawn show open water, so they are also infinitely distant.
	float depth = g_scene_depth.Load(int3(pixel, 0));
	if (depth == g_underwater.clear_depth)
		return asfloat(0x7F800000);

	// Rebuild the camera-space position from the depth value. Orthographic views measure along the view direction.
	float4 ndc = float4(uv.x * 2.0f - 1.0f, 1.0f - uv.y * 2.0f, depth, 1.0f);
	float4 cs = mul(ndc, g_underwater.s2c);
	float3 pos = cs.xyz / cs.w;
	return g_underwater.orthographic != 0 ? abs(pos.z) : length(pos);
}

// Tint, fog, and distort the scene colour.
float4 PSUnderwater(PSIn_PostEffect In) :SV_Target
{
	// Find the distorted sample point, kept inside the scene viewport so no outside pixels leak in.
	float4 vp = g_underwater.viewport;
	float2 uv = (In.ss_vert.xy - vp.xy) / vp.zw;
	float2 sample_uv = saturate(uv + Distortion(uv, vp.z / vp.w));
	float2 pixel = clamp(vp.xy + sample_uv * vp.zw, vp.xy + 0.5f, vp.xy + vp.zw - 0.5f);

	// Read the scene colour (linear, through the sRGB view) at the distorted point.
	float2 target_size;
	g_scene_colour.GetDimensions(target_size.x, target_size.y);
	float4 colour = g_scene_colour.SampleLevel(g_linear_clamp, pixel / target_size, 0);

	// Tint the colour, then fade towards the fog colour. The fog reaches 95% at the visibility distance.
	float dist = SurfaceDistance(int2(pixel), sample_uv);
	float fog = 1.0f - exp(-3.0f * dist / g_underwater.visibility);
	colour.rgb = lerp(colour.rgb * g_underwater.tint.rgb, g_underwater.fog_colour.rgb, fog);
	return colour;
}
