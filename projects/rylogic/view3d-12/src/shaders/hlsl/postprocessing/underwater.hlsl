//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Underwater post-processing pass. See post_processing.h for the effect settings.
#include "view3d-12/src/shaders/hlsl/postprocessing/underwater_cbuf.hlsli"

static const float TAU = 6.28318530718f;
static const float INF = asfloat(0x7F800000);

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

// Length of the part of the view ray through 'pixel' that is in water, or infinity for an unbounded ray in water.
float WaterPathLength(int2 pixel, float2 uv)
{
	// Pixels without scene geometry show open water or sky. Their ray has no end, so only its direction is used,
	// taken from a point at an arbitrary depth that is valid for any depth range.
	float depth = g_underwater.has_depth != 0 ? g_scene_depth.Load(int3(pixel, 0)) : g_underwater.clear_depth;
	bool unbounded = g_underwater.has_depth == 0 || depth == g_underwater.clear_depth;

	// Rebuild the camera-space ray. Perspective rays start at the camera; orthographic rays start on the camera plane.
	float4 ndc = float4(uv.x * 2.0f - 1.0f, 1.0f - uv.y * 2.0f, unbounded ? 0.5f : depth, 1.0f);
	float4 cs = mul(ndc, g_underwater.s2c);
	float3 end = cs.xyz / cs.w;
	float3 start = g_underwater.orthographic != 0 ? float3(end.xy, 0) : float3(0, 0, 0);
	float3 ray = end - start;
	float len = length(ray);
	float3 dir = ray / len;

	// Without a surface plane, the whole ray is in water.
	float3 normal = g_underwater.surface.xyz;
	if (all(normal == 0))
		return unbounded ? INF : len;

	// Signed heights above the water surface of the ray start, and the rate of climb along the ray.
	float h0 = dot(normal, start) + g_underwater.surface.w;
	float climb = dot(normal, dir);

	// An unbounded ray is in water until it rises through the surface, or forever if it never does.
	if (unbounded)
	{
		// Starting in water, the ray leaves when it climbs through the surface.
		if (h0 < 0)
			return climb > 0 ? -h0 / climb : INF;

		// Starting above the water, a descending ray enters it and never leaves.
		return climb < 0 ? INF : 0;
	}

	// A bounded ray is in water for the part of its length below the surface.
	float h1 = h0 + climb * len;
	if (h0 < 0 && h1 < 0)
		return len;
	if (h0 >= 0 && h1 >= 0)
		return 0;

	float t = h0 / (h0 - h1);
	return h0 < 0 ? t * len : (1 - t) * len;
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

	// Tint the colour, then fade towards the fog colour over the part of the view ray that is in water.
	// The fog reaches 95% after the visibility distance.
	float dist = WaterPathLength(int2(pixel), sample_uv);
	float fog = 1.0f - exp(-3.0f * dist / g_underwater.visibility);
	colour.rgb = lerp(colour.rgb * g_underwater.tint.rgb, g_underwater.fog_colour.rgb, fog);
	return colour;
}
