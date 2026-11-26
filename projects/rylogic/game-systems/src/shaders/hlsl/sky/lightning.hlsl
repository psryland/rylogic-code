//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Fork lightning bolt shaders. They replace the geometry and pixel stages of the forward pass, and keep its vertex shader.
// GS: Grows the bolt's channel along its leader, and expands each channel segment into a screen-facing strip with rounded ends.
// PS: Draws a bright core with a soft halo around it. The bolt is transparent, so it draws in the forward pass's weighted transparency sub-pass
//     (PSLightningWeighted), where it blends with other transparent surfaces such as water. PSLightning is used when the window has no
//     transparency buffers, and outputs straight (not premultiplied) colour for the stock alpha blend.
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward.hlsl"
#include "game-systems/src/shaders/hlsl/sky/lightning_cbuf.hlsli"

ConstantBuffer<CBufLightning> g_bolt : register(b3);

// The half width of the drawn strip, as a multiple of the core's half width, so the halo fits inside it.
static const float HaloScale = 4.0;

// The nearest depth to the far plane at which the bolt is drawn. The transparency passes discard fragments at or behind the opaque depth,
// which is the far plane (0, reversed depth) where nothing is drawn, so the bolt must stay just in front of it.
static const float FarDepth = 1e-7;

// The half width of a channel's core in pixels, for a channel 'width' world units wide at clip space 'w', with relative 'brightness'.
// Distant channels are drawn at a minimum size that scales with brightness, so the main channel stays bold while faint tendrils stay fine.
// The channel narrows as the strike fades, down to a thin line that stays visible until its glow ends.
float CoreHalfWidth(float width, float brightness, float w)
{
	float narrowing = (1.0 - g_bolt.fade) * (1.0 - g_bolt.fade);
	float world_hw = 0.5 * width * g_bolt.screen_dim.z / w;
	return max(g_bolt.thin_pixels, max(g_bolt.min_pixels * brightness, world_hw) * narrowing);
}

// One corner of a segment's strip. 'pos' is in pixels from the screen centre. 'local' is the corner's position relative to the segment in pixels,
// along it from its start (x) and across it (y). Depth 'z' is already divided by w. 'ws' is the world position of the corner's end of the segment.
PSIn StripCorner(PSIn vert, float2 pos, float z, float4 ws, float2 local, float seg_len, float core_hw)
{
	// Output positions are already divided by w, so the strip's attributes interpolate linearly in screen space.
	PSIn Out = vert;
	Out.ss_vert = float4(pos / (0.5 * g_bolt.screen_dim.xy), z, 1);
	Out.ws_vert = ws;
	Out.ws_norm = float4(local, seg_len, core_hw);
	return Out;
}

// Each line is one segment of the channel, from a parent node (In[0]) to its child node (In[1]).
// Vertex colours hold (arrival time, brightness, width, clarity). See LightningNode.
[maxvertexcount(4)]
void GSLightning(line PSIn In[2], inout TriangleStream<PSIn> OutStream)
{
	// The leader grows until it reaches the ground. Branches that have not finished by then stay at the length they reached.
	float grown = min(g_bolt.age, g_bolt.leader_time);
	float a0 = In[0].diff.x;
	float a1 = In[1].diff.x;
	if (a0 >= grown)
		return;

	float t = saturate((grown - a0) / max(a1 - a0, 1e-6));
	PSIn v0 = In[0];
	PSIn v1 = In[1];
	v1.ss_vert = lerp(In[0].ss_vert, In[1].ss_vert, t);
	v1.ws_vert = lerp(In[0].ws_vert, In[1].ws_vert, t);
	v1.diff = lerp(In[0].diff, In[1].diff, t);

	// Clip the segment against the near plane. Under reversed depth the near plane is at z == w, and visible points have w - z >= 0.
	// Points behind the camera must be clipped before the divide by 'w', or they project to the far side of the screen.
	float4 p0 = v0.ss_vert;
	float4 p1 = v1.ss_vert;
	float4 w0 = v0.ws_vert;
	float4 w1 = v1.ws_vert;
	float d0 = p0.w - p0.z;
	float d1 = p1.w - p1.z;
	if (d0 < 0.0 && d1 < 0.0)
		return;

	if (d0 < 0.0)
	{
		// Move the start to where the segment crosses the near plane.
		float f = d0 / (d0 - d1);
		p0 = lerp(p0, p1, f);
		w0 = lerp(w0, w1, f);
	}
	else if (d1 < 0.0)
	{
		// Move the end to where the segment crosses the near plane.
		float f = d1 / (d1 - d0);
		p1 = lerp(p1, p0, f);
		w1 = lerp(w1, w0, f);
	}

	// The bolt reaches far into the sky, so parts beyond the far plane are drawn just in front of it rather than clipped.
	// They are still hidden behind any scene geometry because fragments behind the opaque depth are not drawn.
	float z0 = max(p0.z / p0.w, FarDepth);
	float z1 = max(p1.z / p1.w, FarDepth);

	// Work in pixels from the screen centre, where directions and distances are not distorted by the screen's aspect ratio.
	float2 half_dim = 0.5 * g_bolt.screen_dim.xy;
	float2 s0 = p0.xy / p0.w * half_dim;
	float2 s1 = p1.xy / p1.w * half_dim;
	float2 lin = s1 - s0;
	float seg_len = length(lin);
	float2 dir = seg_len > 1e-4 ? lin / seg_len : float2(1, 0);
	float2 perp = float2(-dir.y, dir.x);

	// Size each end for its own distance, and extend the strip past both ends so the segment has rounded caps that join its neighbours.
	float hw0 = CoreHalfWidth(v0.diff.z, v0.diff.y, p0.w);
	float hw1 = CoreHalfWidth(v1.diff.z, v1.diff.y, p1.w);
	float r0 = hw0 * HaloScale;
	float r1 = hw1 * HaloScale;
	OutStream.Append(StripCorner(v0, s0 - (dir + perp) * r0, z0, w0, float2(-r0, -r0), seg_len, hw0));
	OutStream.Append(StripCorner(v0, s0 - (dir - perp) * r0, z0, w0, float2(-r0, +r0), seg_len, hw0));
	OutStream.Append(StripCorner(v1, s1 + (dir - perp) * r1, z1, w1, float2(seg_len + r1, -r1), seg_len, hw1));
	OutStream.Append(StripCorner(v1, s1 + (dir + perp) * r1, z1, w1, float2(seg_len + r1, +r1), seg_len, hw1));
	OutStream.RestartStrip();
}

// The channel's light at one pixel, as straight (not premultiplied) colour. The opacity is the brightest channel, so the light shows at its
// full brightness where it is bright and fades into what is behind it where it is faint.
float4 LightningShade(PSIn In)
{
	// The distance from the pixel to the segment, in units of the core's half width.
	float2 local = In.ws_norm.xy;
	float seg_len = In.ws_norm.z;
	float core_hw = In.ws_norm.w;
	float along = max(0.0, max(-local.x, local.x - seg_len));
	float u = length(float2(along, local.y)) / core_hw;

	// A sharp white-hot core inside a wide, faint, coloured halo. Dim branches fade faster than the main channel as the strike fades.
	float core = exp(-2.0 * u * u);
	float halo = 0.25 * exp(-0.25 * u * u);
	float brightness = In.diff.y;
	float clarity = In.diff.w;
	float level = g_bolt.glow * pow(brightness, 1.0 + 2.0 * g_bolt.fade) * clarity;
	float3 colour = g_bolt.colour.rgb;
	float3 light = level * (core * lerp(colour, 1.0, 0.7) + halo * colour);

	// Convert the light to a colour and opacity. Where the light is brighter than 1, it is drawn opaque at its full brightness.
	float peak = max(light.r, max(light.g, light.b));
	float a = saturate(peak);
	return float4(a > 0 ? light / a : float3(0, 0, 0), a);
}

// Draw the channel over the scene with the stock alpha blend.
PSOut PSLightning(PSIn In)
{
	PSOut Out = (PSOut)0;
	Out.diff = LightningShade(In);
	return Out;
}

// Draw the channel into the weighted transparency buffers. Parts behind opaque surfaces are discarded here, because the sub-pass has no depth test.
PSWeightedOut PSLightningWeighted(PSIn In)
{
	float visibility = AlphaOpaqueVisibility(In.ss_vert, In.ws_vert);
	float4 diff = LightningShade(In);
	diff.a *= visibility;
	return WeightedAlphaOut(In, diff);
}
