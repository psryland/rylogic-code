//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Fork lightning bolt shaders. They replace the geometry and pixel stages of the forward pass, and keep its vertex shader.
// GS: Grows the bolt's channel along its leader, and expands each channel segment into a screen-facing strip with rounded ends. Segments that
//     continue a chain share the round end at their joint, so joints are not lit twice.
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

// A joint where a segment meets the next one in its chain. The two segments share the round end at the joint, divided by the line through
// the joint that halves the angle between them, so no pixel is lit by both. 'normal' points into this segment's side, and is zero where
// the segment has a whole round end of its own.
struct Joint
{
	float2 pos;
	float2 normal;
};

// The joint at screen point 's' (pixels) between a segment arriving along 'dir_in' and one leaving along 'dir_out', with 'sign' choosing the
// side of the leaving segment (+1) or the arriving segment (-1). Near-reversed turns have no useful dividing line, so they keep whole round ends.
Joint MakeJoint(float2 s, float2 dir_in, float2 dir_out, float sign, bool joined)
{
	Joint j;
	j.pos = s;
	float2 bisector = dir_in + dir_out;
	float len = length(bisector);
	j.normal = joined && len > 0.1 ? sign * bisector / len : float2(0, 0);
	return j;
}

// The direction in pixels from screen point 's' to clip space point 'p', or zero when 'p' is behind the near plane and has no useful projection.
float2 ScreenDir(float2 s, float4 p, float2 half_dim, out bool valid)
{
	float2 d = p.xy / p.w * half_dim - s;
	float len = length(d);
	valid = p.w - p.z >= 0.0 && len > 1e-4;
	return valid ? d / len : float2(0, 0);
}

// One corner of a segment's strip. 'pos' is in pixels from the screen centre. 'local' is the corner's position relative to the segment in pixels,
// along it from its start (x) and across it (y). Depth 'z' is already divided by w. 'ws' is the world position of the corner's end of the segment.
// 'tex0' carries the corner's signed distances (in pixels) into the segment's side of its start and end joints. Pixels with a negative distance belong
// to the neighbouring segment.
PSIn StripCorner(PSIn vert, float2 pos, float z, float4 ws, float2 local, float seg_len, float core_hw, Joint j0, Joint j1)
{
	// Output positions are already divided by w, so the strip's attributes interpolate linearly in screen space.
	PSIn Out = vert;
	Out.ss_vert = float4(pos / (0.5 * g_bolt.screen_dim.xy), z, 1);
	Out.ws_vert = ws;
	Out.ws_norm = float4(local, seg_len, core_hw);
	Out.tex0 = float2(dot(pos - j0.pos, j0.normal), dot(pos - j1.pos, j1.normal));
	return Out;
}

// Each primitive is one segment of the channel, from a parent node (In[1]) to its child node (In[2]). In[0] is the node before the parent in the
// same chain, and In[3] the node after the child, or the segment's own end node where the chain does not continue through that end.
// Vertex colours hold (arrival time, brightness, width, clarity). See LightningNode.
[maxvertexcount(4)]
void GSLightning(lineadj PSIn In[4], inout TriangleStream<PSIn> OutStream)
{
	// The leader grows until it reaches the ground. Branches that have not finished by then stay at the length they reached.
	float grown = min(g_bolt.age, g_bolt.leader_time);
	float a0 = In[1].diff.x;
	float a1 = In[2].diff.x;
	if (a0 >= grown)
		return;

	// A segment that is still growing has a free tip, so it has no joint at its end.
	float t = saturate((grown - a0) / max(a1 - a0, 1e-6));
	PSIn v0 = In[1];
	PSIn v1 = In[2];
	v1.ss_vert = lerp(In[1].ss_vert, In[2].ss_vert, t);
	v1.ws_vert = lerp(In[1].ws_vert, In[2].ws_vert, t);
	v1.diff = lerp(In[1].diff, In[2].diff, t);
	bool joined0 = any(In[0].ws_vert != In[1].ws_vert);
	bool joined1 = any(In[3].ws_vert != In[2].ws_vert) && t >= 1.0;

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
		// Move the start to where the segment crosses the near plane. The new start is not a joint.
		float f = d0 / (d0 - d1);
		p0 = lerp(p0, p1, f);
		w0 = lerp(w0, w1, f);
		joined0 = false;
	}
	else if (d1 < 0.0)
	{
		// Move the end to where the segment crosses the near plane. The new end is not a joint.
		float f = d1 / (d1 - d0);
		p1 = lerp(p1, p0, f);
		w1 = lerp(w1, w0, f);
		joined1 = false;
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

	// Divide the round ends at joints with the neighbouring segments of the chain. A neighbour behind the camera leaves a whole round end.
	bool valid0, valid1;
	float2 dir_prev = -ScreenDir(s0, In[0].ss_vert, half_dim, valid0);
	float2 dir_next = ScreenDir(s1, In[3].ss_vert, half_dim, valid1);
	Joint j0 = MakeJoint(s0, dir_prev, dir, +1.0, joined0 && valid0);
	Joint j1 = MakeJoint(s1, dir, dir_next, -1.0, joined1 && valid1);

	// Size each end for its own distance, and extend the strip past both ends so the segment has rounded caps that join its neighbours.
	float hw0 = CoreHalfWidth(v0.diff.z, v0.diff.y, p0.w);
	float hw1 = CoreHalfWidth(v1.diff.z, v1.diff.y, p1.w);
	float r0 = hw0 * HaloScale;
	float r1 = hw1 * HaloScale;
	OutStream.Append(StripCorner(v0, s0 - (dir + perp) * r0, z0, w0, float2(-r0, -r0), seg_len, hw0, j0, j1));
	OutStream.Append(StripCorner(v0, s0 - (dir - perp) * r0, z0, w0, float2(-r0, +r0), seg_len, hw0, j0, j1));
	OutStream.Append(StripCorner(v1, s1 + (dir - perp) * r1, z1, w1, float2(seg_len + r1, -r1), seg_len, hw1, j0, j1));
	OutStream.Append(StripCorner(v1, s1 + (dir + perp) * r1, z1, w1, float2(seg_len + r1, +r1), seg_len, hw1, j0, j1));
	OutStream.RestartStrip();
}

// The channel's light at one pixel, as straight (not premultiplied) colour. The opacity is the brightest channel, so the light shows at its
// full brightness where it is bright and fades into what is behind it where it is faint.
float4 LightningShade(PSIn In)
{
	// Pixels past a joint's dividing line are drawn by the neighbouring segment, so the shared round end is lit once.
	clip(min(In.tex0.x, In.tex0.y));

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
