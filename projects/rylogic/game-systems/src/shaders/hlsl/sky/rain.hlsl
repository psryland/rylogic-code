//************************************
// Game Systems
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Rain shaders. They replace the vertex and pixel stages of the forward pass.
// VS: Places each drop in its repeating tile around the camera, and expands it into a screen-facing streak along its motion relative to the camera.
// PS: Draws the streak with soft sides, brighter toward its head. The streaks are transparent, so they draw in the forward pass's weighted
//     transparency sub-pass (PSRainWeighted), where they blend with other transparent surfaces such as water. PSRain is used when the window
//     has no transparency buffers, and outputs straight (not premultiplied) colour for the stock alpha blend.
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward.hlsl"
#include "game-systems/src/shaders/hlsl/sky/rain_cbuf.hlsli"

ConstantBuffer<CBufRain> g_rain : register(b3);

// Drops nearer than this depth are not drawn, and streaks that reach nearer are cut at this depth.
static const float NearDepth = 0.05;

// The range of intensity over which each drop fades in, so changing the intensity does not make drops pop in or out.
static const float IntensityFade = 0.05;

// A vertex that the rasteriser discards.
PSIn Culled()
{
	PSIn Out = (PSIn)0;
	Out.ss_vert = float4(0, 0, -1, 1);
	return Out;
}

// Each drop is a quad of four vertices that share the drop's data. Vertex colours hold (intensity threshold, corner along the streak (0 = tail,
// 1 = head), corner across the streak (-1 or +1), slot). See CBufRain.
PSIn VSRain(VSIn In)
{
	// Decode the drop. Drops whose threshold is above the intensity are not part of this rain.
	float threshold = In.diff.x;
	float along = In.diff.y;
	float across = In.diff.z;
	int slot = (int)(In.diff.w + 0.5);
	int layer = slot / PR_RAIN_SPEED_CLASSES;
	int speed_class = slot % PR_RAIN_SPEED_CLASSES;
	float shown = saturate((g_rain.intensity - threshold) / IntensityFade);
	if (shown <= 0)
		return Culled();

	// Place the drop in its tile, centred on the camera. The offset moves the tile by the drops' drift relative to the camera, and the drop
	// wraps to the far side of the tile when it leaves. Wrapping is never seen because each layer fades out before the edge of its tile.
	float4 lay = g_rain.layers[layer];
	float tile = lay.x;
	float3 rel = (frac(In.vert.xyz + g_rain.offset[slot].xyz) - 0.5) * tile;

	// Each layer fills a shell around the camera. It fades in where the next smaller layer fades out, so the drop density changes smoothly with distance.
	float dist = length(rel);
	float weight = smoothstep(lay.z, lay.w, dist) * (1 - smoothstep(0.35 * tile, 0.5 * tile, dist));
	if (weight <= 0)
		return Culled();

	// The streak is the path of the drop over the exposure time, relative to the camera. Fast streaks are capped in length, and spread the same
	// light over a longer streak, so a drop gets fainter as its streak gets longer.
	float3 trail = g_rain.velocity[speed_class].xyz * g_rain.streak_time;
	float trail_len = length(trail);
	if (trail_len > g_rain.max_length)
	{
		trail *= g_rain.max_length / trail_len;
		trail_len = g_rain.max_length;
	}
	float energy = min(1.0, g_rain.ref_length / max(trail_len, 1e-6));

	// Find the ends in camera space. Drops place relative to the camera, so only the rotation is needed. Cut the streak where it passes too near.
	float3 head = mul(float4(rel, 0), g_rain.w2c).xyz;
	float3 tail = mul(float4(rel - trail, 0), g_rain.w2c).xyz;
	float head_depth = -head.z;
	float tail_depth = -tail.z;
	if (head_depth < NearDepth && tail_depth < NearDepth)
		return Culled();
	if (head_depth < NearDepth)
		head = lerp(tail, head, (tail_depth - NearDepth) / (tail_depth - head_depth));
	else if (tail_depth < NearDepth)
		tail = lerp(head, tail, (head_depth - NearDepth) / (head_depth - tail_depth));

	// Project the ends into pixels from the screen centre.
	float2 half_screen = 0.5 * g_rain.screen_dim.xy;
	float4 head_ss = mul(float4(head, 1), g_rain.c2s);
	float4 tail_ss = mul(float4(tail, 1), g_rain.c2s);
	float2 head_px = head_ss.xy / head_ss.w * half_screen;
	float2 tail_px = tail_ss.xy / tail_ss.w * half_screen;
	float2 delta = head_px - tail_px;
	float len_px = length(delta);
	float2 dir = len_px > 1e-3 ? delta / len_px : float2(0, 1);
	float2 perp = float2(-dir.y, dir.x);

	// Drops are much thinner than a pixel at most distances. They are drawn at a minimum width, with their opacity lowered so that each covers
	// the screen as much as its true size would. One drop of a larger layer stands for many drops of the smallest layer, so it covers that much more.
	float min_hw = 0.5 * g_rain.min_pixels;
	float true_hw = 0.5 * g_rain.half_width * g_rain.screen_dim.z * (1.0 / head_ss.w + 1.0 / tail_ss.w);
	float drawn_hw = max(true_hw, min_hw);
	float coverage = ((len_px + 2 * true_hw) * true_hw) / ((len_px + 2 * drawn_hw) * drawn_hw);
	float alpha = g_rain.colour.a * weight * min(1.0, coverage * lay.y) * energy * shown;
	if (alpha <= 1.0 / 1024.0)
		return Culled();

	// Place this vertex's corner. The streak's caps extend past its ends by its half width. Each corner keeps its own end's depth.
	bool is_head = along > 0.5;
	float4 end_ss = is_head ? head_ss : tail_ss;
	float2 end_px = is_head ? head_px : tail_px;
	float2 corner = end_px + dir * (is_head ? drawn_hw : -drawn_hw) + perp * (across * drawn_hw);

	// The world position of this corner's end gives the transparency passes the streak's view depth. The inverse of the rotation in 'w2c' is its transpose.
	// Streaks wholly below the clip plane are not drawn; streaks that cross it are cut per pixel. See ClipToPlane.
	float3 head_ws = g_rain.cam_pos.xyz + mul(g_rain.w2c, float4(head, 0)).xyz;
	float3 tail_ws = g_rain.cam_pos.xyz + mul(g_rain.w2c, float4(tail, 0)).xyz;
	if (dot(g_rain.clip_plane.xyz, head_ws) + g_rain.clip_plane.w < 0 && dot(g_rain.clip_plane.xyz, tail_ws) + g_rain.clip_plane.w < 0)
		return Culled();

	PSIn Out = (PSIn)0;
	Out.ss_vert = float4(corner / half_screen * end_ss.w, end_ss.z, end_ss.w);
	Out.ws_vert = float4(is_head ? head_ws : tail_ws, 1);
	Out.diff = float4(g_rain.colour.rgb, alpha);
	Out.tex0 = float2(along, across);
	return Out;
}

// Soft sides, and a tail that is fainter than the head. Returns straight (not premultiplied) colour.
float4 RainShade(PSIn In)
{
	float across = In.tex0.y;
	float alpha = In.diff.a * saturate(1 - across * across) * lerp(0.5, 1.0, saturate(In.tex0.x));
	return float4(In.diff.rgb, alpha);
}

// Discard the parts of a streak below the clip plane, e.g. under a water surface, where the water would otherwise show them dimly.
void ClipToPlane(PSIn In)
{
	// A zero plane gives zero everywhere, so it discards nothing.
	clip(dot(g_rain.clip_plane.xyz, In.ws_vert.xyz) + g_rain.clip_plane.w);
}

// Draw the streak over the scene with the stock alpha blend.
PSOut PSRain(PSIn In)
{
	// Only the part of the streak above the clip plane is drawn
	ClipToPlane(In);
	PSOut Out = (PSOut)0;
	Out.diff = RainShade(In);
	return Out;
}

// Draw the streak into the weighted transparency buffers. Streaks behind opaque surfaces are discarded here, because the sub-pass has no depth test.
PSWeightedOut PSRainWeighted(PSIn In)
{
	// Only the visible part of the streak above the clip plane is drawn
	ClipToPlane(In);
	float visibility = AlphaOpaqueVisibility(In.ss_vert, In.ws_vert);
	float4 diff = RainShade(In);
	diff.a *= visibility;
	return WeightedAlphaOut(In, diff);
}
