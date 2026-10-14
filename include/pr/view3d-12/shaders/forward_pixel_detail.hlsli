//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Detail-normal forward pixel families: caller-written forward pixel shaders for simple materials that have detail normals.
//
// Simple materials with detail normals draw through a stock variant of the simple forward family that tilts the interpolated normal with
// world-projected detail-normal layers before the stock shading. This is the detail-normal equivalent of 'forward_pixel.hlsli'; see that header for
// what a forward pixel family is. Build the shading function on 'ApplyDetailNormals' or 'DetailShade' and expand VIEW3D_FORWARD_PIXEL_ENTRY_POINTS:
//
//   #include "pr/view3d-12/shaders/forward_pixel_detail.hlsli"
//   float4 MyShade(inout PSIn In, bool is_front_face)
//   {
//       In = ApplyDetailNormals(In);                          // Tilt the normal by the detail layers, or call 'DetailShade' for the whole stock shading.
//       ...                                                   // Modify 'In', for example its colour or normal.
//       return ForwardShade(In, is_front_face).diff;          // The stock simple-material shading, in linear colour before dithering.
//   }
//   VIEW3D_FORWARD_PIXEL_ENTRY_POINTS(My, MyShade)
//
// Pass the bytecode to ShaderOptions::m_procedural.m_forward_pixel with 'm_forward_pixel_model' set to EForwardPixelModel::SimpleDetail.
// A detail-normal family is used only by simple materials that have detail normals. Drawing it with any other material fails at render time,
// and a simple material with detail normals rejects a family written for EForwardPixelModel::Simple.
//
// Resources are as described in 'forward_pixel.hlsli'. The detail-normal constants ('g_detail') and maps are also declared by this header.
//
// The stock shaders and their structures are internal to View3D and may change with any package version.
#ifndef PR_VIEW3D_FORWARD_PIXEL_DETAIL_HLSLI
#define PR_VIEW3D_FORWARD_PIXEL_DETAIL_HLSLI
#include "pr/view3d-12/shaders/forward_pixel.hlsli"

// Detail-normal layer constants. The slope map uses the PBR normal-map slot (t12/s7), and the optional strength map uses the PBR
// roughness slot (t5/s5). Simple materials do not otherwise use either slot.
ConstantBuffer<CBufDetailNormals> g_detail : register(b7);

// The number of fixed, evenly spaced directions that a strength map's turns are drawn in. A turn between two of them cross-fades their patterns.
static const int DetailTurnDirections = 16;

// Return the bilinearly filtered (height scale, added slope variance) strength at world position 'ws'. Without a strength map it is (1, 0).
float2 DetailStrength(float3 ws)
{
	// The map's projection puts [0,1] across the grid, and its clamp sampler extends the edge texels beyond it.
	if (g_detail.info.y == 0)
		return float2(1, 0);

	float2 uv = float2(dot(ws, g_detail.strength_row_u.xyz) + g_detail.strength_row_u.w, dot(ws, g_detail.strength_row_v.xyz) + g_detail.strength_row_v.w);
	return max(g_roughness_texture.SampleLevel(g_roughness_sampler, uv, 0).xy, 0.0f);
}

// Return a pseudo-random value in [0,1) for lattice cell 'cell' of the noise pattern selected by 'seed'.
float DetailNoiseHash(int2 cell, uint seed)
{
	// Mix the cell coordinates and seed so neighbouring cells and patterns are uncorrelated.
	uint h = (uint(cell.x) * 0x8DA6B343u) ^ (uint(cell.y) * 0xD8163841u) ^ (seed * 0xCB1AB31Fu);
	h = (h ^ (h >> 16)) * 0x7FEB352Du;
	h = (h ^ (h >> 15)) * 0x846CA68Bu;
	h ^= h >> 16;
	return float(h) * (1.0f / 4294967296.0f);
}

// Return smooth value noise in [0,1] at 'p', measured in noise cells. 'seed' selects an independent pattern.
float DetailValueNoise(float2 p, uint seed)
{
	// Blend the random values at the four surrounding lattice points with a smooth curve, so the noise has no creases at cell edges.
	float2 cell = floor(p);
	float2 f = p - cell;
	f = f * f * (3.0f - 2.0f * f);
	int2 c = int2(cell);
	float n00 = DetailNoiseHash(c, seed);
	float n10 = DetailNoiseHash(c + int2(1, 0), seed);
	float n01 = DetailNoiseHash(c + int2(0, 1), seed);
	float n11 = DetailNoiseHash(c + int2(1, 1), seed);
	return lerp(lerp(n00, n10, f.x), lerp(n01, n11, f.x), f.y);
}

// Return smooth noise in [-1,1] at 'p', measured in noise cells. 'seed' selects an independent pattern.
float DetailNoiseWarp(float2 p, uint seed)
{
	// Centre the value noise on zero so that the average shift is zero.
	return 2.0f * DetailValueNoise(p, seed) - 1.0f;
}

// Add the layers at world position 'ws' to 'gradient', scaled by 'gradient_weight', and to 'variance', scaled by 'variance_weight'. The layers are
// drawn at height scale 'strength' and turned by 'turn' = (cos a, sin a) from their u direction toward their v direction about the world origin.
// 'ws_dx' and 'ws_dy' are the screen derivatives of 'ws', which select the mip level, because callers may run this in non-uniform flow control.
void AccumulateDetailLayers(float3 ws, float3 ws_dx, float3 ws_dy, float2 turn, float strength, float gradient_weight, float variance_weight, inout float3 gradient, inout float variance)
{
	// Turning the pattern turns the projection rows, so the turned rows give the texture coordinates directly.
	for (int i = 0; i != g_detail.info.x; ++i)
	{
		// Find the layer's texture coordinates in its turned projection. The noise coordinates ignore the rows' offsets, so a layer that scrolls
		// (and wraps its offset) does not move or jump its noise.
		float4 row_u = g_detail.row_u[i];
		float4 row_v = g_detail.row_v[i];
		float3 turned_u = turn.x * row_u.xyz + turn.y * row_v.xyz;
		float3 turned_v = turn.x * row_v.xyz - turn.y * row_u.xyz;
		float2 proj = float2(dot(ws, turned_u), dot(ws, turned_v));
		float2 uv = proj + float2(row_u.w, row_v.w);
		float2 noise_uv = proj * g_detail.noise_frequency[i];

		// World noise varies the layer's weight about 1 so that different layers dominate in different places. Each layer uses its own
		// noise pattern, selected by the layer index.
		float noise = DetailValueNoise(noise_uv, uint(i));
		float scale = strength * g_detail.height_scale[i] * (1.0f + g_detail.weight_noise[i] * (2.0f * noise - 1.0f));

		// Two more noise patterns shift the texture coordinates smoothly, which bends the rows of repeated tiles so they do not line up over
		// large areas. Seeds above the layer limit keep these patterns independent of every layer's weight noise.
		float2 warp = float2(DetailNoiseWarp(noise_uv, uint(DetailNormalsMaxLayers + 2 * i)), DetailNoiseWarp(noise_uv, uint(DetailNormalsMaxLayers + 2 * i + 1)));
		uv += g_detail.warp[i] * warp;

		// Each layer's map slope is per texture unit, so the chain rule through the turned rows gives the world-space height gradient.
		// The map's blue channel holds half the mean square slope. Mip filtering averages it as well as the slopes, so the gap between the mean
		// square and the square of the mean is the slope variance that the filtered normal no longer shows. It is scaled by the mean squared
		// length of the rows. The warp's own derivatives are small, so the turned rows give the texture-coordinate derivatives.
		float4 texel = g_normal_texture.SampleGrad(g_normal_sampler, uv, float2(dot(ws_dx, turned_u), dot(ws_dx, turned_v)), float2(dot(ws_dy, turned_u), dot(ws_dy, turned_v)));
		float2 slope = texel.xy * 2.0f - 1.0f;
		float unresolved = max(2.0f * texel.z - dot(slope, slope), 0.0f);
		gradient += gradient_weight * scale * (slope.x * turned_u + slope.y * turned_v);
		variance += variance_weight * scale * scale * 0.5f * (dot(turned_u, turned_u) + dot(turned_v, turned_v)) * unresolved;
	}
}

// Return 'In' with its world normal tilted by the summed height gradient of the detail-normal layers. Also sets the unresolved slope
// variance used by the shading, from the layers' detail that the mip filtering has averaged away plus the base variance.
PSIn ApplyDetailNormals(PSIn In)
{
	// Fragments without an interpolated normal keep the stock fallback normal.
	float3 normal = In.ws_norm.xyz;
	if (dot(normal, normal) == 0.0f)
		return In;

	normal = normalize(normal);

	// Without a strength map the layers are drawn once, unscaled and unturned. The screen derivatives are taken here, in uniform flow control.
	float3 ws = In.ws_vert.xyz;
	float3 ws_dx = ddx(ws);
	float3 ws_dy = ddy(ws);
	float3 gradient = float3(0, 0, 0);
	float variance = g_detail.surface.x;
	if (g_detail.info.y == 0)
	{
		AccumulateDetailLayers(ws, ws_dx, ws_dy, float2(1, 0), 1.0f, 1.0f, 1.0f, gradient, variance);
	}
	else
	{
		// Find the 2x2 texels around the fragment and their bilinear weights. Outside the grid both texels of a column or row clamp to the edge
		// texel, so that texel takes the whole weight and is drawn once.
		uint width, height;
		g_roughness_texture.GetDimensions(width, height);
		int2 last = int2(width, height) - 1;
		float2 size = float2(width, height);
		float2 uv = float2(dot(ws, g_detail.strength_row_u.xyz) + g_detail.strength_row_u.w, dot(ws, g_detail.strength_row_v.xyz) + g_detail.strength_row_v.w);
		float2 t = uv * size - 0.5f;
		float2 f = t - floor(t);
		int2 c0 = clamp(int2(floor(t)), 0, last);
		int2 c1 = clamp(int2(floor(t)) + 1, 0, last);
		if (c0.x == c1.x) f.x = 0.0f;
		if (c0.y == c1.y) f.y = 0.0f;

		// Patterns are drawn only in fixed directions, so a changing angle cross-fades between fixed patterns instead of moving one.
		// Each texel splits its weight between the two fixed directions either side of its angle, and texels that share a direction share its
		// pattern, so each distinct direction is gathered once: its total weight, and its total of weight times strength.
		int dir_index[8];
		float dir_weight[8];
		float dir_strength[8];
		int dir_count = 0;
		for (int k = 0; k != 4; ++k)
		{
			// Select the corner's texel and bilinear weight, and add its base slope variance. Texels without weight or strength add no pattern.
			int2 cell = int2((k & 1) != 0 ? c1.x : c0.x, (k & 2) != 0 ? c1.y : c0.y);
			float weight = ((k & 1) != 0 ? f.x : 1.0f - f.x) * ((k & 2) != 0 ? f.y : 1.0f - f.y);
			float4 texel = g_roughness_texture.Load(int3(cell, 0));
			variance += weight * max(texel.y, 0.0f);
			if (weight == 0.0f || texel.x <= 0.0f)
				continue;

			// Measure the angle in steps between fixed directions. atan2 is in [-pi, pi], so adding a full turn keeps the steps positive.
			float steps = atan2(texel.w, texel.z) * (DetailTurnDirections / 6.28318530718f) + DetailTurnDirections;
			float first = floor(steps);
			float fade = steps - first;
			for (int j = 0; j != 2; ++j)
			{
				// Add this side's share to its direction's totals, adding the direction if no other texel uses it yet.
				float share = weight * (j == 0 ? 1.0f - fade : fade);
				if (share == 0.0f)
					continue;

				int dir = (int(first) + j) % DetailTurnDirections;
				int slot = dir_count;
				for (int s = 0; s != dir_count; ++s)
				{
					// Find the direction among those already gathered.
					if (dir_index[s] == dir)
						slot = s;
				}
				if (slot == dir_count)
				{
					// Start the totals of a new direction.
					dir_index[slot] = dir;
					dir_weight[slot] = 0.0f;
					dir_strength[slot] = 0.0f;
					++dir_count;
				}
				dir_weight[slot] += share;
				dir_strength[slot] += share * texel.x;
			}
		}

		// Patterns in different directions are unrelated, so a weighted sum of them has a smaller spread of slopes than any one of them: the
		// square root of the sum of the squared weights, instead of the sum of the weights. Scaling the slopes by the ratio of the two keeps the
		// contrast steady through a cross-fade. The unresolved variance is an average, so it keeps the plain weights.
		float total_weight = 0.0f;
		float total_square = 0.0f;
		for (int d = 0; d != dir_count; ++d)
		{
			// Sum the weights and their squares.
			total_weight += dir_weight[d];
			total_square += dir_weight[d] * dir_weight[d];
		}
		float contrast = dir_count != 0 ? total_weight * rsqrt(total_square) : 1.0f;

		// Draw each direction once, at the weighted mean strength of the texels that share it.
		for (int n = 0; n != dir_count; ++n)
		{
			// Find the direction's turn, then draw its layers.
			float2 turn;
			sincos(dir_index[n] * (6.28318530718f / DetailTurnDirections), turn.y, turn.x);
			AccumulateDetailLayers(ws, ws_dx, ws_dy, turn, dir_strength[n] / dir_weight[n], contrast * dir_weight[n], dir_weight[n], gradient, variance);
		}
	}
	g_unresolved_slope_variance = variance;

	// A height field displaced along the normal tilts the normal against the tangential part of its gradient.
	gradient -= dot(gradient, normal) * normal;
	In.ws_norm = float4(normalize(normal - gradient), 0);
	return In;
}

// Shade a simple-material fragment with its normal tilted by the detail-normal layers.
float4 DetailShade(inout PSIn In, bool is_front_face)
{
	// Keep the tilted normal in 'In' so the reflection attributes use it too.
	In = ApplyDetailNormals(In);
	float4 diff = ForwardShade(In, is_front_face).diff;

	// The visualisation replaces the shaded colour with the height strength, from blue at 0 through green at 1 to red at 2 or more.
	if (g_detail.info.z != 0)
	{
		float t = saturate(DetailStrength(In.ws_vert.xyz).x * 0.5f);
		diff.rgb = saturate(float3(2.0f * t - 1.0f, 1.0f - abs(2.0f * t - 1.0f), 1.0f - 2.0f * t));
	}
	return diff;
}

#endif
