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

// Return 32 pseudo-random bits for lattice cell 'cell' of the noise patterns selected by 'seed'.
uint DetailNoiseHash(int2 cell, uint seed)
{
	// Mix the cell coordinates and seed so neighbouring cells and patterns are uncorrelated.
	uint h = (uint(cell.x) * 0x8DA6B343u) ^ (uint(cell.y) * 0xD8163841u) ^ (seed * 0xCB1AB31Fu);
	h = (h ^ (h >> 16)) * 0x7FEB352Du;
	h = (h ^ (h >> 15)) * 0x846CA68Bu;
	h ^= h >> 16;
	return h;
}

// Return three independent smooth value noises in [0,1] at 'p', measured in noise cells. 'seed' selects an independent set of patterns.
float3 DetailValueNoise3(float2 p, uint seed)
{
	// Each lattice point's hash is split into three groups of bits, one random value per pattern. Blending the values at the four surrounding
	// lattice points with a smooth curve gives noise without creases at cell edges. Ten or eleven bits per value are far finer than visible.
	float2 cell = floor(p);
	float2 f = p - cell;
	f = f * f * (3.0f - 2.0f * f);
	int2 c = int2(cell);
	uint4 h = uint4(DetailNoiseHash(c, seed), DetailNoiseHash(c + int2(1, 0), seed), DetailNoiseHash(c + int2(0, 1), seed), DetailNoiseHash(c + int2(1, 1), seed));
	float4 n0 = float4(h & 0x7FFu) * (1.0f / 2048.0f);
	float4 n1 = float4((h >> 11) & 0x7FFu) * (1.0f / 2048.0f);
	float4 n2 = float4(h >> 22) * (1.0f / 1024.0f);
	float4 w = float4((1.0f - f.x) * (1.0f - f.y), f.x * (1.0f - f.y), (1.0f - f.x) * f.y, f.x * f.y);
	return float3(dot(n0, w), dot(n1, w), dot(n2, w));
}

// The number of direction slots that one fragment has: the two fixed directions either side of its filtered angle.
static const int DetailMaxFragmentDirections = 2;

// The fixed directions that one fragment draws its layers in, with the weights and strength of each. A slot with zero 'variance_weight' is not
// drawn. Every loop over the slots or layers is unrolled so that all array indices are constant, because a runtime index forces the arrays
// into slow local memory on GPUs.
struct DetailDirections
{
	float2 turn[DetailMaxFragmentDirections];            // (cos a, sin a), turning a layer from its u direction toward its v direction.
	float strength[DetailMaxFragmentDirections];         // Height scale of the direction's pattern.
	float gradient_weight[DetailMaxFragmentDirections];  // Scale applied to the direction's slopes.
	float variance_weight[DetailMaxFragmentDirections];  // Scale applied to the direction's unresolved slope variance. Zero for an unused slot.
};

// Add the layers at world position 'ws', drawn in every direction of 'dirs', to 'gradient' and to 'variance'. 'ws_dx' and 'ws_dy' are the screen
// derivatives of 'ws', which select the mip level, because callers may run this in non-uniform flow control.
void AccumulateDetailLayers(float3 ws, float3 ws_dx, float3 ws_dy, DetailDirections dirs, inout float3 gradient, inout float variance)
{
	[unroll]
	for (int i = 0; i != DetailNormalsMaxLayers; ++i)
	{
		// Unrolling over the largest layer count selects each layer's packed constants by a fixed component.
		if (i == g_detail.info.x)
			break;

		// Project the position and its derivatives onto the layer's unturned rows. A turn by (c, s) maps these to the turned coordinates
		// (c u + s v, c v - s u), so every direction reuses them. The noise coordinates ignore the rows' offsets, so a layer that scrolls (and
		// wraps its offset) does not move or jump its noise.
		float4 row_u = g_detail.row_u[i];
		float4 row_v = g_detail.row_v[i];
		float2 proj = float2(dot(ws, row_u.xyz), dot(ws, row_v.xyz));
		float2 proj_dx = float2(dot(ws_dx, row_u.xyz), dot(ws_dx, row_v.xyz));
		float2 proj_dy = float2(dot(ws_dy, row_u.xyz), dot(ws_dy, row_v.xyz));
		float2 offset = float2(row_u.w, row_v.w);

		// World noise varies the layer's weight about 1 so that different layers dominate in different places, and two more noise patterns shift
		// the texture coordinates smoothly, which bends the rows of repeated tiles so they do not line up over large areas. Each layer uses its
		// own patterns, selected by the layer index. The noise is in the unturned projection, so every direction shares it.
		float3 noise = DetailValueNoise3(proj * g_detail.noise_frequency[i], uint(i));
		float layer_scale = g_detail.height_scale[i] * (1.0f + g_detail.weight_noise[i] * (2.0f * noise.x - 1.0f));
		float2 warp = g_detail.warp[i] * (2.0f * noise.yz - 1.0f);

		// Sum the slopes of every direction, turned back into the unturned (u, v) frame, and their unresolved variance.
		float2 slope_sum = float2(0, 0);
		float unresolved_sum = 0.0f;
		[unroll]
		for (int d = 0; d != DetailMaxFragmentDirections; ++d)
		{
			// Turn the coordinates and derivatives into this direction's frame. The warp's own derivatives are small, so the turned derivatives
			// select the mip level.
			if (dirs.variance_weight[d] == 0.0f)
				continue;

			float2 t = dirs.turn[d];
			float2 uv = float2(t.x * proj.x + t.y * proj.y, t.x * proj.y - t.y * proj.x) + offset + warp;
			float2 uv_dx = float2(t.x * proj_dx.x + t.y * proj_dx.y, t.x * proj_dx.y - t.y * proj_dx.x);
			float2 uv_dy = float2(t.x * proj_dy.x + t.y * proj_dy.y, t.x * proj_dy.y - t.y * proj_dy.x);

			// The map's blue channel holds half the mean square slope. Mip filtering averages it as well as the slopes, so the gap between the mean
			// square and the square of the mean is the slope variance that the filtered normal no longer shows.
			float4 texel = g_normal_texture.SampleGrad(g_normal_sampler, uv, uv_dx, uv_dy);
			float2 slope = texel.xy * 2.0f - 1.0f;
			float unresolved = max(2.0f * texel.z - dot(slope, slope), 0.0f);
			float scale = dirs.strength[d] * layer_scale;
			slope_sum += dirs.gradient_weight[d] * scale * float2(t.x * slope.x - t.y * slope.y, t.y * slope.x + t.x * slope.y);
			unresolved_sum += dirs.variance_weight[d] * scale * scale * unresolved;
		}

		// Each layer's map slope is per texture unit, so the chain rule through the rows gives the world-space height gradient. The variance is
		// scaled by the mean squared length of the rows, which turning does not change.
		gradient += slope_sum.x * row_u.xyz + slope_sum.y * row_v.xyz;
		variance += unresolved_sum * 0.5f * (dot(row_u.xyz, row_u.xyz) + dot(row_v.xyz, row_v.xyz));
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
	float variance = g_detail.surface.x;
	DetailDirections dirs = (DetailDirections)0;
	if (g_detail.info.y == 0)
	{
		dirs.turn[0] = float2(1, 0);
		dirs.strength[0] = 1.0f;
		dirs.gradient_weight[0] = 1.0f;
		dirs.variance_weight[0] = 1.0f;
	}
	else
	{
		// Filter the strength map bilinearly. The filtered angle vector turns smoothly between texel centres; its length does not matter.
		// The map's projection puts [0,1] across the grid, and its clamp sampler extends the edge texels beyond it.
		float2 uv = float2(dot(ws, g_detail.strength_row_u.xyz) + g_detail.strength_row_u.w, dot(ws, g_detail.strength_row_v.xyz) + g_detail.strength_row_v.w);
		float4 texel = g_roughness_texture.SampleLevel(g_roughness_sampler, uv, 0);
		float strength = max(texel.x, 0.0f);
		variance += max(texel.y, 0.0f);

		// Patterns are drawn only in fixed directions, so a changing angle cross-fades between the two fixed patterns either side of it
		// instead of moving one. Opposite angles can filter to a zero vector, which leaves the layers unturned. atan2 is in [-pi, pi], so
		// adding a full turn keeps the steps positive. The second direction is the first turned by one step.
		float2 angle = dot(texel.zw, texel.zw) != 0.0f ? texel.zw : float2(1, 0);
		float steps = atan2(angle.y, angle.x) * (DetailTurnDirections / 6.28318530718f) + DetailTurnDirections;
		float first = floor(steps);
		float fade = steps - first;
		float2 step_turn = float2(cos(6.28318530718f / DetailTurnDirections), sin(6.28318530718f / DetailTurnDirections));
		sincos(first * (6.28318530718f / DetailTurnDirections), dirs.turn[0].y, dirs.turn[0].x);
		dirs.turn[1] = float2(dirs.turn[0].x * step_turn.x - dirs.turn[0].y * step_turn.y, dirs.turn[0].y * step_turn.x + dirs.turn[0].x * step_turn.y);

		// Both directions draw at the filtered strength, and a fragment without strength draws neither. Patterns in different directions are
		// unrelated, so a weighted sum of them has a smaller spread of slopes than either one: the square root of the sum of the squared weights,
		// instead of the sum of the weights, which is 1. Scaling the slopes by the inverse of that keeps the contrast steady through a cross-fade.
		// The unresolved variance is an average, so it keeps the plain weights.
		float2 share = strength > 0.0f ? float2(1.0f - fade, fade) : float2(0, 0);
		float contrast = rsqrt((1.0f - fade) * (1.0f - fade) + fade * fade);
		dirs.strength[0] = strength;
		dirs.strength[1] = strength;
		dirs.variance_weight[0] = share.x;
		dirs.variance_weight[1] = share.y;
		dirs.gradient_weight[0] = contrast * share.x;
		dirs.gradient_weight[1] = contrast * share.y;
	}
	float3 gradient = float3(0, 0, 0);
	AccumulateDetailLayers(ws, ws_dx, ws_dy, dirs, gradient, variance);
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
