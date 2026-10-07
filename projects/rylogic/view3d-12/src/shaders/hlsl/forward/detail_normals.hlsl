// Opt-in simple-material variants that tilt the interpolated normal with world-projected detail-normal layers.
// The stock entry points shade the perturbed fragment unchanged, so materials without detail normals pay nothing.
#include "pr/view3d-12/shaders/forward_pixel.hlsli"

// Detail-normal layer constants. The slope map uses the PBR normal-map slot (t12/s7), which simple materials do not otherwise use.
ConstantBuffer<CBufDetailNormals> g_detail : register(b7);

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

// Return 'In' with its world normal tilted by the summed height gradient of the detail-normal layers. Also sets the unresolved slope
// variance used by the shading, from the layers' detail that the mip filtering has averaged away plus the base variance.
PSIn ApplyDetailNormals(PSIn In)
{
	// Fragments without an interpolated normal keep the stock fallback normal.
	float3 normal = In.ws_norm.xyz;
	if (dot(normal, normal) == 0.0f)
		return In;

	normal = normalize(normal);

	// Each layer's map slope is per texture unit, so the chain rule through the projection rows gives the world-space height gradient.
	// The map's blue channel holds half the mean square slope. Mip filtering averages it as well as the slopes, so the gap between the mean
	// square and the square of the mean is the slope variance that the filtered normal no longer shows.
	float3 gradient = float3(0, 0, 0);
	float variance = g_detail.surface.x;
	for (int i = 0; i != g_detail.info.x; ++i)
	{
		// Find the layer's texture coordinates in its own projection. The noise coordinates ignore the rows' offsets, so a layer that scrolls
		// (and wraps its offset) does not move or jump its noise.
		float4 row_u = g_detail.row_u[i];
		float4 row_v = g_detail.row_v[i];
		float2 proj = float2(dot(In.ws_vert.xyz, row_u.xyz), dot(In.ws_vert.xyz, row_v.xyz));
		float2 uv = proj + float2(row_u.w, row_v.w);
		float2 noise_uv = proj * g_detail.noise_frequency[i];

		// World noise varies the layer's weight about 1 so that different layers dominate in different places. Each layer uses its own
		// noise pattern, selected by the layer index.
		float noise = DetailValueNoise(noise_uv, uint(i));
		float scale = g_detail.height_scale[i] * (1.0f + g_detail.weight_noise[i] * (2.0f * noise - 1.0f));

		// Two more noise patterns shift the texture coordinates smoothly, which bends the rows of repeated tiles so they do not line up over
		// large areas. Seeds above the layer limit keep these patterns independent of every layer's weight noise.
		float2 warp = float2(DetailNoiseWarp(noise_uv, uint(DetailNormalsMaxLayers + 2 * i)), DetailNoiseWarp(noise_uv, uint(DetailNormalsMaxLayers + 2 * i + 1)));
		uv += g_detail.warp[i] * warp;

		// Accumulate the world gradient, and the unresolved variance scaled by the mean squared length of the projection rows.
		float4 texel = g_normal_texture.Sample(g_normal_sampler, uv);
		float2 slope = texel.xy * 2.0f - 1.0f;
		float unresolved = max(2.0f * texel.z - dot(slope, slope), 0.0f);
		gradient += scale * (slope.x * row_u.xyz + slope.y * row_v.xyz);
		variance += scale * scale * 0.5f * (dot(row_u.xyz, row_u.xyz) + dot(row_v.xyz, row_v.xyz)) * unresolved;
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
	return ForwardShade(In, is_front_face).diff;
}

// Generate the forward pixel family for detail-normal materials.
VIEW3D_FORWARD_PIXEL_ENTRY_POINTS(ForwardDetail, DetailShade)
