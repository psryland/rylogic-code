//************************************
// Lost at Sea
//  Copyright (c) Rylogic Ltd 2024
//************************************
// CDLOD terrain vertex shader.
// Transforms unit-grid patch vertices to world space using the instance i2w (g_nugget.o2w),
// samples Perlin noise height, applies geomorphing for seamless LOD transitions,
// and projects to screen via g_frame.cam.w2s.
//
// Vertex layout:
//   m_vert.xy = normalised grid position (i/GridN, j/GridN) ∈ [0, 1]
//   m_vert.z  = 0 (unused — height comes from Perlin noise)
//   m_vert.w  = 1
//
// Instance i2w encoding:
//   g_nugget.o2w[0].x = patch_size (world units)
//   g_nugget.o2w[1].y = patch_size
//   g_nugget.o2w[3].xy = patch world origin
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
#include "src/world/terrain/shaders/terrain_cbuf.hlsli"

#ifdef __cplusplus
namespace las
{
	using namespace pr::hlsl;
#endif

ConstantBuffer<CBufFrame> resource(g_frame, b0);
ConstantBuffer<CBufNugget> resource(g_nugget, b1);
ConstantBuffer<CBufTerrain> resource(g_terrain, b3);

struct PSOut
{
	float4 diff semantic(SV_TARGET);
};

// ---- GPU Perlin noise ----
// 2D integer hash with good distribution (breaks axis-aligned correlations)
int hash2(int x, int y)
{
	int h = x * 374761393 + y * 668265263;
	h = (h ^ (h >> 13)) * 1274126177;
	h = h ^ (h >> 16);
	return h & 0x7fffffff;
}

// Smooth interpolation curve: 6t^5 - 15t^4 + 10t^3
float fade(float t)
{
	return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

// 8 gradient directions: 4 axis-aligned + 4 diagonal for isotropic noise
static const float2 s_grads[8] =
{
	float2(1,0), float2(-1,0), float2(0,1), float2(0,-1),
	float2(1,1), float2(-1,1), float2(1,-1), float2(-1,-1),
};
float grad2d(int hash, float x, float y)
{
	float2 g = s_grads[hash & 7];
	return g.x * x + g.y * y;
}

// Single-octave 2D Perlin noise, returns approximately [-1, 1]
float noise2d(float x, float y)
{
	int ix = (int)floor(x);
	int iy = (int)floor(y);
	float fx = x - floor(x);
	float fy = y - floor(y);

	float u = fade(fx);
	float v = fade(fy);

	// Hash the four corners
	int aa = hash2(ix,     iy);
	int ba = hash2(ix + 1, iy);
	int ab = hash2(ix,     iy + 1);
	int bb = hash2(ix + 1, iy + 1);

	// Gradient dot products at each corner, blended
	float x1 = lerp(grad2d(aa, fx,       fy),       grad2d(ba, fx - 1.0, fy),       u);
	float x2 = lerp(grad2d(ab, fx,       fy - 1.0), grad2d(bb, fx - 1.0, fy - 1.0), u);
	return lerp(x1, x2, v);
}

// Multi-octave (fractal) Perlin noise with weathering effects.
// Domain warping creates flowing, erosion-like features.
// Ridged noise on fine octaves adds sharp ridges and valleys (above sea level only).
// Beach flattening creates gentle sandy shores near the waterline.
float terrain_height(float world_x, float world_y)
{
	int octaves           = (int)g_terrain.noise_params.x;
	float base_frequency  = g_terrain.noise_params.y;
	float persistence     = g_terrain.noise_params.z;
	float amplitude       = g_terrain.noise_params.w;
	float sea_level_bias  = g_terrain.noise_bias.x;

	// Domain warping: warp input coords through low-frequency noise.
	// This bends features into curving valleys and meandering ridges.
	float warp_freq = g_terrain.weather_params.x;
	float warp_strength = g_terrain.weather_params.y;
	float warp_x = noise2d(world_x * warp_freq + 5.2, world_y * warp_freq + 1.3) * warp_strength;
	float warp_y = noise2d(world_x * warp_freq + 8.7, world_y * warp_freq + 2.8) * warp_strength;
	float wx = world_x + warp_x;
	float wy = world_y + warp_y;

	float value = 0.0;
	float freq = base_frequency;
	float amp = 1.0;
	float max_amp = 0.0;
	float ridge_blend = 1.0;
	float detail_atten = 1.0; // reduces fine octaves underwater

	for (int i = 0; i < octaves; ++i)
	{
		float n = noise2d(wx * freq, wy * freq);

		// After the first 4 (coarse) octaves, estimate the base height to determine
		// whether we're above or below sea level. Suppress ridging and fine detail underwater.
		if (i == 4)
		{
			float base_h = (value / max_amp + sea_level_bias) * amplitude;
			ridge_blend = saturate(base_h / g_terrain.weather_params.z);

			// Underwater: smoothly attenuate fine-detail octaves for a smoother sea floor
			float smooth_depth = g_terrain.beach_params.w;
			detail_atten = saturate(base_h / max(smooth_depth, 1.0) + 1.0);
		}

		// Ridged noise for fine octaves: sharp ridges where noise crosses zero.
		// Only applied above sea level to keep coastlines and sea floor smooth.
		if (i >= 4)
		{
			float n_ridged = 1.0 - 2.0 * abs(n);
			n = lerp(n, n_ridged, ridge_blend);
			n *= detail_atten;
		}

		value += n * amp;
		max_amp += amp;
		amp *= persistence;
		freq *= 2.0;
	}

	float raw_h = (value / max_amp + sea_level_bias) * amplitude;

	// Macro height variation: a very low-frequency noise field modulates above-water
	// terrain height across the world. Creates a varied archipelago.
	float macro_freq = g_terrain.weather_params.w;
	float macro = noise2d(world_x * macro_freq + 100.3, world_y * macro_freq + 200.7);
	float height_scale = lerp(g_terrain.beach_params.y, g_terrain.beach_params.z, saturate(macro * 0.5 + 0.5));
	if (raw_h > 0.0)
		raw_h *= height_scale;

	// Beach flattening: create gentle sandy beaches and wide grassy coastal slopes.
	// Uses cubic f(t) = -t³ + 2t² which has:
	//   f(0)=0, f'(0)=0  → perfectly flat at waterline
	//   f(1)=1, f'(1)=1  → smooth join to natural terrain slope
	float beach_height = g_terrain.beach_params.x;
	if (raw_h > 0.0 && raw_h < beach_height)
	{
		float t = raw_h / beach_height;
		raw_h = (-t * t * t + 2.0 * t * t) * beach_height;
	}

	return raw_h;
}

// Terrain normal from finite differences.
// Eps scales with cell_size so distant LODs get smoother normals
// that match their coarser mesh geometry.
float3 terrain_normal(float world_x, float world_y, float cell_size)
{
	float eps = max(1.0, cell_size * 0.5);
	float hL = terrain_height(world_x - eps, world_y);
	float hR = terrain_height(world_x + eps, world_y);
	float hD = terrain_height(world_x, world_y - eps);
	float hU = terrain_height(world_x, world_y + eps);
	return normalize(float3(hL - hR, hD - hU, 2.0 * eps));
}

// Terrain colour based on height and normal slope.
// Designed for tropical islands: wide sandy beaches, lush green coastal
// slopes, rock only at steep cliffs and high peaks.
float4 terrain_colour(float height, float3 normal)
{
	float flatness = normal.z; // z-component of normal = cosine of slope angle

	// Underwater: dark blue-grey
	if (height < 0.0)
		return float4(0.13, 0.25, 0.50, 1.0);

	// Beach: sandy yellow (0–8m, wider band to show the flattened coastline)
	if (height < 8.0)
	{
		// Blend from wet sand near waterline to dry sand
		float t = saturate(height / 8.0);
		return float4(lerp(0.72, 0.86, t), lerp(0.62, 0.78, t), lerp(0.30, 0.42, t), 1.0);
	}

	// Altitude blend factor: 0 at coast → 1 at peaks
	float alt_t = saturate((height - 8.0) / 200.0);

	// Slope-based vegetation/rock blend.
	// Low flatness threshold means only truly steep cliffs show as rock.
	// Higher altitudes require flatter terrain for vegetation (alpine effect).
	float veg_threshold = lerp(0.35, 0.65, alt_t);
	float veg_blend = saturate((flatness - veg_threshold) / 0.15);

	// Rock colour (grey with slight brown tint at lower altitudes)
	float3 rock = float3(0.40 + alt_t * 0.10, 0.38 + alt_t * 0.08, 0.35 + alt_t * 0.10);

	// Vegetation colour: lush green near coast, browner and sparser higher up
	float3 veg_low = float3(0.18, 0.52, 0.10);   // lush tropical green
	float3 veg_high = float3(0.35, 0.40, 0.18);   // sparse alpine scrub
	float3 veg = lerp(veg_low, veg_high, alt_t);

	float3 colour = lerp(rock, veg, veg_blend);
	return float4(colour, 1.0);
}

// Vertex shader: CDLOD grid patch with Perlin noise height and geomorphing
PSIn VSTerrain(VSIn In)
{
	PSIn Out = (PSIn)0;

	float grid_n = g_terrain.patch_config.z;
	float morph_start = g_terrain.patch_config.x;
	float morph_end = g_terrain.patch_config.y;

	// Skirt flag: z=1 means this vertex should drop below the surface to hide LOD cracks
	float is_skirt = In.vert.z;

	// Transform grid vertex [0,1] to world space via g_nugget.o2w (encodes patch origin + scale)
	float4 world_pos = mul(float4(In.vert.xy, 0, 1), g_nugget.o2w);
	float2 world_xy = world_pos.xy;

	// Extract cell size from the instance transform
	float patch_size = g_nugget.o2w[0][0];
	float cell_size = patch_size / grid_n;

	// Sample terrain height at this world position
	float h = terrain_height(world_xy.x, world_xy.y);

	// Geomorphing: odd-indexed vertices morph toward their coarser-LOD neighbours.
	// This eliminates popping when LOD levels change.
	int ix = (int)round(In.vert.x * grid_n);
	int iy = (int)round(In.vert.y * grid_n);
	bool odd_x = (ix & 1) != 0;
	bool odd_y = (iy & 1) != 0;

	if (odd_x || odd_y)
	{
		float dist = length(world_xy - g_terrain.camera_pos.xy);
		float morph = saturate((dist - morph_start) / max(morph_end - morph_start, 0.001));

		float morph_h = h;
		if (odd_x && !odd_y)
		{
			// Morph to average of X-axis neighbours (even indices)
			float hL = terrain_height(world_xy.x - cell_size, world_xy.y);
			float hR = terrain_height(world_xy.x + cell_size, world_xy.y);
			morph_h = (hL + hR) * 0.5;
		}
		else if (!odd_x && odd_y)
		{
			// Morph to average of Y-axis neighbours (even indices)
			float hD = terrain_height(world_xy.x, world_xy.y - cell_size);
			float hU = terrain_height(world_xy.x, world_xy.y + cell_size);
			morph_h = (hD + hU) * 0.5;
		}
		else
		{
			// Both odd: morph to average of 4 diagonal neighbours (all even)
			float h00 = terrain_height(world_xy.x - cell_size, world_xy.y - cell_size);
			float h10 = terrain_height(world_xy.x + cell_size, world_xy.y - cell_size);
			float h01 = terrain_height(world_xy.x - cell_size, world_xy.y + cell_size);
			float h11 = terrain_height(world_xy.x + cell_size, world_xy.y + cell_size);
			morph_h = (h00 + h10 + h01 + h11) * 0.25;
		}

		h = lerp(h, morph_h, morph);
	}

	// Skirt vertices drop below the surface to fill T-junction cracks between LOD levels.
	// Depth scales with cell size so finer LODs have smaller skirts.
	if (is_skirt > 0.5)
		h -= cell_size * 4.0;

	// Final world position with terrain height
	world_pos.z = h;

	// Compute terrain normal from the actual Perlin surface
	float3 n = terrain_normal(world_xy.x, world_xy.y, cell_size);

	// Output
	Out.ws_vert = world_pos;
	Out.ws_norm = float4(n, 0);
	Out.ss_vert = mul(world_pos, g_frame.cam.w2s);
	Out.diff = float4(0, 0, 0, 1);
	Out.tex0 = In.tex0;
	Out.idx0 = In.idx0;

	return Out;
}

// Pixel shader: per-pixel terrain colouring and lighting
PSOut PSTerrain(PSIn In)
{
	PSOut Out = (PSOut)0;

	float3 N = normalize(In.ws_norm.xyz);
	float3 V = normalize(g_frame.cam.c2w[3].xyz - In.ws_vert.xyz);
	float3 L = g_terrain.sun_direction.xyz;

	// ws_vert.z is world height
	float world_z = In.ws_vert.z;

	// Compute terrain colour per-pixel from height and slope
	float4 base_colour = terrain_colour(world_z, N);

	// Diffuse lighting
	float NdotL = saturate(dot(N, L));
	float ambient = 0.25;
	float diffuse = NdotL * 0.65;

	// Soft sky light from above
	float sky_light = saturate(N.z * 0.5 + 0.5) * 0.15;

	float3 colour = base_colour.rgb * (ambient + diffuse + sky_light) * g_terrain.sun_colour.rgb;

	Out.diff = float4(colour, 1.0);
	return Out;
}

#ifdef __cplusplus
}
#endif
