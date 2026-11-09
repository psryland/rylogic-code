//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Projects the opaque scene into the reflection of a plane, so that surfaces on the plane can show the reflection of everything already rendered
// in this frame. Every opaque pixel above the plane is mirrored through it and projected back onto the screen. The pixel it lands on shows the
// plane at the point where that mirrored pixel is seen. When several pixels land on the same target, the one nearest the camera wins, because
// in a mirror nearer objects hide farther ones.
//
// The work is split into passes because a 32-bit atomic cannot hold both a depth and a colour:
//  - 'CSReflectionProject' keeps the greatest depth key that lands on each target.
//  - 'CSReflectionWrite' repeats the projection, and the pixel whose key won writes its colour and distance to the target.
//  - 'CSReflectionFill' fills the small gaps left where the mirror image is stretched, including gaps that a farther surface or the sky shows
//    through, and writes the final reflection.
// The key texture must be cleared to 0 before the first pass. A non-zero key marks a texel that 'CSReflectionWrite' wrote, so the raw
// reflection texture does not need clearing.
//
// The project and write passes can run more than once, each time with a different source image, before the fill pass runs. This lets low
// resolution images of the scene just beyond the viewport add the reflections of objects that are not on screen.
//
// Each reflection texel holds (linear colour, distance). The distance is measured from the camera to the mirrored point, which equals the length
// of the path from the camera to the plane and on to the reflected object. Final texels with distance 0 have no reflection. A separate coverage
// texture holds how much of each final texel to use, which is less than 1 where the smoothed outline of a low resolution source fades out. Sky pixels
// are mirrored as directions, and store 'ReflectionSkyDistance'. Sky keys lose to every surface, because the sky is behind everything.
#include "view3d-12/src/shaders/hlsl/types.hlsli"

struct CBufReflectionPlane
{
	row_major float4x4 s2w; // Screen (normalised device coordinates) to world space
	row_major float4x4 w2s; // World to screen space
	float4 plane;           // xyz = unit normal, w = distance of the plane from the origin along the normal
	float4 viewport;        // The viewport in pixels: (x, y, width, height)
	float4 cam;             // xyz = world-space camera position

	// The source image of the project and write passes. Thread 'DTid' reads source pixel 'DTid + load', which is at normalised device position
	// (dot(src0.xy, p) + src0.z, dot(src1.xy, p) + src1.z) in the main view, where p = DTid + 0.5. A source may be the main view or a lower
	// resolution image of the scene just beyond it, whose pixels each cover a square of 'splat' x 'splat' target pixels.
	float4 src0;            // xy = map row for x, z = offset for x, w = source width in pixels
	float4 src1;            // xy = map row for y, z = offset for y, w = source height in pixels
	float4 src2;            // xy = load offset in pixels, z = splat size in target pixels, w = non-zero to ignore pixels where nothing was rendered
};

// The distance stored for mirrored sky pixels. Half floats hold values up to 65504.
static const float ReflectionSkyDistance = 65000.0f;

// Resources
ConstantBuffer<CBufReflectionPlane> g_cb : register(b0);
Texture2DMS<float4> g_colour : register(t0);    // The opaque scene colour, read as linear colour
Texture2DMS<float> g_depth : register(t1);      // The opaque scene depth. Depth is reversed, with 0 at the far plane
RWTexture2D<uint> g_key : register(u0);         // The greatest depth key that landed on each target
RWTexture2D<float4> g_reflection : register(u1); // The raw reflection: (linear colour, distance). Valid only where the key is non-zero
RWTexture2D<float4> g_final : register(u2);      // The gap-filled reflection: (linear colour, distance)
RWTexture2D<unorm float> g_coverage : register(u3); // The share of each final texel to use; the reader fills the rest from other reflection sources

// The furthest distance, in pixels, that 'CSReflectionFill' looks for a texel to fill a gap with
static const int ReflectionFillRadius = 2;

// The low bits of a key record the splat size of the source that wrote it, so 'CSReflectionFill' can smooth the blocks of low resolution sources.
// The code falls as the splat grows, so where keys are otherwise equal, the sharper source wins. Splat sizes from 1 to 'ReflectionMaxSplat' are allowed.
// 'ReflectionSmoothReach' is the furthest, in pixels, that a block of the largest splat is smoothed beyond its own texels.
static const uint ReflectionKeySplatBits = 0xFu;
static const uint ReflectionMaxSplat = 9u;
static const int ReflectionSmoothReach = (int)ReflectionMaxSplat / 2;

// A written texel counts as a gap in a nearer surface only when that surface is nearer than this fraction of the texel's distance. The margin keeps
// texels within one continuous surface, whose distances change gradually, from replacing each other.
static const float ReflectionGapDistanceRatio = 0.95f;

// The splat size of the source that wrote 'key'
int ReflectionKeySplat(uint key)
{
	return (int)(ReflectionMaxSplat + 1u - (key & ReflectionKeySplatBits));
}

// Find the source pixel read by thread 'DTid' of the project and write passes, and its position in the main view's normalised device coordinates.
// Returns false for threads beyond the source.
bool ReflectionSource(uint2 DTid, out uint2 pix, out float2 ndc)
{
	// Threads beyond the source have no pixel
	pix = DTid + (uint2)g_cb.src2.xy;
	float2 p = (float2)DTid + 0.5f;
	ndc = float2(dot(g_cb.src0.xy, p) + g_cb.src0.z, dot(g_cb.src1.xy, p) + g_cb.src1.z);
	return DTid.x < (uint)g_cb.src0.w && DTid.y < (uint)g_cb.src1.w;
}

// The square of target pixels covered by one source pixel whose mirror image lands on 'target', clipped to the viewport: [lo, hi)
void ReflectionSplat(uint2 target, out uint2 lo, out uint2 hi)
{
	// The square is centred on the target, and a splat size of 1 covers only the target
	int n = (int)g_cb.src2.z;
	int2 vp_lo = (int2)g_cb.viewport.xy;
	int2 vp_hi = vp_lo + (int2)g_cb.viewport.zw;
	lo = (uint2)max((int2)target - n / 2, vp_lo);
	hi = (uint2)min((int2)target - n / 2 + n, vp_hi);
}

// Find the target pixel that the mirror image of the source pixel at 'ndc' lands on. Returns false when the pixel has no mirror image on the screen.
// 'key' orders the candidates for a target; the greatest key is nearest the camera. 'dist' is the distance from the camera to the mirrored point.
bool ReflectionTarget(float2 ndc, float depth, out uint2 target, out uint key, out float dist)
{
	target = uint2(0, 0);
	key = 0;
	dist = 0.0f;

	// Some sources do not render the sky, so their empty pixels are ignored
	if (depth <= 0.0f && g_cb.src2.w != 0.0f)
		return false;

	// Unproject the pixel. Pixels where nothing was rendered (the sky) are mirrored as directions, because the sky is infinitely distant.
	float3 n = g_cb.plane.xyz;
	float4 vp = g_cb.viewport;
	float4 mirrored;
	if (depth > 0.0f)
	{
		// Only points above the plane have a reflection that is seen from above it
		float4 ws = mul(float4(ndc, depth, 1.0f), g_cb.s2w);
		float3 p = ws.xyz / ws.w;
		float h = dot(n, p) - g_cb.plane.w;
		if (h <= 0.0f)
			return false;

		mirrored = float4(p - 2.0f * h * n, 1.0f);
		dist = length(mirrored.xyz - g_cb.cam.xyz);
	}
	else
	{
		// The far point of the view ray, in homogeneous form, gives the ray direction even when it is at infinity.
		// Only sky above the plane's horizon is seen in the reflection.
		float4 ws = mul(float4(ndc, 0.0f, 1.0f), g_cb.s2w);
		float3 dir = (ws.xyz - g_cb.cam.xyz * ws.w) * (ws.w < 0.0f ? -1.0f : 1.0f);
		float up = dot(dir, n);
		if (up <= 0.0f)
			return false;

		mirrored = float4(dir - 2.0f * up * n, 0.0f);
		dist = ReflectionSkyDistance;
	}

	// Project the mirror image. It must be in front of the camera and inside the viewport.
	float4 clip = mul(mirrored, g_cb.w2s);
	if (clip.w <= 0.0f)
		return false;

	float3 ss = clip.xyz / clip.w;
	if (any(abs(ss.xy) >= 1.0f))
		return false;

	// Reversed depth is greater for nearer points, so the greatest key wins. Sky keys hold only the splat code, so they are below every surface key,
	// and no key is 0, which means nothing was written.
	uint splat_code = ReflectionMaxSplat + 1u - (uint)g_cb.src2.z;
	target = (uint2)(vp.xy + float2(ss.x * 0.5f + 0.5f, 0.5f - ss.y * 0.5f) * vp.zw);
	key = (depth > 0.0f ? max(asuint(max(ss.z, 0.0f)) & ~ReflectionKeySplatBits, ReflectionKeySplatBits + 1u) : 0u) | splat_code;
	return true;
}

// Keep the greatest depth key that lands on each target
numthreads(CSReflectionProject, 8, 8, 1)
void CSReflectionProject(uint3 DTid : SV_DispatchThreadID)
{
	// Find the source pixel
	uint2 pix;
	float2 ndc;
	if (!ReflectionSource(DTid.xy, pix, ndc))
		return;

	// Record this pixel's mirror image as a candidate for every target it covers
	uint2 target;
	uint key;
	float dist;
	if (!ReflectionTarget(ndc, g_depth.Load(pix, 0), target, key, dist))
		return;

	uint2 lo, hi;
	ReflectionSplat(target, lo, hi);
	for (uint y = lo.y; y < hi.y; ++y)
	{
		// Cover one row of the splat
		for (uint x = lo.x; x < hi.x; ++x)
			InterlockedMax(g_key[uint2(x, y)], key);
	}
}

// Write the colour of the pixel whose key won each target
numthreads(CSReflectionWrite, 8, 8, 1)
void CSReflectionWrite(uint3 DTid : SV_DispatchThreadID)
{
	// Find the source pixel
	uint2 pix;
	float2 ndc;
	if (!ReflectionSource(DTid.xy, pix, ndc))
		return;

	// Repeat the projection. Pixels with equal keys may both write, but they show the same surface.
	uint2 target;
	uint key;
	float dist;
	if (!ReflectionTarget(ndc, g_depth.Load(pix, 0), target, key, dist))
		return;

	uint2 lo, hi;
	float4 texel = float4(g_colour.Load(pix, 0).rgb, dist);
	ReflectionSplat(target, lo, hi);
	for (uint y = lo.y; y < hi.y; ++y)
	{
		// Write the targets of one row of the splat that this pixel won
		for (uint x = lo.x; x < hi.x; ++x)
		{
			if (g_key[uint2(x, y)] == key)
				g_reflection[uint2(x, y)] = texel;
		}
	}
}

// True when 'key' was written by a surface rather than the sky
bool ReflectionKeyIsSurface(uint key)
{
	return (key & ~ReflectionKeySplatBits) != 0u;
}

// Smooth the reflection at 'pix' over the area of one block of the low resolution surface texel 'texel', whose key is 'key', so a low resolution
// source gives a smooth image rather than blocks. Each source pixel covers a block of its splat size, so a box filter of the same size interpolates
// linearly between the blocks, both inside the surface and across its outline:
//  - Texels from low resolution sources at a similar distance to 'texel' give the surface's colour.
//  - Texels of other surfaces count as the surface, so the outlines between surfaces stay sharp.
//  - Sky texels mix their colour in, so the outline against the mirrored sky is smooth.
//  - Empty texels reduce 'coverage', the share of the result to use. The reader fills the rest from other reflection sources.
// 'pix' need not be one of the surface's texels; it can be an empty or sky texel within one block of it.
float4 ReflectionSmooth(int2 pix, float4 texel, uint key, int2 lo, int2 hi, out float coverage)
{
	// Texels from full resolution sources are not smoothed
	coverage = 1.0f;
	int n = ReflectionKeySplat(key);
	if (n <= 1)
		return texel;

	// Sum the weights of each kind of texel in a box of width 'n'. For an even width, the outermost texels are half inside the box.
	int h = n / 2;
	float edge = (n & 1) == 0 ? 0.5f : 1.0f;
	float tolerance = texel.a * (1.0f - ReflectionGapDistanceRatio);
	float3 surface_sum = float3(0, 0, 0), sky_sum = float3(0, 0, 0);
	float surface_w = 0.0f, other_w = 0.0f, sky_w = 0.0f, empty_w = 0.0f;
	for (int y = -h; y <= h; ++y)
	{
		// Sum one row of the box
		float wy = abs(y) == h ? edge : 1.0f;
		for (int x = -h; x <= h; ++x)
		{
			// Sort the texel by kind
			float w = wy * (abs(x) == h ? edge : 1.0f);
			int2 nbr = clamp(pix + int2(x, y), lo, hi);
			uint k = g_key[nbr];
			if (k == 0u)
			{
				empty_w += w;
				continue;
			}

			float4 t = g_reflection[nbr];
			if (!ReflectionKeyIsSurface(k))
			{
				sky_sum += t.rgb * w;
				sky_w += w;
			}
			else if (ReflectionKeySplat(k) > 1 && abs(t.a - texel.a) <= tolerance)
			{
				surface_sum += t.rgb * w;
				surface_w += w;
			}
			else
			{
				other_w += w;
			}
		}
	}

	// Other surfaces take the surface's colour, because only the outline against the sky and empty texels is blended.
	// The distance is the surface's or the sky's, whichever covers more of the box.
	float3 surface = surface_w > 0.0f ? surface_sum / surface_w : texel.rgb;
	float solid_w = surface_w + other_w;
	float total_w = solid_w + sky_w + empty_w;
	coverage = (solid_w + sky_w) / total_w;
	float3 rgb = (surface * solid_w + sky_sum) / max(solid_w + sky_w, 1e-6f);
	return float4(rgb, solid_w >= sky_w ? texel.a : ReflectionSkyDistance);
}

// Copy each written texel to the final reflection, and fill gaps from the nearest written texel. Texels from low resolution sources, and the texels
// around their outlines, are smoothed.
numthreads(CSReflectionFill, 8, 8, 1)
void CSReflectionFill(uint3 DTid : SV_DispatchThreadID)
{
	// Threads beyond the viewport have no pixel
	if (DTid.x >= (uint)g_cb.viewport.z || DTid.y >= (uint)g_cb.viewport.w)
		return;

	// The mirror image is stretched where the reflected surface is seen at a steeper angle than the plane, which leaves gaps between the texels
	// it reaches. A gap can be empty, or it can hold a farther surface or the sky, whose mirror images land between those of the nearer surface.
	int2 pix = (int2)DTid.xy + (int2)g_cb.viewport.xy;
	int2 lo = (int2)g_cb.viewport.xy;
	int2 hi = lo + (int2)g_cb.viewport.zw - 1;
	float4 result = float4(0, 0, 0, 0);
	uint result_key = 0u;
	if (g_key[pix] != 0)
	{
		// A written texel is a gap when much nearer texels lie on both sides of it along some line. Requiring both sides keeps the true outline
		// of a nearer surface from growing over what is behind it. Sky texels store a large distance, so every surface is nearer than them.
		float4 own = g_reflection[pix];
		float limit = own.a * ReflectionGapDistanceRatio;
		static const int2 axes[4] = { int2(1, 0), int2(0, 1), int2(1, 1), int2(1, -1) };
		float4 fill = own;
		uint fill_key = g_key[pix];
		int fill_d2 = 0x7FFFFFFF;
		for (int a = 0; a != 4; ++a)
		{
			// Find the nearest much-nearer texel on each side of this texel along the line
			float4 side[2] = { float4(0, 0, 0, 0), float4(0, 0, 0, 0) };
			uint side_key[2] = { 0u, 0u };
			int steps[2] = { 0, 0 };
			for (int s = 0; s != 2; ++s)
			{
				// Step outwards on one side, stopping at the first texel that is much nearer
				int2 dir = s == 0 ? axes[a] : -axes[a];
				for (int i = 1; i <= ReflectionFillRadius && steps[s] == 0; ++i)
				{
					// Unwritten texels do not close a gap
					int2 nbr = clamp(pix + dir * i, lo, hi);
					uint k = g_key[nbr];
					if (k == 0)
						continue;

					float4 texel = g_reflection[nbr];
					if (texel.a < limit)
					{
						side[s] = texel;
						side_key[s] = k;
						steps[s] = i;
					}
				}
			}

			// Fill from the closer of the two sides when both close the gap, preferring the closest of all lines
			if (steps[0] == 0 || steps[1] == 0)
				continue;

			int s = steps[0] <= steps[1] ? 0 : 1;
			int d2 = steps[s] * steps[s] * (a < 2 ? 1 : 2);
			if (d2 < fill_d2)
			{
				fill_d2 = d2;
				fill = side[s];
				fill_key = side_key[s];
			}
		}
		result = fill;
		result_key = fill_key;
	}
	else
	{
		// Fill each empty texel from the nearest written texel. Larger gaps stay empty, and the reader falls back to other reflection sources there.
		int best = 0x7FFFFFFF;
		for (int y = -ReflectionFillRadius; y <= ReflectionFillRadius; ++y)
		{
			// Search one row of the neighbourhood
			for (int x = -ReflectionFillRadius; x <= ReflectionFillRadius; ++x)
			{
				// Keep the nearest written texel
				int2 nbr = clamp(pix + int2(x, y), lo, hi);
				int d2 = x * x + y * y;
				uint k = g_key[nbr];
				if (d2 >= best || k == 0)
					continue;

				best = d2;
				result = g_reflection[nbr];
				result_key = k;
			}
		}
	}

	// Texels that show no surface (empty or sky) within one block of a low resolution surface are smoothed with it, so the surface's outline
	// fades over one block instead of stepping from block to block.
	if (!ReflectionKeyIsSurface(result_key))
	{
		// Find the nearest low resolution surface texel whose block reaches this texel
		int best = 0x7FFFFFFF;
		for (int y = -ReflectionSmoothReach; y <= ReflectionSmoothReach; ++y)
		{
			// Search one row of the neighbourhood
			for (int x = -ReflectionSmoothReach; x <= ReflectionSmoothReach; ++x)
			{
				// Keep the nearest surface texel whose block reaches this texel
				int2 nbr = clamp(pix + int2(x, y), lo, hi);
				int d2 = x * x + y * y;
				uint k = g_key[nbr];
				if (d2 >= best || !ReflectionKeyIsSurface(k) || max(abs(x), abs(y)) > ReflectionKeySplat(k) / 2)
					continue;

				best = d2;
				result = g_reflection[nbr];
				result_key = k;
			}
		}
	}

	// Smooth texels from low resolution sources, and record how much of each texel to use
	float coverage = 0.0f;
	g_final[pix] = result_key != 0u ? ReflectionSmooth(pix, result, result_key, lo, hi, coverage) : result;
	g_coverage[pix] = coverage;
}
