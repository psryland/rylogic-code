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
// The project and write passes can run more than once, each time with a different source image, before the fill pass runs. Besides the main view,
// a mirror source adds the true reflection: the scene drawn from the mirror image of the camera, clipped at the plane. It holds the reflections of
// surfaces that near objects hide in the main view, and of objects beyond the edges of the screen.
//
// Each reflection texel holds (linear colour, distance). The distance is measured from the camera to the mirrored point, which equals the length
// of the path from the camera to the plane and on to the reflected object. Final texels with distance 0 have no reflection. Sky pixels are
// mirrored as directions, and store 'ReflectionSkyDistance'. Sky keys lose to every surface, because the sky is behind everything.
#include "view3d-12/src/shaders/hlsl/types.hlsli"

struct CBufReflectionPlane
{
	row_major float4x4 s2w; // The source's normalised device coordinates to world space. For the mirror source this undoes the mirroring too
	row_major float4x4 w2s; // World to the main view's screen space
	float4 plane;           // xyz = unit normal, w = distance of the plane from the origin along the normal
	float4 viewport;        // The viewport in pixels: (x, y, width, height)
	float4 cam;             // xyz = world-space camera position of the main view, w unused

	// The source image of the project and write passes. Thread 'DTid' reads source pixel 'DTid + load', which is at normalised device position
	// (dot(src0.xy, p) + src0.z, dot(src1.xy, p) + src1.z) in the source's own view, where p = DTid + 0.5. Keys of the source's candidates are
	// scaled by 'key scale', so that a scale below 1 makes the source lose to others that land at almost the same depth.
	float4 src0;            // xy = map row for x, z = offset for x, w = source width in pixels
	float4 src1;            // xy = map row for y, z = offset for y, w = source height in pixels
	float4 src2;            // xy = load offset in pixels, z = key scale, w unused
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

// The furthest distance, in pixels, that 'CSReflectionFill' looks for a texel to fill a gap with
static const int ReflectionFillRadius = 2;

// The key of a mirrored sky pixel. Surface keys are greater, and 0 means nothing was written.
static const uint ReflectionSkyKey = 1u;

// A written texel counts as a gap in a nearer surface only when that surface is nearer than this fraction of the texel's distance. The margin keeps
// texels within one continuous surface, whose distances change gradually, from replacing each other.
static const float ReflectionGapDistanceRatio = 0.95f;

// Find the source pixel read by thread 'DTid' of the project and write passes, and its position in the source's normalised device coordinates.
// Returns false for threads beyond the source.
bool ReflectionSource(uint2 DTid, out uint2 pix, out float2 ndc)
{
	// Threads beyond the source have no pixel
	pix = DTid + (uint2)g_cb.src2.xy;
	float2 p = (float2)DTid + 0.5f;
	ndc = float2(dot(g_cb.src0.xy, p) + g_cb.src0.z, dot(g_cb.src1.xy, p) + g_cb.src1.z);
	return DTid.x < (uint)g_cb.src0.w && DTid.y < (uint)g_cb.src1.w;
}

// Find the target pixel that the mirror image of the source pixel at 'ndc' lands on. Returns false when the pixel has no mirror image on the screen.
// 'key' orders the candidates for a target; the greatest key is nearest the camera. 'dist' is the distance from the camera to the mirrored point.
bool ReflectionTarget(float2 ndc, float depth, out uint2 target, out uint key, out float dist)
{
	target = uint2(0, 0);
	key = 0;
	dist = 0.0f;

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
		// The far point of the view ray, in homogeneous form, gives the ray direction even when it is at infinity. The ray starts at the source's
		// own eye, which is the point that 's2w' maps the clip-space direction (0,0,1,0) to. Clip w is positive for every pixel, so the far point is
		// in front of that eye. Only sky above the plane's horizon is seen in the reflection.
		float4 ws = mul(float4(ndc, 0.0f, 1.0f), g_cb.s2w);
		float4 e = mul(float4(0.0f, 0.0f, 1.0f, 0.0f), g_cb.s2w);
		float3 eye = abs(e.w) > 0.0f ? e.xyz / e.w : g_cb.cam.xyz;
		float3 dir = (ws.xyz - eye * ws.w) * (ws.w < 0.0f ? -1.0f : 1.0f);
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

	// Reversed depth is greater for nearer points, so the greatest key wins. The bits of a non-negative float order the same way as its value.
	target = (uint2)(vp.xy + float2(ss.x * 0.5f + 0.5f, 0.5f - ss.y * 0.5f) * vp.zw);
	key = depth > 0.0f ? max(asuint(max(ss.z * g_cb.src2.z, 0.0f)), ReflectionSkyKey + 1u) : ReflectionSkyKey;
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

	// Record this pixel's mirror image as a candidate for its target
	uint2 target;
	uint key;
	float dist;
	if (!ReflectionTarget(ndc, g_depth.Load(pix, 0), target, key, dist))
		return;

	InterlockedMax(g_key[target], key);
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

	if (g_key[target] == key)
		g_reflection[target] = float4(g_colour.Load(pix, 0).rgb, dist);
}

// Copy each written texel to the final reflection, and fill gaps from the nearest written texel
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
	if (g_key[pix] != 0)
	{
		// A written texel is a gap when much nearer texels lie on both sides of it along some line. Requiring both sides keeps the true outline
		// of a nearer surface from growing over what is behind it. Sky texels store a large distance, so every surface is nearer than them.
		float4 own = g_reflection[pix];
		float limit = own.a * ReflectionGapDistanceRatio;
		static const int2 axes[4] = { int2(1, 0), int2(0, 1), int2(1, 1), int2(1, -1) };
		float4 fill = own;
		int fill_d2 = 0x7FFFFFFF;
		for (int a = 0; a != 4; ++a)
		{
			// Find the nearest much-nearer texel on each side of this texel along the line
			float4 side[2] = { float4(0, 0, 0, 0), float4(0, 0, 0, 0) };
			int steps[2] = { 0, 0 };
			for (int s = 0; s != 2; ++s)
			{
				// Step outwards on one side, stopping at the first texel that is much nearer
				int2 dir = s == 0 ? axes[a] : -axes[a];
				for (int i = 1; i <= ReflectionFillRadius && steps[s] == 0; ++i)
				{
					// Unwritten texels do not close a gap
					int2 nbr = clamp(pix + dir * i, lo, hi);
					if (g_key[nbr] == 0)
						continue;

					float4 texel = g_reflection[nbr];
					if (texel.a < limit)
					{
						side[s] = texel;
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
			}
		}
		result = fill;
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
				if (d2 >= best || g_key[nbr] == 0)
					continue;

				best = d2;
				result = g_reflection[nbr];
			}
		}
	}
	g_final[pix] = result;
}
