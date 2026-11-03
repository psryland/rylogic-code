//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
// Screen-space ambient occlusion passes. See post_processing.h for the effect settings and postprocessing.md for the design.
// The passes are:
//  1. CSAOPrefilter: convert the scene depth to linear view depth at the AO resolution.
//  2. CSAOMain: estimate the visible fraction of the hemisphere above each AO texel by searching for horizons in a few screen directions.
//  3. CSAOBlur: average 4x4 texels with similar depth, which removes the 4x4 noise pattern used to vary the search between texels.
//  4. PSAOApply/PSAODebug: upsample to full resolution using depth to avoid bleeding across edges, then multiply into the opaque colour.
#include "view3d-12/src/shaders/hlsl/postprocessing/ambient_occlusion_cbuf.hlsli"

static const float PI = 3.14159265359f;
static const float HALF_PI = 1.57079632679f;

// View depth stored for texels without scene geometry. Any depth at or beyond 'BACKGROUND_TEST' is background.
static const float BACKGROUND = 3.0e38f;
static const float BACKGROUND_TEST = 1.0e38f;

// Largest relative depth difference between texels treated as the same surface by the blur and upsample.
static const float SAME_SURFACE = 0.05f;

ConstantBuffer<CBufAmbientOcclusion> resource(g_ao, b0);
Texture2DMS<float> resource(g_scene_depth, t0); // The multi-sampled scene depth buffer
Texture2D<float> resource(g_view_depth, t1);    // Linear view depth at the AO resolution
Texture2D<float> resource(g_ao_in, t2);         // Occlusion at the AO resolution (1 = fully visible)
RWTexture2D<float> resource(g_out, u0);         // Output of the compute passes

// Convert a depth buffer value to view-space z (distance in front of the camera).
float LinearDepth(float d)
{
	// Unproject only the z and w rows; for the projections used by view3d they do not depend on screen x or y.
	return -(g_ao.depth_params.x * d + g_ao.depth_params.y) / (g_ao.depth_params.z * d + g_ao.depth_params.w);
}

// The view-space position of AO texel 'uv' (a texel index, possibly fractional) at view depth 'z'.
float3 ViewPosition(float2 uv, float z)
{
	// The xy coefficients are for a view ray through the texel; perspective rays spread with depth.
	float2 xy = g_ao.view_params.xy * uv + g_ao.view_params.zw;
	return float3(g_ao.orthographic != 0 ? xy : xy * z, z);
}

// Read the linear view depth of AO texel 'uv', clamped to the AO buffer.
float ViewDepth(int2 uv)
{
	return g_view_depth.Load(int3(clamp(uv, int2(0, 0), g_ao.ao_size - 1), 0));
}

// Two independent noise values in [0,1) for the position of 'uv' within its 4x4 block.
float2 Noise(uint2 uv)
{
	// Order the 16 cells so that neighbouring cells differ a lot. The first value is evenly spaced over the cells: the blur averages
	// the cells, so the slice angles of a block are evenly spread and the average over angles is exact for an unoccluded tilted surface.
	// The second value uses the golden ratio so that the pair forms a well-spread 2D pattern and the step offset is not correlated with the angle.
	static const uint bayer[16] = { 0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5 };
	float index = bayer[(uv.y & 3) * 4 + (uv.x & 3)];
	return float2((index + 0.5f) / 16.0f, frac(0.5f + index * 0.6180339887f));
}

// Write the linear view depth for each AO texel, taken from the top-left pixel of its block and the first depth sample.
numthreads(CSAOPrefilter, 8, 8, 1)
void CSAOPrefilter(uint3 dtid : SV_DispatchThreadID)
{
	// Threads beyond the AO buffer have no texel.
	if (any(dtid.xy >= uint2(g_ao.ao_size)))
		return;

	// The same pixel and sample are used by the upsample, so AO texels line up exactly with the full-resolution depth.
	int2 pixel = int2(g_ao.viewport.xy) + int2(dtid.xy) * g_ao.scale;
	float d = g_scene_depth.Load(pixel, 0);
	g_out[dtid.xy] = d == g_ao.clear_depth ? BACKGROUND : LinearDepth(d);
}

// Estimate the visible fraction of the hemisphere above each AO texel.
numthreads(CSAOMain, 8, 8, 1)
void CSAOMain(uint3 dtid : SV_DispatchThreadID)
{
	// Threads beyond the AO buffer have no texel. Background texels are fully visible.
	if (any(dtid.xy >= uint2(g_ao.ao_size)))
		return;

	int2 uv = int2(dtid.xy);
	float z = ViewDepth(uv);
	if (z >= BACKGROUND_TEST)
	{
		g_out[uv] = 1.0f;
		return;
	}

	// Reconstruct the surface normal from the neighbouring depths. On each axis, use the neighbour nearest in depth
	// so that the normal is not bent by a depth edge next to the texel.
	float3 P = ViewPosition(uv, z);
	float3 normal;
	{
		float3 pl = ViewPosition(uv + int2(-1, 0), ViewDepth(uv + int2(-1, 0)));
		float3 pr = ViewPosition(uv + int2(+1, 0), ViewDepth(uv + int2(+1, 0)));
		float3 pt = ViewPosition(uv + int2(0, -1), ViewDepth(uv + int2(0, -1)));
		float3 pb = ViewPosition(uv + int2(0, +1), ViewDepth(uv + int2(0, +1)));
		float3 dx = abs(pr.z - z) < abs(z - pl.z) ? pr - P : P - pl;
		float3 dy = abs(pb.z - z) < abs(z - pt.z) ? pb - P : P - pt;
		normal = normalize(cross(dy, dx));
	}

	// The direction from the surface to the camera. Orthographic view rays are parallel.
	float3 view_vec = g_ao.orthographic != 0 ? float3(0, 0, -1) : normalize(-P);
	if (dot(normal, view_vec) < 0)
		normal = -normal;

	// Convert the world radius to AO texels at this depth. Skip texels where the search would not reach a neighbour.
	float texel_radius = g_ao.radius * g_ao.texels_per_unit / (g_ao.orthographic != 0 ? 1.0f : z);
	texel_radius = min(texel_radius, g_ao.max_radius);
	if (texel_radius < 1.0f)
	{
		g_out[uv] = 1.0f;
		return;
	}

	// The first sample is at least about one texel away, so the texel does not occlude itself.
	float2 noise = Noise(dtid.xy);
	float min_s = 1.3f / texel_radius;

	// Search for the horizon on both sides of the texel in each slice, and integrate the visible arc between the horizons.
	// Each slice is a plane containing the view vector. The slices must be spread evenly in angle around the view vector for the average to be the
	// cosine-weighted visible fraction. Evenly spaced screen directions are not evenly spaced around the view vector of an off-axis perspective texel,
	// which would darken unoccluded surfaces seen at an angle. So the slice is chosen in view space, and its screen direction is derived from it.
	// 'tangent' and 'bitangent' are perpendicular to the view vector and match screen x and y for an orthographic view.
	float3 tangent = normalize(cross(view_vec, float3(0, 1, 0)));
	float3 bitangent = cross(tangent, view_vec);
	float visibility = 0;
	for (int slice = 0; slice != g_ao.slice_count; ++slice)
	{
		// The view-space direction of this slice, perpendicular to the view vector.
		float phi = (slice + noise.x) * PI / g_ao.slice_count;
		float3 ortho_dir = cos(phi) * tangent + sin(phi) * bitangent;
		float3 axis = cross(ortho_dir, view_vec);

		// The screen direction, in texels, of moving from P along 'ortho_dir'. A perspective texel position is xy/z, so moving along
		// 'ortho_dir' changes it in proportion to 'ortho_dir.xy - (P.xy / P.z) * ortho_dir.z'. Orthographic texel positions only depend on xy.
		float2 screen_dir = g_ao.orthographic != 0 ? ortho_dir.xy : ortho_dir.xy - (P.xy / P.z) * ortho_dir.z;
		float2 omega = normalize(screen_dir / g_ao.view_params.xy);

		// Project the normal into the slice plane. 'n' is its angle from the view vector, signed towards 'ortho_dir'.
		float3 proj_normal = normal - axis * dot(normal, axis);
		float proj_normal_len = length(proj_normal);
		float sign_norm = sign(dot(ortho_dir, proj_normal));
		float cos_norm = saturate(dot(proj_normal, view_vec) / proj_normal_len);
		float n = sign_norm * acos(cos_norm);

		// The horizons start at the surface tangent plane, so unoccluded directions give full visibility.
		float low0 = cos(n + HALF_PI);
		float low1 = cos(n - HALF_PI);
		float hc0 = low0;
		float hc1 = low1;

		// Step outwards on both sides. Samples are denser near the texel, where occluders matter most.
		for (int j = 0; j != g_ao.step_count; ++j)
		{
			// Distance along the screen direction, rounded to whole texels so samples read exact depths.
			float s = (j + noise.y) / g_ao.step_count;
			s = s * s + min_s;
			float2 offset = round(s * texel_radius * omega);
			float2 uv0 = uv + offset;
			float2 uv1 = uv - offset;

			// Measure the angle of each sample above the view vector. Distant samples fade towards the low horizon, so geometry beyond the radius does not occlude.
			// Rounding moves samples slightly out of the slice plane, so the angle is measured on the projection into that plane. Otherwise samples on a flat
			// surface seen off-axis by a perspective camera would rise above the tangent horizon and darken the surface.
			float z0 = ViewDepth(int2(uv0));
			float z1 = ViewDepth(int2(uv1));
			float3 d0 = ViewPosition(uv0, z0) - P;
			float3 d1 = ViewPosition(uv1, z1) - P;
			float len0 = length(d0);
			float len1 = length(d1);
			float3 s0 = d0 - axis * dot(d0, axis);
			float3 s1 = d1 - axis * dot(d1, axis);
			float w0 = z0 >= BACKGROUND_TEST ? 0.0f : saturate(len0 * g_ao.falloff_mul + g_ao.falloff_add);
			float w1 = z1 >= BACKGROUND_TEST ? 0.0f : saturate(len1 * g_ao.falloff_mul + g_ao.falloff_add);
			float shc0 = lerp(low0, dot(s0, view_vec) * rsqrt(max(dot(s0, s0), 1e-12f)), w0);
			float shc1 = lerp(low1, dot(s1, view_vec) * rsqrt(max(dot(s1, s1), 1e-12f)), w1);
			hc0 = max(hc0, shc0);
			hc1 = max(hc1, shc1);
		}

		// Convert the horizon cosines to angles, limited to the hemisphere around the normal, and integrate the cosine-weighted visible arc.
		float h0 = -acos(clamp(hc1, -1.0f, 1.0f));
		float h1 = acos(clamp(hc0, -1.0f, 1.0f));
		h0 = n + clamp(h0 - n, -HALF_PI, HALF_PI);
		h1 = n + clamp(h1 - n, -HALF_PI, HALF_PI);
		float iarc0 = (cos_norm + 2 * h0 * sin(n) - cos(2 * h0 - n)) / 4;
		float iarc1 = (cos_norm + 2 * h1 * sin(n) - cos(2 * h1 - n)) / 4;
		visibility += proj_normal_len * (iarc0 + iarc1);
	}

	// Average the slices and apply the intensity here, at the lower resolution, rather than per output pixel. Do not clamp to 1:
	// for a normal tilted away from the view vector, single slices can exceed 1 and only the average over slice angles is 1.
	// The blur averages over every noise cell, so clamping before it would bias tilted surfaces dark. The blur clamps instead.
	visibility = max(visibility / g_ao.slice_count, 1e-4f);
	g_out[uv] = pow(visibility, g_ao.intensity);
}

// Average the occlusion over a 4x4 block of texels with similar depth. The block contains each noise cell once, so the noise cancels on flat surfaces.
numthreads(CSAOBlur, 8, 8, 1)
void CSAOBlur(uint3 dtid : SV_DispatchThreadID)
{
	// Threads beyond the AO buffer have no texel. Background texels stay fully visible.
	if (any(dtid.xy >= uint2(g_ao.ao_size)))
		return;

	int2 uv = int2(dtid.xy);
	float z = ViewDepth(uv);
	if (z >= BACKGROUND_TEST)
	{
		g_out[uv] = 1.0f;
		return;
	}

	// Weight texels by depth similarity so occlusion does not leak across depth edges.
	float sum = 0;
	float weight = 0;
	float tolerance = SAME_SURFACE * z;
	for (int y = -1; y != 3; ++y)
	{
		for (int x = -1; x != 3; ++x)
		{
			// Texels outside the buffer are clamped, which repeats edge texels.
			int2 t = clamp(uv + int2(x, y), int2(0, 0), g_ao.ao_size - 1);
			float w = saturate(1.0f - abs(g_view_depth.Load(int3(t, 0)) - z) / tolerance);
			sum += w * g_ao_in.Load(int3(t, 0));
			weight += w;
		}
	}

	// The centre texel always has full weight, so 'weight' is at least 1. Occlusion only darkens, so the average is limited to 1.
	g_out[uv] = min(sum / weight, 1.0f);
}

// Pixel shader input from the full screen triangle vertex shader 'VSPostEffect' (see underwater.hlsl).
struct PSIn_AO
{
	float4 ss_vert :SV_Position;
};

// The occlusion at render target pixel 'pixel', upsampled from the AO buffer.
float UpsampleAO(int2 pixel)
{
	// Pixels without scene geometry are not occluded.
	float d = g_scene_depth.Load(pixel, 0);
	if (d == g_ao.clear_depth)
		return 1.0f;

	// Find the four AO texels around the pixel. AO texel 't' was computed at viewport pixel 't * scale', so that is its position.
	float z = LinearDepth(d);
	float2 pos = float2(pixel - int2(g_ao.viewport.xy)) / g_ao.scale;
	int2 t0 = int2(floor(pos));
	float2 f = pos - t0;

	// Blend the four texels with bilinear weights, reduced for texels on a different surface.
	float tolerance = SAME_SURFACE * z;
	float sum = 0;
	float weight = 0;
	float best_diff = BACKGROUND;
	float best_ao = 1.0f;
	[unroll] for (int i = 0; i != 4; ++i)
	{
		// The bilinear weight of corner 'o', times the depth similarity.
		int2 o = int2(i & 1, i >> 1);
		int2 t = clamp(t0 + o, int2(0, 0), g_ao.ao_size - 1);
		float tz = g_view_depth.Load(int3(t, 0));
		float ao = g_ao_in.Load(int3(t, 0));
		float2 bw = lerp(1.0f - f, f, float2(o));
		float diff = abs(tz - z);
		float w = bw.x * bw.y * saturate(1.0f - diff / tolerance);
		sum += w * ao;
		weight += w;

		// Remember the texel nearest in depth, for pixels with no similar texel (e.g. thin features missed at the lower resolution).
		if (diff < best_diff)
		{
			best_diff = diff;
			best_ao = ao;
		}
	}
	return weight > 1e-4f ? sum / weight : best_ao;
}

// Multiply the occlusion into the opaque colour. The blend state does the multiply, so the output is the occlusion in every channel.
float4 PSAOApply(PSIn_AO In) : SV_Target0
{
	return UpsampleAO(int2(In.ss_vert.xy)).xxxx;
}

// Show the occlusion as grey scale, replacing the opaque colour.
float4 PSAODebug(PSIn_AO In) : SV_Target0
{
	float ao = UpsampleAO(int2(In.ss_vert.xy));
	return float4(ao, ao, ao, 1);
}
