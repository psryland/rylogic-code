//***********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2010
//***********************************************
#ifndef PR_VIEW3D_SHADER_ENV_MAP_HLSLI
#define PR_VIEW3D_SHADER_ENV_MAP_HLSLI
#include "view3d-12/src/shaders/hlsl/types.hlsli"

// Parallax correction finds where a reflected ray meets the captured scene by marching along the ray and comparing each point's distance from the
// capture centre with the distance the map stores in that direction. A cube map lookup by direction alone treats the environment as infinitely
// distant, so nearby objects appear shifted in reflections. Only geometry inside the parallax bounds is marched; anything the ray reaches beyond
// them, such as the sky, is treated as infinitely distant. See 'EnvMapDirections'.

// The most coarse march samples one pixel takes. The pixels of a 2x2 quad test different positions, so the quad tests four times as many.
static const int EnvMapMarchMaxLaneSteps = 8;

// The most halving steps used to refine a hit
static const int EnvMapMarchMaxRefineSteps = 6;

// The most extra samples used to search near the closest approach of a ray whose coarse samples all missed. Each one shrinks the searched
// interval by about 0.618.
static const int EnvMapMarchMaxNearMissSteps = 5;

// Stored distances at or above this are treated as infinitely distant. Nothing rendered (the far plane) stores exactly 1, but filtering between
// far-plane texels and distant geometry gives values just below it.
static const float EnvMapMaxDistance = 1.0f - 4.0f / 65535.0f;

// The marched part of a reflected ray, parameterised by the angle at which the current map's capture centre sees each point on it.
// Equal steps in this angle move equal distances across the map, so near and far parts of the ray are sampled at the same density on the map.
// The angle is measured from the ray's closest point to the centre, so a ray point at parameter 't' is seen at 'atan((t - t_closest) / height)'.
struct EnvMapRay
{
	float3 pos;       // World-space ray start
	float3 dir;       // Normalised world-space ray direction
	float3 centre;    // World-space capture centre of the current map
	float  t_closest; // Ray parameter of the point closest to 'centre'
	float  height;    // Distance from 'centre' to the ray's closest point, kept above zero
	float  angle0;    // Angle of the start of the marched part
	float  span;      // Angle swept by the marched part. 0 when the ray misses the parallax bounds
	float  scale;     // Scale 'S' of the distances stored in the map's distance cube
	float  lod;       // Mip level that distances are read from
};

// Return the world-space point at fraction 's' of the marched part of 'ray'
float3 EnvMapRayPoint(EnvMapRay ray, float s)
{
	float t = ray.t_closest + ray.height * tan(ray.angle0 + s * ray.span);
	return ray.pos + t * ray.dir;
}

// Return how far the point at fraction 's' of 'ray' is in front of the captured surface seen in the same direction. Positive means the ray has
// not reached the surface yet. Distances are compared in their stored form, 'd / (d + S)', which keeps their order without decoding them.
// 'on_surface' is false when the centre sees nothing in that direction (the sky or the far plane).
float EnvMapRayGap(EnvMapRay ray, float s, out bool on_surface)
{
	// Distances near 1 are the sky or the far plane, which no ray point can be behind
	float3 v = EnvMapRayPoint(ray, s) - ray.centre;
	float d = length(v);
	float a = g_envmap_distance.SampleLevel(g_envmap_sampler, mul(float4(v, 0.0f), g_frame.env_map.w2env).xyz, ray.lod);
	on_surface = a < EnvMapMaxDistance;
	a = on_surface ? a : 1.0f;
	return a - d / (d + ray.scale);
}
float EnvMapRayGap(EnvMapRay ray, float s)
{
	bool on_surface;
	return EnvMapRayGap(ray, s, on_surface);
}

// Return the part [t0, t1] of the ray from 'pos' in normalised direction 'dir' that is inside the box from 'lo' to 'hi'. The part starts no
// earlier than the ray start, and is empty (t0 > t1) when the ray misses the box.
float2 EnvMapClipRay(float3 pos, float3 dir, float3 lo, float3 hi)
{
	// Intersect the ray with each pair of box faces. A tiny replacement for a zero direction component keeps the face distances ordered.
	float3 safe_dir = select(abs(dir) > 1e-12f, dir, 1e-12f);
	float3 ta = (lo - pos) / safe_dir;
	float3 tb = (hi - pos) / safe_dir;
	float3 t_min = min(ta, tb);
	float3 t_max = max(ta, tb);
	return float2(max(max(t_min.x, t_min.y), max(t_min.z, 0.0f)), min(min(t_max.x, t_max.y), t_max.z));
}

// Return the world-space directions from the current and previous maps' capture centres to the environment seen along the ray from 'ws_pos' in
// direction 'ws_dir', and convert them to env-map space for sampling. 'ss_pos' is the pixel position, which places the pixel within its 2x2 quad.
// 'importance' is the weight the reflection will have in the final colour, in [0,1]; weaker reflections take fewer march samples.
// 'lod' is the mip level the reflection will be sampled at; coarser mips need fewer samples to resolve.
// Every pixel of a quad must call this with the same frame constants, because the pixels share their march results.
void EnvMapDirections(float3 ws_pos, float3 ws_dir, float2 ss_pos, float importance, float lod, out float3 dir, out float3 dir_prev)
{
	// Without stored distances or parallax bounds, the environment is infinitely distant, so the ray direction alone selects it.
	// The branch is uniform across the frame.
	float3 n = normalize(ws_dir);
	float3 ws_hit_dir = n;
	float3 ws_hit_dir_prev = n;
	if (g_frame.env_map.centre.w > 0.0f && g_frame.env_map.bounds_min.w > 0.0f)
	{
		// Describe the part of the ray inside the parallax bounds by the angles at which the capture centre sees its ends
		float2 t = EnvMapClipRay(ws_pos, n, g_frame.env_map.bounds_min.xyz, g_frame.env_map.bounds_max.xyz);
		EnvMapRay ray;
		ray.pos = ws_pos;
		ray.dir = n;
		ray.centre = g_frame.env_map.centre.xyz;
		ray.t_closest = -dot(ws_pos - ray.centre, n);
		ray.height = max(length(ws_pos - ray.centre + ray.t_closest * n), 1e-4f);
		ray.angle0 = atan2(t.x - ray.t_closest, ray.height);
		ray.span = t.x < t.y ? atan2(t.y - ray.t_closest, ray.height) - ray.angle0 : 0.0f;
		ray.scale = g_frame.env_map.centre.w;
		ray.lod = lod;

		// Choose the number of coarse samples. Sampling finer than one texel of the sampled mip gains nothing, and weak reflections get a smaller
		// budget. A face texel spans about 2/size radians. The quad uses the largest count of its pixels, so all four sample on the same grid.
		uint face_w, face_h, mips;
		g_envmap_texture.GetDimensions(0, face_w, face_h, mips);
		float texel = 2.0f / face_w * exp2(lod);
		int budget = max(1, (int)ceil(EnvMapMarchMaxLaneSteps * saturate(importance)));
		int lane_steps = clamp((int)ceil(ray.span / (4.0f * texel)), 1, budget);
		lane_steps = max(max(lane_steps, QuadReadAcrossX(lane_steps)), max(QuadReadAcrossY(lane_steps), QuadReadAcrossDiagonal(lane_steps)));
		lane_steps = clamp(lane_steps, 1, EnvMapMarchMaxLaneSteps);

		// March the coarse samples. The quad's pixels interleave their samples, so pixel 'k' tests fractions (4j + k + 1)/N of its own ray, where
		// N is the quad's total sample count. The march stops at the first sample behind the captured surface. Fraction 0 is the reflecting
		// surface itself, which is treated as in front.
		int quad_lane = (int(ss_pos.x) & 1) + 2 * (int(ss_pos.y) & 1);
		float step = 1.0f / (4 * lane_steps);
		float s_own = 2.0f;
		float s_near = 2.0f;
		float gap_near = 1e30f;
		if (ray.span > 0.0f)
		{
			for (int j = 0; j != lane_steps; ++j)
			{
				// Stop at the first sample behind the surface
				float s = (4 * j + quad_lane + 1) * step;
				bool on_surface;
				float gap = EnvMapRayGap(ray, s, on_surface);
				if (gap <= 0.0f)
				{
					s_own = s;
					break;
				}

				// Remember the sample that came closest to a surface, in case the ray passes behind it between samples
				if (on_surface && gap < gap_near)
				{
					s_near = s;
					gap_near = gap;
				}
			}
		}

		// Neighbouring rays are nearly parallel, so the quad's earliest hit fraction is a good guess for each pixel's hit, at four times the
		// resolution of the pixel's own samples. Neighbour values are only hints, so values from inactive or discarded pixels are ignored.
		float s_quad = s_own;
		float s_x = QuadReadAcrossX(s_own);
		float s_y = QuadReadAcrossY(s_own);
		float s_d = QuadReadAcrossDiagonal(s_own);
		s_quad = s_x > 0.0f && s_x < s_quad ? s_x : s_quad;
		s_quad = s_y > 0.0f && s_y < s_quad ? s_y : s_quad;
		s_quad = s_d > 0.0f && s_d < s_quad ? s_d : s_quad;

		// Check the guess on this pixel's own ray to find an interval [lo, hi] whose start is in front of the surface and whose end is behind it.
		// Every pixel sample before 's_own' was in front, which bounds the interval when the guess is wrong.
		float lo = 0.0f;
		float hi = 2.0f;
		if (s_quad <= 1.0f)
		{
			if (s_quad == s_own || EnvMapRayGap(ray, s_quad) <= 0.0f)
			{
				// The guess is behind the surface. Use the interval since the previous quad sample if that sample is in front. Otherwise the surface
				// is earlier still, after this pixel's last sample in front of it.
				float s_prev = s_quad - step;
				if (s_prev <= 0.0f || EnvMapRayGap(ray, s_prev) > 0.0f)
				{
					lo = max(s_prev, 0.0f);
					hi = s_quad;
				}
				else
				{
					int j = (int)floor((s_prev / step - quad_lane - 1) / 4.0f);
					lo = j >= 0 ? (4 * j + quad_lane + 1) * step : 0.0f;
					hi = s_prev;
				}
			}
			else if (s_own <= 1.0f)
			{
				// The guess is in front of the surface, so this pixel's own hit is later
				lo = max(s_quad, s_own - 4.0f * step);
				hi = s_own;
			}
		}

		// A ray can pass behind a thin part of a surface, such as just below the top edge of a wall, entirely between its samples. If no sample
		// found a hit but one came close to a surface, search around it for a point behind that surface. Each step keeps the part of the interval
		// that holds the smaller gap (a golden-section search), and stops at the first point behind the surface. Rays that only saw the sky skip this.
		if (hi > 1.0f && s_near <= 1.0f)
		{
			// Search between the neighbouring samples of this pixel, which were both in front of the surface
			const float golden = 0.618034f;
			float a = max(s_near - 4.0f * step, 0.0f);
			float b = min(s_near + 4.0f * step, 1.0f);
			float x1 = b - golden * (b - a);
			float x2 = a + golden * (b - a);
			float f1 = EnvMapRayGap(ray, x1);
			float f2 = EnvMapRayGap(ray, x2);
			float s_hit = 2.0f;
			for (int i = 0; i != EnvMapMarchMaxNearMissSteps + 1; ++i)
			{
				// Stop at the first point found behind the surface
				if (f1 <= 0.0f || f2 <= 0.0f)
				{
					s_hit = f1 <= 0.0f ? x1 : x2;
					break;
				}

				// Narrow the interval towards the smaller gap, reusing the inner point that stays inside it
				if (i == EnvMapMarchMaxNearMissSteps)
					break;

				if (f1 < f2)
				{
					b = x2;
					x2 = x1;
					f2 = f1;
					x1 = b - golden * (b - a);
					f1 = EnvMapRayGap(ray, x1);
				}
				else
				{
					a = x1;
					x1 = x2;
					f1 = f2;
					x2 = a + golden * (b - a);
					f2 = EnvMapRayGap(ray, x2);
				}
			}

			// Bracket the crossing with a point known to be in front: 's_near' if it comes before the hit, else this pixel's previous sample.
			if (s_hit <= 1.0f)
			{
				lo = s_near < s_hit ? s_near : max(s_near - 4.0f * step, 0.0f);
				hi = s_hit;
			}
		}

		// Halve the interval until it spans about one texel of the sampled mip, then use its middle as the hit. A ray that reaches no surface
		// inside the bounds sees the infinitely distant environment, so it keeps its own direction.
		if (hi <= 1.0f)
		{
			int refine_steps = clamp((int)ceil(log2(max((hi - lo) * ray.span / texel, 1.0f))), 0, EnvMapMarchMaxRefineSteps);
			for (int i = 0; i != refine_steps; ++i)
			{
				// Keep the half that still crosses the surface
				float mid = 0.5f * (lo + hi);
				if (EnvMapRayGap(ray, mid) > 0.0f)
					lo = mid;
				else
					hi = mid;
			}

			// The previous map, which is only sampled while it fades out, sees the same hit from its own capture centre
			float3 hit = EnvMapRayPoint(ray, 0.5f * (lo + hi));
			ws_hit_dir = hit - ray.centre;
			ws_hit_dir_prev = hit - g_frame.env_map.centre_prev.xyz;
		}
	}
	dir = mul(float4(normalize(ws_hit_dir), 0.0f), g_frame.env_map.w2env).xyz;
	dir_prev = mul(float4(normalize(ws_hit_dir_prev), 0.0f), g_frame.env_map.w2env).xyz;
}

// Sample the environment map in env-map space directions 'dir' (current map) and 'dir_prev' (previous map). While a new map fades in, the
// result blends from the previous map. The blend weight is the same for every pixel in the frame, so the branch does not diverge.
// 'dir_dx' and 'dir_dy' are the screen-space changes in the direction that set the filtering; the same values are used for both maps.
float4 SampleEnvMap(float3 dir, float3 dir_prev, float3 dir_dx, float3 dir_dy)
{
	float4 col = g_envmap_texture.SampleGrad(g_envmap_sampler, dir, dir_dx, dir_dy);

	float blend = g_frame.env_map.blend.x;
	if (blend < 1.0f)
	{
		// Blend from the previous map
		float4 prev = g_envmap_prev_texture.SampleGrad(g_envmap_sampler, dir_prev, dir_dx, dir_dy);
		col = lerp(prev, col, blend);
	}
	return col;
}

// Sample mip level 'lod' of the environment maps, blending from the previous map like 'SampleEnvMap'
float4 SampleEnvMapLevel(float3 dir, float3 dir_prev, float lod)
{
	float4 col = g_envmap_texture.SampleLevel(g_envmap_sampler, dir, lod);

	float blend = g_frame.env_map.blend.x;
	if (blend < 1.0f)
	{
		// Blend from the previous map
		float4 prev = g_envmap_prev_texture.SampleLevel(g_envmap_sampler, dir_prev, lod);
		col = lerp(prev, col, blend);
	}
	return col;
}

// Blend the reflected environment into 'initial_diff'. 'env_reflectivity' is the reflectivity when viewed straight on; the reflection
// strengthens towards a full mirror at grazing angles, so a value of 1 is a perfect mirror from every direction.
// 'slope_variance' is the mean square slope of surface detail too fine for 'ws_norm' to show. Zero gives a sharp mirror reflection; larger
// values blur the reflection and weaken it at grazing angles, the way a rough surface does.
// 'ss_pos' is the pixel position (see 'EnvMapDirections').
float4 EnvironmentMap(float4 ws_pos, float4 ws_norm, float4 ws_cam, float2 ss_pos, float4 initial_diff, float slope_variance)
{
	// A rough surface acts like many tiny mirrors tilted about the normal, so it reflects a cone of directions. A coarser mip averages
	// that cone. Roughness is the fourth root of the mean square slope, which is the scale the PBR path uses to choose its mip level.
	uint env_w, env_h, env_mips;
	g_envmap_texture.GetDimensions(0, env_w, env_h, env_mips);
	float roughness = sqrt(sqrt(max(slope_variance, 0.0f)));
	float lod = roughness * (env_mips - 1);

	// Weight the reflection with the Schlick approximation to the Fresnel term, so reflection rises from the straight-on value as the view
	// becomes parallel to the surface. On a rough surface, some facets still face the viewer at grazing angles, so the limit falls from 1
	// to 1 - roughness. 'abs' treats both sides of the surface the same.
	float4 to_surface = ws_pos - ws_cam;
	float reflectivity = saturate(g_nugget.env_reflectivity);
	float cos_theta = saturate(abs(dot(normalize(to_surface.xyz), normalize(ws_norm.xyz))));
	float grazing = max(1.0f - roughness, reflectivity);
	float fresnel = reflectivity + (grazing - reflectivity) * pow(1.0f - cos_theta, 5.0f);

	// Sample the environment at the parallax-corrected hit. The filtering follows the plain mirror direction, which changes smoothly across the
	// surface; the hit directions jump between near and far surfaces at their edges, which would otherwise select a needlessly blurred mip.
	// Scaling the changes by 2^lod moves the selected mip 'lod' levels coarser. The Fresnel weight sets how much effort the parallax correction gets.
	float3 ws_mirror = reflect(to_surface, ws_norm).xyz;
	float3 mirror = mul(float4(normalize(ws_mirror), 0.0f), g_frame.env_map.w2env).xyz;
	float lod_scale = exp2(lod);
	float3 dir, dir_prev;
	EnvMapDirections(ws_pos.xyz, ws_mirror, ss_pos, fresnel, lod, dir, dir_prev);
	float4 col = SampleEnvMap(dir, dir_prev, ddx(mirror) * lod_scale, ddy(mirror) * lod_scale);
	return lerp(initial_diff, col, fresnel);
}

#endif