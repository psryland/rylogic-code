//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2025
//************************************
// Procedural atmospheric background shader.
// VS: Reconstructs world view directions from a full-screen triangle.
// PS: Computes the atmosphere from the sun position, then adds stars and layered clouds.
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/forward/forward_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/sky/procedural_sky_cbuf.hlsli"
#include "view3d-12/src/shaders/hlsl/sky/cloud_field.hlsli"
#include "view3d-12/src/shaders/hlsl/utility/colour_space.hlsli"

#ifdef __cplusplus
namespace pr::rdr12::sky
{
	using namespace pr::hlsl;
#endif

ConstantBuffer<CBufFrame> resource(g_frame, b0);
ConstantBuffer<CBufProceduralSky> resource(g_sky, b3);
TextureCube<float4> resource(g_background, t18);
SamplerState resource(g_background_sampler, s1);

// The low cloud accumulated over previous frames, and its update for this frame. One texel per 2x2 pixel quad. See AccumulateLowCloud.
Texture2D<float4> resource(g_sky_history, t22);
RWTexture2D<float4> resource(g_sky_history_out, u3);

struct PSOut
{
	float4 diff semantic(SV_TARGET);
};

// Relative luminance of a linear colour
float Luminance(float3 colour)
{
	return dot(colour, float3(0.2126, 0.7152, 0.0722));
}

// Optical depths of the whole atmosphere straight up, for red, green and blue light: Rayleigh scattering by air (blue scatters most),
// Mie scattering by haze (grey), and ozone absorption (removes orange, which keeps twilight skies blue rather than grey).
static const float3 TauRayleigh = float3(5.8e-3, 13.5e-3, 33.1e-3) * 8.0;
static const float3 TauOzone = float3(0.65e-6, 1.881e-6, 0.085e-6) * 15000.0;
static const float TauMie = 2.4e-3;

// Scale from the caller's sun colour and intensity to sky radiance. Radiance is tone mapped with 1 - exp(-L).
static const float SkyExposure = 3.5;

// The relative length of the path through the atmosphere toward 'sin_elevation', compared with straight up.
// Paths lengthen quickly near the horizon, but stay finite because the atmosphere is curved.
float AirMass(float sin_elevation)
{
	// Fitted curve for a curved atmosphere, valid from the horizon to straight up.
	float s = saturate(sin_elevation);
	float elevation_deg = degrees(asin(s));
	return 1.0 / (s + 0.50572 * pow(elevation_deg + 6.07995, -1.6364));
}

// Sunlight that reaches the ground through the atmosphere, as a fraction of the light above it.
float3 SunTransmittance(float sun_z, float path_scale)
{
	// Low sun light passes through more air, so blue is scattered out first, then green, leaving orange and red.
	return exp(-(TauRayleigh + 1.1 * TauMie + TauOzone) * AirMass(sun_z) * path_scale);
}

// Linear sky radiance along 'view_dir' without the sun disc or clouds. 'sun_light' is the light above the atmosphere.
float3 SkyRadiance(float3 view_dir, float3 sun_dir, float3 sun_light)
{
	// Sunlight scattered toward the eye along the view path. Light scattered high in the sky has a shorter path from the sun,
	// so upward views use less reddened sunlight; this keeps the twilight zenith blue while the horizon turns orange.
	float view_z = saturate(view_dir.z);
	float3 sun_trans = SunTransmittance(sun_dir.z, 1.0 - 0.9 * sqrt(view_z));
	float3 extinction = TauRayleigh + 1.1 * TauMie;
	float3 view_scatter = 1.0 - exp(-extinction * AirMass(view_z));

	// Air scatters almost evenly; haze scatters strongly forward, which makes the bright glow around the sun.
	float c = dot(view_dir, sun_dir);
	float phase_rayleigh = 0.75 * (1.0 + c * c);
	float g = 0.8;
	float phase_mie = 0.5 * (1.0 - g * g) / pow(abs(1.0 + g * g - 2.0 * g * c), 1.5);
	float3 radiance = sun_light * sun_trans * (TauRayleigh * phase_rayleigh + TauMie * phase_mie) / extinction * view_scatter;

	// At twilight the sky depends on the direction relative to the sun. Toward the sun the low sky glows, while the opposite sky is darker.
	// Light reaching the opposite sky has scattered more often, so it is violet rather than orange. The effects are strongest near the horizon.
	float twilight = smoothstep(0.2, 0.0, sun_dir.z) * lerp(0.5, 1.0, 1.0 - view_z);
	float toward = 0.5 + 0.5 * c;
	radiance *= lerp(1.0, lerp(0.3, 1.3, toward * toward), twilight);
	radiance = lerp(radiance, Luminance(radiance) * float3(0.85, 0.68, 1.2), twilight * (1.0 - toward) * 0.75);

	// After sunset the Earth's shadow rises from the horizon opposite the sun as a dark blue-grey band. Its top edge is at about the sun's depression angle.
	float shadow_top = -sun_dir.z;
	float in_shadow = (1.0 - smoothstep(shadow_top - 0.01, shadow_top + 0.04, view_z)) * smoothstep(0.3, -0.5, c);
	radiance = lerp(radiance, Luminance(radiance) * float3(0.55, 0.6, 0.85) * 0.35, in_shadow);

	// At dusk the sky darkens quickly to a deep blue, before the sun has set. Only the low sky around the sun keeps its bright orange glow.
	float dusk = smoothstep(0.17, 0.0, sun_dir.z);
	float glow = pow(saturate(toward), 6.0) * pow(1.0 - view_z, 3.0);
	float3 deep_blue = Luminance(radiance) * float3(0.45, 0.62, 1.0);
	radiance = lerp(radiance, lerp(deep_blue * 0.3, radiance, glow), dusk);

	// Fade to night as the sun sets below the horizon, leaving a faint blue night sky.
	float day = saturate(sun_dir.z * 12.0 + 1.0);
	return radiance * day * day + float3(0.002, 0.003, 0.006);
}

// Add the sun disc and its glow to the sky radiance 'sky' along 'view_dir', and darken views below the horizon.
// 'time' is in seconds, wrapped at PR_SKY_TIME_PERIOD, and slowly moves the soft rays in the glow.
float3 AtmosphericSky(float3 sky, float3 view_dir, float3 sun_dir, float3 sun_light, float time)
{
	// The sun disc, reddened by the same path through the atmosphere as sunlight on the ground.
	float cos_sun = dot(view_dir, sun_dir);
	float3 sun_seen = sun_light * SunTransmittance(sun_dir.z, 1.0);
	sky += 40.0 * sun_seen * smoothstep(0.99985, 0.99993, cos_sun) * step(-0.02, sun_dir.z);

	// A bright hazy glow around the sun, from light scattered slightly forward by haze. The phase function above is too broad to show it.
	// The angle from the sun (radians) is approximated by the chord length, which is accurate near the sun where the glow matters.
	// A tight bright core blends into a wide faint halo. Clouds are composited over this, so thick cloud hides the glow.
	float sun_angle = sqrt(max(2.0 * (1.0 - cos_sun), 0.0));

	// Soft rays that slowly drift and shimmer in the halo. The angle around the sun is measured in a world-fixed frame so the rays do not turn
	// with the camera. That frame has no fixed reference when the sun is exactly overhead, so another axis is used then.
	// Only whole multiples of the angle are used, so there is no seam where atan2 wraps. Time rates are whole cycles per PR_SKY_TIME_PERIOD,
	// so the motion is continuous when the CPU wraps the time. The rays fade out near the centre, where the angle around the sun is undefined.
	float3 side = cross(sun_dir, float3(0, 0, 1));
	side = dot(side, side) > 1e-6 ? normalize(side) : float3(1, 0, 0);
	float around = atan2(dot(view_dir, cross(sun_dir, side)), dot(view_dir, side));
	float w = 6.2831853 * time / PR_SKY_TIME_PERIOD;
	float rays = 0.5 * sin(7.0 * around + 10.0 * w) + 0.3 * sin(13.0 * around - 17.0 * w) + 0.2 * sin(23.0 * around + 29.0 * w);

	// A low sun is seen through much more haze, so its glow and shimmer are stronger. This also keeps the glow visible against the bright
	// twilight sky, even though the sunlight reaching the eye is heavily dimmed.
	float low = smoothstep(0.2, 0.0, sun_dir.z);
	float halo = 1.0 + lerp(0.3, 0.6, low) * rays * smoothstep(0.0, 0.03, sun_angle);

	// A tight bright core blends into a wide faint halo carrying the rays.
	float glow = (1.0 + 3.0 * low) * (1.125 * exp(-sun_angle / 0.02) + 0.2625 * halo * exp(-sun_angle / 0.12));
	sky += sun_seen * glow * smoothstep(-0.03, 0.0, sun_dir.z);

	// Below the horizon the background is darker, standing in for ground or sea.
	float below = saturate(-view_dir.z * 3.0);
	return lerp(sky, sky * 0.3, below);
}

// Scramble an integer lattice point and seed into a well-mixed 32-bit value.
uint LatticeHash(int3 cell, uint seed)
{
	// Multiply by large odd constants and fold the high bits down so neighbouring cells are uncorrelated.
	uint h = (uint)cell.x * 0x8DA6B343u ^ (uint)cell.y * 0xD8163841u ^ (uint)cell.z * 0xCB1AB31Fu ^ seed * 0x165667B1u;
	h ^= h >> 13;
	h *= 0x5BD1E995u;
	h ^= h >> 15;
	return h;
}

// Map a hash to [0,1]
float Unorm(uint h)
{
	return (h & 0xFFFFFFu) / 16777215.0;
}

// Twinkling point stars on a sphere of hashed cells. 'pixel_size' is the pixel footprint on that sphere.
float3 Stars(float3 dir, float pixel_size, float time)
{
	// One possible star per cell of a grid around a sphere of radius 300; most cells are empty.
	float3 p = dir * 300.0;
	int3 cell = int3(floor(p));
	uint h = LatticeHash(cell, 7u);
	if (Unorm(h) < 0.993)
		return 0;

	// Place the star inside the cell, then measure its distance along the sphere.
	float3 r = float3(Unorm(LatticeHash(cell, 11u)), Unorm(LatticeHash(cell, 13u)), Unorm(LatticeHash(cell, 17u)));
	float3 star = normalize(floor(p) + 0.25 + 0.5 * r) * 300.0;
	float d = length(p - star);

	// Keep stars at least a pixel wide so they do not flicker as the camera turns. Most stars are faint, a few are bright.
	float sigma = max(pixel_size * 0.6, 0.04);
	float shape = exp(-d * d / (sigma * sigma));
	float brightness = 0.15 + 2.0 * pow(r.x, 6.0);

	// Twinkle rates are whole cycles per PR_SKY_TIME_PERIOD, so the twinkle is continuous when the CPU wraps the time.
	float cycles = 15.0 + floor(40.0 * r.y);
	float twinkle = 0.75 + 0.25 * sin(6.2831853 * cycles * time / PR_SKY_TIME_PERIOD + r.z * 6.283);
	float3 colour = lerp(float3(0.75, 0.85, 1.0), float3(1.0, 0.9, 0.75), r.y);
	return colour * brightness * twinkle * shape;
}

// The colour and radiance of lightning light in cloud at brightness 1.
static const float3 LightningColour = float3(0.75, 0.82, 1.0) * 4.0;

// Lightning light at 'pos' on a cloud layer, from the flashes in g_sky.lightning. Each flash lights a soft patch around its position,
// and a faint wide glow spreads through the rest of the deck.
float LightningGlow(float2 pos)
{
	// Sum the flashes. Unused slots have zero brightness.
	float glow = 0;
	[unroll] for (int i = 0; i != PR_SKY_LIGHTNING_MAX; ++i)
	{
		// Skip unused slots so their radius is never read.
		float4 flash = g_sky.lightning[i];
		if (flash.w <= 0)
			continue;

		float2 d = (pos - flash.xy) / flash.z;
		float r2 = dot(d, d);
		glow += flash.w * (exp(-r2) + 0.15 * exp(-0.05 * r2));
	}
	return glow;
}

// Find where the ray from 'cam' along 'dir' leaves the spherical shell at 'altitude' around the planet centre. The camera must be inside the shell.
// Returns false when the camera is outside the shell. 'mu' is the cosine between the ray and the shell's upward normal there, which is small
// where the ray grazes the shell near the horizon. The shell curves down to meet the horizon at a finite distance, which limits how squashed
// distant cloud becomes. The terms are arranged to avoid losing precision with the large planet radius.
bool ShellExit(float3 cam, float3 dir, float altitude, out float t, out float mu)
{
	// Solve |cam + t*dir - centre| = radius for the positive root.
	float radius = PR_SKY_PLANET_RADIUS + altitude;
	float b = dot(cam.xy, dir.xy) + (cam.z + PR_SKY_PLANET_RADIUS) * dir.z;
	float c = dot(cam.xy, cam.xy) + (cam.z - altitude) * (cam.z + altitude + 2.0 * PR_SKY_PLANET_RADIUS);
	float disc = sqrt(max(b * b - c, 0.0));
	t = b >= 0 ? -c / (b + disc) : disc - b;
	mu = max((b + t) / radius, 0.02);
	return c < 0;
}

// The height of 'p' above the spherical shell at 'altitude'. Accurate for points near the camera compared with the planet radius.
float ShellHeight(float3 p, float altitude)
{
	// The shell drops below the tangent plane by the squared horizontal distance over twice the radius.
	return p.z + dot(p.xy, p.xy) / (2.0 * PR_SKY_PLANET_RADIUS) - altitude;
}

// The mip level of the cloud noise for 'layer' at distance 't' along a ray meeting the layer at 'mu' (see ShellExit).
float CloudLod(int layer, float t, float pixel_angle, float mu)
{
	// The pixel footprint on the layer stretches where the ray grazes it.
	float footprint = t * pixel_angle / sqrt(mu);
	return log2(max(footprint * PR_SKY_CLOUD_NOISE_SIZE / min(CloudLayers[layer].y, CloudLayers[layer].z), 1e-3));
}

// How strongly a sun just above the horizon shines under the clouds. Their bases, which are usually the shaded side, are then lit in sunset colours.
// Storm decks block the low sun, so they stay shaded.
float UnderLit(float3 sun_dir, float storm)
{
	// Only a sun close to the horizon lights the bases.
	return smoothstep(0.15, 0.03, sun_dir.z) * smoothstep(-0.05, 0.0, sun_dir.z) * (1.0 - storm);
}

// Scattering by cloud droplets toward the viewer, relative to even scattering, for a ray at 'cos_angle' to the light. 'g' > 0 favours forward scattering.
float PhaseHG(float cos_angle, float g)
{
	// The Henyey-Greenstein phase function, scaled so that g = 0 gives 1.
	return (1.0 - g * g) / pow(abs(1.0 + g * g - 2.0 * g * cos_angle), 1.5);
}

// A pseudo-random value in [0,1) for an integer cell, with no visible pattern between neighbouring cells.
float CellHash(int2 cell)
{
	// Mix the cell coordinates with an integer hash, which has no repeating structure at the scales used.
	uint2 p = asuint(cell);
	uint h = p.x * 1973u + p.y * 9277u;
	h = (h << 13) ^ h;
	h = h * (h * h * 15731u + 789221u) + 1376312589u;
	return float(h & 0x00ffffffu) / 16777216.0;
}

// A pseudo-random value in [0,1) fixed to the cloud field, at noise coordinate 'uv1' of a layer read at mip level 'lod' (see CloudLod).
// Used to offset ray march samples, so the error left by widely spaced samples shows as fine grain rather than bands or stripes.
// The grain is made of cells in the layer's noise coordinates, so it drifts with the clouds instead of staying fixed on the screen.
// The cells are about one noise texel at 'lod', which is about one pixel. Values from the two nearest cell sizes are blended, so the
// grain does not jump as the cell size changes with distance.
float CloudGrain(float2 uv1, float lod)
{
	// Hash the cells at the two mip levels around 'lod'.
	float level = floor(lod);
	float2 texels = uv1 * PR_SKY_CLOUD_NOISE_SIZE;
	float g0 = CellHash(int2(floor(texels * exp2(-level))));
	float g1 = CellHash(int2(floor(texels * exp2(-level - 1.0))) + int2(7919, 104729));
	return lerp(g0, g1, lod - level);
}

// Finish the colour of a cloud layer: fade it into the air, tint storm cloud, and fade it out at the horizon. Returns premultiplied radiance and alpha.
// 'radiance' is not premultiplied. 't' is the typical distance to the visible cloud, and 'darkening' how much the layer darkens in a storm.
float4 CloudFinish(float3 radiance, float alpha, float t, float3 dir, float storm, float darkening, float3 haze)
{
	// Distant cloud fades into the air in front of it, which is dim under a storm.
	float fog = 1.0 - exp(-t / 70000.0);
	radiance = lerp(radiance, haze * lerp(1.0, 0.35, storm), fog);

	// Storm cloud is lit mostly by blue skylight scattered through the cloud deck, so it is a cool slate grey rather than a warm grey.
	radiance = lerp(radiance, Luminance(radiance) * float3(0.86, 0.93, 1.08), storm * darkening * 0.85);
	alpha *= saturate(dir.z * 50.0);
	return float4(radiance * alpha, alpha);
}

// A thin mid-level (1) or cirrus (2) cloud layer seen along 'dir' from 'cam'. Returns premultiplied radiance and alpha.
// 'lower_scale' scales the altitude of the mid layer (see PR_SKY_CLOUD_LOWER_SCALE). These layers are thin sheets shaded from the cloud field.
float4 CloudSheet(int layer, float3 cam, float3 dir, float pixel_angle, float lower_scale, float3 sun_dir, float3 sun_light, float3 ambient, float3 haze)
{
	// Find the layer, and read its cover and noise coordinates where the ray meets it.
	CloudConstants clouds = g_sky.clouds;
	float t, mu;
	if (!ShellExit(cam, dir, CloudLayerAltitude(layer, lower_scale), t, mu))
		return 0;

	float3 hit = cam + dir * t;
	float2 uv1, uv2;
	CloudUV(clouds, layer, hit.xy, uv1, uv2);
	float lod = CloudLod(layer, t, pixel_angle, mu);
	CloudShape s = CloudLayerShape(layer, CloudCover(clouds, hit.xy), uv1, lod, lower_scale);
	if (s.layer_cover <= 0)
		return 0;

	// Opacity rises over the soft band above the cloud edge.
	float field = CloudField(uv1, uv2, lod, s.lump_scale);
	float coverage = saturate(CloudEdge(s, field, CloudFray(layer, uv1, lod)) / s.softness);
	if (coverage <= 0)
		return 0;

	// Light reaching the visible base has crossed the cloud above it, so thick cloud is darker than its thin edges.
	// 'depth' measures how far the field rises above the cloud edge, including the lumps, so bulges and creases are shaded too.
	// 'shade' is the amount of greying, from 0 (white) to 1 (the darkest grey for this cover). A low sun lights the bases instead.
	float depth = saturate((field - s.threshold) / s.depth_range);
	float under_lit = UnderLit(sun_dir, s.storm);
	float shade = lerp(depth, 0.2 * depth, under_lit);

	// These layers are thinner than the low cloud, so they darken less. When the sun shines from below, the unlit tops are much darker,
	// which gives strong contrast with the glowing bases.
	float darkening = layer == 1 ? 0.6 : 0.25;
	float dark_max = lerp(lerp(lerp(0.25, 0.55, s.build), 0.9, s.storm), 0.85, under_lit) * darkening;
	float grey = 1.0 - dark_max * smoothstep(0.0, 1.0, shade);

	// Thin cloud near the sun glows from light scattered forward through it (silver lining). Storm cloud lets little direct sunlight through.
	float cos_sun = dot(dir, sun_dir);
	float phase = 1.0 + 2.0 * pow(saturate(cos_sun), 8.0) * (1.0 - coverage);
	float sunlit = lerp(1.0, 0.3, s.storm * darkening) + 1.2 * under_lit * (1.0 - shade);
	float3 radiance = grey * (sun_light * (0.65 * sunlit * phase) + ambient * 0.9) * lerp(1.0, 0.75, s.storm * darkening);

	// Toward a low sun, the viewer sees the unlit side of the cloud, so thick cloud there is a dark silhouette. Its thin edges still glow.
	float backlit = pow(saturate(cos_sun), 3.0) * smoothstep(0.25, 0.02, sun_dir.z);
	radiance *= lerp(1.0, 0.3, backlit * coverage);

	// Lightning below lights mid-level cloud from within, patchily where it is thick. Cirrus is far above the flashes and catches little of their light.
	radiance += LightningColour * LightningGlow(hit.xy) * (layer == 1 ? 0.5 : 0.1) * lerp(0.4, 1.0, depth);

	// Denser cloud, and longer paths where the ray grazes the layer, are more opaque. Squaring the opacity softens the edges.
	// Sparse cloud is thinner, so it is partly translucent.
	float thickness = (layer == 1 ? 3.0 : 0.8) * lerp(0.5, 1.0, s.build);
	float path = 1.0 + min(0.3 / mu, 3.0);
	float alpha = 1.0 - exp(-coverage * coverage * thickness * path);
	return CloudFinish(radiance, alpha, t, dir, s.storm, darkening, haze);
}

// The low cloud seen along 'dir' from 'cam', which must be below the cloud. Returns premultiplied radiance and alpha.
// The low cloud is a slab of finite height (see CloudLayerShape). Each column of the slab has a flat-ish base near the layer plane and a rounded top
// whose height follows the cloud field, so clouds have lit tops, shaded bases, and sides. The ray is marched through the slab, front to back,
// and each sample is lit by the sunlight that passes through the cloud toward the sun, the sky light, and any lightning.
// 'lower_scale' scales the layer's altitude (see PR_SKY_CLOUD_LOWER_SCALE). 'quad_slot' is the pixel's index, 0 to 3, within its 2x2 pixel quad.
// 'frame_jitter', in [0,1), moves every sample along its step, so that frames sample different points. Use 0 when frames are not accumulated.
// 'anchor' receives the point where the ray meets the layer plane. It is left unchanged when the ray does not meet the plane.
// The result is noisy per pixel and is meant to be averaged over the quad with QuadAverage, and over frames with AccumulateLowCloud.
float4 CloudSlab(float3 cam, float3 dir, float pixel_angle, float quad_slot, float frame_jitter, float lower_scale, float3 sun_dir, float3 sun_light, float3 ambient, float3 haze, inout float3 anchor)
{
	// The cloud's shape is set by the cover where the ray meets the layer plane, which is where most of the bases sit.
	CloudConstants clouds = g_sky.clouds;
	float altitude = CloudLayerAltitude(0, lower_scale);
	float t_plane, mu;
	if (!ShellExit(cam, dir, altitude, t_plane, mu))
		return 0;

	anchor = cam + dir * t_plane;
	float2 plane_xy = anchor.xy;
	float2 uv1, uv2;
	CloudUV(clouds, 0, plane_xy, uv1, uv2);
	float lod = CloudLod(0, t_plane, pixel_angle, mu);
	CloudShape s = CloudLayerShape(0, CloudCover(clouds, plane_xy), uv1, lod, lower_scale);
	if (s.layer_cover <= 0)
		return 0;

	// The part of the ray inside the slab, from below the lowest base to above the highest top.
	float t0, t1, mu_bounds;
	float floor_z = -CloudSlabSkirt * s.height;
	if (!ShellExit(cam, dir, altitude + floor_z, t0, mu_bounds))
		t0 = 0;

	ShellExit(cam, dir, altitude + s.height, t1, mu_bounds);

	// Steps are spaced at about a fifth of the slab height, so rays that graze the slab near the horizon take more steps and do not skip over
	// whole clouds, up to a limit that keeps the cost bounded. Accumulating frames averages the remaining gaps away. Footprints this wide are
	// a reflection probe or a distant view, where fewer samples are enough. Coarser noise on long steps keeps the gaps between samples from
	// showing as noise. The step count is fractional, with a shorter last step, so the spacing changes smoothly between neighbouring rays.
	// A whole number of steps would change the spacing suddenly and leave visible seams.
	float max_steps = pixel_angle > 0.003 ? 12.0 : 24.0;
	float step_count = clamp((t1 - t0) / (0.2 * s.height), 8.0, max_steps);
	int steps = (int)ceil(step_count);
	float dt = (t1 - t0) / step_count;
	float march_lod = max(lod, log2(max(dt * PR_SKY_CLOUD_NOISE_SIZE / min(CloudLayers[0].y, CloudLayers[0].z), 1e-3)) - 1.0);

	// Each pixel of a 2x2 pixel quad samples a different quarter of every step, so the average over the quad holds one sample from each
	// quarter (see QuadAverage). The cloud-fixed grain picks the point within the quarter. The grain's cells are sized for the pixel's
	// longest footprint on the layer, which is across the horizon where the ray grazes it, so they are no bigger than a pixel in any
	// direction. Larger cells would share one offset along a row of pixels and show as horizontal streaks.
	float grain_lod = lod - 0.5 * log2(max(mu, 1e-4));
	float jitter = frac((quad_slot + CloudGrain(uv1, grain_lod)) * 0.25 + frame_jitter);

	// Lighting that is the same for the whole ray. Light is scattered mainly forward, which makes thin cloud glow near the sun (silver lining),
	// with some even scattering. Sky light is weaker low in the cloud, because the cloud above blocks it, and weaker in larger and storm cloud.
	// Lightning is read once at the layer plane, because a flash lights a wide patch of cloud.
	float cos_sun = dot(dir, sun_dir);
	float phase = lerp(PhaseHG(cos_sun, 0.0), PhaseHG(cos_sun, 0.7), 0.2);
	float under_lit = UnderLit(sun_dir, s.storm);
	float ambient_floor = lerp(lerp(0.6, 0.4, s.build), 0.25, s.storm);
	float3 lightning = LightningColour * LightningGlow(plane_xy);

	// Samples toward the sun reach about half way through the slab. A low sun crosses the slab at a long slant, so its samples are limited to
	// stay near the cloud they shade.
	float sun_reach = 0.5 * s.height / max(sun_dir.z, 0.25);
	static const float SunSampleAt[3] = { 0.1, 0.35, 1.0 };
	static const float SunSampleSpan[3] = { 0.2, 0.3, 0.5 };

	// March front to back. 'transmit' is the fraction of light from behind that still reaches the camera.
	float transmit = 1.0;
	float3 radiance = 0;
	float t_sum = 0.0;
	[loop] for (int i = 0; i != steps; ++i)
	{
		// Skip samples outside the slab's possible height range without reading the noise. The last step is shortened to end at the slab exit.
		// The offset within each step moves on by the golden ratio, so samples of neighbouring rays do not line up at the same heights,
		// which would show as stacked slices. The quad's four offsets stay a quarter step apart.
		float ta = t0 + i * dt;
		float step_len = min(dt, t1 - ta);
		float t = ta + frac(jitter + i * 0.618034) * step_len;
		float3 p = cam + dir * t;
		float hz = ShellHeight(p, altitude);
		if (hz < floor_z || hz > s.height)
			continue;

		float density = CloudSlabDensityAt(clouds, s, p.xy, hz, march_lod, true);
		if (density <= 0.001)
			continue;

		// Sunlight is dimmed by the cloud between this sample and the sun. Light scattered more than once still gets through thick cloud, so it is
		// never fully dark: a weaker, more slowly falling term stands in for that light.
		float optical = 0.0;
		[unroll] for (int k = 0; k != 3; ++k)
		{
			// Coarser noise is enough, because the shading is soft.
			float3 q = p + sun_dir * (SunSampleAt[k] * sun_reach);
			optical += CloudSlabDensityAt(clouds, s, q.xy, ShellHeight(q, altitude), march_lod + 1.0, false) * SunSampleSpan[k] * sun_reach;
		}
		optical *= s.sigma;
		float sun_through = max(exp(-optical), 0.3 * exp(-0.25 * optical));

		// A low sun shines in under the cloud, lighting the bases. That light fades with depth above the base.
		float h_rel = saturate((hz - floor_z) / (s.height - floor_z));
		float base_light = under_lit * 1.2 * exp(-0.5 * s.sigma * (hz - floor_z));
		float3 light = sun_light * (0.65 * sun_through * phase + base_light) + ambient * (0.9 * lerp(ambient_floor, 1.0, h_rel)) + lightning * lerp(0.4, 1.0, density);

		// Add the light scattered toward the camera over this step, dimmed by the cloud in front. Every droplet scatters the light it stops,
		// so the light a step adds is its light times the fraction of light it stops.
		float stopped = 1.0 - exp(-s.sigma * density * step_len);
		radiance += transmit * stopped * light;
		t_sum += transmit * stopped * t;
		transmit *= 1.0 - stopped;

		// Stop once the cloud in front hides everything behind it.
		if (transmit < 0.02)
			break;
	}

	// Rays that met no cloud add nothing.
	float alpha = 1.0 - transmit;
	if (alpha <= 0.001)
		return 0;

	radiance /= alpha;

	// Storm cloud lets little light through overall. Toward a low sun, the viewer sees the unlit side of the cloud, so thick cloud there
	// is a dark silhouette. Its thin edges still glow.
	float backlit = pow(saturate(cos_sun), 3.0) * smoothstep(0.25, 0.02, sun_dir.z);
	radiance *= lerp(1.0, 0.75, s.storm) * lerp(1.0, 0.3, backlit * alpha);
	return CloudFinish(radiance, alpha, t_sum / alpha, dir, s.storm, 1.0, haze);
}

// The average of 'value' over the 2x2 pixel quad that contains 'pixel'. Must be called where every pixel of the quad runs the same code,
// because it reads the neighbouring pixels' values through the screen-space derivatives.
float4 QuadAverage(float4 value, uint2 pixel)
{
	// A fine derivative is the difference between the two pixels of a row (or column) of the quad, so half of it moves the value to the pair's mean.
	// Averaging the rows, then the columns, gives the mean of all four pixels.
	float2 side = 0.5 - float2(pixel & 1);
	value += side.x * ddx_fine(value);
	value += side.y * ddy_fine(value);
	return value;
}

// The per-channel range of 'value' over the 2x2 pixel quad. Must be called where every pixel of the quad runs the same code.
void QuadRange(float4 value, out float4 lo, out float4 hi)
{
	// Combine with the pixel across the row, then with the pixel pair across the column.
	float4 across = QuadReadAcrossX(value);
	lo = min(value, across);
	hi = max(value, across);
	lo = min(lo, QuadReadAcrossY(lo));
	hi = max(hi, QuadReadAcrossY(hi));
}

// Blend this frame's quad-averaged low cloud 'low_cloud' with the low cloud of previous frames, which averages away the grain of the ray march
// (see CloudSlab). Returns the blended low cloud, and stores it for the next frame. 'lo' and 'hi' are the range of this frame's samples over the
// quad (see QuadRange). 'anchor' is a point on the cloud seen through this pixel, in the atmosphere frame. 'drifts' is 1 when the anchor moves
// with the low cloud's wind, otherwise 0. 'pixel' is the screen pixel.
// Must be called where every pixel of the quad runs the same code, because the history is shared by the quad.
float4 AccumulateLowCloud(float4 low_cloud, float4 lo, float4 hi, float3 anchor, float drifts, uint2 pixel)
{
	// Frames are not accumulated in views whose camera jumps between frames, such as reflection probe captures.
	SkyHistory history = g_frame.sky_history;
	if (history.enabled == 0)
		return low_cloud;

	// Move the anchor back to where the cloud was in the previous frame. The low cloud's offset is in noise tiles of the wind frame (see CloudUV),
	// so a change in the offset moves the cloud by the change times the tile size, along and across the wind. The change is wrapped, because
	// the offset wraps at the noise period.
	CloudConstants clouds = g_sky.clouds;
	float2 wind;
	sincos(clouds.wind_direction, wind.y, wind.x);
	float2 moved = clouds.offset01.xy - history.prev_cloud_offset;
	moved -= PR_SKY_CLOUD_PERIOD * round(moved / PR_SKY_CLOUD_PERIOD);
	moved *= CloudLayers[0].yz;
	float2 shift = moved.x * wind + moved.y * float2(-wind.y, wind.x);
	float3 prev_anchor = anchor - drifts * float3(shift, 0);

	// Find where that point was on screen in the previous frame. The atmosphere frame is a rotation of the world frame, so the transpose of
	// 'world_to_sky' maps it back to the world. History texel 'i' holds the quad covering pixels 2i and 2i+1, so its centre is at pixel 2i+1.
	float3 prev_world = mul((float3x3)clouds.world_to_sky, prev_anchor);
	float4 prev_clip = mul(float4(prev_world, 1), history.prev_w2s);
	float2 prev_ndc = prev_clip.xy / max(prev_clip.w, 1e-6);
	float2 prev_pixel = history.viewport.xy + (prev_ndc * float2(0.5, -0.5) + 0.5) * history.viewport.zw;
	float2 texel = 0.5 * prev_pixel - 0.5;

	// Read the history with bilinear filtering. The history is unusable if the point was behind the camera, outside the history, or behind
	// scene geometry. Pixels behind geometry are not drawn by the sky, so they keep the cleared value, which has a negative alpha.
	uint2 size;
	g_sky_history.GetDimensions(size.x, size.y);
	int2 i0 = (int2)floor(texel);
	float2 f = texel - i0;
	float usable = history.valid * (prev_clip.w > 0) * all(i0 >= 0) * all(i0 + 1 < (int2)size);
	int2 c = clamp(i0, 0, (int2)size - 2);
	float4 h00 = g_sky_history.Load(int3(c, 0));
	float4 h10 = g_sky_history.Load(int3(c + int2(1, 0), 0));
	float4 h01 = g_sky_history.Load(int3(c + int2(0, 1), 0));
	float4 h11 = g_sky_history.Load(int3(c + int2(1, 1), 0));
	usable *= min(min(h00.a, h10.a), min(h01.a, h11.a)) >= 0;
	float4 prev = lerp(lerp(h00, h10, f.x), lerp(h01, h11, f.x), f.y);

	// Share one history between the pixels of the quad, so the quad keeps a single value. Any unusable pixel makes the whole quad unusable.
	prev = QuadAverage(prev, pixel);
	usable = min(usable, QuadReadAcrossX(usable));
	usable = min(usable, QuadReadAcrossY(usable));

	// Limit the history to the range of this frame's samples, which removes stale cloud left by changes the reprojection does not follow, such as
	// the clouds evolving or lightning. Then blend in a small part of this frame. Each frame's samples sit at different points along the ray
	// (see CloudSlab), so the blend averages over many sample points.
	static const float NewWeight = 0.1;
	float4 result = usable > 0 ? lerp(clamp(prev, lo, hi), low_cloud, NewWeight) : low_cloud;
	g_sky_history_out[pixel >> 1] = result;
	return result;
}

// The full procedural sky along 'dir' from 'cam' (both in the atmosphere frame). Returns linear colour in [0,1].
// 'pixel_angle' is the angle one pixel covers. 'pixel' is the screen pixel, used to share the low cloud's samples within each 2x2 pixel quad.
// Must be called where every pixel of a quad runs the same code (see QuadAverage).
float3 ProceduralSkyColour(float3 dir, float3 cam, float pixel_angle, uint2 pixel)
{
	// Sky radiance, lit by the sun above the atmosphere. 'haze' is the air without the sun disc, which distant clouds fade into.
	CloudConstants clouds = g_sky.clouds;
	float3 sun_dir = clouds.sun_direction.xyz;
	float3 sun_top = g_sky.sun_colour.rgb * g_sky.sun_intensity * SkyExposure;
	float3 haze = SkyRadiance(dir, sun_dir, sun_top);
	float3 sky = AtmosphericSky(haze, dir, sun_dir, sun_top, g_sky.time);

	// Stars appear in the darkening sky before sunset and fade out near the horizon where the air is thick.
	float star_visibility = smoothstep(0.08, -0.08, sun_dir.z) * saturate(dir.z * 5.0);
	if (star_visibility > 0)
		sky += Stars(dir, pixel_angle * 300.0, g_sky.time) * star_visibility;

	// Under heavy cloud little sunlight reaches the air below it, so the clear air seen toward the horizon and below it is dim and grey.
	float heavy = smoothstep(0.6, 1.0, CloudCover(clouds, cam.xy));
	float gloom = lerp(1.0, 0.17, heavy);
	sky = lerp(sky, Luminance(sky) * float3(0.85, 0.88, 0.92), 0.7 * heavy) * gloom;
	haze = lerp(haze, Luminance(haze) * float3(0.85, 0.88, 0.92), 0.7 * heavy) * gloom;

	// Lightning also lights the air below the cloud, faintly, by the flashes near the camera.
	sky += LightningColour * 0.03 * LightningGlow(cam.xy);

	// The low cloud's anchor for accumulating frames (see AccumulateLowCloud). Rays that miss the layer anchor to a distant point in their
	// direction, which does not drift with the wind.
	float4 low_cloud = 0;
	float3 anchor = cam + dir * 1e6;
	float drifts = 0;
	if (dir.z > 0)
	{
		// Sunlight at the clouds is reddened by its path through the atmosphere. Clouds keep a little light just after sunset.
		float3 sun_light = sun_top * SunTransmittance(sun_dir.z, 0.8) * saturate(sun_dir.z * 15.0 + 1.0);
		float lower_scale = CloudLowerScale(clouds, cam.xy);

		// Cloud bases are lit by the sky overhead and the air in the same direction, which is warm toward a setting sun.
		// Sky light is mostly grey in effect, because it arrives from all over the sky.
		float3 ambient = 0.5 * (SkyRadiance(float3(0, 0, 1), sun_dir, sun_top) * gloom + haze);
		ambient = lerp(ambient, Luminance(ambient), 0.5 + 0.35 * heavy);

		// Composite from the highest layer down, because the lowest layer is nearest to an observer below the clouds. Hidden layers are skipped.
		[unroll] for (int layer = 2; layer >= 1; --layer)
		{
			// Each sheet covers what is behind it.
			if ((clouds.hidden_layers >> layer) & 1)
				continue;

			float4 cloud = CloudSheet(layer, cam, dir, pixel_angle, lower_scale, sun_dir, sun_light, ambient, haze);
			sky = sky * (1.0 - cloud.a) + cloud.rgb;
		}
		if ((clouds.hidden_layers & 1) == 0)
		{
			// The low cloud is nearest. It is composited below, once all pixels of the quad have their samples.
			float quad_slot = (pixel.x & 1) + 2 * (pixel.y & 1);
			low_cloud = CloudSlab(cam, dir, pixel_angle, quad_slot, g_frame.sky_history.frame_jitter, lower_scale, sun_dir, sun_light, ambient, haze, anchor);
			drifts = 1;
		}
	}

	// Blend the low cloud's samples over each 2x2 pixel quad, then over frames, which removes the grain left by the widely spaced march steps.
	// Then put the low cloud in front of the rest of the sky.
	float4 lo, hi;
	QuadRange(low_cloud, lo, hi);
	low_cloud = QuadAverage(low_cloud, pixel);
	low_cloud = AccumulateLowCloud(low_cloud, lo, hi, anchor, drifts, pixel);
	sky = sky * (1.0 - low_cloud.a) + low_cloud.rgb;

	// Compress the radiance into display range; bright areas roll off smoothly instead of clipping.
	return 1.0 - exp(-sky);
}
// Vertex shader: derive world directions independently of camera translation and object transforms.
PSIn VSProceduralSky(VSIn In)
{
	PSIn Out = (PSIn)0;

	// Orthographic rays are parallel; perspective rays also account for off-centre projection.
	float3 camera_direction = float3(0, 0, -1);
	if (g_frame.cam.c2s[3][3] == 0)
	{
		camera_direction.xy = (In.vert.xy + float2(g_frame.cam.c2s[2][0], g_frame.cam.c2s[2][1])) /
			float2(g_frame.cam.c2s[0][0], g_frame.cam.c2s[1][1]);
	}
	Out.ws_norm = mul(float4(camera_direction, 0), g_frame.cam.c2w);

	// Far depth (0 under reversed depth) fills only background pixels; interpolation preserves the unnormalized ray until the pixel shader.
	Out.ss_vert = float4(In.vert.xy, 0, 1);
	Out.diff = float4(0, 0, 0, 1);
	Out.tex0 = In.tex0;
	Out.idx0 = In.idx0;

	return Out;
}

// Pixel shader: procedural atmospheric sky.
// The history writes would otherwise disable the early depth test. The test is still correct early because the sky neither writes depth nor discards pixels.
[earlydepthstencil]
PSOut PSProceduralSky(PSIn In)
{
	PSOut Out = (PSOut) 0;

	// The angle covered by one pixel, taken before any branching so the screen-space derivatives are valid.
	float3 view_dir = normalize(In.ws_norm.xyz);
	float pixel_angle = max(length(fwidth(view_dir)), 1e-5);

	// Evaluate only the sources that contribute, blending linear colour in one opaque background draw.
	float3 sky = 0;
	if (g_sky.blend_weight > 0)
	{
		float3 sky_dir = mul(float4(view_dir, 0), g_sky.clouds.world_to_sky).xyz;
		float3 cam = mul(float4(g_frame.cam.c2w[3].xyz, 0), g_sky.clouds.world_to_sky).xyz;
		sky = ProceduralSkyColour(sky_dir, cam, pixel_angle, uint2(In.ss_vert.xy));
	}
	if (g_sky.blend_weight < 1)
	{
		float3 cube_dir = mul(float4(view_dir, 0), g_sky.world_to_cube).xyz;
		float3 background = g_background.SampleLevel(g_background_sampler, cube_dir, 0).rgb;
		sky = lerp(background, sky, g_sky.blend_weight);
	}

	Out.diff = float4(sky, 1.0);

	// Dither the smooth sky gradient before 8-bit output. See DitherSrgb8.
	Out.diff.rgb = DitherSrgb8(Out.diff.rgb, uint2(In.ss_vert.xy), g_frame.output.x, 0);
	return Out;
}

#ifdef __cplusplus
}
#endif
