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
#include "view3d-12/src/shaders/hlsl/utility/colour_space.hlsli"

#ifdef __cplusplus
namespace pr::rdr12::sky
{
	using namespace pr::hlsl;
#endif

ConstantBuffer<CBufFrame> resource(g_frame, b0);
ConstantBuffer<CBufProceduralSky> resource(g_sky, b3);
TextureCube<float4> resource(g_background, t18);
Texture2D<float> resource(g_weather, t19);
SamplerState resource(g_background_sampler, s1);

// Cloud layers, lowest first. See PR_SKY_CLOUD_LAYER0.
static const float4 CloudLayers[3] = { float4(PR_SKY_CLOUD_LAYER0), float4(PR_SKY_CLOUD_LAYER1), float4(PR_SKY_CLOUD_LAYER2) };

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

	// Fade to night as the sun sets below the horizon, leaving a faint blue night sky.
	float day = saturate(sun_dir.z * 12.0 + 1.0);
	return radiance * day * day + float3(0.002, 0.003, 0.006);
}

// Linear sky radiance including the sun disc, darkened below the horizon.
float3 AtmosphericSky(float3 view_dir, float3 sun_dir, float3 sun_light)
{
	float3 sky = SkyRadiance(view_dir, sun_dir, sun_light);

	// The sun disc, reddened by the same path through the atmosphere as sunlight on the ground.
	float cos_sun = dot(view_dir, sun_dir);
	sky += 40.0 * sun_light * SunTransmittance(sun_dir.z, 1.0) * smoothstep(0.99985, 0.99993, cos_sun) * step(-0.02, sun_dir.z);

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

// The slope of the ramp at a lattice corner toward the offset 'd'. The hash picks one of 12 directions toward the edges of a cube.
float LatticeRamp(int3 cell, uint seed, float3 d)
{
	uint h = LatticeHash(cell, seed) & 15u;
	float u = h < 8u ? d.x : d.y;
	float v = h < 4u ? d.y : (h == 12u || h == 14u) ? d.x : d.z;
	return ((h & 1u) ? -u : u) + ((h & 2u) ? -v : v);
}

// Smooth gradient noise in [0,1] with mean 0.5 and about one feature per unit. It repeats every 'period' units in x and y,
// and every 'z_period' units in z. Gradient noise has no flat-topped cells, so it shows less of the square lattice than value noise.
float PeriodicNoise(float3 p, int period, int z_period, uint seed)
{
	// Wrap the lattice so the pattern repeats exactly, which lets the CPU wrap cloud offsets and evolution without a visible jump.
	float3 i = floor(p);
	float3 f = p - i;
	float3 u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
	int3 per = int3(period, period, z_period);
	int3 c0 = ((int3(i) % per) + per) % per;
	int3 c1 = (c0 + 1) % per;

	// Blend the ramps from the eight corners, first along x, then y, then z.
	float x00 = lerp(LatticeRamp(int3(c0.x, c0.y, c0.z), seed, f - float3(0, 0, 0)), LatticeRamp(int3(c1.x, c0.y, c0.z), seed, f - float3(1, 0, 0)), u.x);
	float x10 = lerp(LatticeRamp(int3(c0.x, c1.y, c0.z), seed, f - float3(0, 1, 0)), LatticeRamp(int3(c1.x, c1.y, c0.z), seed, f - float3(1, 1, 0)), u.x);
	float x01 = lerp(LatticeRamp(int3(c0.x, c0.y, c1.z), seed, f - float3(0, 0, 1)), LatticeRamp(int3(c1.x, c0.y, c1.z), seed, f - float3(1, 0, 1)), u.x);
	float x11 = lerp(LatticeRamp(int3(c0.x, c1.y, c1.z), seed, f - float3(0, 1, 1)), LatticeRamp(int3(c1.x, c1.y, c1.z), seed, f - float3(1, 1, 1)), u.x);
	float n = lerp(lerp(x00, x10, u.y), lerp(x01, x11, u.y), u.z);

	// The raw noise lies in about [-1,1] with a spread of 0.27. Scaling to a spread of about 0.185 keeps cover thresholds meaningful.
	return saturate(0.5 + 0.68 * n);
}

// The frequency gain from one noise octave to the next. See CloudFbm.
static const float OctaveGain = 2.2360680; // sqrt(5)

// The average of a smoothly folded noise octave (see CloudFbm), given the spread of PeriodicNoise.
static const float BillowMean = 0.865;

// Sum of noise octaves in [0,1] with mean 0.5 (BillowMean when folded), from 'first' to 'last' octave. 'p' is in feature sizes; xy repeats every PR_SKY_CLOUD_PERIOD
// and z every PR_SKY_CLOUD_EVOLVE_PERIOD. 'last' may be fractional: the final octave fades out, which removes shimmer at a distance.
// With 'billow', each octave is folded smoothly (1 - (2n - 1)^2), which gives rounded, heaped lumps with soft creases between them.
float CloudFbm(float3 p, int first, float last, bool billow, uint seed)
{
	// Each octave rotates by about 27 degrees and scales by OctaveGain, so the lattices of different octaves do not line up.
	// The integer matrix maps whole periods to whole periods, so every octave repeats with the base period. Higher octaves also evolve faster.
	float mean = billow ? BillowMean : 0.5;
	float sum = 0;
	float total = 0;
	float amp = 1.0;
	int z_period = PR_SKY_CLOUD_EVOLVE_PERIOD;
	[loop] for (int o = 0; o != 6; ++o)
	{
		// Octaves before 'first' only advance the frequency.
		float weight = saturate(last + 1 - o);
		if (weight <= 0)
			break;

		if (o >= first)
		{
			float n = PeriodicNoise(p, PR_SKY_CLOUD_PERIOD, z_period, seed + o);
			n = billow ? 1.0 - (2.0 * n - 1.0) * (2.0 * n - 1.0) : n;
			sum += amp * lerp(mean, n, weight);
			total += amp;
		}
		p = float3(2.0 * p.x + p.y, 2.0 * p.y - p.x, 2.0 * p.z) + float3(17.31, 9.13, 5.71);
		z_period *= 2;
		amp *= 0.4;
	}

	// Faded octaves contribute their mean, so fading detail does not change the average coverage.
	return total > 0 ? sum / total : mean;
}
// Cloud cover in [0,1] at a position in the atmosphere frame. Matches WeatherMap::CoverAt.
float WeatherCover(float2 xy)
{
	// Without a map, the cover is uniform.
	if (g_sky.has_weather == 0)
		return g_sky.cloud_cover;

	// Clamp to texel centres so filtering never wraps to the opposite edge.
	uint w, h;
	g_weather.GetDimensions(w, h);
	float2 uv = (xy - g_sky.weather_area.xy) * g_sky.weather_area.zw;
	float2 half_texel = 0.5 / float2(w, h);
	float map_cover = g_weather.SampleLevel(g_background_sampler, clamp(uv, half_texel, 1.0 - half_texel), 0);

	// Fade to the default cover near and beyond the edges so the map has no visible boundary.
	float edge = min(min(uv.x, 1.0 - uv.x), min(uv.y, 1.0 - uv.y));
	return lerp(g_sky.cloud_cover, map_cover, smoothstep(0.0, PR_SKY_WEATHER_EDGE_FADE, edge));
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

// Cloud shape at noise position 'p' (xy in feature sizes, z = evolution phase), with detail up to octave 'last'.
// Returns x = opacity in [0,1]; y = thickness, which is 0 at the edge and keeps growing inside the cloud; z = mass in [0,1], which is how far
// inside a large cloud the point is; and w = lumps in [0,1], which is high on rounded bulges and low in the creases between them.
float4 CloudShape(float3 p, float last, float threshold, float softness, bool heaped, uint seed)
{
	// The low octaves set the overall cloud masses. A wide soft edge gives soft clouds instead of hard-edged blobs.
	float coverage = (CloudFbm(p, 0, min(last, 2.0), false, seed) - threshold) / softness;

	// Lumps can push the edge out by at most this much, so points further outside are clear.
	float lump_reach = heaped ? 0.6 : 0.0;
	if (coverage <= -lump_reach)
		return 0;

	// Heaped cloud (cumulus) is built from rounded lumps. The lumps change the thickness everywhere, not only at the edge,
	// so outlines are lumpy and the base has bulges and creases. Lumps are centred on their average, so they do not change the cover.
	// They start one octave above the cloud masses, so each cloud has a few large bulges rather than fine texture.
	float lumps = heaped ? CloudFbm(p, 1, last, true, seed + 50u) : BillowMean;
	float thickness = coverage + 4.0 * (lumps - BillowMean);

	// Mass grows more slowly than thickness and ignores the lumps, so only large clouds have dark middles.
	return float4(saturate(thickness), max(thickness, 0.0), saturate(coverage * 0.3), lumps);
}

// One cloud layer seen along 'dir' from 'cam'. Returns premultiplied radiance and alpha.
float4 CloudLayer(int layer, float3 cam, float3 dir, float pixel_angle, float3 sun_dir, float3 sun_light, float3 ambient, float3 haze)
{
	// Intersect the layer's spherical shell around the planet centre. The shell curves down to meet the horizon at a finite distance,
	// which limits how squashed distant cloud becomes. The terms are arranged to avoid losing precision with the large planet radius.
	float4 config = CloudLayers[layer];
	float radius = PR_SKY_PLANET_RADIUS + config.x;
	float b = dot(cam.xy, dir.xy) + (cam.z + PR_SKY_PLANET_RADIUS) * dir.z;
	float c = dot(cam.xy, cam.xy) + (cam.z - config.x) * (cam.z + config.x + 2.0 * PR_SKY_PLANET_RADIUS);
	if (c >= 0)
		return 0;

	float disc = sqrt(b * b - c);
	float t = b >= 0 ? -c / (b + disc) : disc - b;
	float3 hit = cam + dir * t;

	// 'mu' is the cosine between the ray and the shell's upward normal. It is small where the ray grazes the layer near the horizon.
	float mu = max((b + t) / radius, 0.02);

	// Cover comes from the weather at the hit point, adjusted per layer so higher layers appear at lower cover
	// and thin out before the low layer closes over.
	float cover = WeatherCover(hit.xy);
	float storm = smoothstep(0.7, 1.0, cover);
	float layer_cover = layer == 0 ? cover : layer == 1 ? saturate(cover * 1.2 - 0.2) : saturate(cover * 1.5) * 0.6;
	if (layer_cover <= 0)
		return 0;

	// Position in noise space. The CPU integrates the wind into the offsets and the shape changes into the evolution phase.
	float2 offset = layer == 0 ? g_sky.cloud_offset01.xy : layer == 1 ? g_sky.cloud_offset01.zw : g_sky.cloud_offset2;
	float3 p = float3(hit.xy / config.yz - offset, g_sky.cloud_evolve[layer]);
	uint seed = 31u * (layer + 1);
	bool heaped = layer != 2;

	// Limit detail to what a pixel can resolve. Footprints stretch where the ray grazes the layer.
	float footprint = t * pixel_angle / mu / min(config.y, config.z);
	float last = clamp(-log2(footprint * 2.0) / log2(OctaveGain) - 1.0, 0.0, 5.0);

	// Cirrus is streaked by warping along its long axis.
	if (layer == 2)
		p.x += 1.5 * (CloudFbm(p, 0, 2.0, false, 101u) - 0.5);

	// Bend the noise space with a broad, slowly changing flow, so cloud outlines and cloud fields do not follow the noise lattice.
	float3 pw = p * 0.5;
	p.xy += 0.5 * (float2(
		PeriodicNoise(pw, PR_SKY_CLOUD_PERIOD / 2, PR_SKY_CLOUD_EVOLVE_PERIOD / 2, seed + 70u),
		PeriodicNoise(pw, PR_SKY_CLOUD_PERIOD / 2, PR_SKY_CLOUD_EVOLVE_PERIOD / 2, seed + 71u)) - 0.5);

	// Group clouds into fields with clear gaps between them. A large, slowly changing field raises or lowers the local threshold.
	float cluster = PeriodicNoise(p * 0.25, PR_SKY_CLOUD_PERIOD / 4, PR_SKY_CLOUD_EVOLVE_PERIOD / 4, seed + 90u);

	// Threshold the noise by cover: none at 0, about half the sky at 0.5, and all of it at 1. Storms fill in the gaps.
	float threshold = lerp(0.72, 0.25, layer_cover) + 0.4 * (0.5 - cluster) * (1.0 - storm);
	float softness = layer == 2 ? 0.2 : 0.1;
	float4 shape = CloudShape(p, last, threshold, softness, heaped, seed);
	if (layer == 0)
	{
		shape.x = lerp(shape.x, 1.0, storm * 0.8);
		shape.z = lerp(shape.z, 1.0, storm);
	}
	if (shape.x <= 0)
		return 0;

	// Sunlight reaching the base passes through the cloud above it. A high sun shines through the full thickness of large clouds;
	// a low sun lights the bases from the side, which gives warm bases at sunset. Cloud toward the sun adds shadow.
	// Higher layers are thinner, so they darken less than the low layer.
	float darkening = layer == 0 ? 1.0 : layer == 1 ? 0.5 : 0.15;
	float2 to_sun = sun_dir.xy * 0.2;
	float shadow = saturate((CloudFbm(p + float3(to_sun, 0), 0, min(last, 2.0), false, seed) - threshold) / softness * 0.3);
	float depth = darkening * (shape.z * lerp(0.8, 3.0, saturate(sun_dir.z * 2.0)) + 1.5 * shadow) + 2.0 * storm;
	float sunlit = exp(-depth);

	// Bulges that face the sun catch more light than those facing away, and creases between bulges are shaded from the sky.
	// Both use the unclamped thickness with only the coarse detail, so the whole base has broad, soft shading rather than fine streaks.
	float lighting_detail = min(last, 2.0);
	float4 coarse = heaped ? CloudShape(p, lighting_detail, threshold, softness, true, seed) : shape;
	float sunward = heaped ? CloudShape(p + float3(sun_dir.xy * 0.15, 0), lighting_detail, threshold, softness, true, seed).y : coarse.y;
	float bump = clamp(1.0 + 1.5 * (coarse.y - sunward), 0.5, 1.5);
	float crease = heaped ? lerp(0.7, 1.15, saturate((coarse.w - 0.6) / 0.35)) : 1.0;

	// Thin cloud near the sun glows from light scattered forward through it (silver lining).
	float cos_sun = dot(dir, sun_dir);
	float phase = 0.6 + 3.0 * pow(saturate(cos_sun), 10.0) * exp(-2.0 * shape.y);

	// Sky light reaches thin cloud from all sides but is blocked inside large clouds. Storm cloud absorbs more.
	float albedo = layer == 0 ? lerp(1.0, 0.3, storm) : lerp(1.0, 0.6, storm);
	float3 radiance = albedo * crease * (sun_light * sunlit * bump * phase * 0.6 + ambient * lerp(0.4, 0.15, shape.z * darkening));

	// Denser cloud, and longer paths where the ray grazes the layer, are more opaque. Squaring the opacity softens the edges.
	float thickness = layer == 0 ? 6.0 : layer == 1 ? 3.0 : 0.8;
	float path = 1.0 + min(0.3 / mu, 3.0);
	float alpha = 1.0 - exp(-shape.x * shape.x * thickness * path);

	// Distant cloud fades into the air in front of it, which is dim under a storm.
	float fog = 1.0 - exp(-t / 70000.0);
	radiance = lerp(radiance, haze * lerp(1.0, 0.35, storm), fog);
	alpha *= saturate(dir.z * 50.0);

	return float4(radiance * alpha, alpha);
}

// The full procedural sky along 'dir' from 'cam' (both in the atmosphere frame). Returns linear colour in [0,1].
float3 ProceduralSkyColour(float3 dir, float3 cam, float pixel_angle)
{
	// Sky radiance, lit by the sun above the atmosphere.
	float3 sun_dir = g_sky.sun_direction.xyz;
	float3 sun_top = g_sky.sun_colour.rgb * g_sky.sun_intensity * SkyExposure;
	float3 sky = AtmosphericSky(dir, sun_dir, sun_top);
	float3 haze = SkyRadiance(dir, sun_dir, sun_top);

	// Stars fade in at dusk and fade out near the horizon where the air is thick.
	float star_visibility = smoothstep(0.05, -0.12, sun_dir.z) * saturate(dir.z * 5.0);
	if (star_visibility > 0)
		sky += Stars(dir, pixel_angle * 300.0, g_sky.time) * star_visibility;

	// Under heavy cloud little sunlight reaches the air below it, so the clear air seen toward the horizon and below it is dim and grey.
	float heavy = smoothstep(0.6, 1.0, WeatherCover(cam.xy));
	float gloom = lerp(1.0, 0.35, heavy);
	sky = lerp(sky, Luminance(sky) * float3(0.85, 0.88, 0.92), 0.7 * heavy) * gloom;
	haze = lerp(haze, Luminance(haze) * float3(0.85, 0.88, 0.92), 0.7 * heavy) * gloom;
	if (dir.z > 0)
	{
		// Sunlight at the clouds is reddened by its path through the atmosphere. Clouds keep a little light just after sunset.
		float3 sun_light = sun_top * SunTransmittance(sun_dir.z, 0.8) * saturate(sun_dir.z * 15.0 + 1.0);

		// Cloud bases are lit by the sky overhead and the horizon in the same direction, which is warm toward a setting sun.
		float3 horizon_dir = normalize(float3(normalize(dir.xy + float2(1e-4, 0)), 0.1));
		float3 ambient = 0.5 * (SkyRadiance(float3(0, 0, 1), sun_dir, sun_top) + SkyRadiance(horizon_dir, sun_dir, sun_top));
		ambient = lerp(ambient, Luminance(ambient), 0.5 + 0.35 * heavy);

		// Composite from the highest layer down, because the lowest layer is nearest to an observer below the clouds.
		[unroll] for (int layer = 2; layer >= 0; --layer)
		{
			float4 cloud = CloudLayer(layer, cam, dir, pixel_angle, sun_dir, sun_light, ambient, haze);
			sky = sky * (1.0 - cloud.a) + cloud.rgb;
		}
	}

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

	// Far depth fills only background pixels; interpolation preserves the unnormalized ray until the pixel shader.
	Out.ss_vert = float4(In.vert.xy, 1, 1);
	Out.diff = float4(0, 0, 0, 1);
	Out.tex0 = In.tex0;
	Out.idx0 = In.idx0;

	return Out;
}

// Pixel shader: procedural atmospheric sky
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
		float3 sky_dir = mul(float4(view_dir, 0), g_sky.world_to_sky).xyz;
		float3 cam = mul(float4(g_frame.cam.c2w[3].xyz, 0), g_sky.world_to_sky).xyz;
		sky = ProceduralSkyColour(sky_dir, cam, pixel_angle);
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
