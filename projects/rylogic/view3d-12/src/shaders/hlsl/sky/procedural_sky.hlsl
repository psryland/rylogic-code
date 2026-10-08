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
Texture2D<float4> resource(g_cloud_noise, t20);
SamplerState resource(g_background_sampler, s1);
SamplerState resource(g_noise_sampler, s8);

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

// The cloud field from the two noise samples of a layer (see CloudLayer). Masses come from the broad noise; lumps add rounded bulges,
// scaled by 'lump_scale'. Both samples are averaged, so they are rescaled to keep the spread of a single sample.
float CloudFieldFromNoise(float4 n1, float4 n2, float lump_scale)
{
	// Combine the broad masses with the heaped lumps.
	return 0.5 + 0.72 * (n1.r + n2.g - 1.0) + lump_scale * (n1.b + n2.a - 1.0);
}

// The cloud field at the noise coordinates 'uv1' and 'uv2' of a layer (see CloudLayer).
float CloudField(float2 uv1, float2 uv2, float lod, float lump_scale)
{
	// Read both noise samples at the layer's mip level.
	float4 n1 = g_cloud_noise.SampleLevel(g_noise_sampler, uv1, lod);
	float4 n2 = g_cloud_noise.SampleLevel(g_noise_sampler, uv2, lod + 0.5);
	return CloudFieldFromNoise(n1, n2, lump_scale);
}

// One cloud layer seen along 'dir' from 'cam'. Returns premultiplied radiance and alpha.
// The cloud field is read from a tileable noise texture (see ProceduralSky), so each layer costs a few texture reads:
// R and G hold independent broad noise that sets the cloud masses, and B and A hold independent heaped lumps (rounded bulges with creases).
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
	// and thin out before the low layer closes over. Zero cover is always clear sky. Cirrus grows with cover up to 0.25, then stays light.
	float cover = WeatherCover(hit.xy);
	float storm = smoothstep(0.7, 1.0, cover);
	float layer_cover = layer == 0 ? cover : layer == 1 ? saturate(cover * 1.2 - 0.2) : min(cover * 1.5, 0.375);
	if (layer_cover <= 0)
		return 0;

	// Texture coordinates in noise tiles of a wind-aligned frame (x along the wind), so stretched clouds streak along the wind in every layer.
	// The second sample is rotated by 45 degrees, scaled by sqrt(2), and drifts with the evolution phase,
	// so the sum of the two samples changes shape over time and hides the tiling of each. Its integer matrix maps whole tiles to whole tiles,
	// so it repeats with the CPU's offset wrapping.
	float2 wind_dir;
	sincos(g_sky.wind_direction, wind_dir.y, wind_dir.x);
	float2 wind_perp = float2(-wind_dir.y, wind_dir.x);
	float2 offset = layer == 0 ? g_sky.cloud_offset01.xy : layer == 1 ? g_sky.cloud_offset01.zw : g_sky.cloud_offset2;
	float2 uv1 = float2(dot(hit.xy, wind_dir), dot(hit.xy, wind_perp)) / config.yz - offset;
	float2 uv2 = float2(uv1.x + uv1.y, uv1.y - uv1.x) + g_sky.cloud_evolve[layer] * float2(1.0, 0.5) + float2(0.37, 0.61);

	// Choose the mip level from the pixel footprint on the layer, which stretches where the ray grazes the layer.
	float footprint = t * pixel_angle / sqrt(mu);
	float lod = log2(max(footprint * PR_SKY_CLOUD_NOISE_SIZE / min(config.y, config.z), 1e-3));

	// Group clouds into fields with clear gaps between them. A broad, slowly changing field raises or lowers the local threshold.
	// Threshold the noise by cover: none at 0, about half the sky at 0.5, and all of it at 1. Storms fill in the gaps.
	// At mid cover the threshold is near the middle of the noise, so every small wiggle would cross it and scatter many small clouds.
	// Grouping is strongest there, so low and mid-level clouds gather into large masses with wide gaps. Cirrus keeps light grouping.
	float cluster = g_cloud_noise.SampleLevel(g_noise_sampler, uv1 * 0.18, max(lod - 2.5, 4.0)).g;
	float mid_cover = layer == 2 ? 0.0 : smoothstep(0.1, 0.35, cover) * (1.0 - smoothstep(0.55, 0.8, cover));
	float threshold = lerp(0.85, 0.15, layer_cover) + lerp(0.3, 1.3, mid_cover) * (0.5 - cluster) * (1.0 - storm);

	// Masses come from the broad noise; lumps add rounded bulges to cumulus, and only slight texture to cirrus.
	// Both samples are averaged, so they are rescaled to keep the spread of a single sample.
	float4 n1 = g_cloud_noise.SampleLevel(g_noise_sampler, uv1, lod);
	float4 n2 = g_cloud_noise.SampleLevel(g_noise_sampler, uv2, lod + 0.5);
	float lump_scale = layer == 2 ? 0.1 : 0.3;
	float field = CloudFieldFromNoise(n1, n2, lump_scale);

	// Opacity rises over a soft band above the threshold. Cirrus has a wide band, so it is wispy. Low and mid-level cloud are fluffy: their band is
	// widened, and fine lumpy noise frays their edges. Storm cloud has a narrow band and no fraying, so it has well defined edges.
	float build = smoothstep(0.2, 0.6, cover);
	float fluff = 1.0 - storm;
	float fray = layer == 2 ? 0.0 : g_cloud_noise.SampleLevel(g_noise_sampler, uv1 * 3.0 + float2(0.21, 0.43), lod + 1.585).b - 0.5;
	float softness = layer == 2 ? 0.2 : layer == 1 ? lerp(0.18, 0.05, storm) * lerp(1.0, 2.0, fluff) : lerp(lerp(0.1, 0.07, build), 0.05, storm) * lerp(1.0, 3.5, fluff);
	float coverage = saturate((field - threshold + 0.35 * fray * fluff) / softness);

	// Light reaching the visible base has crossed the cloud above it, so thick cloud is darker than its thin edges.
	// 'depth' measures how far the field rises above the cloud edge, including the lumps, so bulges and creases are shaded too.
	// 'shade' is the amount of greying, from 0 (white) to 1 (the darkest grey for this cover).
	float depth_range = lerp(0.35, 0.2, storm);
	float depth = saturate((field - threshold) / depth_range);

	// A sun just above the horizon shines under the clouds. Their bases, which are usually the shaded side, are then lit in sunset colours.
	// Storm decks block the low sun, so they stay shaded.
	float under_lit = smoothstep(0.15, 0.03, sun_dir.z) * smoothstep(-0.05, 0.0, sun_dir.z) * (1.0 - storm);
	float shade = lerp(depth, 0.2 * depth, under_lit);
	if (layer == 0)
	{
		// Low cloud is a height field: each cloud has a flat base on the layer and rises to a height that grows with 'depth'.
		// Where the ray meets the base under a cloud, the grey base is visible. Where it passes through a gap, it rises past the base and can meet
		// the side of a cloud farther away. A side is lit from above, so it is white near the top and greys toward the base.
		// Seen from below, the far edge of each base is a sharp bottom edge with the white side of a farther cloud below it on the screen,
		// so overlapping clouds form layered tiers. Ray samples at fixed heights find the first side the ray meets.
		float height = lerp(lerp(600.0, 1500.0, build), 3000.0, storm);
		float tan_elev = mu * rsqrt(max(1.0 - mu * mu, 1e-4));
		float2 away = float2(dot(dir.xy, wind_dir), dot(dir.xy, wind_perp));
		away *= rsqrt(max(dot(away, away), 1e-8));

		// Find where the ray meets a cloud side. 'gap' is the ray height above the cloud top, so the ray is inside a cloud where it is negative.
		// Each sample is a soft hit: it is partly opaque over a fluffy, frayed band around the cloud top, and it hides the samples after it.
		// The hit height is the opacity-weighted crossing height, so it changes smoothly as hits move between samples. A hard first-hit test
		// would make the height jump between samples, which draws crisp contours across the sides.
		float band = lerp(0.05, 0.45, fluff) * height;
		float bias = 0.5 * height * fray * fluff;
		float z_prev = 0.0;
		float gap_prev = max(-height * (field - threshold) / depth_range, 0.0);
		float clear = 1.0;
		float z_sum = 0.0;
		[unroll] for (int i = 0; i != 3; ++i)
		{
			// Each sample is at a fixed fraction of the cloud height, at the distance where the ray reaches that height.
			float z = height * (0.2 + 0.35 * i);
			float2 duv1 = away * (z / tan_elev) / config.yz;
			float2 duv2 = float2(duv1.x + duv1.y, duv1.y - duv1.x);
			float field_i = CloudField(uv1 + duv1, uv2 + duv2, lod, lump_scale);
			float gap = z - height * saturate((field_i - threshold) / depth_range);

			// Interpolate the crossing height between samples, then add this sample's share of the hit.
			float z_cross = lerp(z_prev, z, saturate(gap_prev / max(gap_prev - min(gap, 0.0), 1e-3)));
			float hit = saturate((bias - gap) / band) * clear;
			z_sum += z_cross * hit;
			clear -= hit;
			z_prev = z;
			gap_prev = max(gap, 0.0);
		}

		// Sides are white at the top and grey at the base. The base is grey, darker under thick cloud. The base hides any side behind it.
		// When the sun is near the horizon it shines under the cloud, so the gradient inverts: the base is lit and the side tops are in shade.
		float side = (1.0 - clear) * (1.0 - coverage);
		float side_height = saturate(z_sum / max(1.0 - clear, 1e-4) / (0.8 * height));
		float side_shade = lerp(0.85 * (1.0 - side_height), 0.2 + 0.7 * side_height, under_lit);
		float base_shade = lerp(0.6 + 0.4 * depth, 0.1 * depth, under_lit);
		float total = coverage + side;
		shade = (coverage * base_shade + side * side_shade) / max(total, 1e-4);

		// Storms close the gaps with a dark, even deck, so lighter upper layers cannot show through. The sides still mark the tiers.
		float fill = (1.0 - total) * storm;
		shade = (total * shade + fill * 0.85) / max(total + fill, 1e-4);
		coverage = total + fill;

		// Full cover is one continuous deck with no gaps or sides. Its shade follows the field across the former gaps, so the deck keeps
		// broad light and dark regions without outlines.
		float deck_shade = 0.55 + 0.45 * saturate((field - threshold + 0.3) / 0.5);
		shade = lerp(shade, deck_shade, storm * storm);
	}

	if (coverage <= 0)
		return 0;

	// The thickest cloud has light grey bases in sparse cloud, mid grey bases at half cover, and dark grey bases in a storm.
	// Higher layers are thinner, so they darken less than the low layer. When the sun shines from below, the unlit tops are much darker, which gives
	// strong contrast with the glowing bases.
	float darkening = layer == 0 ? 1.0 : layer == 1 ? 0.6 : 0.25;
	float dark_max = lerp(lerp(lerp(0.25, 0.55, build), 0.9, storm), 0.85, under_lit) * darkening;
	float grey = 1.0 - dark_max * smoothstep(0.0, 1.0, shade);

	// Sides facing the sun are brighter than sides facing away. Compare the cloud here with the cloud a short way toward the sun.
	float2 to_sun = float2(dot(sun_dir.xy, wind_dir), dot(sun_dir.xy, wind_perp)) * 0.02;
	float4 n1s = g_cloud_noise.SampleLevel(g_noise_sampler, uv1 + to_sun, lod);
	float field_sunward = CloudFieldFromNoise(n1s, n2, lump_scale);
	float facing = clamp(1.0 - 2.0 * (field_sunward - field), 0.75, 1.25);

	// Thin cloud near the sun glows from light scattered forward through it (silver lining). Storm cloud lets little direct sunlight through.
	float cos_sun = dot(dir, sun_dir);
	float phase = 1.0 + 2.0 * pow(saturate(cos_sun), 8.0) * (1.0 - coverage);
	float sunlit = lerp(1.0, 0.3, storm * darkening) + 1.2 * under_lit * (1.0 - shade);
	float3 radiance = grey * (sun_light * (0.65 * sunlit * phase * facing) + ambient * 0.9) * lerp(1.0, 0.75, storm * darkening);

	// Toward a low sun, the viewer sees the unlit side of the cloud, so thick cloud there is a dark silhouette. Its thin edges still glow.
	float backlit = pow(saturate(cos_sun), 3.0) * smoothstep(0.25, 0.02, sun_dir.z);
	radiance *= lerp(1.0, 0.3, backlit * coverage);

	// Denser cloud, and longer paths where the ray grazes the layer, are more opaque. Squaring the opacity softens the edges.
	// Sparse cloud is thinner, so it is partly translucent.
	float thickness = (layer == 0 ? lerp(3.0, 6.0, storm) : layer == 1 ? 3.0 : 0.8) * lerp(0.5, 1.0, build);
	float path = 1.0 + min(0.3 / mu, 3.0);
	float alpha = 1.0 - exp(-coverage * coverage * thickness * path);

	// Distant cloud fades into the air in front of it, which is dim under a storm.
	float fog = 1.0 - exp(-t / 70000.0);
	radiance = lerp(radiance, haze * lerp(1.0, 0.35, storm), fog);

	// Storm cloud is lit mostly by blue skylight scattered through the cloud deck, so it is a cool slate grey rather than a warm grey.
	float luminance = dot(radiance, float3(0.2126, 0.7152, 0.0722));
	radiance = lerp(radiance, luminance * float3(0.86, 0.93, 1.08), storm * darkening * 0.85);
	alpha *= saturate(dir.z * 50.0);

	return float4(radiance * alpha, alpha);
}

// The full procedural sky along 'dir' from 'cam' (both in the atmosphere frame). Returns linear colour in [0,1].
float3 ProceduralSkyColour(float3 dir, float3 cam, float pixel_angle)
{
	// Sky radiance, lit by the sun above the atmosphere. 'haze' is the air without the sun disc, which distant clouds fade into.
	float3 sun_dir = g_sky.sun_direction.xyz;
	float3 sun_top = g_sky.sun_colour.rgb * g_sky.sun_intensity * SkyExposure;
	float3 haze = SkyRadiance(dir, sun_dir, sun_top);
	float3 sky = AtmosphericSky(haze, dir, sun_dir, sun_top, g_sky.time);

	// Stars appear in the darkening sky before sunset and fade out near the horizon where the air is thick.
	float star_visibility = smoothstep(0.08, -0.08, sun_dir.z) * saturate(dir.z * 5.0);
	if (star_visibility > 0)
		sky += Stars(dir, pixel_angle * 300.0, g_sky.time) * star_visibility;

	// Under heavy cloud little sunlight reaches the air below it, so the clear air seen toward the horizon and below it is dim and grey.
	float heavy = smoothstep(0.6, 1.0, WeatherCover(cam.xy));
	float gloom = lerp(1.0, 0.17, heavy);
	sky = lerp(sky, Luminance(sky) * float3(0.85, 0.88, 0.92), 0.7 * heavy) * gloom;
	haze = lerp(haze, Luminance(haze) * float3(0.85, 0.88, 0.92), 0.7 * heavy) * gloom;
	if (dir.z > 0)
	{
		// Sunlight at the clouds is reddened by its path through the atmosphere. Clouds keep a little light just after sunset.
		float3 sun_light = sun_top * SunTransmittance(sun_dir.z, 0.8) * saturate(sun_dir.z * 15.0 + 1.0);

		// Cloud bases are lit by the sky overhead and the air in the same direction, which is warm toward a setting sun.
		// Sky light is mostly grey in effect, because it arrives from all over the sky.
		float3 ambient = 0.5 * (SkyRadiance(float3(0, 0, 1), sun_dir, sun_top) * gloom + haze);
		ambient = lerp(ambient, Luminance(ambient), 0.5 + 0.35 * heavy);

		// Composite from the highest layer down, because the lowest layer is nearest to an observer below the clouds.
		[unroll] for (int layer = 2; layer >= 0; --layer)
		{
			// Hidden layers are skipped entirely.
			if ((g_sky.hidden_cloud_layers >> layer) & 1)
				continue;

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

	// Far depth (0 under reversed depth) fills only background pixels; interpolation preserves the unnormalized ray until the pixel shader.
	Out.ss_vert = float4(In.vert.xy, 0, 1);
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
