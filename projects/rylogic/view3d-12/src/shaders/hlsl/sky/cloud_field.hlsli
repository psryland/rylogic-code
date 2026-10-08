//************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The procedural sky's cloud field. The sky shader draws it, and the scene lighting uses it for cloud shadows, so the shadows match the visible clouds.
// HLSL only. The includer's root signature must provide the weather map at t19, the cloud noise at t20 (see ProceduralSky), and a wrapping linear sampler at s8.
#ifndef PR_VIEW3D_CLOUD_FIELD_HLSLI
#define PR_VIEW3D_CLOUD_FIELD_HLSLI
#include "pr/hlsl/interop.hlsli"
#include "view3d-12/src/shaders/hlsl/sky/cloud_cbuf.hlsli"

Texture2D<float> resource(g_weather, t19);
Texture2D<float4> resource(g_cloud_noise, t20);
SamplerState resource(g_noise_sampler, s8);

// Cloud layers, lowest first. See PR_SKY_CLOUD_LAYER0.
static const float4 CloudLayers[3] = { float4(PR_SKY_CLOUD_LAYER0), float4(PR_SKY_CLOUD_LAYER1), float4(PR_SKY_CLOUD_LAYER2) };

// The depth of the low cloud slab below its layer altitude, as a fraction of the slab's height. Thick cloud dips a little below the layer.
static const float CloudSlabSkirt = 0.12;

// Cloud cover in [0,1] at a position in the atmosphere frame. Matches WeatherMap::CoverAt.
float CloudCover(CloudConstants clouds, float2 xy)
{
	// Without a map, the cover is uniform.
	if (clouds.has_weather == 0)
		return clouds.cover;

	// Clamp to texel centres so filtering never wraps to the opposite edge. That also makes the wrapping noise sampler safe to use here.
	uint w, h;
	g_weather.GetDimensions(w, h);
	float2 uv = (xy - clouds.weather_area.xy) * clouds.weather_area.zw;
	float2 half_texel = 0.5 / float2(w, h);
	float map_cover = g_weather.SampleLevel(g_noise_sampler, clamp(uv, half_texel, 1.0 - half_texel), 0);

	// Fade to the default cover near and beyond the edges so the map has no visible boundary.
	float edge = min(min(uv.x, 1.0 - uv.x), min(uv.y, 1.0 - uv.y));
	return lerp(clouds.cover, map_cover, smoothstep(0.0, PR_SKY_WEATHER_EDGE_FADE, edge));
}

// How much the low and mid layers are lowered for a camera at 'cam_xy' in the atmosphere frame. See PR_SKY_CLOUD_LOWER_START.
float CloudLowerScale(CloudConstants clouds, float2 cam_xy)
{
	// Storm cloud hangs lower over the camera.
	return lerp(1.0, PR_SKY_CLOUD_LOWER_SCALE, smoothstep(PR_SKY_CLOUD_LOWER_START, 1.0, CloudCover(clouds, cam_xy)));
}

// The altitude of 'layer' in world units. For the low cloud slab, this is the plane its bases sit on.
float CloudLayerAltitude(int layer, float lower_scale)
{
	// Cirrus keeps its altitude in a storm.
	return layer == 2 ? CloudLayers[2].x : CloudLayers[layer].x * lower_scale;
}

// Map a horizontal vector in the atmosphere frame into the wind frame, where x points along the wind.
float2 CloudWindFrame(CloudConstants clouds, float2 v)
{
	// Rotate by the wind direction.
	float2 wind;
	sincos(clouds.wind_direction, wind.y, wind.x);
	return float2(dot(v, wind), dot(v, float2(-wind.y, wind.x)));
}

// Map a change in the first noise coordinate of a layer to the matching change in the second. See CloudUV.
float2 CloudUV2Delta(float2 duv1)
{
	// The second sample is rotated by 45 degrees and scaled by sqrt(2).
	return float2(duv1.x + duv1.y, duv1.y - duv1.x);
}

// Noise coordinates of the two samples of 'layer' at 'xy' in the atmosphere frame.
// The coordinates are in noise tiles of a wind-aligned frame (x along the wind), so stretched clouds streak along the wind in every layer.
// The second sample is rotated by 45 degrees, scaled by sqrt(2), and drifts with the evolution phase, so the sum of the two samples changes
// shape over time and hides the tiling of each. Its integer matrix maps whole tiles to whole tiles, so it repeats with the CPU's offset wrapping.
void CloudUV(CloudConstants clouds, int layer, float2 xy, out float2 uv1, out float2 uv2)
{
	// Move with the layer's wind offset, then derive the evolving second sample.
	float2 offset = layer == 0 ? clouds.offset01.xy : layer == 1 ? clouds.offset01.zw : clouds.offset2;
	uv1 = CloudWindFrame(clouds, xy) / CloudLayers[layer].yz - offset;
	uv2 = CloudUV2Delta(uv1) + clouds.evolve[layer] * float2(1.0, 0.5) + float2(0.37, 0.61);
}

// The cloud field from the two noise samples of a layer. Masses come from the broad noise (R, G); lumps add rounded bulges from the heaped noise (B, A),
// scaled by 'lump_scale'. Both samples are averaged, so they are rescaled to keep the spread of a single sample.
float CloudFieldFromNoise(float4 n1, float4 n2, float lump_scale)
{
	// Combine the broad masses with the heaped lumps.
	return 0.5 + 0.72 * (n1.r + n2.g - 1.0) + lump_scale * (n1.b + n2.a - 1.0);
}

// The cloud field at the noise coordinates 'uv1' and 'uv2' of a layer, at mip level 'lod'.
float CloudField(float2 uv1, float2 uv2, float lod, float lump_scale)
{
	// Read both noise samples. The second sample's tiles are larger, so it is read half a level coarser.
	float4 n1 = g_cloud_noise.SampleLevel(g_noise_sampler, uv1, lod);
	float4 n2 = g_cloud_noise.SampleLevel(g_noise_sampler, uv2, lod + 0.5);
	return CloudFieldFromNoise(n1, n2, lump_scale);
}

// Fine lumpy noise in [-0.5, 0.5] that frays the edges of low and mid-level cloud. Cirrus is not frayed.
float CloudFray(int layer, float2 uv1, float lod)
{
	// A finer, offset copy of the heaped noise.
	return layer == 2 ? 0.0 : g_cloud_noise.SampleLevel(g_noise_sampler, uv1 * 3.0 + float2(0.21, 0.43), lod + 1.585).b - 0.5;
}

// The parameters that turn a layer's cloud field into cloud. They depend on the local cover, but not on the exact point in the layer.
struct CloudShape
{
	float layer_cover; // This layer's share of the cover. The layer is clear where this is 0.
	float storm;       // 0 = no storm, 1 = full storm deck.
	float build;       // How far the clouds have built up, from small sparse cloud (0) to large cloud (1).
	float fluff;       // 1 for fluffy fair-weather cloud, 0 for well-defined storm cloud.
	float threshold;   // The field value at the cloud edge.
	float softness;    // The width, in field units, of the soft band where opacity rises above the edge.
	float depth_range; // The field range from the edge to the thickest cloud.
	float lump_scale;  // The weight of the heaped lumps in the field.
	float height;      // Low cloud only: the slab's height above its base plane, in world units.
	float sigma;       // Low cloud only: the extinction per world unit of the densest cloud.
};

// The shape of 'layer' where the cover is 'cover'. 'uv1' is the layer's first noise coordinate there, and 'lod' its mip level.
CloudShape CloudLayerShape(int layer, float cover, float2 uv1, float lod, float lower_scale)
{
	CloudShape s;

	// Cover is adjusted per layer, so higher layers appear at lower cover and thin out before the low layer closes over.
	// Zero cover is always clear sky. Cirrus grows with cover up to 0.25, then stays light.
	s.layer_cover = layer == 0 ? cover : layer == 1 ? saturate(cover * 1.2 - 0.2) : min(cover * 1.5, 0.375);
	s.storm = smoothstep(0.7, 1.0, cover);
	s.build = smoothstep(0.2, 0.6, cover);
	s.fluff = 1.0 - s.storm;

	// Group clouds into fields with clear gaps between them. A broad, slowly changing field raises or lowers the local threshold.
	// Threshold the noise by cover: none at 0, about half the sky at 0.5, and all of it at 1. Storms fill in the gaps.
	// At mid cover the threshold is near the middle of the noise, so every small wiggle would cross it and scatter many small clouds.
	// Grouping is strongest there, so low and mid-level clouds gather into large masses with wide gaps. Cirrus keeps light grouping.
	float cluster = g_cloud_noise.SampleLevel(g_noise_sampler, uv1 * 0.18, max(lod - 2.5, 4.0)).g;
	float mid_cover = layer == 2 ? 0.0 : smoothstep(0.1, 0.35, cover) * (1.0 - smoothstep(0.55, 0.8, cover));
	s.threshold = s.layer_cover > 0 ? lerp(0.85, 0.15, s.layer_cover) + lerp(0.3, 1.3, mid_cover) * (0.5 - cluster) * s.fluff : 2.0;

	// Opacity rises over a soft band above the threshold. Cirrus has a wide band, so it is wispy. Low and mid-level cloud are fluffy: their band is
	// widened, and fine lumpy noise frays their edges. Storm cloud has a narrow band and no fraying, so it has well defined edges.
	// The low cloud's band is narrower, because its soft rounded top already softens its outline, and a wide band would leave small cloud see-through.
	s.softness = layer == 2 ? 0.2 : layer == 1 ? lerp(0.18, 0.05, s.storm) * lerp(1.0, 2.0, s.fluff) : lerp(lerp(0.1, 0.07, s.build), 0.05, s.storm) * lerp(1.0, 1.8, s.fluff);
	s.depth_range = lerp(0.35, 0.2, s.storm);
	s.lump_scale = layer == 2 ? 0.1 : 0.3;

	// Low cloud is a slab. Sparse fair-weather cloud is shallow. Clouds grow taller as they build up, mostly at higher cover, and storm cloud is
	// the tallest. Lowered storm cloud is thinner in proportion, so its tops stay well below the mid layer. All low cloud is dense enough to be
	// opaque through its full height, and larger cloud absorbs a little more light over the same height.
	s.height = lerp(lerp(400.0, 1500.0, s.build * s.build), 3000.0, s.storm) * lower_scale;
	s.sigma = lerp(9.0, 12.0, s.storm) * lerp(0.8, 1.0, s.build) / s.height;
	return s;
}

// How far a point with cloud field 'field' and fray 'fray' lies inside the cloud edge, in field units. Negative outside the cloud.
float CloudEdge(CloudShape s, float field, float fray)
{
	// Fraying moves the edge in and out of fluffy cloud.
	return field - s.threshold + 0.35 * fray * s.fluff;
}

// The low cloud's coverage and relative thickness (0 at the edge, 1 for the thickest cloud) at a point with edge distance 'edge' and field 'field'.
// A storm closes the gaps with a continuous deck. The deck's thickness follows the field, so it keeps broad thick and thin regions without outlines.
float2 CloudSlabColumn(CloudShape s, float edge, float field)
{
	// Blend from separate clouds to the deck as the storm builds.
	float coverage = saturate(edge / s.softness);
	float q = saturate(edge / s.depth_range);
	float deck = s.storm * s.storm;
	float deck_q = 0.45 + 0.55 * saturate((field - s.threshold + 0.3) / 0.5);
	return float2(lerp(coverage, 1.0, deck), lerp(q, max(q, deck_q), deck));
}

// The base and top heights, relative to the layer plane, of a low cloud column with relative thickness 'q'.
// The top is rounded, rising steeply near the edge and levelling off over the thickest cloud. The base is flat-ish and dips a little under thick cloud.
float2 CloudSlabBounds(CloudShape s, float q)
{
	// Both bounds scale with the slab height.
	return float2(-CloudSlabSkirt * q, q * (2.0 - q)) * s.height;
}

// The density in [0,1] of the low cloud at height 'hz' above the layer plane, in the column described by 'column' (see CloudSlabColumn).
float CloudSlabDensity(CloudShape s, float2 column, float hz)
{
	// The base is a fairly sharp boundary. The top fades over a wide band in fluffy cloud, so its outline is soft, and a narrow band in storm cloud.
	// The bands are fractions of the column's own thickness, so short columns still reach full density in their middle.
	float2 bounds = CloudSlabBounds(s, column.y);
	float thickness = max(bounds.y - bounds.x, 1.0);
	float base_band = 0.1 * thickness;
	float top_band = lerp(0.1, 0.4, s.fluff) * thickness;
	return column.x * saturate((hz - bounds.x) / base_band) * saturate((bounds.y - hz) / top_band);
}

// The density in [0,1] of the low cloud with shape 's' at 'xy' in the atmosphere frame and height 'hz' above the layer plane, read at mip level 'lod'.
// 'frayed' adds the fine fraying of the cloud edges, which is only worth reading for the visible cloud.
float CloudSlabDensityAt(CloudConstants clouds, CloudShape s, float2 xy, float hz, float lod, bool frayed)
{
	// Read the field for the column through 'xy', then the density at its height.
	float2 uv1, uv2;
	CloudUV(clouds, 0, xy, uv1, uv2);
	float field = CloudField(uv1, uv2, lod, s.lump_scale);

	// The fraying is read at a point that circles as the height rises, so the frayed edges change with height. Without this, the fraying would be
	// the same at every height and the sides of the cloud would look like vertical streaks. One circle spans the slab height, and its radius is
	// about the size of the fraying's lumps.
	float turn = 6.2832 * hz / s.height;
	float fray = frayed ? CloudFray(0, uv1 + 0.012 * float2(cos(turn), sin(turn)), lod) : 0.0;
	return CloudSlabDensity(s, CloudSlabColumn(s, CloudEdge(s, field, fray), field), hz);
}

// The fraction of direct light from the sky's sun that passes through the clouds to 'ws_pos'. 'ws_to_light' is the normalised direction toward
// the light, and 'ws_cam' the camera position, both in the scene frame. Light that does not come from the sky's sun is not shaded.
// This is a soft, low-detail estimate: it reads the cloud at a coarse mip level and integrates the low cloud column vertically.
float CloudShadow(CloudConstants clouds, float3 ws_pos, float3 ws_to_light, float3 ws_cam)
{
	// Without cloud shadows, all the light passes.
	if (clouds.shadow_strength <= 0)
		return 1.0;

	// Only a light from the sky's sun is shaded by its clouds. Light from below the horizon does not pass through them.
	float3 to_sun = mul(float4(ws_to_light, 0), clouds.world_to_sky).xyz;
	float aligned = smoothstep(0.95, 0.99, dot(to_sun, clouds.sun_direction.xyz));
	if (aligned <= 0 || to_sun.z <= 0.01)
		return 1.0;

	// Follow the light back from the point to each layer. Shadows are near the camera compared with the planet radius, so the layers are flat here.
	// The path through a layer lengthens as the sun gets lower, but not without limit, because thin cloud edges let low light through.
	float3 pos = mul(float4(ws_pos, 0), clouds.world_to_sky).xyz;
	float lower_scale = CloudLowerScale(clouds, mul(float4(ws_cam, 0), clouds.world_to_sky).xy);
	float2 travel = to_sun.xy / to_sun.z;
	float slant = 1.0 / max(to_sun.z, 0.15);
	float lod = 3.0;
	float optical = 0.0;
	float storm = 0.0;
	if ((clouds.hidden_layers & 1) == 0)
	{
		// The low cloud's shape is set by the cover where the light enters its base.
		float altitude = CloudLayerAltitude(0, lower_scale);
		float2 xy = pos.xy + travel * (altitude - pos.z);
		float2 uv1, uv2;
		CloudUV(clouds, 0, xy, uv1, uv2);
		float cover = CloudCover(clouds, xy);
		CloudShape s = CloudLayerShape(0, cover, uv1, lod, lower_scale);
		storm = s.storm;

		// Average the column thickness at two heights along the light, which softens the shadow edges where the light crosses the slab at an angle.
		[unroll] for (int i = 0; i != 2; ++i)
		{
			// The second sample is half way up the slab.
			float2 duv1 = CloudWindFrame(clouds, travel * (0.5 * i * s.height)) / CloudLayers[0].yz;
			float field = CloudField(uv1 + duv1, uv2 + CloudUV2Delta(duv1), lod, s.lump_scale);
			float2 column = CloudSlabColumn(s, CloudEdge(s, field, 0.0), field);
			float2 bounds = CloudSlabBounds(s, column.y);
			optical += 0.5 * s.sigma * column.x * (bounds.y - bounds.x);
		}
	}
	float transmit = exp(-optical * slant);
	if ((clouds.hidden_layers & 2) == 0)
	{
		// Mid-level cloud is thin, so it casts weaker shadows than the low cloud.
		float altitude = CloudLayerAltitude(1, lower_scale);
		float2 xy = pos.xy + travel * (altitude - pos.z);
		float2 uv1, uv2;
		CloudUV(clouds, 1, xy, uv1, uv2);
		CloudShape s = CloudLayerShape(1, CloudCover(clouds, xy), uv1, lod, lower_scale);
		float coverage = saturate(CloudEdge(s, CloudField(uv1, uv2, lod, s.lump_scale), 0.0) / s.softness);
		float alpha = 1.0 - exp(-coverage * coverage * 3.0 * lerp(0.5, 1.0, s.build) * slant);
		transmit *= 1.0 - 0.4 * alpha;
	}

	// A storm deck spreads the sunlight over the whole sky, so it casts no distinct shadows. The scene's storm lighting dims the light instead.
	return lerp(1.0, transmit, clouds.shadow_strength * aligned * (1.0 - storm));
}

#endif
