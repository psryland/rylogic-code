//************************************
// Ocean Surface Forward Pixel
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Forward shading for the world-view water surface: the stock detail-normal shading, with a dark lip where the water laps the terrain, and
// textured foam on the shore and on breaking wave crests.
//
// The water's vertical depth is read from a caller-supplied shore height source (see ShoreDepth). A flat surface meets the terrain in a
// knife edge, so the thinnest water at the waterline is darkened, as if the front were a rounded lip in its own shadow.
//
// The includer must define the shore height source before including this file:
//   float OceanShoreHeight(float2 world_xy, uint4 shore_source[3])
// It returns the ground height (m) below 'world_xy', or NaN where none is known. 'shore_source' is OceanConstants.shore_source, which the
// caller sets with the ocean's shore source. The source may read its own resources, such as a buffer in VIEW3D_PROCEDURAL_BUFFER1_REGISTER.
// This file expands the forward pixel entry points 'PSWaterOpaque...' and 'PSWater...'.
//
// All foam is one foam texture, revealed by a foam amount in [0, 1]. The texture's alpha channel is a foam pattern in which the values are spread
// evenly, so the pattern above the threshold '1 - amount' covers that fraction of the surface. The highest values lie on a network of filaments
// and furthest from the centres of round bubbles, so as the amount falls, dense foam opens into round bubbles, the bubbles merge into open water,
// and thin filaments remain before they vanish, the way real foam ages. Water under and between the foam is milky with fine bubbles.
// The amount is the largest of:
//  - Shore foam, from the water's depth. Shallow water at the waterline is fully covered, and the amount falls to zero across a band whose
//    width grows with the swash, so rough seas foam further out. Shore foam and the lip fade out with camera distance.
//  - Crest foam, where the crest steepness rises above the crest threshold (see OceanCrestFoam). Where crest foam is on, the crest
//    steepness is rebuilt at pixel resolution so far crests stay narrow (see PixelCrest).
//  - Foam history: crest foam left behind by passing crests, which fades where it formed, leaving trails (see ocean_foam_cs.hlsl).
// The texture is drawn in two layers of different sizes, warped by broad noise, and broad noise varies the coverage and the shore foam depth,
// so the foam does not visibly repeat. Where the pixels are too large to show the pattern, the foam is a plain matte cover of the amount.
#include "pr/view3d-12/shaders/forward_pixel_detail.hlsli"
#include "pr/game-systems/ocean/ocean_surface.hlsli"

ConstantBuffer<OceanConstants> g_ocean : register(VIEW3D_PROCEDURAL_FORWARD_CONSTANTS_REGISTER);

// Wind-sea wave elements, bathymetry heights, the wind-sea table and winds, and the foam history. See OceanConstants for the layout.
ByteAddressBuffer g_ocean_buffer : register(VIEW3D_PROCEDURAL_BUFFER0_REGISTER);
#include "pr/game-systems/ocean/ocean_buffer.hlsli"

// Linear colour and opacity of fully covered foam. Foam is lit by the stock shading, so this is its albedo, not its final colour.
static const float3 FoamColour = float3(0.9f, 0.92f, 0.94f);
static const float FoamOpacity = 0.95f;

// Slope variance added by fully covered foam. Foam is a rough surface, so it blurs the environment reflection instead of mirroring it.
// Foam too far away to show its pattern is seen mostly at low angles, where even a blurred reflection would hide it, so it is made fully matte.
static const float FoamSlopeVariance = 0.3f;
static const float UnresolvedSlopeVariance = 1.0f;

// Size of the foam texture in texels, and the number of its largest filament cells across one tile.
static const float FoamTextureSize = 512.0f;
static const float FoamCellsPerTile = 4.0f;

// Linear colour of water clouded by fine bubbles, and how far the foam amount moves the water's colour and opacity towards it. Bubbles scatter
// light back out of the water, so foamy water is paler and less transparent than clear water.
static const float3 MilkColour = float3(0.32f, 0.55f, 0.56f);
static const float MilkStrength = 0.6f;

// Breakup of the foam's repeats, in foam tiles: the wavelengths of the coverage noise, and the wavelength of the shore foam depth noise.
// The texture coordinates are also warped (see OceanFoamWarpWavelength).
static const float2 FoamCoverWavelengths = float2(1.7f, 5.3f);
static const float FoamBandWavelength = 4.3f;

// Return a random value in [0, 1) for the integer lattice cell 'cell'. A fixed hash makes the value the same every frame, and the values repeat
// every OceanFoamNoisePeriod cells.
float FoamHash(float2 cell)
{
	// Mix both coordinates into one well-scrambled value.
	uint2 q = asuint(int2(cell)) & uint(OceanFoamNoisePeriod - 1);
	uint h = (q.x * 0x8da6b343u) ^ (q.y * 0xd8163841u);
	h = (h ^ (h >> 16)) * 0x7feb352du;
	h = (h ^ (h >> 15)) * 0x846ca68bu;
	h ^= h >> 16;
	return float(h) * (1.0f / 4294967296.0f);
}

// Return smooth noise in [0, 1] at 'p', with one random value per unit lattice cell, blended smoothly between them.
float FoamNoise(float2 p)
{
	// Blend the random values at the four surrounding lattice points.
	float2 cell = floor(p);
	float2 f = p - cell;
	float2 s = f * f * (3.0f - 2.0f * f);
	float a = FoamHash(cell);
	float b = FoamHash(cell + float2(1.0f, 0.0f));
	float c = FoamHash(cell + float2(0.0f, 1.0f));
	float d = FoamHash(cell + float2(1.0f, 1.0f));
	return lerp(lerp(a, b, s.x), lerp(c, d, s.x), s.y);
}

// Foam pattern at one pixel, from the foam texture. See FoamTexture.
struct FoamTextureSample
{
	float value;     // Pattern value in [0, 1].
	float2 gradient; // Change in 'value' per metre in world x and y.
	float resolved;  // How well the pixel shows the pattern, from 1 (fully) to 0 (not at all, so 'value' is meaningless).
};

// Return (value, d/du, d/dv) of the foam pattern at texture coordinate 'uv', where 'uv_dx' and 'uv_dy' are the changes in 'uv' across the pixel.
// The gradient is measured over one texel of the filtered texture, so it smooths out with the pattern as the pixels grow.
float3 FoamLayer(float2 uv, float2 uv_dx, float2 uv_dy)
{
	// Explicit derivatives choose the mip level, so this is safe inside branches.
	float texel = 1.0f / FoamTextureSize;
	float h = g_normal_texture.SampleGrad(g_normal_sampler, uv, uv_dx, uv_dy).a;
	float hu = g_normal_texture.SampleGrad(g_normal_sampler, uv + float2(texel, 0.0f), uv_dx, uv_dy).a;
	float hv = g_normal_texture.SampleGrad(g_normal_sampler, uv + float2(0.0f, texel), uv_dx, uv_dy).a;
	return float3(h, (hu - h) * FoamTextureSize, (hv - h) * FoamTextureSize);
}

// Return the foam pattern at world position 'ws_xy', where 'ws_dx' and 'ws_dy' are the changes in 'ws_xy' across the pixel and 'footprint' is
// the pixel size in metres. The small layer repeats every tile; the large layer is larger and turned (see OceanFoamLargeScale), so their
// repeats do not line up, and broad noise shifts both by different amounts in different places, so the repeats are not on a regular grid.
// Where the small layer's cells are too small for the pixel, the large layer alone shows, and where its cells are too small too, the pattern is
// unresolved.
FoamTextureSample FoamTexture(float2 ws_xy, float2 ws_dx, float2 ws_dy, float footprint)
{
	// Measure how well each layer's coarsest cells are resolved. Cells a few pixels across still show the pattern, smaller cells would shimmer.
	float tile = g_ocean.foam_pattern.x;
	float small_cell = tile / FoamCellsPerTile;
	float large_cell = OceanFoamLargeScale * small_cell;
	float small_resolved = 1.0f - smoothstep(0.1f, 0.4f, footprint / small_cell);
	float large_resolved = 1.0f - smoothstep(0.1f, 0.4f, footprint / large_cell);
	FoamTextureSample result;
	result.value = 0.5f;
	result.gradient = float2(0.0f, 0.0f);
	result.resolved = large_resolved;
	if (large_resolved <= 0.0f)
		return result;

	// Shift the sample position by broad noise. The shift changes slowly, so the pixel's derivatives and the gradient stay close enough.
	// The noise of each component moves along its own direction, so the pattern wanders in place (see OceanFoamChurnDirX).
	float2 warp_p = ws_xy / (OceanFoamWarpWavelength * tile);
	float2 warp = float2(FoamNoise(warp_p + g_ocean.foam_churn.xy), FoamNoise(warp_p + g_ocean.foam_churn.zw + float2(37.1f, 11.7f)));
	ws_xy += (2.0f * OceanFoamWarpShift * tile) * (warp - 0.5f);

	// Sample the large layer in its turned frame. Its gradient turns back to world axes through the transpose of the turn.
	float c = cos(OceanFoamLargeAngle);
	float s = sin(OceanFoamLargeAngle);
	float2x2 turn = float2x2(c, -s, s, c);
	float large_scale = 1.0f / (OceanFoamLargeScale * tile);
	float3 large = FoamLayer(mul(turn, ws_xy) * large_scale, mul(turn, ws_dx) * large_scale, mul(turn, ws_dy) * large_scale);
	large.yz = mul(large.yz, turn) * large_scale;

	// Add the small layer where it is resolved. It dominates, and the large layer breaks up its repeats.
	float3 sum = large;
	if (small_resolved > 0.0f)
	{
		// Both layers are in the same units, so blending the values and gradients blends the patterns.
		float3 small = FoamLayer(ws_xy / tile, ws_dx / tile, ws_dy / tile);
		small.yz /= tile;
		sum = lerp(large, 0.7f * small + 0.3f * large, small_resolved);
	}
	result.value = sum.x;
	result.gradient = sum.yz * large_resolved;
	return result;
}

// Return the foam history at world position 'ws_xy': crest foam left behind by earlier crests (see ocean_foam_cs.hlsl). The near window has fine
// texels; towards its edge the far window's coarse texels take over, and beyond the far window there is no history. An empty history is not read.
float FoamHistory(float2 ws_xy)
{
	// An empty history has no foam to read.
	if (g_ocean.foam_history.z == 0u)
		return 0.0f;

	// Measure how far the position is towards the near window's edge, as a fraction of its half side.
	uint slot = g_ocean.foam_history.y;
	float half_side = 0.5f * float(OceanFoamHistorySide);
	float2 near_p = ws_xy / OceanFoamNearTexel;
	float2 near_local = abs(near_p - float2(g_ocean.foam_origins.xy) - half_side) / half_side;
	float to_far = smoothstep(0.8f, 0.95f, max(near_local.x, near_local.y));

	// Blend from the near window to the far one. Each is read only where it contributes.
	float near = to_far < 1.0f ? OceanFoamHistorySample(0u, slot, g_ocean.foam_origins.xy, near_p) : 0.0f;
	float far = to_far > 0.0f ? OceanFoamHistorySample(1u, slot, g_ocean.foam_origins.zw, ws_xy / OceanFoamFarTexel) : 0.0f;
	return lerp(near, far, to_far);
}

// Horizontal distances from the camera (m) over which crests fade from pixel resolution to the vertices' resolution (see PixelCrest).
static const float2 PixelCrestRange = float2(60.0f, 120.0f);

// Return the crest steepness (see OceanWaveSample) at world position 'ws_xy', 'distance' metres horizontally from the camera, for a pixel 'footprint' metres across.
// 'vertex_steepness' and 'vertex_spacing' are the vertices' crest steepness and lattice spacing (m).
// The vertex lattice cannot show waves shorter than a few vertex spacings, and the lattice coarsens with distance, so far crests interpolated
// from the vertices are broad and soft. Waves that are too short for the lattice but long enough for the pixel are added back here, so crests
// within the pixel crest range are as narrow as the pixels allow.
float PixelCrest(float2 ws_xy, float distance, float footprint, float vertex_steepness, float vertex_spacing)
{
	// The vertex steepness is a sum over the vertices' waves, so more waves can be added to it. The added waves fade out over the pixel crest
	// range, because far water covers much of the screen and its narrow crests are hard to see.
	float sum = vertex_steepness;
	float wave_fade = OceanWaveFade(distance, g_ocean.level_radii.w) * (1.0f - smoothstep(PixelCrestRange.x, PixelCrestRange.y, distance));

	// A wave is added only if the pixel keeps more of it than the lattice does (see OceanWaveFilter). That needs a pixel finer than the lattice,
	// and a wavelength above four pixel footprints but below six vertex spacings, where the lattice keeps the wave in full.
	if (wave_fade <= 0.0f || footprint >= vertex_spacing || OceanCalm(g_ocean.waves.y))
		return sum;

	// The elements are in order of increasing wavelength, so the waves to add start at the first wave longer than four footprints, and end at the
	// first wave the lattice keeps in full. Most far and near pixels have no such wave, and skip the costly sea lookup.
	float pixel_lost = 4.0f * footprint;
	float lattice_kept = 6.0f * vertex_spacing;
	uint first = 0;
	for (; first != g_ocean.layout.z; ++first)
	{
		// Stop at the first wave that the pixel shows.
		if (asfloat(g_ocean_buffer.Load(first * 64 + 36)) > pixel_lost)
			break;
	}
	if (first != g_ocean.layout.z && asfloat(g_ocean_buffer.Load(first * 64 + 36)) < lattice_kept)
	{
		// Each wave gets the share of its amplitude that the pixel allows but the lattice removed. The wavelength alone decides this, so most
		// elements are skipped after one load. Short waves are deep-water waves, so the depth correction is left out.
		OceanSeaData data;
		OceanBufferSource source;
		source.sea = OceanSeaAt(data, g_ocean.wind_sea, ws_xy);
		float time = g_ocean.camera_time.w;
		for (uint i = first; i != g_ocean.layout.z; ++i)
		{
			// Add the wave's crest steepness, as the vertex shader does (see OceanWaves).
			float wavelength = asfloat(g_ocean_buffer.Load(i * 64 + 36));
			if (wavelength >= lattice_kept)
				break;

			float share = saturate(OceanWaveFilter(wavelength, footprint) - OceanWaveFilter(wavelength, vertex_spacing)) * wave_fade;
			if (share <= 0.0f)
				continue;

			WaterFieldElement element = source.Element(i);
			element.wave.x *= share;
			element.wave.w = 0.0f;
			element.timing.x *= OceanWaveFilter(wavelength / 3.0f, footprint);
			sum += OceanCrestSteepness(element, WaterFieldElementHeightAndGradient(element, ws_xy, time).x);
		}
	}
	return sum;
}

// The most coarse reflection march samples per water pixel (see g_env_map_max_lane_steps). A 2x2 pixel group tests four times as many.
static const int WaterReflectionLaneSteps = 2;

// The most halving steps that refine a reflection hit on water (see g_env_map_max_refine_steps)
static const int WaterReflectionRefineSteps = 3;

// The local wind speeds (m/s) over which the projected reflection fades out of the water seen from above (see g_reflection_plane_weight). The
// capillary roughness has already faded the projected reflection to about a tenth at the lower speed and a twentieth at the upper. The renderer
// stops rendering the projected reflection when every water cell's wind is above the upper speed (see WaterReflectionGate.cs).
static const float2 ReflectionFadeWinds = float2(20.0f, 25.0f);

// Return the water's vertical depth (m) under the fragment 'In', from the caller's shore height source, so it does not depend on the view direction.
// The result is negative where no ground surface is known below.
float ShoreDepth(PSIn In)
{
	// The shore source reports NaN where it knows no ground height.
	float height = OceanShoreHeight(In.ws_vert.xy, g_ocean.shore_source);
	return isnan(height) ? -1.0f : In.ws_vert.z - height;
}

// Return how much, in [0, 1], to darken the shaded water at the waterline, so the water's edge reads as a rounded lip in shadow instead of a
// flat knife edge. 'water_depth' is the water's depth under the fragment (see ShoreDepth). 'fade' scales the lip with the camera distance.
float ShoreLipDarkness(float water_depth, float fade)
{
	// Only water thinner than the lip height darkens.
	float height = g_ocean.shore_lip.x;
	float strength = g_ocean.shore_lip.y * fade;
	if (strength <= 0.0f || water_depth < 0.0f || water_depth >= height)
		return 0.0f;

	// At a fraction 'u' of the lip height, the darkness is (1 - u)². This is zero with zero rate of change at the top, so the lip fades in without
	// a visible edge, and it is greatest along the terrain.
	float w = 1.0f - water_depth / height;
	return saturate(strength * w * w);
}

// Darken the shaded colour 'colour' by 'darkness', making it more opaque so the dark edge shows over the terrain behind it.
float4 Darken(float4 colour, float darkness)
{
	// Darkness 0 leaves the colour unchanged.
	return float4(colour.rgb * (1.0f - darkness), lerp(colour.a, 1.0f, darkness));
}

// Cover the surface of 'In' with foam by 'foam', in [0, 1], over water clouded by 'milk', in [0, 1]. Foam is white, nearly opaque, and rough,
// so it hides the water colour and blurs the reflections. 'slope_variance' is the roughness of fully covered foam (see FoamSlopeVariance).
void ApplyFoam(inout PSIn In, float foam, float milk, float slope_variance)
{
	// Cloud the water first, so gaps in the foam show milky water. Zero foam and milk leave the surface unchanged.
	In.diff.rgb = lerp(In.diff.rgb, MilkColour, milk * MilkStrength);
	In.diff.a = lerp(In.diff.a, 1.0f, milk * MilkStrength);
	In.diff.rgb = lerp(In.diff.rgb, FoamColour, foam);
	In.diff.a = lerp(In.diff.a, FoamOpacity, foam);
	g_unresolved_slope_variance += foam * slope_variance;
}

// Return the ripple strength at surface position 'xy', as a detail strength texel (see ApplyDetailNormals): the height scale relative to a fresh breeze,
// the capillary slope variance, and the downwind direction. The ripples follow the wind of the wind sea's first level, which is the least smoothed.
float4 RippleStrength(float2 xy)
{
	// Blend the first level's wind of the four surrounding nodes. Calm water has no strength, so its direction does not matter.
	WaterBathymetryCell cell = WindSeaLocate(g_ocean.wind_sea, xy);
	uint levels = uint(g_ocean.wind_sea.level_count);
	OceanSeaData data;
	float2 wind = WindSeaBlendWind(cell, data.Wind(uint(cell.index.x) * levels), data.Wind(uint(cell.index.y) * levels), data.Wind(uint(cell.index.z) * levels), data.Wind(uint(cell.index.w) * levels));
	float speed = length(wind);
	return float4(OceanRippleSteepness(speed) / OceanRippleReferenceSteepness, OceanRippleSlopeVariance(speed), wind);
}

// Shade the opaque curtain at the domain wall: the stock shading with the ripples, and no foam or lip.
float4 RippleShade(inout PSIn In, bool is_front_face)
{
	// Keep the tilted normal in 'In' so the reflection attributes use it too.
	float4 strength = RippleStrength(In.ws_vert.xy);
	if ((g_ocean.layout.y & OceanFlagRipples) != 0)
		In = ApplyDetailNormals(In, strength);

	// The ripple strength visualisation replaces the shading.
	float4 diff = ForwardShade(In, is_front_face).diff;
	if ((g_ocean.layout.y & OceanFlagShowRippleStrength) != 0)
		diff.rgb = DetailStrengthColour(strength.x);

	return diff;
}

// Shade a water surface fragment with the stock detail-normal shading, adding the dark lip at the waterline and textured foam.
float4 WaterShade(inout PSIn In, bool is_front_face)
{
	// Ripples and foam break up reflections and shadow edges, so the water takes a short, coarse reflection march and a single shadow sample.
	// These are a large part of the water's cost, because it covers much of the screen.
	g_env_map_max_lane_steps = WaterReflectionLaneSteps;
	g_env_map_max_refine_steps = WaterReflectionRefineSteps;
	g_env_map_near_miss_search = false;
	g_shadow_filter_max = 1;

	// The water lies on the window's reflection plane, so it reflects the opaque scene on screen, including the trees, and uses the environment
	// map only where the reflected surface is off screen or hidden.
	g_reflection_plane_surface = true;

	// The ripple strength visualisation replaces the shading, foam included.
	float4 ripple = RippleStrength(In.ws_vert.xy);
	if ((g_ocean.layout.y & OceanFlagShowRippleStrength) != 0)
		return RippleShade(In, is_front_face);

	// Seen from above, the projected reflection fades out in strong local winds (see ReflectionFadeWinds), where the rough surface already leaves
	// little of it. From below, the reflection plane is not rendered, and the surface keeps its usual reflection.
	if (g_frame.cam.c2w[3].z > g_ocean.level_radii.x)
		g_reflection_plane_weight = 1.0f - smoothstep(ReflectionFadeWinds.x, ReflectionFadeWinds.y, length(ripple.zw));

	// Tilt the normal by the detail layers, and measure the pixel's extent while the control flow is still uniform. The wave normal without the
	// detail layers is kept, because foam floats over the ripples and hides them.
	float3 wave_normal = normalize(In.ws_norm.xyz);
	if ((g_ocean.layout.y & OceanFlagRipples) != 0)
		In = ApplyDetailNormals(In, ripple);

	// Without foam the water has only the stock shading, and none of the foam's costs.
	if ((g_ocean.layout.y & OceanFlagFoam) == 0)
		return ForwardShade(In, is_front_face).diff;

	float2 ws_dx = ddx(In.ws_vert.xy);
	float2 ws_dy = ddy(In.ws_vert.xy);
	float footprint = length(fwidth(In.ws_vert.xyz));

	// Crests get their steepness at pixel resolution, so they stay narrow out to the pixel crest range (see PixelCrest). Keys without crest foam, and
	// shallow water, skip the cost.
	float depth_fade = saturate(In.tex0.y);
	float crest = 0.0f;
	if (depth_fade > 0.0f && g_ocean.foam_crest.x > 0.0f)
	{
		// The crest foam follows the pixel's steepness.
		float steepness = PixelCrest(In.ws_vert.xy, length(In.ws_vert.xy - g_frame.cam.c2w[3].xy), footprint, In.tex0.x, In.idx0.y);
		crest = OceanCrestFoam(steepness, depth_fade);
	}

	// Shore foam and the lip fade out towards the fade radius, because measuring the water's depth is costly. Beyond it the depth is not measured.
	float distance = length(In.ws_vert.xyz - g_frame.cam.c2w[3].xyz);
	float radius = g_ocean.foam_pattern.z;
	float fade = 1.0f - smoothstep(max(radius - g_ocean.foam_pattern.w, 0.0f), radius, distance);
	float water_depth = fade > 0.0f ? ShoreDepth(In) : -1.0f;
	float darkness = ShoreLipDarkness(water_depth, fade);

	// Shore foam covers the water fully at the waterline and falls to zero at the shore foam depth. The foam amount is the largest of the shore
	// foam, the crest foam, and the trails that earlier crests left. Broad noise varies the depth along the shore between half and one and a half
	// times, so the foam's edge is not a smooth contour.
	float tile = g_ocean.foam_pattern.x;
	float band = g_ocean.foam_shore.x;
	band *= 0.5f + FoamNoise(In.ws_vert.xy / (FoamBandWavelength * tile) + float2(71.3f, 29.9f));
	float shore = water_depth >= 0.0f ? (1.0f - saturate(water_depth / band)) * fade : 0.0f;
	float amount = saturate(max(max(shore, crest), FoamHistory(In.ws_vert.xy)));
	if (amount > 0.0f)
	{
		// Broad noise raises the amount to a power between about a third and three, so neighbouring repeats of the texture show different coverage.
		// A power keeps no foam at zero amount and full foam at a full amount.
		float2 cover_p = In.ws_vert.xy / tile;
		float cover = 0.6f * FoamNoise(cover_p / FoamCoverWavelengths.x + float2(13.7f, 53.1f)) + 0.4f * FoamNoise(cover_p / FoamCoverWavelengths.y + float2(91.3f, 7.9f));
		amount = pow(amount, exp2(3.2f * (cover - 0.5f)));

		// Foam covers the pattern where it is above a threshold that falls as the amount rises: none at zero amount, all of it at full amount. The
		// softness blurs the foam's edge over a range of pattern values. Where the pattern is unresolved, the foam is a plain cover of the amount.
		FoamTextureSample pattern = FoamTexture(In.ws_vert.xy, ws_dx, ws_dy, footprint);
		float soft = g_ocean.foam_shore.y;
		float edge = lerp(1.0f + soft, -soft, amount);
		float t = saturate((pattern.value - (edge - soft)) / (2.0f * soft));
		float foam = lerp(amount, t * t * (3.0f - 2.0f * t), pattern.resolved);
		float foam_slope = pattern.resolved * 6.0f * t * (1.0f - t) / (2.0f * soft);

		// Foam stands above the water by up to the bump height, and its top follows the pattern: height = bump * foam * (0.5 + pattern). A height
		// field tilts the normal against the tangential part of its gradient, so the foam's edges and lumps catch the light. Foam hides the ripples,
		// so the base normal blends from the detailed normal to the wave normal as the foam covers the pixel.
		float3 normal = normalize(lerp(normalize(In.ws_norm.xyz), wave_normal, foam));
		float bump = g_ocean.foam_pattern.y;
		float3 gradient = float3(bump * (foam_slope * (0.5f + pattern.value) + foam) * pattern.gradient, 0.0f);
		gradient -= dot(gradient, normal) * normal;
		In.ws_norm = float4(normalize(normal - gradient), 0.0f);

		// Foam that shows its pattern is rough; unresolved foam is matte. The milkiness follows the amount, not the pattern, so it fills the gaps.
		ApplyFoam(In, foam, amount, lerp(UnresolvedSlopeVariance, FoamSlopeVariance, pattern.resolved));
	}

	// Shade once, outside the foam branch: the reflection picks its blur from the change in the mirror direction across neighbouring pixels,
	// which is only defined where every pixel of the 2x2 group runs the same code. Shading inside the branch would outline the foam with sharp reflections.
	return Darken(ForwardShade(In, is_front_face).diff, darkness);
}

// The opaque entry points draw the curtain at the domain wall, where there is no opaque depth to measure the water against. The alpha entry points
// draw the transparent surface.
VIEW3D_FORWARD_PIXEL_ENTRY_POINTS(WaterOpaque, RippleShade)
VIEW3D_FORWARD_PIXEL_ENTRY_POINTS(Water, WaterShade)
