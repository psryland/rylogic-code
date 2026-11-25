//************************************
// Ocean Surface
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Per-frame constants of the world-view water surface and the lattice and wave evaluation, shared by the native object owner and the vertex shader.
#ifndef PR_GAME_SYSTEMS_OCEAN_SURFACE_HLSLI
#define PR_GAME_SYSTEMS_OCEAN_SURFACE_HLSLI
#include "pr/hlsl/interop.hlsli"
#include "pr/physics/terrain/water/water_field.hlsli"
#include "pr/physics/terrain/water/water_depth.hlsli"
#include "pr/physics/terrain/water/wind_sea.hlsli"

#ifdef __cplusplus
namespace pr::game_systems::ocean
{
	using namespace pr::hlsl;
	using namespace pr::physics::terrain::water::shared;

	// Prefer the HLSL overloads to the global C maths functions, which the using-directives would otherwise make ambiguous.
	using pr::hlsl::abs;
	using pr::hlsl::pow;
	using pr::hlsl::sqrt;
#endif

// Maximum number of wave elements in the buffer. The element region always has room for this many, so the other regions never move.
static const uint OceanMaxElements = 40;

// Maximum number of wind-sea wavelength bands, and so of wind levels per node.
static const uint OceanMaxBands = 5;

// Correct wave amplitudes for the local water depth (shoaling and breaking). Without it all water is treated as deep.
static const uint OceanFlagDepthEffects = 1;

// Draw the small-scale ripple normals. Without it the surface shows only the wave normals.
static const uint OceanFlagRipples = 2;

// Draw the foam and the dark lip at the waterline. Without it the foam history is not updated either.
static const uint OceanFlagFoam = 4;

// Draw the ripple strength as colour instead of the shaded water: blue for calm, green for a fresh breeze (8 m/s), and red for twice its roughness or more.
static const uint OceanFlagShowRippleStrength = 8;

// Number of quads in the opaque curtain that closes the water against the domain wall, from the wave surface down to the domain floor.
// Curtain vertex IDs follow the lattice's, as top and bottom pairs ordered anticlockwise from above.
static const uint OceanCurtainSegments = 8192;

// The procedural constants block. View3D copies exactly 1024 bytes, so the reserved tail pads the block to that size.
// The procedural buffer holds five regions at fixed byte offsets: 'layout.z' wind-sea wave elements of 64 bytes from offset 0 (with room
// for OceanMaxElements) in order of increasing wavelength, the bathymetry heights as row-major floats from 'layout.w', the wind-sea
// table rows (float4 each) from 'wind_sea_offsets.x', the wind-sea node winds (float2 each) from 'wind_sea_offsets.y', and the foam history
// from 'foam_history.x'.
// See wind_sea.hlsli for the wind-sea layout, and OceanFoamHistorySide for the foam history.
// World positions are world metres with the domain centred on the origin.
struct OceanConstants
{
	float4 camera_time;     // Camera world position (xyz) and the wave clock in seconds (w).
	float4 level_radii;     // Still-water level, domain radius, mesh outer radius, and the radius beyond which the surface is flat.
	float4 mesh;            // Minimum vertex spacing (m), radial warp power, breaking height/depth ratio, and the domain floor height (m) where the curtain ends.
	uint4 layout;           // Lattice vertices per side, OceanFlag* bits, wave element count, and the bathymetry byte offset.
	float4 bathymetry;      // Bathymetry grid origin (xy) and node spacing (z).
	uint4 bathymetry_dims;  // Bathymetry node counts along x and y.
	float4 colour;          // Linear RGBA tint of the surface.
	WindSeaGrid wind_sea;   // Layout of the wind-sea winds and table.
	uint4 wind_sea_offsets; // Byte offsets of the wind-sea table (x) and winds (y).
	float4 waves;           // Swash allowance in metres (x, see WaterFieldSwashDepth), the largest rise or fall of the drawn surface from the still-water level in metres (y), and the shore crest sharpness (z, see WaterFieldShoreSharpness). w is unused.
	float4 foam_shore;      // Shore foam: vertical water depth (m) at which the shore foam fades out (x), and edge softness (y). z and w are unused.
	float4 foam_crest;      // Crest foam: strength (x), and the crest steepness score at which foam starts (y) and the further score over which it reaches full (z); w is unused.
	float4 foam_pattern;    // Foam texture: tile size (m, x), bump height (m, y), the camera distance (m) beyond which there is no shore foam or lip (z), and the distance (m) inside it over which they fade out (w).
	float4 shore_lip;       // Dark lip at the waterline: water thickness (m) over which the water darkens (x) and its darkness at the terrain (y), and the still-water depth (m) at which crest foam is gone (z). w is unused.
	float4 foam_fade;       // The time step (s) of the last foam history update (x) and the foam's fade rate (amount per second, y). zw are unused.
	int4 foam_origins;      // Lowest corner, in texels, of the near (xy) and far (zw) foam history windows. See OceanFoamHistorySide.
	int4 foam_prev_origins; // The windows' corners at the previous history update. Texels outside them hold no history.
	uint4 foam_history;     // Byte offset of the foam history region (x), the slot (0 or 1) that holds the latest history (y), and 1 while the history may hold foam, or 0 when it is empty and is not updated or read (z). w is unused.
	float4 foam_churn;      // Offsets, in warp noise cells, of the noise that warps the foam texture in x (xy) and in y (zw). They move slowly so the foam pattern wanders in place. Each is wrapped to [0, OceanFoamNoisePeriod).
	uint4 shore_source[3];  // Parameters of the caller's shore depth source, which the ocean passes through unchanged. See ocean_forward_ps.hlsli.
	float4 rain;            // Rain rings: intensity in [0, 1] (x), the largest ring radius in metres (y), the steepest slope of a new ring (z), and the wavelength in metres of a new ring's ripples (w). Zero in any component draws no rings.
	float4 reserved[39];
};

// Foam history: two square windows of foam amounts that follow the camera, a near window with fine texels and a far window with coarse ones.
// Each window wraps around in both directions, so the texel for grid cell 'g' is at 'g' modulo the side, and cells that stay inside a moving
// window keep their texels. Each window has two slots, so one update reads one slot and writes the other. The history region holds the near
// window's two slots and then the far window's two, each a row-major array of floats.
static const int OceanFoamHistorySide = 256;
static const float OceanFoamNearTexel = 0.5f;
static const float OceanFoamFarTexel = 4.0f;

// The foam texture is drawn in two layers: the small layer at the tile size, and a large layer this many times larger and turned by this angle,
// so the two layers' repeats do not line up.
static const float OceanFoamLargeScale = 4.3f;
static const float OceanFoamLargeAngle = 0.61f;

// The foam texture coordinates are bent by broad noise: its wavelength and the largest shift, in foam tiles. The shift is small against the wavelength,
// so it bends the pattern only slightly while moving each repeat by a different amount. The noise moves along two fixed directions, one for each
// component of the shift, so the pattern wanders in circles rather than sliding.
static const float OceanFoamWarpWavelength = 3.1f;
static const float OceanFoamWarpShift = 0.3f;
static const float2 OceanFoamChurnDirX = float2(0.8f, 0.6f);
static const float2 OceanFoamChurnDirY = float2(-0.6f, 0.8f);

// The foam noise repeats every this many lattice cells, which is far larger than any world, so moving noise offsets can wrap without a jump.
static const int OceanFoamNoisePeriod = 4096;

// The lattice is a square of vertices mapped to a disk around the camera, with ring spacing that grows with distance. These helpers describe
// that mapping, so the vertex shader and CPU queries of the drawn surface agree. 'unit_radius' is a lattice position's distance from the
// disk centre in [0, 1], and 'half_cells' is half the number of cells along one side of the lattice.

// Return the radius (m) of the uniformly spaced central part of the disk.
odr float OceanCentralExtent(float min_spacing, float half_cells, float outer_radius)
{
	return min(min_spacing * half_cells, outer_radius);
}

// Return the radial warp power for a camera 'camera_height' metres above or below the still-water level.
// The warp flattens towards uniform spacing as the camera rises, because distant rings then cover a larger share of the screen.
odr float OceanWarpPower(float warp_power, float camera_height, float outer_radius)
{
	return lerp(warp_power, 1.0f, saturate(abs(camera_height) / outer_radius));
}

// Return the distance (m) from the camera of the ring at 'unit_radius'.
odr float OceanRingRadius(float unit_radius, float central_extent, float outer_radius, float warp_power)
{
	return central_extent * unit_radius + (outer_radius - central_extent) * pow(unit_radius, warp_power);
}

// Return the spacing (m) between neighbouring rings at 'unit_radius'. It limits the shortest wave the lattice can show.
odr float OceanRingSpacing(float unit_radius, float central_extent, float outer_radius, float warp_power, float half_cells)
{
	return (central_extent + (outer_radius - central_extent) * warp_power * pow(max(unit_radius, 1.0e-6f), warp_power - 1.0f)) / half_cells;
}

// Return the ring spacing (m) at 'distance' metres from the camera. Rings beyond the outer radius take the outermost ring's spacing.
// Points on the domain wall use this, so they get the same waves whether they are on the curtain or on the surface's edge.
odr float OceanSpacingAtDistance(float distance, float central_extent, float outer_radius, float warp_power, float half_cells)
{
	// The ring radius grows steadily with the unit radius, so halving the bracket each step finds the ring at 'distance'.
	float lo = 0.0f;
	float hi = 1.0f;
	for (int i = 0; i != 20; ++i)
	{
		// Keep the half of the bracket that contains the ring.
		float mid = 0.5f * (lo + hi);
		if (OceanRingRadius(mid, central_extent, outer_radius, warp_power) < distance)
			lo = mid;
		else
			hi = mid;
	}
	return OceanRingSpacing(0.5f * (lo + hi), central_extent, outer_radius, warp_power, half_cells);
}

// Return the wave weight at 'radius' metres from the camera. Waves fade out over the last 30% of the wave radius, so the far surface is flat.
odr float OceanWaveFade(float radius, float wave_radius)
{
	return 1.0f - saturate((radius - 0.7f * wave_radius) / (0.3f * wave_radius));
}

// The largest rise or fall (m) of the drawn surface (see OceanConstants::waves.y) at which the whole surface is drawn flat. Waves this low
// cannot be seen, so the wave sums are skipped.
static const float OceanCalmHeight = 0.001f;

// True when the whole surface is drawn flat, given the largest rise or fall (m) of the drawn surface, 'wave_height'.
odr bool OceanCalm(float wave_height)
{
	return wave_height <= OceanCalmHeight;
}

// Return the weight of a wave of 'wavelength' metres on a lattice with 'spacing' metres between vertices. Triangles interpolate the
// per-vertex normals linearly, so waves with fewer than about four vertices per wavelength alias into patterns that follow the lattice
// rows. Such waves are removed, and waves fade in until they have six vertices per wavelength.
odr float OceanWaveFilter(float wavelength, float spacing)
{
	return saturate((wavelength / spacing - 4.0f) / 2.0f);
}

// Return the largest slope one full-weight ripple layer adds in a wind of 'wind_speed' m/s. Ripples are absent in light air, appear as the wind
// breaks the surface (about 3 m/s), rise smoothly to full strength by 8 m/s, and keep growing linearly into a gale (40 m/s).
odr float OceanRippleSteepness(float wind_speed)
{
	float onset = saturate((wind_speed - 2.5f) / (8.0f - 2.5f));
	float gale = saturate((wind_speed - 8.0f) / (40.0f - 8.0f));
	return 0.4f * onset * onset * (3.0f - 2.0f * onset) + 0.6f * gale;
}

// The ripple steepness at 8 m/s, a fresh breeze, at which the ripple layers are set. Ripple strengths are relative to it, so a strength of 1 is a
// fresh breeze and calm water is 0.
static const float OceanRippleReferenceSteepness = 0.4f;

// Return the mean square slope of the capillary roughness finer than the ripple layers in a wind of 'wind_speed' m/s. It is zero up to 4 m/s,
// so light air stays glossy, then grows linearly with the wind like measured sea-surface slope statistics, at about half their rate because
// the waves and the ripple layers already show part of the slope.
odr float OceanRippleSlopeVariance(float wind_speed)
{
	return wind_speed > 4.0f ? 0.0025f * (wind_speed - 4.0f) : 0.0f;
}

// The state of every wind-sea band at one surface position.
struct OceanSea
{
	WindSeaBand bands[OceanMaxBands];
};

// Return the state of every band of the wind sea laid out by 'grid' at 'world_xy'. 'data.Wind(i)' returns wind vector 'i' and
// 'data.Row(i)' returns table row 'i', both indexed as in wind_sea.hlsli. Bands beyond the grid's band count are calm.
template <typename TData>
OceanSea OceanSeaAt(in_(TData) data, WindSeaGrid grid, float2 world_xy)
{
	// Blend the four node winds of each level once, because several bands share each level.
	WaterBathymetryCell cell = WindSeaLocate(grid, world_xy);
	int levels = grid.level_count;
	float2 level_winds[OceanMaxBands];
	hlsl_unroll
	for (int l = 0; l != (int)OceanMaxBands; ++l)
	{
		// Unused levels stay calm.
		level_winds[l] = float2(0.0f, 0.0f);
		if (l < levels)
			level_winds[l] = WindSeaBlendWind(cell, data.Wind(cell.index.x * levels + l), data.Wind(cell.index.y * levels + l), data.Wind(cell.index.z * levels + l), data.Wind(cell.index.w * levels + l));
	}

	// Each band takes the wind between its two levels, then the table rows at that wind's speed.
	OceanSea sea;
	hlsl_unroll
	for (int b = 0; b != (int)OceanMaxBands; ++b)
	{
		// Unused bands have no waves.
		sea.bands[b].direction = float2(0.0f, 0.0f);
		sea.bands[b].amplitude = 0.0f;
		sea.bands[b].exponent = 0.0f;
		sea.bands[b].sharpness = 0.0f;
		if (b >= grid.band_count)
			continue;

		// Select the two levels by comparison in an unrolled loop, so that every array index is constant (see hlsl_unroll).
		int l1;
		float level_frac;
		int l0 = WindSeaLevelIndex(grid, b, l1, level_frac);
		float2 wind0 = float2(0.0f, 0.0f);
		float2 wind1 = float2(0.0f, 0.0f);
		hlsl_unroll
		for (int l = 0; l != (int)OceanMaxBands; ++l)
		{
			// Keep the winds of the bracketing levels.
			wind0 = l == l0 ? level_winds[l] : wind0;
			wind1 = l == l1 ? level_winds[l] : wind1;
		}
		float2 wind = lerp(wind0, wind1, level_frac);
		float row_frac;
		int row = WindSeaTableRowIndex(grid, b, length(wind), row_frac);
		sea.bands[b] = WindSeaBandState(wind, data.Row(row), data.Row(row + 1), row_frac);
	}
	return sea;
}

// Return wave 'element' with the amplitude and crest sharpness of its band in 'sea'. Elements outside the wind sea keep their own values.
odr WaterFieldElement OceanSeaElement(WaterFieldElement element, in_(OceanSea) sea)
{
	// Only wind-sea elements name a band.
	int band = WaterFieldWindSeaBand(element);
	if (band < 0)
		return element;

	// Select the band by comparison in an unrolled loop, so that every array index is constant (see hlsl_unroll).
	WindSeaBand state = sea.bands[0];
	hlsl_unroll
	for (int b = 1; b != (int)OceanMaxBands; ++b)
	{
		// Keep the element's band.
		if (b == band)
			state = sea.bands[b];
	}
	return WindSeaElement(element, state);
}

// The drawn wave surface at one position, with the values the pixel shader needs to place crest foam.
struct OceanWaveSample
{
	WaterFieldSurfaceSample surface;  // Height and normal of the drawn surface. displacement_foam.w is unused.
	float crest_steepness;            // Sum of the drawn waves' crest steepness here (see OceanCrestSteepness). 0 where there are no waves.
	float crest_depth_fade;           // Fraction of the crest foam that the still-water depth allows, in [0, 1]. See OceanWaves.
};

// Return a flat wave sample with no crests.
odr OceanWaveSample OceanWaveSampleZero()
{
	// Flat water has no crests.
	OceanWaveSample result;
	result.surface = WaterFieldSurfaceSampleZero();
	result.crest_steepness = 0.0f;
	result.crest_depth_fade = 0.0f;
	return result;
}

// Return the crest steepness of the sine or Gerstner wave 'element' at a point where its height is 'height': the slope of the wave's steepest
// face, rising from 0 at mid height to its full value at the crest, and 0 below mid height. The steepest face of a crest with sharpness r is
// steeper than that of a sine wave of the same height, and becomes very steep as r nears one (see WaterFieldWaveProfile), so sharp crests score
// far higher than rolling waves of the same height.
odr float OceanCrestSteepness(WaterFieldElement element, float height)
{
	// Flat elements have no crest.
	float r = element.timing.x;
	float crest = element.wave.x * (1.0f + r);
	if (crest <= 0.0f)
		return 0.0f;

	// The steepest slope is (tau/wavelength) * amplitude * F(r), where F is the slope gain of the sharpened profile over a sine wave. F is approximated
	// by (1 + r*(-0.4536 + 0.8406*r)) / (1 - r), which is exactly 1 at r = 0 and within about 3% of F for 0 <= r <= 0.9. Multiplying by the height
	// weight clamp(height, 0, crest) / crest cancels the amplitude and turns (1 - r)(1 + r) into 1 - r^2, so the score needs only one division.
	float numerator = 1.0f + r * (-0.4536178322f + 0.8405956717f * r);
	return (6.28318530718f * numerator * clamp(height, 0.0f, crest)) / (element.wave.y * (1.0f - r * r));
}

// Return the drawn wave height and normal at the surface position 'world_xy', where the still water is 'depth' metres deep.
// 'source.Element(i)' returns element 'i' with its local amplitude and crest sharpness (see OceanSeaElement). Vertices move only vertically, and the heights are the physics water
// field's heights: the same elements, depth correction, and breaking limit. The lattice cannot show waves too short for 'spacing', so those
// are filtered (see OceanWaveFilter), and every wave is scaled by 'wave_fade'; elsewhere the drawn surface is the surface that moves bodies.
// 'breaking_ratio' is the breaking height/depth ratio, and crests sharpen towards 'shore_sharpness' as waves near it (see WaterFieldShoreSharpness). 'swash' is the swash allowance (see WaterFieldSwashDepth), which lets waves run up the
// shore above the still-water level. Depth effects correct the amplitudes for 'depth' plus 'swash'; without them the water is treated as deep.
// The result is zero where 'depth' plus 'swash' is not positive.
// The crest steepness is the sum of OceanCrestSteepness over the drawn waves, so it peaks where the crests of steep, sharp waves meet, not
// on the tops of tall rolling waves. The crest depth fade falls from 1 to 0 as the still water shoals from twice 'crest_depth' (positive) to
// 'crest_depth', so shallow water near the shore shows only shore foam. Together they let the pixel shader put foam on breaking crests.
template <typename TSource>
OceanWaveSample OceanWaves(in_(TSource) source, uint count, float2 world_xy, float time, float depth, float swash, float spacing, float wave_fade, bool depth_effects, float breaking_ratio, float shore_sharpness, float crest_depth)
{
	// Dry positions beyond the swash and positions beyond the wave radius stay flat.
	OceanWaveSample result = OceanWaveSampleZero();
	WaterFieldSurfaceSample water = WaterFieldSurfaceSampleZero();
	float wave_depth = depth + swash;
	if (wave_fade <= 0.0f || wave_depth <= 0.0f)
		return result;

	// Total the shoaled amplitudes of every element, unfiltered, so the breaking limit matches the physics water field.
	float correction_depth = depth_effects ? wave_depth : WaterFieldDeepWater;
	float total_amplitude = 0.0f;
	for (uint i = 0; i != count; ++i)
	{
		WaterFieldElement element = source.Element(i);
		float shoaled = WaterFieldElementAmplitudeBound(element) * WaterFieldShoaling(element, correction_depth);
		total_amplitude += shoaled;
	}

	// Accumulate the corrected elements with zero steepness, so each contributes only its physics height and slope, and total their crest steepness.
	float min_sharpness = WaterFieldShoreSharpness(total_amplitude, correction_depth, breaking_ratio, shore_sharpness);
	float breaking_scale = WaterFieldBreakingScale(total_amplitude, min_sharpness, correction_depth, breaking_ratio);
	float steepness = 0.0f;
	for (uint j = 0; j != count; ++j)
	{
		// Apply the distance fade, the depth correction, and the lattice filter. Filtered waves cost nothing more.
		WaterFieldElement element = source.Element(j);
		float lattice = OceanWaveFilter(element.wave.y, spacing);
		element.wave.x *= wave_fade * lattice;
		if (element.wave.x == 0.0f)
			continue;

		// A sharp crest carries detail at a third of the wavelength and shorter. Soften the crest, including any shore sharpening, where the
		// lattice cannot show that detail, so sharp crests do not alias into lattice patterns.
		element.wave.w = 0.0f;
		WaterFieldElement corrected = WaterFieldDepthCorrected(element, correction_depth, breaking_scale, min_sharpness);
		corrected.timing.x *= OceanWaveFilter(element.wave.y / 3.0f, spacing);
		float height_before = water.displacement_foam.z;
		WaterFieldAccumulateSurface(corrected, world_xy, time, water);
		steepness += OceanCrestSteepness(corrected, water.displacement_foam.z - height_before);
	}

	// Whitecaps fade out in shallow water near the shore, where the shore foam takes over.
	result.surface = water;
	result.crest_steepness = steepness;
	result.crest_depth_fade = saturate(depth / crest_depth - 1.0f);
	return result;
}

#ifdef __cplusplus
	static_assert(sizeof(OceanConstants) == 1024);
}
#endif
#endif
