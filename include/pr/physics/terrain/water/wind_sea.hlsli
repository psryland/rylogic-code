//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Stage-neutral evaluation of a wind sea that varies across the world, shared by C++ and HLSL.
// A wind sea is a set of wave elements with fixed directions over the full circle, grouped into wavelength bands. Each element names its band
// in info.y (see WaterFieldWindSeaBand). The local wind of each band sets the amplitude and crest sharpness of every element in that band, so
// the waves follow the wind at each place without any element changing its direction, wavelength, or phase. Changing the wind therefore only
// changes amplitudes, and the surface never jumps.
// The data has two parts, both owned and loaded by the consumer:
//   * Winds: a regular grid of nodes, where node (i, j) is at origin + cell_size * (i, j). Each node holds 'level_count' wind vectors
//     (m/s, downwind), stored at [(j * dims.x + i) * level_count + level]. The levels are spread evenly over the bands, from the first band
//     to the last, and each band uses the wind interpolated between its two nearest levels (see WindSeaLevelIndex). So one level gives every
//     band the same wind, and one level per band gives each band its own wind. Callers usually smooth the winds so that they change slowly
//     over one wavelength of the bands that use them. Positions outside the grid use the nearest edge node.
//   * Table: for each band, 'speed_count' rows of WindSeaTableRow at wind speeds 0, speed_step, 2 * speed_step, ..., stored at
//     [band * speed_count + row]. Speeds beyond the last row use the last row.
// To evaluate the sea at a point:
//   1. Find the cell with WindSeaLocate, blend the four node winds of each level with WindSeaBlendWind, and interpolate each band's wind
//      between its two levels from WindSeaLevelIndex.
//   2. Find the two table rows for each band's wind speed with WindSeaTableRow, and make the band state with WindSeaBandState.
//   3. Use WindSeaElement to give each wind-sea element its local amplitude and sharpness, then evaluate it as usual (see water_field.hlsli).
// The local amplitude ignores how it changes with position, so slopes and velocities are accurate while the winds change slowly over one wavelength.
#ifndef PR_PHYSICS_WIND_SEA_HLSLI
#define PR_PHYSICS_WIND_SEA_HLSLI
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/core.hlsli"
#include "pr/physics/terrain/water/water_field_types.hlsli"
#include "pr/physics/terrain/water/water_depth.hlsli"

#ifdef __cplusplus
namespace pr::physics::terrain::water::shared
{
#endif

// The layout of a wind sea's wind grid and table. See the file comment for how the data is stored.
struct WindSeaGrid
{
	float2 origin;    // World position of node (0, 0), in metres.
	float cell_size;  // Node spacing in metres, positive.
	float speed_step; // Wind speed between table rows, in m/s, positive.
	int2 dims;        // Node count in x and y, each at least two.
	int band_count;   // Number of wavelength bands, positive.
	int speed_count;  // Table rows per band, at least two.
	int level_count;  // Wind vectors per node, in [1, band_count].
	int3 pad;
};

// One table row: the state of a band's waves when the band's wind blows steadily at the row's speed.
//   x = amplitude (m) of an element travelling exactly downwind.
//   y = alignment exponent e. An element at angle delta from downwind has amplitude x * cos(delta/2)^(2e), so larger values keep the waves
//       closer to the wind direction. Zero gives the same amplitude in every direction.
//   z = crest sharpness of an element travelling exactly downwind, in [0, WaterFieldMaxCrestSharpness] (see WaterFieldWaveProfile). Other
//       elements are less sharp; see WindSeaSharpness.
//   w = sum over the band's directions of cos(delta/2)^(4e), for directions spread evenly over the full circle. The band's variance is
//       0.5 * x² * w, so x * x * w is twice the band's variance.
typedef float4 WindSeaTableRow;

// The state of one band's waves at a point.
struct WindSeaBand
{
	float2 direction; // Unit downwind direction, or zero in calm air.
	float amplitude;  // Amplitude (m) of an element travelling exactly downwind.
	float exponent;   // Alignment exponent; see WindSeaTableRow.
	float sharpness;  // Crest sharpness of an element travelling exactly downwind.
};

// Return the wind-sea band of 'element', or -1 when the element is not part of a wind sea and keeps its own amplitude.
odr int WaterFieldWindSeaBand(WaterFieldElement element)
{
	return element.info.y - 1;
}

// Return the cell of the wind grid containing 'world_xy', clamped to the grid. Indices are node indices; multiply by level_count for the wind data.
odr WaterBathymetryCell WindSeaLocate(WindSeaGrid grid, float2 world_xy)
{
	// The wind grid has the same node layout as a bathymetry grid.
	WaterBathymetryGrid nodes;
	nodes.origin = grid.origin;
	nodes.cell_size = grid.cell_size;
	nodes.pad = 0.0f;
	nodes.dims = grid.dims;
	nodes.pad2 = int2(0, 0);
	return WaterBathymetryLocate(nodes, world_xy);
}

// Return the bilinear blend of one band's wind at the four nodes of a located cell.
// Blending vectors makes the wind weaker where neighbouring nodes disagree on direction, so a sharp turn in the wind calms the waves between them.
odr float2 WindSeaBlendWind(WaterBathymetryCell cell, float2 w00, float2 w10, float2 w01, float2 w11)
{
	float2 lo = lerp(w00, w10, cell.frac.x);
	float2 hi = lerp(w01, w11, cell.frac.x);
	return lerp(lo, hi, cell.frac.y);
}

// Return the first of the two wind levels that 'band' interpolates between. The second level is in 'l1' and the fraction towards it in 'frac'.
// With a single level, both levels are level zero.
odr int WindSeaLevelIndex(WindSeaGrid grid, int band, out_(int) l1, out_(float) frac)
{
	// Spread the levels evenly from the first band to the last.
	float level = grid.band_count > 1 ? (float)(band * (grid.level_count - 1)) / (float)(grid.band_count - 1) : 0.0f;
	int l0 = min((int)level, max(grid.level_count - 2, 0));
	l1 = min(l0 + 1, grid.level_count - 1);
	frac = saturate(level - (float)l0);
	return l0;
}

// Return the index of the first of the two table rows that bracket 'speed' for 'band', and the fraction between them in 'frac'.
odr int WindSeaTableRowIndex(WindSeaGrid grid, int band, float speed, out_(float) frac)
{
	// Speeds beyond the table use its last row.
	float row = clamp(speed / grid.speed_step, 0.0f, (float)(grid.speed_count - 1));
	int r0 = min((int)row, grid.speed_count - 2);
	frac = saturate(row - (float)r0);
	return band * grid.speed_count + r0;
}

// Return the state of a band whose local wind is 'wind', given the two table rows at 'row0' and the next row, and the fraction between them.
odr WindSeaBand WindSeaBandState(float2 wind, WindSeaTableRow row0, WindSeaTableRow row1, float frac)
{
	// Interpolating the rows keeps every value continuous in the wind speed.
	WindSeaTableRow row = lerp(row0, row1, frac);
	float speed = length(wind);

	WindSeaBand band;
	band.direction = speed > 1.0e-6f ? wind / speed : float2(0.0f, 0.0f);
	band.amplitude = row.x;
	band.exponent = row.y;
	band.sharpness = row.z;
	return band;
}

// Return the amplitude of a wind-sea element travelling in unit 'direction' in a band whose state is 'band'.
odr float WindSeaAmplitude(float2 direction, WindSeaBand band)
{
	// cos(delta/2)^2 = (1 + cos(delta)) / 2. The small floor keeps pow finite for an element directly upwind, where the amplitude is zero anyway.
	float alignment = max(0.5f + 0.5f * dot(direction, band.direction), 1.0e-6f);
	return band.amplitude * pow(alignment, band.exponent);
}

// Return the crest sharpness of a wind-sea element travelling in unit 'direction' in a band whose state is 'band'.
// The sharpness is full downwind and falls quickly away from it to zero across the wind and upwind, so only waves the wind drives have peaked crests.
odr float WindSeaSharpness(float2 direction, WindSeaBand band)
{
	// cos(delta)^4 keeps most of the sharpness near downwind (0.56 at 30 degrees), then decays with a long tail (0.25 at 45, 0.06 at 60 degrees).
	// Sharp crests travelling across each other would otherwise form a visible lattice of crossing ridges. Calm air has no direction, so no sharpness.
	float c = saturate(dot(direction, band.direction));
	float c2 = c * c;
	return band.sharpness * c2 * c2;
}

// Return a copy of the wind-sea 'element' with the amplitude and crest sharpness of its band's state 'band'.
odr WaterFieldElement WindSeaElement(WaterFieldElement element, WindSeaBand band)
{
	// Only the amplitude and sharpness depend on the wind; the direction, wavelength, and phase are fixed.
	WaterFieldElement result = element;
	result.wave.x = WindSeaAmplitude(element.position.xy, band);
	result.timing.x = WindSeaSharpness(element.position.xy, band);
	return result;
}

#ifdef __cplusplus
	static_assert(sizeof(WindSeaGrid) == 48);
}
#endif
#endif
