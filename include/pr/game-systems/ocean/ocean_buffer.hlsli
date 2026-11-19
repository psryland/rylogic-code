//************************************
// Ocean Buffer
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Reads the water surface's procedural buffer: wave elements, bathymetry, the wind-sea table and winds, and the foam history (see OceanConstants
// for the layout). Samples the drawn waves, the crest foam they make, and the foam history. The includer declares 'g_ocean' (OceanConstants) and 'g_ocean_buffer' (a ByteAddressBuffer or
// RWByteAddressBuffer over the procedural buffer) before including this file.
#ifndef PR_GAME_SYSTEMS_OCEAN_BUFFER_HLSLI
#define PR_GAME_SYSTEMS_OCEAN_BUFFER_HLSLI
#include "pr/physics/terrain/water/water_field.hlsli"
#include "pr/physics/terrain/water/water_depth.hlsli"
#include "pr/game-systems/ocean/ocean_surface.hlsli"

// Read one wave element as stored, before the local wind sets its amplitude and crest sharpness.
WaterFieldElement OceanLoadElement(uint index)
{
	// Elements are 64 bytes each, starting at the beginning of the buffer.
	uint offset = index * 64;
	WaterFieldElement element;
	element.info = asint(g_ocean_buffer.Load4(offset + 0));
	element.position = asfloat(g_ocean_buffer.Load4(offset + 16));
	element.wave = asfloat(g_ocean_buffer.Load4(offset + 32));
	element.timing = asfloat(g_ocean_buffer.Load4(offset + 48));
	return element;
}

// Supplies the wind-sea winds and table rows from the buffer to OceanSeaAt.
struct OceanSeaData
{
	float2 Wind(uint index)
	{
		return asfloat(g_ocean_buffer.Load2(g_ocean.wind_sea_offsets.y + 8 * index));
	}
	float4 Row(uint index)
	{
		return asfloat(g_ocean_buffer.Load4(g_ocean.wind_sea_offsets.x + 16 * index));
	}
};

// Supplies the wave elements, with their local amplitudes and crest sharpness, to OceanWaves.
struct OceanBufferSource
{
	OceanSea sea;

	WaterFieldElement Element(uint index)
	{
		return OceanSeaElement(OceanLoadElement(index), sea);
	}
};

// Return the terrain height below 'world_xy' from the bathymetry grid.
float OceanTerrainHeight(float2 world_xy)
{
	// Interpolate the four surrounding nodes; positions outside the grid use the edge heights.
	WaterBathymetryGrid grid;
	grid.origin = g_ocean.bathymetry.xy;
	grid.cell_size = g_ocean.bathymetry.z;
	grid.pad = 0;
	grid.dims = int2(g_ocean.bathymetry_dims.xy);
	grid.pad2 = int2(0, 0);
	WaterBathymetryCell cell = WaterBathymetryLocate(grid, world_xy);
	uint base = g_ocean.layout.w;
	float h00 = asfloat(g_ocean_buffer.Load(base + 4 * cell.index.x));
	float h10 = asfloat(g_ocean_buffer.Load(base + 4 * cell.index.y));
	float h01 = asfloat(g_ocean_buffer.Load(base + 4 * cell.index.z));
	float h11 = asfloat(g_ocean_buffer.Load(base + 4 * cell.index.w));
	return WaterBathymetryInterpolate(cell, h00, h10, h01, h11);
}

// Return the drawn wave sample at the surface position 'world_xy', 'distance' metres horizontally from the camera, on a lattice with 'spacing'
// metres between vertices. Waves fade out towards the wave radius so the far surface is flat, and they vanish where the terrain is above the
// reach of the swash. A calm sea (see OceanCalm) is flat everywhere.
OceanWaveSample OceanSample(float2 world_xy, float distance, float spacing)
{
	// Positions beyond the wave radius, and a calm sea, are flat, so they need neither the terrain depth nor the local wind sea.
	float wave_fade = OceanWaveFade(distance, g_ocean.level_radii.w);
	if (wave_fade <= 0.0f || OceanCalm(g_ocean.waves.y))
		return OceanWaveSampleZero();

	float depth = g_ocean.level_radii.x - OceanTerrainHeight(world_xy);
	bool depth_effects = (g_ocean.layout.y & OceanFlagDepthEffects) != 0;
	OceanSeaData data;
	OceanBufferSource source;
	source.sea = OceanSeaAt(data, g_ocean.wind_sea, world_xy);
	return OceanWaves(source, g_ocean.layout.z, world_xy, g_ocean.camera_time.w, depth, g_ocean.waves.x, spacing, wave_fade, depth_effects, g_ocean.mesh.z, g_ocean.waves.z, g_ocean.shore_lip.z);
}

// Return the crest foam amount, in [0, 1], where the crest steepness is 'steepness' and the crest depth fade is 'depth_fade' (see OceanWaveSample).
// There is no foam up to the crest threshold, and the amount rises to the crest strength over the crest range above it.
float OceanCrestFoam(float steepness, float depth_fade)
{
	// Zero strength, or water too shallow for whitecaps, has no crest foam.
	float strength = g_ocean.foam_crest.x * depth_fade;
	if (strength <= 0.0f)
		return 0.0f;

	return strength * saturate((steepness - g_ocean.foam_crest.y) / g_ocean.foam_crest.z);
}

// Return the byte offset of grid cell 'cell' in slot 'slot' of foam history window 'window' (0 near, 1 far). See OceanFoamHistorySide.
uint OceanFoamHistoryAddress(uint window, uint slot, int2 cell)
{
	// Cells wrap around the window. The side is a power of two, so masking gives the non-negative remainder, even for negative cells.
	int mask = OceanFoamHistorySide - 1;
	int2 texel = cell & mask;
	uint slot_bytes = uint(OceanFoamHistorySide * OceanFoamHistorySide) * 4u;
	return g_ocean.foam_history.x + (2u * window + slot) * slot_bytes + uint(texel.y * OceanFoamHistorySide + texel.x) * 4u;
}

// Return the foam amount of grid cell 'cell' in slot 'slot' of window 'window', whose lowest corner is cell 'origin'. Cells outside the window have none.
float OceanFoamHistoryLoad(uint window, uint slot, int2 origin, int2 cell)
{
	// Texels hold only the cells inside the window; outside it they belong to other cells.
	int2 local = cell - origin;
	if (any(local < 0) || any(local >= OceanFoamHistorySide))
		return 0.0f;

	return asfloat(g_ocean_buffer.Load(OceanFoamHistoryAddress(window, slot, cell)));
}

// Return the bilinearly filtered foam amount at 'p', measured in cells of window 'window', from slot 'slot' of the window with lowest corner 'origin'.
// Cell 'c' covers [c, c + 1), so its value is at its centre.
float OceanFoamHistorySample(uint window, uint slot, int2 origin, float2 p)
{
	// Blend the four cells whose centres surround 'p'.
	float2 q = p - 0.5f;
	float2 base = floor(q);
	float2 f = q - base;
	int2 c = int2(base);
	float a = OceanFoamHistoryLoad(window, slot, origin, c);
	float b = OceanFoamHistoryLoad(window, slot, origin, c + int2(1, 0));
	float d = OceanFoamHistoryLoad(window, slot, origin, c + int2(0, 1));
	float e = OceanFoamHistoryLoad(window, slot, origin, c + int2(1, 1));
	return lerp(lerp(a, b, f.x), lerp(d, e, f.x), f.y);
}
#endif
