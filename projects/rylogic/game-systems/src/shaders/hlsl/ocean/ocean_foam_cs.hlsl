//************************************
// Ocean Foam History
//  Copyright (c) Rylogic Ltd 2026
//************************************
// Updates the water surface's foam history, the crest foam that passing crests leave behind (see OceanFoamHistorySide).
//
// Each thread updates one texel of one window: it fades the previous foam where it formed, and adds the crest foam of the waves at the
// texel now. The foam fades at a constant rate, so a trail behind a crest thins from dense foam to lace to fine lines in the foam texture before
// it vanishes (see ocean_forward_ps.hlsli). The update reads the slot that is not 'foam_history.y' and writes slot 'foam_history.y'.
#include "pr/game-systems/ocean/ocean_surface.hlsli"

ConstantBuffer<OceanConstants> g_ocean : register(b0);

// Wind-sea wave elements, bathymetry heights, the wind-sea table and winds, and the foam history. See OceanConstants for the layout.
RWByteAddressBuffer g_ocean_buffer : register(u0);
#include "pr/game-systems/ocean/ocean_buffer.hlsli"

// Update texel 'id.xy' of window 'id.z' (0 near, 1 far).
numthreads(Main, 8, 8, 1)
void Main(uint3 id : SV_DispatchThreadID)
{
	// Find the grid cell that this texel holds. The window wraps, so the cell is the one inside the window whose position modulo the side is the texel.
	uint window = id.z;
	int2 origin = window == 0 ? g_ocean.foam_origins.xy : g_ocean.foam_origins.zw;
	int2 prev_origin = window == 0 ? g_ocean.foam_prev_origins.xy : g_ocean.foam_prev_origins.zw;
	float texel = window == 0 ? OceanFoamNearTexel : OceanFoamFarTexel;
	int2 cell = origin + ((int2(id.xy) - origin) & (OceanFoamHistorySide - 1));
	float2 world_xy = (float2(cell) + 0.5f) * texel;

	// Read the cell's foam at the previous update; cells outside the previous window had no history. The foam then fades at the fade rate.
	uint slot = g_ocean.foam_history.y;
	float dt = g_ocean.foam_fade.x;
	float foam = OceanFoamHistorySample(window, 1u - slot, prev_origin, float2(cell) + 0.5f);
	foam = max(foam - g_ocean.foam_fade.y * dt, 0.0f);

	// Crests passing the cell now renew its foam. Waves shorter than a few texels are filtered out, as for a lattice with the texel spacing.
	// Without crest foam the trails only fade, so the costly wave sample is skipped.
	if (g_ocean.foam_crest.x > 0.0f)
	{
		float distance = length(world_xy - g_ocean.camera_time.xy);
		OceanWaveSample waves = OceanSample(world_xy, distance, texel);
		foam = max(foam, OceanCrestFoam(waves.crest_steepness, waves.crest_depth_fade));
	}
	g_ocean_buffer.Store(OceanFoamHistoryAddress(window, slot, cell), asuint(foam));
}
