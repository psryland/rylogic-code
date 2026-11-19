//************************************
// Ocean Surface Forward Pixel (default)
//  Copyright (c) Rylogic Ltd 2026
//************************************
// The ocean's default forward pixel shaders, for callers that have no shore height source. The water has no shore depth, so it has no shore
// lip or shore foam. Callers with a ground height source compile their own pixel shaders from 'ocean_forward_ps.hlsli'.

// Report that no ground is known below the water.
float OceanShoreHeight(float2 world_xy, uint4 shore_source[3])
{
	// A quiet NaN means "unknown ground height".
	return asfloat(0x7FC00000u);
}

#include "pr/game-systems/ocean/ocean_forward_ps.hlsli"
