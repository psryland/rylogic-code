//*********************************************
// Physics Terrain Water
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
// Stage-neutral water-depth corrections for wave elements, shared by C++ and HLSL.
// Waves entering shallow water change height (shoaling) and break when they become too tall for the depth.
// These helpers scale element amplitudes by the local water depth so the unchanged evaluators in water_field.hlsli can be reused:
//   1. For every element, add Amplitude * WaterFieldShoaling(element, depth) to a running total.
//   2. Compute scale = WaterFieldBreakingScale(total, depth, breaking_ratio).
//   3. Evaluate WaterFieldDepthCorrected(element, depth, scale) instead of the element itself.
// The depth is the still-water level minus the terrain height at the sample point, plus the swash allowance (see WaterFieldSwashDepth).
// A very large depth (for example 1e30) disables every correction.
// The wavelength, direction, and speed are not changed (no refraction). Surface slopes and velocities use the corrected amplitude at the sample point
// but ignore the rate at which the amplitude changes with position; this is accurate when the depth changes slowly over one wavelength.
#ifndef PR_PHYSICS_WATER_DEPTH_HLSLI
#define PR_PHYSICS_WATER_DEPTH_HLSLI
#include "pr/hlsl/interop.hlsli"
#include "pr/hlsl/core.hlsli"
#include "pr/physics/terrain/water/water_field_types.hlsli"

#ifdef __cplusplus
namespace pr::physics::terrain::water::shared
{
#endif

// Depth value that disables all depth corrections.
static const float WaterFieldDeepWater = 1.0e30f;

// Largest amplitude gain from shoaling. Linear theory grows without limit as the depth goes to zero, but breaking limits real waves first.
static const float WaterFieldMaxShoaling = 2.0f;

// Ratio of the swash allowance to the significant wave amplitude. See WaterFieldSwashDepth.
static const float WaterFieldSwashRatio = 2.0f;

// Return the swash allowance for elements whose squared amplitudes add up to 'amplitude_squares'.
// Real waves run up and down a beach, so the waterline moves with them. Adding this allowance to the still-water depth before the depth
// corrections keeps waves alive at the still shoreline and lets them reach dry ground up to this height above the still level. The surface
// there is below the terrain except where a crest rises above the ground, so the waterline moves with the waves. The allowance is the
// significant amplitude (half the significant wave height, sqrt(2 * sum of A²)) times WaterFieldSwashRatio, so calm water keeps a still shoreline.
odr float WaterFieldSwashDepth(float amplitude_squares)
{
	return WaterFieldSwashRatio * sqrt(2.0f * max(amplitude_squares, 0.0f));
}

// Return the amplitude gain of a wave with wave number 'k' (rad/m) in water of 'depth' metres.
// The gain keeps the wave's energy flux constant as the wave slows down: it is 1 in deep water, dips slightly below 1 at intermediate depth,
// and grows as the fourth root of 1/depth in shallow water. It is zero on dry land (depth <= 0).
// Linear theory uses the local (shorter) wavelength in shallow water. Here the deep-water wave number is used, which gives the correct deep and shallow limits.
odr float WaterFieldShoalingGain(float k, float depth)
{
	// Waves cannot exist without water, and in deep water the gain is exactly one.
	if (depth <= 0.0f)
		return 0.0f;

	float kh = k * depth;
	if (kh > 10.0f)
		return 1.0f;

	// Group speed ratio n and phase speed ratio sqrt(tanh(kh)) relative to deep water, which has n = 0.5.
	float n = 0.5f * (1.0f + 2.0f * kh / sinh(2.0f * kh));
	float speed_ratio = sqrt(tanh(kh));
	return min(sqrt(0.5f / (n * speed_ratio)), WaterFieldMaxShoaling);
}

// Return the shoaling gain for one element. Only sine and Gerstner waves shoal; other elements keep their amplitude.
odr float WaterFieldShoaling(WaterFieldElement element, float depth)
{
	switch (element.info.x)
	{
		case WaterFieldElementSineWave:
		case WaterFieldElementGerstnerWave:
		{
			return WaterFieldShoalingGain(tau / element.wave.y, depth);
		}
		case WaterFieldElementRadialPacket:
		{
			return depth > 0.0f ? 1.0f : 0.0f;
		}
		default:
		{
			return 0.0f;
		}
	}
}

// Return the factor that limits the total shoaled amplitude to a breaking wave height.
// A wave breaks when its crest-to-trough height exceeds 'breaking_ratio' * depth (about 0.78 for real waves). The amplitude is half the height,
// so every element is scaled down by the same factor until the sum of amplitudes is at most 0.5 * breaking_ratio * depth.
odr float WaterFieldBreakingScale(float total_amplitude, float depth, float breaking_ratio)
{
	// No waves, or no water, need no limiting.
	float limit = 0.5f * breaking_ratio * max(depth, 0.0f);
	return total_amplitude > limit ? limit / total_amplitude : 1.0f;
}

// Return a copy of 'element' with its amplitude corrected for 'depth' and limited by 'breaking_scale'.
odr WaterFieldElement WaterFieldDepthCorrected(WaterFieldElement element, float depth, float breaking_scale)
{
	// The evaluators read the amplitude from wave.x for every element type.
	WaterFieldElement result = element;
	result.wave.x *= WaterFieldShoaling(element, depth) * breaking_scale;
	return result;
}

// Return how close waves of 'total_amplitude' are to breaking in water 'depth' deep, from zero (half the breaking height or less) to one (breaking).
// Renderers use it to sharpen crests as waves approach the breaking height, see WaterFieldGerstnerSteepness.
odr float WaterFieldBreakingCloseness(float total_amplitude, float depth, float breaking_ratio)
{
	// Closeness grows over the last half of the range up to the breaking height.
	float limit = 0.5f * breaking_ratio * max(depth, 0.0f);
	return limit > 0.0f ? saturate(2.0f * total_amplitude / limit - 1.0f) : 0.0f;
}

// Return the per-element Gerstner steepness for a sum of waves whose amplitudes add up to 'total_amplitude' and whose slopes k*A add up to
// 'total_slope', in water 'depth' metres deep. 'steepness' in [0,1] scales the horizontal displacement of each element relative to its amplitude;
// one gives trochoid-shaped crests, so crests sharpen in proportion to how steep the waves really are and small waves stay smooth.
// The surface folds over when the sum of k*A*steepness exceeds one, which sets the fold limit 1/total_slope. As 'breaking_closeness' rises
// to one the steepness moves to the fold limit, so waves visibly sharpen before they break. The result never exceeds the fold limit, and the
// total horizontal displacement never exceeds 'depth'. The displacement therefore reaches zero where the waves end, and it changes across the
// surface no faster than the depth does, so displaced water meets the flat surface beyond without folding.
odr float WaterFieldGerstnerSteepness(float steepness, float breaking_closeness, float total_slope, float total_amplitude, float depth)
{
	// A flat surface has unbounded limits; the tiny floors keep the divisions finite.
	float fold_limit = 1.0f / max(total_slope, 1.0e-6f);
	float depth_limit = max(depth, 0.0f) / max(total_amplitude, 1.0e-6f);
	return min(min(lerp(steepness, fold_limit, breaking_closeness), fold_limit), depth_limit);
}

// A regular grid of terrain heights covering part of the world XY plane. Node (i, j) is at origin + cell_size * (i, j).
// Node heights are stored by the consumer in row-major order (j * dims.x + i). Positions outside the grid use the nearest edge value.
struct WaterBathymetryGrid
{
	float2 origin;
	float cell_size;
	float pad;
	int2 dims;
	int2 pad2;
};

// The four nodes surrounding a position, and the position's fraction across the cell.
struct WaterBathymetryCell
{
	int4 index; // Row-major node indices of (i,j), (i+1,j), (i,j+1), (i+1,j+1).
	float2 frac;
};

// Return the cell containing 'world_xy', clamped to the grid.
odr WaterBathymetryCell WaterBathymetryLocate(WaterBathymetryGrid grid, float2 world_xy)
{
	// Clamp to the grid so positions outside it use the edge heights.
	float2 max_node = float2((float)(grid.dims.x - 1), (float)(grid.dims.y - 1));
	float2 node = clamp((world_xy - grid.origin) / grid.cell_size, float2(0.0f, 0.0f), max_node);
	int i0 = min((int)node.x, grid.dims.x - 2);
	int j0 = min((int)node.y, grid.dims.y - 2);
	i0 = max(i0, 0);
	j0 = max(j0, 0);
	int i1 = min(i0 + 1, grid.dims.x - 1);
	int j1 = min(j0 + 1, grid.dims.y - 1);

	WaterBathymetryCell cell;
	cell.index = int4(j0 * grid.dims.x + i0, j0 * grid.dims.x + i1, j1 * grid.dims.x + i0, j1 * grid.dims.x + i1);
	cell.frac = saturate(node - float2((float)i0, (float)j0));
	return cell;
}

// Return the bilinear interpolation of the four node heights of a located cell.
odr float WaterBathymetryInterpolate(WaterBathymetryCell cell, float h00, float h10, float h01, float h11)
{
	float lo = lerp(h00, h10, cell.frac.x);
	float hi = lerp(h01, h11, cell.frac.x);
	return lerp(lo, hi, cell.frac.y);
}

#ifdef __cplusplus
	static_assert(sizeof(WaterBathymetryGrid) == 32);
}
#endif
#endif
