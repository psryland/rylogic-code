//*********************************************
// Physics Terrain
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#ifndef PR_TERRAIN_TERRAIN_DECALS_HLSLI
#define PR_TERRAIN_TERRAIN_DECALS_HLSLI
#include "pr/physics/terrain/landscape/baseline_surface.hlsli"
#ifdef __cplusplus
namespace pr::physics::terrain::landscape::shared
{
#endif

// Terrain decals are circular height overrides on the baseline surface. Each decal holds its target height inside 'pad radius' and blends back to the
// surface across the ring out to 'blend radius'. A decal's weight is w = s(t), where t = (blend - distance) / (blend - pad) clamped to [0, 1], and
// s(t) = t²(3 - 2t). Overlapping decals combine as h = P·h_surface + (1 - P)·(Σ wᵢhᵢ / Σ wᵢ), with P = Π(1 - wᵢ). Inside any single pad P is zero, so the
// height is exactly the pad height when no other decal reaches that point. Order does not matter, and the height and its derivatives are continuous.

// Byte layout of the single-precision local decal blob prepared by BaselineSurface::LocalDecals(). All members are 32-bit scalars, read from a
// ByteAddressBuffer. The blob is a header, a grid of cells, an index list, then the decal records. Positions are relative to the blob's origin.
struct BaselineDecalLocalHeader
{
	uint m_decal_count;     // Number of decal records. Zero means no decals, and nothing else is valid.
	uint m_cells_x;         // Grid cells along X.
	uint m_cells_y;         // Grid cells along Y.
	float m_cell_size;      // Grid cell side length in metres.
	float m_grid_origin_x;  // Local X of the grid's minimum corner.
	float m_grid_origin_y;  // Local Y of the grid's minimum corner.
	uint m_indices_offset;  // Byte offset of the index list from the start of the blob.
	uint m_decals_offset;   // Byte offset of the decal records from the start of the blob.
};

// One decal in the local blob. 'm_height' is absolute (the same datum as BaselineLocalResult::m_height). 'm_user' is the caller's value, unchanged.
struct BaselineDecalLocalRecord
{
	float m_centre_x;
	float m_centre_y;
	float m_pad_radius;
	float m_blend_radius;
	float m_height;
	uint m_user;
	uint m_reserved0;
	uint m_reserved1;
};

// Byte sizes of the blob parts. Each cell is a (first index, index count) pair into the index list; each index is one decal record number.
static const uint BaselineDecalHeaderSize = 32;
static const uint BaselineDecalCellSize = 8;
static const uint BaselineDecalRecordSize = 32;

// Running sums for combining every decal that reaches one sample. 'm_keep' is P, 'm_weight' is Σ wᵢ and 'm_weighted_height' is Σ wᵢhᵢ.
struct BaselineDecalBlend
{
	BaselineField m_keep;
	BaselineField m_weight;
	BaselineField m_weighted_height;
};

// Return the blend state for a sample that no decal reaches.
inline BaselineDecalBlend BaselineDecalBlendInit()
{
	BaselineDecalBlend blend;
	blend.m_keep = BaselineConstant(1);
	blend.m_weight = BaselineConstant(0);
	blend.m_weighted_height = BaselineConstant(0);
	return blend;
}

// Add one decal to 'blend'. ('dx', 'dy') is the sample position minus the decal centre. Callers guarantee 0 <= pad_radius < blend_radius.
inline BaselineDecalBlend BaselineDecalAccumulate(BaselineDecalBlend blend, BaselineReal dx, BaselineReal dy, BaselineReal pad_radius, BaselineReal blend_radius, BaselineReal height)
{
	// Samples on or beyond the blend radius have zero weight.
	BaselineReal distance_sq = dx * dx + dy * dy;
	if (!(distance_sq < blend_radius * blend_radius))
		return blend;

	// The weight is one inside the pad. In the ring it is the smooth step of the normalised distance, whose gradient points towards the centre.
	BaselineField weight = BaselineConstant(1);
#if !defined(__cplusplus) && defined(PR_TERRAIN_FP32) && PR_TERRAIN_FP32
	BaselineReal distance = sqrt(distance_sq);
#else
	BaselineReal distance = (BaselineReal)BaselineSqrt(distance_sq);
#endif
	if (distance > pad_radius)
	{
		BaselineReal width = blend_radius - pad_radius;
		BaselineReal t = (blend_radius - distance) / width;
		BaselineReal slope = (6 * t * (1 - t)) / (width * distance);
		weight = BaselineField(t * t * (3 - 2 * t), -slope * dx, -slope * dy);
	}

	// Combine with the product rule so derivatives stay analytic.
	blend.m_keep = BaselineProduct(blend.m_keep, BaselineConstant(1) - weight);
	blend.m_weight = blend.m_weight + weight;
	blend.m_weighted_height = blend.m_weighted_height + height * weight;
	return blend;
}

// Apply the accumulated decals to the undecorated surface field 'surface' (absolute height, d/dx, d/dy).
inline BaselineField BaselineDecalResolve(BaselineDecalBlend blend, BaselineField surface)
{
	// No decal reached the sample.
	if (!(blend.m_weight.x > 0))
		return surface;

	// The weighted mean of the decal heights, with the quotient rule for its derivatives.
	BaselineReal sum = blend.m_weight.x;
	BaselineReal mean = blend.m_weighted_height.x / sum;
	BaselineField target = BaselineField(mean,
		(blend.m_weighted_height.y - mean * blend.m_weight.y) / sum,
		(blend.m_weighted_height.z - mean * blend.m_weight.z) / sum);

	// Keep the surface share P and give the rest to the decal heights.
	return BaselineProduct(blend.m_keep, surface) + BaselineProduct(BaselineConstant(1) - blend.m_keep, target);
}

#ifdef __cplusplus
	static_assert(sizeof(BaselineDecalLocalHeader) == BaselineDecalHeaderSize && offsetof(BaselineDecalLocalHeader, m_decals_offset) == 28);
	static_assert(sizeof(BaselineDecalLocalRecord) == BaselineDecalRecordSize && offsetof(BaselineDecalLocalRecord, m_user) == 20);
	static_assert(std::is_trivially_copyable_v<BaselineDecalLocalHeader> && std::is_trivially_copyable_v<BaselineDecalLocalRecord>);
}
#endif
#endif
