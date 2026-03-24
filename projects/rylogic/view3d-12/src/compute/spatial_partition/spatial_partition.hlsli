// Spatial partition
#ifndef PR_COMPUTE_SPATIAL_PARTITION_HLSLI
#define PR_COMPUTE_SPATIAL_PARTITION_HLSLI

#include "pr/hlsl/core.hlsli"

// Convert a floating point position into a grid cell coordinate
inline int3 GridCell(float4 position, uniform float grid_scale)
{
	return int3(ceil(position.xyz * grid_scale));
}

// Generate a hash from a grid cell coordinate.
inline uint CellHash(int3 grid, uniform uint cell_count)
{
	// This performs better than using the same prime for each component
	uint h1 = Hash(grid.x);
	uint h2 = Hash(grid.y);
	uint h3 = Hash(grid.z);
	const uint prime1 = 73856093;
	const uint prime2 = 19349663;
	const uint prime3 = 83492791;
	
	// The last cell is reserved for 'nan' positions
	return (h1 * prime1 + h2 * prime2 + h3 * prime3) % (cell_count - 1);
}

// Return the index range for a given cell hash
inline int2 IndexRange(uint cell_hash, RWStructuredBuffer<uint> idx_start, RWStructuredBuffer<uint> idx_count)
{
	return int2(idx_start[cell_hash], idx_start[cell_hash] + idx_count[cell_hash]);
}

// A coroutine context for iterating over neighbouring particles
struct FindIter
{
	int3 cell;      // Current cell
	int3 lwr;       // Inclusive lower bound
	int3 upr;       // Exclusive upper bound
	int2 idx_range; // Range of indices in the spatially sorted index buffer
};

// Find all particles within 'radius' of 'position'
inline FindIter Find(float4 position, float4 radius, uniform float grid_scale, uniform uint cell_count)
{
	FindIter iter;
	iter.lwr = GridCell(position - radius, grid_scale);
	iter.upr = GridCell(position + radius, grid_scale) + uint3(1, 1, 1);
	iter.cell = uint3(iter.lwr.x - 1, iter.lwr.y, iter.lwr.z);
	iter.idx_range = int2(0, 0);
	return iter;
}

// Advance the find iterator to the next cell. On return the 'idx_range' field contains the range
// of indices in the spatially sorted index buffer.
inline bool DoFind(inout FindIter iter, uniform uint cell_count, RWStructuredBuffer<uint> idx_start, RWStructuredBuffer<uint> idx_count)
{
	// Advance to the next cell
	++iter.cell.x;
	if (iter.cell.x == iter.upr.x)
	{
		++iter.cell.y;
		iter.cell.x = iter.lwr.x;
		if (iter.cell.y == iter.upr.y)
		{
			++iter.cell.z;
			iter.cell.y = iter.lwr.y;
			if (iter.cell.z == iter.upr.z)
				return false;
		}
	}

	// Get the range of indices in 'cell'
	uint cell_hash = CellHash(iter.cell, cell_count);
	iter.idx_range = IndexRange(cell_hash, idx_start, idx_count);
	return true;
}

#endif
