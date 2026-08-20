//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
// Algorithm:
//  1. Find the cell hash value for each position
//  2. Build a histogram of positions per cell
//  3. Sort the 'm_grid_hash' buffer by hash value so that all positions in the same cell are contiguous
//  4. Create a lookup from cell hash to the start index of the cell in the sorted buffer
#include "src/compute/particle_collision/particle.hlsli"
#include "src/compute/spatial_partition/spatial_partition.hlsli"

static const uint ThreadGroupSize = 1024;

// Constants
cbuffer cbGridPartition : register(b0)
{
	// The length of 'm_positions'
	int NumPositions;

	// The maximum number of grid cells.
	// The last cell is reserved for NaN particles, so size must be >= 2
	// Primes + 1 are a good choice: 1021 + 1, 65521 + 1, 1048573 + 1, 16777213 + 1
	int CellCount;

	// The quantising factor to apply to the positions
	float GridScale;
	
	int pad;
};

// The positions to sort into the grid
StructuredBuffer<PositionType> m_positions : register(t0);

// The grid cell hash for each position. (length of m_positions)
RWStructuredBuffer<uint> m_grid_hash : register(u0);

// The spatially sorted index buffer (length m_positions)
RWStructuredBuffer<uint> m_spatial : register(u1);

// The lowest index (from m_positions) for each cell hash (length CellCount)
RWStructuredBuffer<uint> m_idx_start : register(u2);

// The number of positions for each cell hash (length CellCount)
RWStructuredBuffer<uint> m_idx_count : register(u3);

// Reset the start/count arrays
[numthreads(ThreadGroupSize, 1, 1)]
void Init(uint3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= CellCount)
		return;
	
	m_idx_start[dtid.x] = 0xFFFFFFFF;
	m_idx_count[dtid.x] = 0;
}

// Populate the grid hash buffer with the hash value for each position
[numthreads(ThreadGroupSize, 1, 1)]
void CalculateHashes(uint3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= NumPositions)
		return;

	// Handle 'nan' positions by placing them in the last cell
	int3 grid = GridCell(m_positions[dtid.x].pos, GridScale);
	uint hash = any(isnan(m_positions[dtid.x].pos)) ? CellCount - 1 : CellHash(grid, CellCount);
	m_grid_hash[dtid.x] = hash;
	m_spatial[dtid.x] = dtid.x;
}

// Build the lookup structure (run post-sort)
[numthreads(ThreadGroupSize, 1, 1)]
void BuildLookup(uint3 dtid : SV_DispatchThreadID)
{
	if (dtid.x >= NumPositions)
		return;

	uint hash = m_grid_hash[dtid.x];
	
	// Record the smallest index for each cell hash value
	InterlockedMin(m_idx_start[hash], dtid.x);
	
	// Record the number of positions for each cell hash value
	InterlockedAdd(m_idx_count[hash], 1);
}
