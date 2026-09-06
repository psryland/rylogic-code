//*********************************************
// Physics Engine — Shared Articulation Mobility Operations
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once

#ifdef __cplusplus
namespace PR_ARTICULATION_MOBILITY_OPS_CPP_NAMESPACE {
#endif

// Return a zeroed packed mobility.
GpuArticulationSpatialMobility MobilityZero()
{
	GpuArticulationSpatialMobility mobility;
	for (int index = 0; index != 6; ++index)
		mobility.packed[index] = float4(0.0f, 0.0f, 0.0f, 0.0f);
	return mobility;
}

// Return the packed upper-triangular scalar index for a symmetric matrix component.
int MobilityPackedIndex(int row, int column)
{
	int low = min(row, column);
	int high = max(row, column);
	return low * 6 - low * (low - 1) / 2 + high - low;
}

// Return one symmetric mobility component.
float MobilityComponent(GpuArticulationSpatialMobility mobility, int row, int column)
{
	int packed_index = MobilityPackedIndex(row, column);
	return mobility.packed[packed_index / 4][packed_index % 4];
}

// Store one upper-triangular mobility component.
void MobilitySetComponent(inout_(GpuArticulationSpatialMobility) mobility, int row, int column, float value)
{
	int packed_index = MobilityPackedIndex(row, column);
	mobility.packed[packed_index / 4][packed_index % 4] = value;
}

// Apply one packed force-to-motion mobility.
GpuArticulationSpatialVector MobilityApply(GpuArticulationSpatialMobility mobility, GpuArticulationSpatialVector force)
{
	GpuArticulationSpatialVector motion = AbaZeroSpatialVector();
	for (int column = 0; column != 6; ++column)
	{
		float force_component = AbaSpatialComponent(force, column);
		for (int row = 0; row != 6; ++row)
			AbaSetSpatialComponent(
				motion,
				row,
				AbaSpatialComponent(motion, row) + MobilityComponent(mobility, row, column) * force_component);
	}
	return motion;
}

#ifdef __cplusplus
}
using namespace PR_ARTICULATION_MOBILITY_OPS_CPP_NAMESPACE;
#endif
