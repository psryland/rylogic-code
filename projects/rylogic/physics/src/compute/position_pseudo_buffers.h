//*********************************************
// Physics Engine
//  Copyright (C) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "src/utility/gpu.h"

namespace pr::physics
{
	// UAVs for world-space rigid and link-local articulated position-only velocities.
	struct GpuPositionPseudoBuffers
	{
		D3DPtr<ID3D12Resource> m_rigid_velocities;
		D3DPtr<ID3D12Resource> m_link_velocities;
		D3DPtr<ID3D12Resource> m_generalized_velocities;
	};
}
