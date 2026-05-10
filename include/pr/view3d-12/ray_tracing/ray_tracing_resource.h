//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Create a BLAS/TLAS result buffer in the only state that acceleration-structure resources can occupy.
	D3DPtr<ID3D12Resource> CreateRayTracingAccelerationStructure(ResourceFactory& factory, uint64_t size_in_bytes, std::string_view name);
}
