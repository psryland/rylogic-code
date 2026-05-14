//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/gpu_transfer_buffer.h"
#include "pr/view3d-12/utility/cmd_list.h"

namespace pr::rdr12
{
	// Create a BLAS/TLAS result buffer in the only state that acceleration-structure resources can occupy.
	D3DPtr<ID3D12Resource> CreateRayTracingAccelerationStructure(Renderer& rdr, uint64_t size_in_bytes, std::string_view name);

	// Create a normal GPU resource used by ray tracing setup and record its initialisation into 'cmd_list'.
	D3DPtr<ID3D12Resource> CreateRayTracingResource(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, ResDesc const& desc, std::string_view name);
}
