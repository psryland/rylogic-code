//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ray_tracing/ray_tracing_resource.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/resource/resource_factory.h"

namespace pr::rdr12
{
	// Create a BLAS/TLAS result buffer in the only state that acceleration-structure resources can occupy.
	D3DPtr<ID3D12Resource> CreateRayTracingAccelerationStructure(ResourceFactory& factory, uint64_t size_in_bytes, std::string_view name)
	{
		D3DPtr<ID3D12Resource> res;
		auto desc = ResDesc::Buf(s_cast<int64_t>(size_in_bytes), 1, std::span<std::byte const>{}, 1)
			.usage(EUsage::UnorderedAccess)
			.misc_flags(ResDesc::EMiscFlags::RayTracingStruct)
			.def_state(D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE);

		Check(factory.rdr().D3DDevice()->CreateCommittedResource(
			&desc.HeapProps,
			desc.HeapFlags,
			&desc,
			desc.DefaultState,
			nullptr,
			__uuidof(ID3D12Resource),
			(void**)res.address_of()));

		DefaultResState(res.get(), desc.DefaultState);
		DebugName(res, name);
		return res;
	}
}
