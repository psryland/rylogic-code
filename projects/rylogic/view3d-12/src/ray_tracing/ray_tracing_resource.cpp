//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/ray_tracing/ray_tracing_resource.h"
#include "pr/view3d-12/main/renderer.h"

namespace pr::rdr12
{
	using EFinalState = ::pr::compute::EFinalState;
	using EUsage = ::pr::compute::EUsage;
	using GfxUpdateSubresourceScope = ::pr::compute::GfxUpdateSubresourceScope;
	using BarrierBatch = ::pr::compute::BarrierBatch<D3D12_COMMAND_LIST_TYPE_DIRECT>;
	using ::pr::compute::DebugName;
	using ::pr::compute::DefaultResState;

	// Create a BLAS/TLAS result buffer in the only state that acceleration-structure resources can occupy.
	D3DPtr<ID3D12Resource> CreateRayTracingAccelerationStructure(Renderer& rdr, uint64_t size_in_bytes, std::string_view name)
	{
		D3DPtr<ID3D12Resource> res;
		auto desc = ResDesc::Buf(s_cast<int64_t>(size_in_bytes), 1, std::span<std::byte const>{}, 1)
			.usage(EUsage::UnorderedAccess)
			.misc_flags(ResDesc::EMiscFlags::RayTracingStruct)
			.def_state(D3D12_RESOURCE_STATE_RAYTRACING_ACCELERATION_STRUCTURE);

		Check(rdr.D3DDevice()->CreateCommittedResource(
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

	// Create a normal GPU resource used by ray tracing setup and record its initialisation into 'cmd_list'.
	D3DPtr<ID3D12Resource> CreateRayTracingResource(Renderer& rdr, GfxCmdList& cmd_list, GpuUploadBuffer& upload, ResDesc const& desc, std::string_view name)
	{
		if (desc.Width == 0)
			return {};

		D3DPtr<ID3D12Resource> res;
		auto rd = desc;
		if (desc.Dimension == D3D12_RESOURCE_DIMENSION_BUFFER)
			rd.Width *= desc.ElemStride;

		Check(rdr.D3DDevice()->CreateCommittedResource(
			&desc.HeapProps,
			desc.HeapFlags,
			&rd,
			D3D12_RESOURCE_STATE_COMMON,
			nullptr,
			__uuidof(ID3D12Resource),
			(void**)res.address_of()));

		DefaultResState(res.get(), D3D12_RESOURCE_STATE_COMMON);
		DebugName(res, name);

		// Initial data is uploaded through the frame upload buffer so the copy executes before the AS build that consumes it.
		if (!desc.Data.empty())
		{
			if (desc.MipLevels != 1)
				throw std::runtime_error("Ray tracing setup resources do not support generated mip maps");

			auto array_length = desc.Dimension == D3D12_RESOURCE_DIMENSION_TEXTURE3D ? 1 : s_cast<int>(desc.DepthOrArraySize);
			for (auto i = 0; i != array_length; ++i)
			{
				GfxUpdateSubresourceScope map(cmd_list, upload, res.get(), i, 0, 1, desc.DataAlignment);
				map.Write(desc.Data[i], AllSet(desc.MiscFlags, ResDesc::EMiscFlags::PartialInitData));
				map.Commit(EFinalState::Override, desc.DefaultState);
			}
		}
		else
		{
			BarrierBatch barriers(cmd_list);
			barriers.Transition(res.get(), desc.DefaultState);
			barriers.Commit();
		}

		DefaultResState(res.get(), desc.DefaultState);
		return res;
	}
}
