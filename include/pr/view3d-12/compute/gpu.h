// HyperPose Tools
// Copyright (c) 2025
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/utility/gpu_sync.h"
#include "pr/view3d-12/utility/cmd_alloc.h"
#include "pr/view3d-12/utility/cmd_list.h"
#include "pr/view3d-12/resource/gpu_transfer_buffer.h"

namespace pr::rdr12
{
	// A DirectX 12 instance for running compute shaders
	template <D3D12_COMMAND_LIST_TYPE ListType>
	class Gpu
	{
		// Notes:
		//  - Use this type when you want to run compute shaders on the GPU but don't
		//    want to create a full 'Renderer' instance.

		// D3D Device and command queue
		D3DPtr<ID3D12Device4> m_device;
		D3DPtr<ID3D12CommandQueue> m_cmd_queue;

		// GPU fence helper
		GpuSync m_gsync;

		// Command lists/allocators
		CmdAllocPool<ListType> m_cmd_alloc_pool;
		CmdListPool<ListType> m_cmd_list_pool;

		// Upload memory buffer for initialising resources
		GpuUploadBuffer m_upload_buffer;
		
	public:

		Gpu(ID3D12Device4* existing_device, ID3D12CommandQueue* existing_queue);
		Gpu(ID3D12Device4* existing_device = nullptr)
			: Gpu(existing_device, nullptr)
		{}

		ID3D12Device4 const* device() const
		{
			return m_device.get();
		}
		ID3D12Device4* device()
		{
			return m_device.get();
		}

		ID3D12CommandQueue const* queue() const
		{
			return m_cmd_queue.get();
		}
		ID3D12CommandQueue* queue()
		{
			return m_cmd_queue.get();
		}

		// Allow use as a device
		ID3D12Device4 const* operator -> () const
		{
			return device();
		}
		ID3D12Device4* operator ->()
		{
			return device();
		}
		operator ID3D12Device4 const* () const
		{
			return device();
		}
		operator ID3D12Device4* ()
		{
			return device();
		}

		// Access the GPU upload buffer
		GpuUploadBuffer& UploadBuffer()
		{
			return m_upload_buffer;
		}

		// Allocate a DX resource
		D3DPtr<ID3D12Resource> CreateResource(ResDesc const& desc, CmdList<ListType>& cmd_list, std::string_view name);
	};

	using GfxGpu = Gpu<D3D12_COMMAND_LIST_TYPE_DIRECT>;
	using ComGpu = Gpu<D3D12_COMMAND_LIST_TYPE_COMPUTE>;
}
