//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/utility/wrappers.h"
#include "pr/compute/utility/gpu_sync.h"

namespace pr::compute
{
	// A 'page' in the upload buffer
	struct GpuTransferBufferPage
	{
		D3DPtr<ID3D12Resource> m_res; // The upload buffer resource
		uint8_t* m_mem;               // The mapped CPU memory
		int64_t m_capacity;           // The sizeof the resource buffer
		int64_t m_size;               // The consumed space in this block
		int64_t m_ref_count;          // Track the number of external references to this block
		uint64_t m_sync_point;        // The highest sync point recorded while this was the head block

		GpuTransferBufferPage()
			: m_res()
			, m_mem()
			, m_capacity()
			, m_size()
			, m_ref_count()
			, m_sync_point()
		{
		}
		GpuTransferBufferPage(ID3D12Device* device, D3D12_HEAP_TYPE heap_type, int64_t size, int alignment, int64_t sync_point)
			: m_res()
			, m_mem()
			, m_capacity(size)
			, m_size(0)
			, m_ref_count(0)
			, m_sync_point(sync_point)
		{
			HeapProps heap_props(heap_type);
			ResDesc desc = ResDesc::Buf(size, 1, {}, alignment).def_state(D3D12_RESOURCE_STATE_GENERIC_READ);
			assert(desc.Check());

			// Create the transfer buffer resource
			Check(device->CreateCommittedResource(
				&heap_props, D3D12_HEAP_FLAG_NONE,
				&desc, D3D12_RESOURCE_STATE_COMMON, nullptr,
				__uuidof(ID3D12Resource), (void**)m_res.address_of()));
			Check(m_res->SetName(L"GpuTransferBuffer:Block"));

			// Upload buffers can live mapped
			Check(m_res->Map(0U, nullptr, (void**)&m_mem));
		}
		GpuTransferBufferPage(GpuTransferBufferPage&&) = default;
		GpuTransferBufferPage(GpuTransferBufferPage const&) = delete;
		GpuTransferBufferPage& operator=(GpuTransferBufferPage&&) = default;
		GpuTransferBufferPage& operator=(GpuTransferBufferPage const&) = delete;
		~GpuTransferBufferPage()
		{
			if (m_res != nullptr)
				m_res->Unmap(0U, nullptr);
		}

		// Remaining free space in the page
		int64_t free() const
		{
			return m_capacity - m_size;
		}
	};
}

