//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/view3d-12/resource/gpu_transfer_buffer_page.h"

namespace pr::rdr12
{
	// An allocation in a GpuTransferBuffer
	struct GpuTransferAllocation
	{
		// Notes:
		//  - The allocation is a linear block of memory, but for images can be interpreted as an array of mips.
		//  - Allocations are reference counted, so as long as a reference to the allocation exists then it will
		//    stay alive. However, recycling of the memory page will be blocked by long lived allocations.
		//  - Allocations are automatically reclaimed whenever 'AddSyncPoint' is called on the GpuSync object
		//    referenced by the owning GpuTransferBuffer.

		GpuTransferBufferPage* m_page; // The memory page this allocation is from
		ID3D12Resource* m_res; // The upload resource that contains this allocation.
		uint8_t* m_mem;        // The system memory address, mapped to m_res->GetGPUAddress().
		int64_t m_ofs;         // The offset from 'm_mem' (aka 'm_buf->GetGPUAddress()') to the start of the allocation.
		int64_t m_size;        // The size of the allocation (in bytes).

		GpuTransferAllocation()
			:GpuTransferAllocation(nullptr, nullptr, nullptr, 0, 0)
		{
		}
		GpuTransferAllocation(GpuTransferBufferPage* page, ID3D12Resource* res, uint8_t* mem, int64_t ofs, int64_t size)
			: m_page(page)
			, m_res(res)
			, m_mem(mem)
			, m_ofs(ofs)
			, m_size(size)
		{
			if (m_page)
				++m_page->m_ref_count;
		}
		GpuTransferAllocation(GpuTransferAllocation&& rhs) noexcept
			: GpuTransferAllocation()
		{
			std::swap(m_page, rhs.m_page);
			std::swap(m_res, rhs.m_res);
			std::swap(m_mem, rhs.m_mem);
			std::swap(m_ofs, rhs.m_ofs);
			std::swap(m_size, rhs.m_size);
		}
		GpuTransferAllocation(GpuTransferAllocation const&) = delete;
		GpuTransferAllocation& operator=(GpuTransferAllocation&& rhs) noexcept
		{
			std::swap(m_page, rhs.m_page);
			std::swap(m_res, rhs.m_res);
			std::swap(m_mem, rhs.m_mem);
			std::swap(m_ofs, rhs.m_ofs);
			std::swap(m_size, rhs.m_size);
			return *this;
		}
		GpuTransferAllocation& operator=(GpuTransferAllocation const& rhs) = delete;
		~GpuTransferAllocation()
		{
			if (m_page)
				--m_page->m_ref_count;
		}

		template <typename T> std::span<T const> span(int64_t start = 0, int64_t count = std::numeric_limits<int64_t>::max()) const
		{
			assert(is_valid_range<T>(start, count));
			return std::span<T const>{ ptr<T>() + start, s_cast<size_t>(std::min<int64_t>(count, m_size / sizeof(T))) };
		}
		template <typename T> std::span<T> span(int64_t start = 0, int64_t count = std::numeric_limits<int64_t>::max())
		{
			assert(is_valid_range<T>(start, count));
			return std::span<T>{ ptr<T>() + start, s_cast<size_t>(std::min<int64_t>(count, m_size / sizeof(T))) };
		}

		template <typename T> T const* ptr(int64_t offset = 0) const
		{
			assert(is_valid_range<T>(offset, 0));
			return type_ptr<T>(m_mem + m_ofs) + offset;
		}
		template <typename T> T* ptr(int64_t offset = 0)
		{
			return const_call(ptr<T>(offset));
		}
		template <typename T> T const* end() const
		{
			return type_ptr<T>(m_mem + m_ofs + m_size);
		}
		template <typename T> T* end()
		{
			return const_call(end<T>());
		}

		// Check that '[start, start + count)' is within the valid range
		template <typename T> bool is_valid_range(int64_t start, int64_t count) const
		{
			if (m_size % sizeof(T) != 0)
				throw std::runtime_error("Buffer size is not a multiple of sizeof(T)");
			if (start < 0 || start > s_cast<int64_t>(m_size / sizeof(T)))
				throw std::runtime_error("'start' is out of range");
			if (count != std::numeric_limits<int64_t>::max() && (start + count < 0 || start + count > s_cast<int64_t>(m_size / sizeof(T))))
				throw std::runtime_error("'count' is out of range");

			return true;
		}
	};
}
