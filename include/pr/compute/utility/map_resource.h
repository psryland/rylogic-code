//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/utility/conversion.h"

namespace pr::compute
{
	// A scope object for locking (or "mapping") a resource so that the CPU can access it.
	// Mapping: Gets a pointer to the data contained in a sub-resource, and denies the GPU access to that sub-resource.
	// Unmapping: Invalidate the pointer to a resource and re-enable the GPU's access to that resource.
	struct MapResource
	{
		// Notes:
		// 
		//  - THIS WILL STALL THE GPU IF THE RESOURCE IS CURRENTLY IN USE BY THE GPU.
		//  - Are you sure you don't want 'UpdateSubresourceScope' instead?
		// 
		//  - SDK Notes from Dx11: *DON'T read from a sub-resource mapped for writing*
		//    When you pass D3D11_MAP_WRITE, D3D11_MAP_WRITE_DISCARD, or D3D11_MAP_WRITE_NO_OVERWRITE to the MapType parameter,
		//    you must ensure that your app does not read the sub-resource data to which the pData member of D3D11_MAPPED_SUBRESOURCE
		//    points because doing so can cause a significant performance penalty. The memory region to which pData points can be
		//    allocated with PAGE_WRITECOMBINE, and your app must honour all restrictions that are associated with such memory.
		// The SDK recommends using volatile pointers (but struct assignment for volatiles requires CV-qualified assignment operators)
		// Just don't read from m_data...

		ID3D12Resource* m_res;       // The resource to be locked
		UINT            m_sub;       // e.g. mip level (use 0 for V/I buffers)
		int             m_elem_size; // The size of each element (in bytes)
		Range           m_wrange;    // The write range (while mapped), the range modified after Unmap (in elements, not bytes)
		Range           m_rrange;    // The read range (while mapped), the range modified after Unmap (in elements, not bytes)
		void*           m_data;      // The pointer to the mapped resource data

		MapResource()
			:m_res()
			,m_sub()
			,m_elem_size()
			,m_wrange()
			,m_rrange()
			,m_data()
		{}
		MapResource(ID3D12Resource* res, int sub, int elem_size, Range read_range = RangeZero)
			:MapResource()
		{
			Map(res, sub, elem_size, read_range);
		}
		MapResource(MapResource&& rhs) noexcept
			:m_res(rhs.m_res)
			,m_sub(rhs.m_sub)
			,m_elem_size(rhs.m_elem_size)
			,m_rrange(rhs.m_rrange)
			,m_wrange(rhs.m_wrange)
			,m_data(rhs.m_data)
		{
			rhs.m_res = nullptr;
			rhs.m_sub = 0;
			rhs.m_elem_size = 0;
			rhs.m_rrange = Range {};
			rhs.m_wrange = Range {};
			rhs.m_data = nullptr;
		}
		MapResource(MapResource const&) = delete;
		MapResource& operator =(MapResource&& rhs) noexcept
		{
			if (this == &rhs) return *this;
			std::swap(m_res, rhs.m_res);
			std::swap(m_sub, rhs.m_sub);
			std::swap(m_elem_size, rhs.m_elem_size);
			std::swap(m_rrange, rhs.m_rrange);
			std::swap(m_wrange, rhs.m_wrange);
			std::swap(m_data, rhs.m_data);
			return *this;
		}
		MapResource& operator =(MapResource const&) = delete;
		~MapResource()
		{
			Unmap();
		}

		// Write access
		std::byte* data(uint64_t ofs = 0)
		{
			return byte_ptr(m_data) + m_elem_size * m_wrange.m_beg + ofs;
		}
		std::byte* end()
		{
			return byte_ptr(m_data) + m_elem_size * m_wrange.m_end;
		}
		template <typename T> T* ptr()
		{
			return type_ptr<T>(data());
		}
		template <typename T> T* end()
		{
			return type_ptr<T>(end());
		}
		
		// Read access
		std::byte const* data(uint64_t ofs = 0) const
		{
			return byte_ptr(m_data) + m_elem_size * m_rrange.m_beg + ofs;
		}
		std::byte const* end() const
		{
			return byte_ptr(m_data) + m_elem_size * m_rrange.m_end;
		}
		template <typename T> T const* ptr() const
		{
			return type_ptr<T>(data());
		}
		template <typename T> T const* end() const
		{
			return type_ptr<T>(end());
		}

		// Interpret the memory at 'byte_offset' as type 'T&'
		template <typename T> T& at(int byte_offset)
		{
			return *type_ptr<T>(data(byte_offset));
		}

		// Maps a resource to CPU accessible memory.
		// 'read_range' (in units of elements) indicates the region the CPU might read,
		// and the coordinates are subresource-relative. Pass Range::Zero() if the CPU will not read.
		bool Map(ID3D12Resource* res, int sub, int elem_size, Range read_range = Range::Zero())
		{
			PR_ASSERT(PR_DBG_RDR, m_res == nullptr, "Already mapped");

			// Get the write range
			auto desc = res->GetDesc();
			auto write_range = Range(0, s_cast<size_t>(desc.Width * desc.Height * desc.DepthOrArraySize));

			// Get the pointer to the memory
			void* data = nullptr;
			auto rrange = To<D3D12_RANGE>(read_range);
			Check(res->Map(sub, &rrange, &data));

			// Update only on success
			m_res       = res;
			m_sub       = sub;
			m_elem_size = elem_size;
			m_rrange    = read_range;
			m_wrange    = write_range;
			m_data      = data;
			return true;
		}
		void Unmap()
		{
			if (!m_res) return;
			D3D12_RANGE range = {};
			m_res->Unmap(m_sub, &range);
			m_rrange = Scale(To<Range>(range), 1, m_elem_size);
			m_wrange = Scale(To<Range>(range), 1, m_elem_size);
			m_data = nullptr;
			m_res = nullptr;
		}
	};
}

