//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/gpu_transfer_allocation.h"
#include "pr/view3d-12/resource/gpu_transfer_buffer_page.h"
#include "pr/view3d-12/utility/lookup.h"
#include "pr/view3d-12/utility/wrappers.h"
#include "pr/view3d-12/utility/gpu_sync.h"

namespace pr::rdr12
{
	// A buffer of GPU memory transfer resources
	template <D3D12_HEAP_TYPE HeapType>
	struct GpuTransferBuffer :RefCounted<GpuTransferBuffer<HeapType>>
	{
		// Notes:
		//  - In Dx11, setting the shader constants would copy to a new area of memory, behind the scenes, for each Map/Unmap.
		//    In Dx12, we have to do this ourselves, you can't use the same bit of memory in 'SetGraphicsRootConstantBufferView'
		//    calls (for e.g.). It's not making any copies.
		//  - This class is a deque of ID3D12Resource buffers (blocks) used to store data until the GPU has finished with it.
		//    It's a bit like the GpuDescriptorHeap, except that it is a container of Upload resource memory.
		//  - This type is used for uploading constant buffers for shaders, initialising textures, initialising V/I buffers, etc.
		//  - The 'block_size' parameter only controls the default size of each block. Larger blocks are created as needed.

		// A 'page' in the upload buffer
		using Block = GpuTransferBufferPage;
		using Lookup = Lookup<int, D3D12_GPU_VIRTUAL_ADDRESS>;
		using SyncPoint = struct { Block const* m_block; int64_t m_offset; };
		using SyncPoints = pr::deque<SyncPoint, 4>;
		using UsedBlocks = pr::deque<Block, 4>;
		using FreeBlocks = pr::vector<Block, 4>;
		using Allocation = GpuTransferAllocation;

		UsedBlocks m_used;      // The set of blocks in use by the GPU (or currently being added to).
		FreeBlocks m_free;      // Blocks that the GPU has finished with and can be recycled.
		int64_t    m_blk_size;  // The default size of each block. (Will auto create large blocks if needed).
		int32_t    m_blk_align; // The alignment to create blocks with.
		GpuSync*   m_gsync;     // The GPU fence marking GPU progress.
		Lookup     m_lookup;    // A lookup for buffer reuse (since the last sync point).
		AutoSub    m_eh0;       // Event subscription

		GpuTransferBuffer(GpuSync& gsync, int64_t block_size, int block_alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT)
			:m_used()
			,m_free()
			,m_blk_size(block_size)
			,m_blk_align(block_alignment)
			,m_gsync(&gsync)
			,m_lookup()
			,m_eh0()
		{
			// Sign up for notifications when a sync point is added
			m_eh0 = m_gsync->SyncPointAdded += [this](GpuSync&, EmptyArgs const&)
			{
				// This begins a new "frame" of allocations
				if (m_used.empty()) return;
				m_used.back().m_sync_point = m_gsync->LastAddedSyncPoint();
				m_lookup.clear();
				PurgeCompleted(false);
			};
		}
		GpuTransferBuffer(GpuTransferBuffer&& rhs) noexcept
			: m_used(std::move(rhs.m_used))
			, m_free(std::move(rhs.m_free))
			, m_blk_size(rhs.m_blk_size)
			, m_blk_align(rhs.m_blk_align)
			, m_gsync(rhs.m_gsync)
			, m_lookup(std::move(rhs.m_lookup))
			, m_eh0(std::move(rhs.m_eh0))
		{
			rhs.m_blk_size = 0;
			rhs.m_blk_align = 0;
			rhs.m_gsync = nullptr;
		}
		GpuTransferBuffer(GpuTransferBuffer const&) = delete;
		GpuTransferBuffer& operator=(GpuTransferBuffer&& rhs) noexcept
		{
			if (this == &rhs) return *this;
			std::swap(m_used, rhs.m_used);
			std::swap(m_free, rhs.m_free);
			std::swap(m_blk_size, rhs.m_blk_size);
			std::swap(m_blk_align, rhs.m_blk_align);
			std::swap(m_gsync, rhs.m_gsync);
			std::swap(m_lookup, rhs.m_lookup);
			std::swap(m_eh0, rhs.m_eh0);
			return *this;
		}
		GpuTransferBuffer& operator=(GpuTransferBuffer const&) = delete;
		~GpuTransferBuffer()
		{
			for (; !m_used.empty();)
			{
				auto& used = m_used.front();
				m_gsync->Wait(used.m_sync_point);
				PurgeCompleted(true);
			}
		}

		// Allocate some upload buffer space
		Allocation Alloc(int64_t size, int alignment)
		{
			if (alignment > m_blk_align)
				throw std::runtime_error("Cannot use alignment larger than the block alignment");

			// Make sure there's space
			EnsureCapacity(size, alignment);
			auto& block = m_used.back();

			// Allocate space
			Allocation alex(&block, block.m_res.get(), block.m_mem, PadTo(block.m_size, alignment), size);

			// Consume from the block
			block.m_size = PadTo(block.m_size, alignment) + size;

			// Return the allocation
			return alex;
		}
		template <typename T> Allocation Alloc(int count)
		{
			return Alloc(count * sizeof(T), alignof(T));
		}

		// Copy an object into upload buffer memory, and return the GPU pointer to it's location
		template <typename Item>
		D3D12_GPU_VIRTUAL_ADDRESS Add(Item const& item, int alignment, bool might_reuse)
		{
			auto key = 0;

			// See if 'item' is already in the buffer
			if (might_reuse)
			{
				// Generate a key
				key = hash::HashBytes32(&item, &item + 1);
				auto iter = m_lookup.find(key);
				if (iter != end(m_lookup))
					return iter->second;
			}

			// Add 'item' to the upload buffer
			auto alex = Alloc(sizeof(item), alignment);
			memcpy(alex.m_mem + alex.m_ofs, &item, sizeof(item));
			auto gpu_address = alex.m_res->GetGPUVirtualAddress() + alex.m_ofs;

			// Save in the lookup if this object might be reused
			if (might_reuse)
			{
				m_lookup[key] = gpu_address;
			}

			// Return the handle to the GPU memory
			return gpu_address;
		}

		// Recycle blocks that the GPU has finished with
		void PurgeCompleted()
		{
			PurgeCompleted(false);
		}

	private:

		// Recycle blocks that the GPU has finished with
		void PurgeCompleted(bool shutdown)
		{
			auto completed = m_gsync->CompletedSyncPoint();

			// Any blocks with sync points <= 'completed' are ready to be recycled.
			for (; !m_used.empty() && m_used.front().m_sync_point <= completed; )
			{
				// To reduce allocations, keep the last used block active (unless destructing)
				if (!shutdown && m_used.size() == 1)
					break;

				// The front block still has external references
				if (!shutdown && m_used.front().m_ref_count != 0)
					break;

				if (m_used.front().m_ref_count != 0)
					throw std::runtime_error("Allocation still in use at shutdown");

				// Remove from the used list
				auto block = std::move(m_used.front());
				m_used.pop_front();

				// Add to the free list
				block.m_size = 0;
				block.m_sync_point = completed;
				m_free.push_back(std::move(block));
			}
		}

		// D3D device
		ID3D12Device* device() const
		{
			return m_gsync->D3DDevice();
		}

		// Make sure there is room in the buffers for 'sz' bytes of shader constant data.
		void EnsureCapacity(int64_t size, int alignment)
		{
			// Is there space to add 'size' bytes (after alignment)?
			if (!m_used.empty() && PadTo(m_used.back().m_size, alignment) + size <= m_used.back().m_capacity)
				return;

			// Can we recycle a block from the free list
			auto iter = find_if(m_free, [=](auto& blk) { return blk.m_capacity >= size; });
			if (iter != m_free.end())
			{
				// Remove from the free list
				auto block = std::move(*iter);
				m_free.erase_fast(iter);

				// Add to the 'in-use' list
				block.m_sync_point = m_gsync->LastAddedSyncPoint();
				m_used.push_back(std::move(block));
				return;
			}

			// Create a new block
			auto blk_size = PadTo(std::max(size, m_blk_size), m_blk_align);
			Block block(device(), HeapType, blk_size, m_blk_align, m_gsync->LastAddedSyncPoint());
			m_used.push_back(std::move(block));

			// Sanity check for unchecked growth
			if (m_used.size() > 500)
			{
				// Can put a breakpoint here before it throws
				if (m_used.size() > 1000)
					throw std::runtime_error("GPU transfer buffer memory growing. There is probably a reference count leak");
			}
		}

		// Ref-counting clean up function
		static void RefCountZero(RefCounted<GpuTransferBuffer>* doomed)
		{
			auto upload_buffer = static_cast<GpuTransferBuffer*>(doomed);
			rdr12::Delete(upload_buffer);
		}
		friend struct RefCount<GpuTransferBuffer>;
	};

	using GpuUploadBuffer = GpuTransferBuffer<D3D12_HEAP_TYPE_UPLOAD>;
	using GpuReadbackBuffer = GpuTransferBuffer<D3D12_HEAP_TYPE_READBACK>;
}