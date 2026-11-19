//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/resource/gpu_transfer_allocation.h"
#include "pr/compute/resource/gpu_address.h"
#include "pr/compute/resource/gpu_transfer_buffer_page.h"
#include "pr/compute/utility/wrappers.h"
#include "pr/compute/utility/gpu_sync.h"

namespace pr::compute
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

		// Hashes the bytes of a cached item. Lookups can pass a span, so a cache hit does not allocate.
		struct BytesHash
		{
			using is_transparent = void;
			size_t operator()(std::span<std::byte const> bytes) const
			{
				return s_cast<size_t>(hash::HashBytes32(bytes.data(), bytes.data() + bytes.size()));
			}
		};

		// Compares cached item bytes exactly, so two different items that have the same hash never share an upload.
		struct BytesEqual
		{
			using is_transparent = void;
			bool operator()(std::span<std::byte const> lhs, std::span<std::byte const> rhs) const
			{
				return std::ranges::equal(lhs, rhs);
			}
		};

		// A 'page' in the upload buffer
		using Block = GpuTransferBufferPage;
		using Lookup = std::unordered_map<std::vector<std::byte>, GpuAddress, BytesHash, BytesEqual>;
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
		int64_t    m_reuse_id;  // Identifies the current reuse window. See 'ReuseId'.
		AutoSub    m_eh0;       // Event subscription

		// The source of reuse window ids. Ids are unique across all buffers of this heap type, so a cached id never matches another buffer.
		inline static std::atomic_int64_t s_reuse_ids = 0;

		GpuTransferBuffer(GpuSync& gsync, int64_t block_size, int block_alignment = D3D12_DEFAULT_RESOURCE_PLACEMENT_ALIGNMENT)
			:m_used()
			,m_free()
			,m_blk_size(block_size)
			,m_blk_align(block_alignment)
			,m_gsync(&gsync)
			,m_lookup()
			,m_reuse_id(++s_reuse_ids)
			,m_eh0()
		{
			// Sign up for notifications when a sync point is added
			m_eh0 = m_gsync->SyncPointAdded += [this](GpuSync&, EmptyArgs const&)
			{
				// This begins a new "frame" of allocations
				if (m_used.empty()) return;
				m_used.back().m_sync_point = m_gsync->LastAddedSyncPoint();
				EndReuseWindow();
				PurgeCompleted();
			};
		}
		// The fence callback captures this address, so the buffer must stay at its construction address.
		GpuTransferBuffer(GpuTransferBuffer&&) = delete;
		GpuTransferBuffer(GpuTransferBuffer const&) = delete;
		GpuTransferBuffer& operator=(GpuTransferBuffer&&) = delete;
		GpuTransferBuffer& operator=(GpuTransferBuffer const&) = delete;
		~GpuTransferBuffer()
		{
			// The owner must retire submitted work and release allocation handles before destruction. Destroy pages directly;
			// recycling them into the free list would allocate memory during teardown.
			for (auto const& used : m_used)
			{
				m_gsync->Wait(used.m_sync_point);
				assert(used.m_ref_count == 0 && "Allocation still in use at shutdown");
			}
		}

		// Allocate some upload buffer space
		Allocation Alloc(int64_t size, int alignment)
		{
			// Make sure there's space
			CheckAlignment(alignment);
			EnsureCapacity(size, alignment);
			auto& block = m_used.back();

			// Allocate space
			Allocation alex(&block, block.m_res.get(), block.m_mem, PadTo(block.m_size, alignment), size);

			// Consume from the block
			block.m_size = PadTo(block.m_size, alignment) + size;

			// Mark the block as in-use until at least the next sync point completes. The commands recorded against this
			// allocation are queued in a command list that will not execute until the next flush. Without this update, a
			// block's m_sync_point would only ever be set from its creation time (or from the SyncPointAdded handler for
			// the back block) which can be a sync point that has already been completed. Once another block is appended,
			// the front block loses size==1 protection and PurgeCompleted would recycle it, even though its queued
			// commands have not yet executed. Recycling allows the block memory to be overwritten by subsequent Alloc
			// callers, which corrupts the source data for the queued copies.
			block.m_sync_point = m_gsync->NextSyncPoint();

			// Return the allocation
			return alex;
		}
		template <typename T> Allocation Alloc(int count)
		{
			return Alloc(count * sizeof(T), alignof(T));
		}

		// Copy an object into upload buffer memory, and return its GPU location.
		// When 'might_reuse' is true, an earlier upload is returned only if it has identical bytes (padding included) and satisfies 'alignment'.
		template <typename Item>
		GpuAddress Add(Item const& item, int alignment, bool might_reuse) requires (std::is_trivially_copyable_v<Item> && std::is_standard_layout_v<Item>)
		{
			// Reuse an identical upload made since the last sync point. The cache test uses 'alignment' as a divisor, so validate it first.
			CheckAlignment(alignment);
			auto bytes = std::as_bytes(std::span{ &item, 1 });
			if (might_reuse)
			{
				auto iter = m_lookup.find(bytes);
				if (iter != end(m_lookup) && iter->second.VirtualAddress() % alignment == 0)
					return iter->second;
			}

			// Add 'item' to the upload buffer
			auto alex = Alloc(sizeof(item), alignment);
			memcpy(alex.m_mem + alex.m_ofs, &item, sizeof(item));
			auto gpu_address = GpuAddress(alex);

			// Save in the lookup if this object might be reused
			if (might_reuse)
			{
				m_lookup.insert_or_assign(std::vector<std::byte>(bytes.begin(), bytes.end()), gpu_address);
			}

			// Return the handle to the GPU memory
			return gpu_address;
		}

		// Identifies the current reuse window: the span between sync points in which uploads stay valid for later commands recorded on this buffer.
		// An address returned by 'Add' or 'Alloc' may be bound again by later commands while 'ReuseId' still returns the id read when it was uploaded.
		// Ids are never repeated, so a caller can cache an upload against an id without also tracking the buffer.
		int64_t ReuseId() const
		{
			return m_reuse_id;
		}

		// Recycle blocks that the GPU has finished with
		void PurgeCompleted()
		{
			// Keep the active tail and any page whose allocation handles are still in use.
			auto completed = m_gsync->CompletedSyncPoint();
			for (; m_used.size() > 1 && m_used.front().m_sync_point <= completed && m_used.front().m_ref_count == 0; )
			{
				// Allocate the destination before removing the source, so allocation failure cannot lose a page.
				m_free.emplace_back();
				m_free.back() = std::move(m_used.front());
				m_used.pop_front();
				m_free.back().m_size = 0;
				m_free.back().m_sync_point = completed;
			}
		}

		// Cancel reservations beyond the last submitted fence point. The caller must exclusively own recording on this
		// buffer/fence and first abandon every unsubmitted command list. Allocation handles may remain alive, but must no longer record work. Never call after Execute
		// without a successful queue Signal. A page shared with earlier submissions stays protected by their last fence point.
		void CancelUnsubmitted() noexcept
		{
			// Remove only the unsubmitted reservation; do not rewind offsets or claim that earlier GPU work has completed.
			auto last_submitted = m_gsync->LastAddedSyncPoint();
			for (auto& block : m_used)
			{
				if (block.m_sync_point > last_submitted)
					block.m_sync_point = last_submitted;
			}
			EndReuseWindow();
		}

	private:

		// Forget uploads that later commands may no longer share
		void EndReuseWindow()
		{
			// A new id invalidates addresses that callers cached against the old one
			m_lookup.clear();
			m_reuse_id = ++s_reuse_ids;
		}

		// D3D device
		ID3D12Device* device() const
		{
			return m_gsync->D3DDevice();
		}

		// Throw unless 'alignment' is a positive power of two no larger than the block alignment.
		void CheckAlignment(int alignment) const
		{
			// Padding and the reuse test both require a non-zero power-of-two divisor that blocks can satisfy
			if (alignment <= 0 || (alignment & (alignment - 1)) != 0)
				throw std::runtime_error("Upload alignment must be a positive power of two");
			if (alignment > m_blk_align)
				throw std::runtime_error("Cannot use alignment larger than the block alignment");
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
			pr::compute::Delete(upload_buffer);
		}
		friend struct RefCount<GpuTransferBuffer>;
	};

	using GpuUploadBuffer = GpuTransferBuffer<D3D12_HEAP_TYPE_UPLOAD>;
	using GpuReadbackBuffer = GpuTransferBuffer<D3D12_HEAP_TYPE_READBACK>;

	// Moving either buffer would leave its fence subscription bound to a different owner.
	static_assert(!std::is_move_constructible_v<GpuUploadBuffer> && !std::is_move_assignable_v<GpuUploadBuffer>);
	static_assert(!std::is_move_constructible_v<GpuReadbackBuffer> && !std::is_move_assignable_v<GpuReadbackBuffer>);
}
