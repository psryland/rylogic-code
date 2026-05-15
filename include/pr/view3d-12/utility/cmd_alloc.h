//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/view3d-12/utility/gpu_sync.h"

namespace pr::rdr12
{
	// Forward declare the pool
	template <D3D12_COMMAND_LIST_TYPE ListType>
	struct CmdAllocPool;

	// An allocator instance
	template <D3D12_COMMAND_LIST_TYPE ListType>
	struct CmdAlloc
	{
		// Explanation:
		//   Allocators basically point to a 'std::deque' of memory blocks in GPU memory. When you add things to a command
		//   list, space gets allocated by the allocator and the command list just records the pointers.
		//   When a command list is executed, the list is copied to GPU memory so the pointers it contains are still
		//   valid. This is why allocators can't be reset until the GPU has finished with them, but command lists can.
		//   So, use one allocator per thread, per frame, per command queue.
		//   Instances of command lists can live wherever you need them, but must be reset to use the appropriate
		//   allocator for the frame.
		//
		// Reuse efficiency:
		//   We want to reuse allocators to prevent too much dynamic allocation.
		//   We want allocators to automatically recycle when their sync point is reached.
		// 
		// Notes:
		//  - One allocator per thread
		//  - Each allocator can only be recording one command list at a time.
		//  - Only reset allocator when GPU sync is <= m_sync_point
		//  - Can reset a command list immediately after executing it, but it has to use a different allocator
		using pool_t = typename CmdAllocPool<ListType>;

		D3DPtr<ID3D12CommandAllocator> m_alloc; // The allocator pointer
		std::thread::id m_thread_id;            // The thread id of the last thread to call Reset
		uint64_t m_sync_point;                  // The sync point after which 'alloc' can be reused.
		pool_t* m_pool;                         // The pool to return this allocator too
		
		CmdAlloc()
			: m_alloc()
			, m_thread_id()
			, m_sync_point()
			, m_pool()
		{}
		CmdAlloc(D3DPtr<ID3D12CommandAllocator> alloc, uint64_t sync_point, pool_t* pool)
			: m_alloc(alloc)
			, m_thread_id(std::this_thread::get_id())
			, m_sync_point(sync_point)
			, m_pool(pool)
		{}
		CmdAlloc(CmdAlloc&& rhs) noexcept
			:CmdAlloc()
		{
			std::swap(m_alloc, rhs.m_alloc);
			std::swap(m_thread_id, rhs.m_thread_id);
			std::swap(m_sync_point, rhs.m_sync_point);
			std::swap(m_pool, rhs.m_pool);
		}
		CmdAlloc(CmdAlloc const&) = delete;
		CmdAlloc& operator =(CmdAlloc&& rhs) noexcept
		{
			if (&rhs == this)
				return *this;

			std::swap(m_alloc, rhs.m_alloc);
			std::swap(m_thread_id, rhs.m_thread_id);
			std::swap(m_sync_point, rhs.m_sync_point);
			std::swap(m_pool, rhs.m_pool);
			return *this;
		}
		CmdAlloc& operator =(CmdAlloc const&) = delete;
		~CmdAlloc()
		{
			if (m_pool != nullptr && m_alloc != nullptr)
				m_pool->Return(std::move(*this));
		}

		// Set the ID of the thread using this command allocator
		void UseThisThread()
		{
			m_thread_id = std::this_thread::get_id();
		}

		// Access the allocator
		ID3D12CommandAllocator const* get() const
		{
			auto const this_thread = std::this_thread::get_id();
			if (this_thread != m_thread_id)
				throw std::runtime_error("Cross thread use of a command allocator");

			return m_alloc.get();
		}
		ID3D12CommandAllocator* get()
		{
			return const_call(get());
		}
		ID3D12CommandAllocator const* operator ->() const
		{
			return get();
		}
		ID3D12CommandAllocator* operator ->()
		{
			return get();
		}
		operator ID3D12CommandAllocator const* () const
		{
			return get();
		}
		operator ID3D12CommandAllocator* ()
		{
			return get();
		}
	};

	// A pool of allocators
	template <D3D12_COMMAND_LIST_TYPE ListType>
	struct CmdAllocPool
	{
		using pool_t = pr::vector<CmdAlloc<ListType>, 4, false>;

		GpuSync* m_gsync;
		pool_t m_pool;

		explicit CmdAllocPool(GpuSync& gsync)
			:m_gsync(&gsync)
			,m_pool()
		{}
		CmdAllocPool(CmdAllocPool&&) = default;
		CmdAllocPool(CmdAllocPool const&) = delete;
		CmdAllocPool& operator =(CmdAllocPool&&) = default;
		CmdAllocPool& operator =(CmdAllocPool const&) = delete;
		~CmdAllocPool()
		{
			m_gsync = nullptr; // Used to catch return to destructed pool
		}

		// Get an allocator that's available to be used
		CmdAlloc<ListType> Get()
		{
			// Move the available allocators to the back of the pool.
			auto sync_point = m_gsync->CompletedSyncPoint();
			auto iter = std::partition(m_pool.begin(), m_pool.end(), [=](auto& alloc) { return alloc.m_sync_point > sync_point; });

			// Create a new allocator if there isn't one available
			if (iter == m_pool.end())
			{
				auto device = m_gsync->D3DDevice();

				// Create a command allocator
				D3DPtr<ID3D12CommandAllocator> cmd_alloc;
				Check(device->CreateCommandAllocator(ListType, __uuidof(ID3D12CommandAllocator), (void**)cmd_alloc.address_of()));
				Check(cmd_alloc->SetName(L"CmdAllocPool:CmdAlloc"));
				m_pool.emplace_back(cmd_alloc, sync_point, nullptr);
			}

			// Get a command allocator from the pool
			PR_ASSERT(PR_DBG_RDR, m_pool.back().m_sync_point <= sync_point, "This allocator is still in use");
			auto alloc = std::move(m_pool.back());
			m_pool.pop_back();

			// Mark the allocator as unusable until the next sync point is completed
			alloc.UseThisThread();
			alloc.m_sync_point = m_gsync->LastAddedSyncPoint() + 1;
			alloc.m_pool = this;
			Check(alloc->Reset()); // Reset it ready for use
			return std::move(alloc);
		}

		// Return an allocator to the pool
		void Return(CmdAlloc<ListType>&& cmd_alloc)
		{
			PR_ASSERT(PR_DBG_RDR, m_gsync != nullptr, "This pool has already been destructed");
			PR_ASSERT(PR_DBG_RDR, cmd_alloc != nullptr, "Don't add null allocators to the pool");
			PR_ASSERT(PR_DBG_RDR, cmd_alloc.m_pool == this, "Returned object didn't come from this pool");
			cmd_alloc.m_pool = nullptr;
			m_pool.push_back(std::move(cmd_alloc));
		}
	};

	// Flavours
	using GfxCmdAllocPool = CmdAllocPool<D3D12_COMMAND_LIST_TYPE_DIRECT>;
	using ComCmdAllocPool = CmdAllocPool<D3D12_COMMAND_LIST_TYPE_COMPUTE>;
	using GfxCmdAlloc = CmdAlloc<D3D12_COMMAND_LIST_TYPE_DIRECT>;
	using ComCmdAlloc = CmdAlloc<D3D12_COMMAND_LIST_TYPE_COMPUTE>;
}
