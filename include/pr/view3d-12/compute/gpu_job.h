//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/compute/gpu.h"
#include "pr/view3d-12/utility/gpu_sync.h"
#include "pr/view3d-12/utility/cmd_alloc.h"
#include "pr/view3d-12/utility/cmd_list.h"
#include "pr/view3d-12/resource/gpu_transfer_buffer.h"
#include "pr/view3d-12/resource/gpu_descriptor_heap.h"
#include "pr/view3d-12/utility/cmd_list_collection.h"
#include "pr/view3d-12/utility/barrier_batch.h"
#include "pr/view3d-12/utility/keep_alive.h"

namespace pr::rdr12
{
	// System Values:
	//   'dtid' = dispatch thread index (i.e. global thread index)
	//   'gtid' = group thread index (i.e. group relative thread index)
	//   'gpid' = group index (i.e. index of the group witihn the dispatch)
	//   'gpsz' = group size (i.e. number of threads per group)
	// 
	// Dispatch:
	//   Dispatch = ThreadGroups[N]
	//      where N = ([0,65536), [0,65536), [0,65536)) for Dx11 and Dx12
	//   ThreadGroup = Threads[N]
	//      where N = ([1,A), [1,B), [1,C)) where A*B*C <= 1024 for Dx11 and Dx12
	//     
	//   WaveSize = Threads[N]
	//      where N = 32/64 threads (depending on hardware)
	//   ThreadGroup = Waves[N]
	//      where N = (ThreadGroup + WaveSize - 1) / WaveSize
	//
	//   Groups are divided into Waves and Waves are managed by a scheduler on the hardware.
	//   Wave execution is hidden by the hardware, but conceptually all Waves run in parallel
	//   so that all threads in a Group conceptually run in parallel.
	//   
	//   Groups conceptually run in parallel as well, but there is no cross-group synchronization.
	//   There is also no shared memory between groups, only within a group ("group" shared memory).
	//   However, there is a group index so data can be stored per group in a RWStructuredBuffer.
	// 
	// Waves:
	//   A Wave is 32/64 threads running in lock-step. Each thread in a wave is called a Lane.
	//   Waves can be treated as Sub-Groups within a Group, with Wave intrinsic functions used
	//   to share data between Lanes in the Wave. This means it's often possible to store data
	//   in group shared memory per Wave, rather than per Thread.
	//
	//   To get a "Wave Index" use:
	//      int dispatch_wave_index = dtid.x / WaveGetLaneCount();
	//      int group_wave_index = gpid.x / WaveGetLaneCount();
	//   Use "WaveActiveSum" to calculate totals across all active lanes in a wave.
	//   Use "WavePrefixSum" to determine an offset based on Lane index.
	//
	// Group Shared Memory:
	//   - Group shared memory for one thread group is entirely independent of the group shared memory
	//     for any other thread group. There is no way for one group to access or interfere with the
	//     shared memory of another group.
	//   - The lifetime of the group shared memory is limited to the duration of the thread group execution.
	//     Once all the threads in a group have completed their execution, the contents of the group shared
	//     memory are discarded.
	//   - All threads within a single group can read from and write to the group shared memory.
	//     This allows for efficient communication and synchronization among threads within the same group.
	
	// Calculate the number of dispatches needed to process 'total' items in groups of 'group_size'
	inline int DispatchCount(int total, int group_size)
	{
		return (total + group_size - int(1)) / group_size;
	}
	inline iv3 DispatchCount(iv3 total, iv3 group_size)
	{
		return (total + group_size - iv3(1)) / group_size;
	}

	template <D3D12_COMMAND_LIST_TYPE QueueType>
	struct GpuJob
	{
		// Notes:
		//  - Creating a CommandQueue is slow. GpuJob is meant to be a long-lived object.
		//    You call Run() on it each frame rather than creating a new GpuJob each frame.
		//
		using Gpu = Gpu<QueueType>;
		using GpuViewHeap = GpuDescriptorHeap<D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV>;
		using CmdListCollection = CmdListCollection<QueueType>;
		using BarrierBatch = BarrierBatch<QueueType>;
		using CmdAllocPool = CmdAllocPool<QueueType>;
		using CmdList = CmdList<QueueType>;

		D3DPtr<ID3D12Device4>      m_device;     // The device to use for the GPU operations
		D3DPtr<ID3D12CommandQueue> m_queue;      // The command queue to use for the GPU operations
		GpuSync                    m_gsync;      // The GPU fence
		GpuViewHeap                m_view_heap;  // A GPU visible descriptor heap
		CmdAllocPool               m_cmd_pool;   // Command allocator pool for the compute shader
		CmdList                    m_cmd_list;   // Command list for the compute shader
		BarrierBatch               m_barriers;   // Barrier batch for the compute shader
		KeepAlive                  m_keep_alive; // Keep alive for temporary resources
		GpuUploadBuffer            m_upload;     // Upload buffer for the compute shader
		GpuReadbackBuffer          m_readback;   // Read back buffer for the compute shader

		// Note: can be constructed with a 'Gpu' instance or 'Renderer::D3DDevice()'
		GpuJob(ID3D12Device4* device, ID3D12CommandQueue* queue, char const* name, uint32_t pix_colour, int view_heap_capacity = 1)
			: m_device(device, true)
			, m_queue(queue ? D3DPtr<ID3D12CommandQueue>{queue, true} : GetQueue(device))
			, m_gsync(device)
			, m_view_heap(view_heap_capacity, m_gsync)
			, m_cmd_pool(m_gsync)
			, m_cmd_list(device, m_cmd_pool.Get(), nullptr, name, pix_colour)
			, m_barriers(m_cmd_list)
			, m_keep_alive(m_gsync)
			, m_upload(m_gsync, 0)
			, m_readback(m_gsync, 0)
		{
			auto heaps = { m_view_heap.get() };
			m_cmd_list.SetDescriptorHeaps({ heaps.begin(), heaps.size() });
		}
		GpuJob(ID3D12Device4* device, char const* name, uint32_t pix_colour, int view_heap_capacity = 1)
			: GpuJob(device, nullptr, name, pix_colour, view_heap_capacity)
		{}
		GpuJob(Gpu& gpu, char const* name, uint32_t pix_colour, int view_heap_capacity = 1)
			: GpuJob(gpu.device(), gpu.queue(), name, pix_colour, view_heap_capacity)
		{}

		// Run the job and block till complete
		void Run()
		{
			// Job complete
			m_barriers.Commit();
			m_cmd_list.Close();

			// Run the sort job
			CmdListCollection cmd_lists = { m_cmd_list.get() };
			m_queue->ExecuteCommandLists(cmd_lists.count(), cmd_lists.data());

			// Record the sync point for when the command will be finished
			auto sync_point = m_gsync.AddSyncPoint(m_queue.get());
			m_cmd_list.SyncPoint(sync_point);

			// Reset for the next job
			m_cmd_list.Reset(m_cmd_pool.Get());

			// Rebind the view heap after reset
			auto heaps = { m_view_heap.get() };
			m_cmd_list.SetDescriptorHeaps({ heaps.begin(), heaps.size() });

			// Wait for the GPU to finish
			m_gsync.Wait();
		}

		// Get a pointer to the queue
		static D3DPtr<ID3D12CommandQueue> GetQueue(ID3D12Device4* device)
		{
			D3D12_COMMAND_QUEUE_DESC queue_desc = {
				.Type = QueueType,
				.Priority = D3D12_COMMAND_QUEUE_PRIORITY_NORMAL,
				.Flags = D3D12_COMMAND_QUEUE_FLAG_NONE,
				.NodeMask = 0,
			};
			D3DPtr<ID3D12CommandQueue> m_queue;
			Check(device->CreateCommandQueue(&queue_desc, __uuidof(ID3D12CommandQueue), (void**)m_queue.address_of()));
			return m_queue;
		}
	};

	using GraphicsJob = GpuJob<D3D12_COMMAND_LIST_TYPE_DIRECT>;
	using ComputeJob = GpuJob<D3D12_COMMAND_LIST_TYPE_COMPUTE>;
}