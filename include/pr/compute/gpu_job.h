//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/gpu.h"
#include "pr/compute/utility/gpu_sync.h"
#include "pr/compute/utility/cmd_alloc.h"
#include "pr/compute/utility/cmd_list.h"
#include "pr/compute/resource/gpu_transfer_buffer.h"
#include "pr/compute/resource/gpu_descriptor_heap.h"
#include "pr/compute/utility/cmd_list_collection.h"
#include "pr/compute/utility/barrier_batch.h"
#include "pr/compute/utility/keep_alive.h"

namespace pr::compute
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

		struct RunProfile
		{
			double m_prepare_ms = 0;
			double m_execute_ms = 0;
			double m_wait_ms = 0;
			double m_reset_ms = 0;
		};
		struct RunHandle
		{
			uint64_t m_sync_point = 0;
			explicit operator bool() const { return m_sync_point != 0; } // Return true if this handle represents an in-flight GPU submission.
		};

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
			, m_pending_sync_point()
		{
			auto heaps = { m_view_heap.get() };
			m_cmd_list.SetDescriptorHeaps({ heaps.begin(), heaps.size() });
		}
		GpuJob(ID3D12Device4* device, char const* name, uint32_t pix_colour, int view_heap_capacity = 1)
			: GpuJob(device, nullptr, name, pix_colour, view_heap_capacity)
		{
		}
		GpuJob(Gpu& gpu, char const* name, uint32_t pix_colour, int view_heap_capacity = 1)
			: GpuJob(gpu.device(), gpu.queue(), name, pix_colour, view_heap_capacity)
		{
		}

		uint64_t m_pending_sync_point; // The submitted-but-not-yet-completed sync point.

		// Submit the recorded commands and return without waiting for completion.
		RunHandle Submit(RunProfile* profile = nullptr)
		{
			if (m_pending_sync_point != 0)
				throw std::runtime_error("GpuJob::Submit called while a previous submission is still pending");

			auto const timestamp = [profile]
			{
				return profile != nullptr ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
			};
			auto const prepare_beg = timestamp();

			// Job complete
			m_barriers.Commit();
			m_cmd_list.Close();

			auto const execute_beg = timestamp();

			// Run the job.
			m_pending_sync_point = ExecuteRecorded();

			if (profile != nullptr)
			{
				auto const submit_end = timestamp();
				auto elapsed_ms = [](auto beg, auto end)
				{
					return std::chrono::duration<double, std::milli>(end - beg).count();
				};
				profile->m_prepare_ms = elapsed_ms(prepare_beg, execute_beg);
				profile->m_execute_ms = elapsed_ms(execute_beg, submit_end);
			}

			return RunHandle{ m_pending_sync_point };
		}

		// Execute the commands recorded so far without waiting, then continue recording into a fresh command list.
		// The queue executes work in order, so the handle from the next 'Submit' also covers the flushed commands.
		// Resource-state tracking restarts as it does after 'Submit', so callers must not rely on a non-default state set before the flush.
		void Flush()
		{
			// Flushing is part of recording, which is not possible while a submission is pending.
			if (m_pending_sync_point != 0)
				throw std::runtime_error("GpuJob::Flush called while a submission is pending");

			// Execute the recorded prefix, then record later commands with an allocator the GPU is not using.
			m_barriers.Commit();
			m_cmd_list.Close();
			ExecuteRecorded();
			ResetRecording();
		}

		// Wait for a submitted job, then reset command recording for the next submission.
		void Complete(RunHandle& handle, RunProfile* profile = nullptr)
		{
			if (!handle || handle.m_sync_point != m_pending_sync_point)
				throw std::runtime_error("GpuJob::Complete called with an invalid or non-current run handle");

			auto const timestamp = [profile]
			{
				return profile != nullptr ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
			};
			auto const wait_beg = timestamp();

			// Wait for the GPU to finish
			if (!m_gsync.Wait(handle.m_sync_point))
			{
				auto reason = m_gsync.DeviceRemovedReason();
				throw DeviceRemovedException(m_device.get(), reason);
			}

			auto const reset_beg = timestamp();

			// Notify that sync points have been reached (recycles command allocators,
			// sweeps KeepAlive references, etc). Without this, resources accumulate
			// indefinitely when GpuJob is used without a Renderer's periodic timer.
			m_gsync.Poll();

			// Reset for the next job
			ResetRecording();

			if (profile != nullptr)
			{
				auto const reset_end = timestamp();
				auto elapsed_ms = [](auto beg, auto end)
				{
					return std::chrono::duration<double, std::milli>(end - beg).count();
				};
				profile->m_wait_ms = elapsed_ms(wait_beg, reset_beg);
				profile->m_reset_ms = elapsed_ms(reset_beg, reset_end);
			}

			m_pending_sync_point = 0;
			handle = {};
		}

		// Wait for submitted work during terminal cleanup without touching thread-affine recording state.
		void Abandon(RunHandle& handle)
		{
			if (!handle || handle.m_sync_point != m_pending_sync_point)
				throw std::runtime_error("GpuJob::Abandon called with an invalid or non-current run handle");

			(void)m_gsync.Wait(handle.m_sync_point);
			m_pending_sync_point = 0;
			handle = {};
		}

		// Submit and retire a partially recorded list so staged allocations and tracked resource states reach a completed fence boundary.
		void RetireRecordedWork()
		{
			// Executing the valid prefix keeps CPU-side resource-state tracking consistent with the actual GPU state.
			auto handle = Submit();

			// Complete normally so transient allocations are reclaimed and the command list is reset for reuse.
			Complete(handle);
		}

		// Run the job and block till complete.
		void Run(RunProfile* profile = nullptr)
		{
			auto handle = Submit(profile);
			Complete(handle, profile);
		}

	private:

		// Execute the closed command list and return the sync point that marks its completion.
		uint64_t ExecuteRecorded()
		{
			// The allocator cannot be reused until the GPU reaches the sync point that follows these commands.
			CmdListCollection cmd_lists = { m_cmd_list.get() };
			m_queue->ExecuteCommandLists(cmd_lists.count(), cmd_lists.data());
			auto sync_point = m_gsync.AddSyncPoint(m_queue.get());
			m_cmd_list.SyncPoint(sync_point);
			return sync_point;
		}

		// Reopen the command list with an available allocator, ready for further recording.
		void ResetRecording()
		{
			// Reset clears the bound descriptor heaps, so rebind the view heap.
			m_cmd_list.Reset(m_cmd_pool.Get());
			auto heaps = { m_view_heap.get() };
			m_cmd_list.SetDescriptorHeaps({ heaps.begin(), heaps.size() });
		}

	public:

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

		// Upload 'data' to 'res'
		template <typename T> friend void UploadData(GpuJob& job, ID3D12Resource* res, std::span<T const> data)
		{
			assert(job.m_cmd_list.ResState(res).Mip0State() == D3D12_RESOURCE_STATE_COPY_DEST && "'res' should be in the COPY_DEST state");

			auto upload_buffer = job.m_upload.Alloc<T>(static_cast<int>(data.size()));
			memcpy(upload_buffer.ptr<T>(), data.data(), data.size() * sizeof(T));
			job.m_cmd_list.CopyBufferRegion(res, 0, upload_buffer);
		}

		// Upload 'data' to 'res' piecemeal
		template <typename T> friend void UploadData(GpuJob& job, ID3D12Resource* res, size_t count, std::invocable<T*, T*> auto cb)
		{
			assert(job.m_cmd_list.ResState(res).Mip0State() == D3D12_RESOURCE_STATE_COPY_DEST && "'res' should be in the COPY_DEST state");

			auto upload_buffer = job.m_upload.Alloc<T>(count);
			cb(upload_buffer.ptr<T>(), upload_buffer.end<T>());
			job.m_cmd_list.CopyBufferRegion(res, 0, upload_buffer);
		}

		// Read back 'count' elements of T from 'res'
		template <typename T> friend GpuTransferAllocation ReadBack(GpuJob& job, int64_t count, ID3D12Resource* res)
		{
			assert(job.m_cmd_list.ResState(res).Mip0State() == D3D12_RESOURCE_STATE_COPY_SOURCE && "'res' should be in the COPY_SOURCE state");

			auto readback_buffer = job.m_readback.Alloc<T>(count);
			job.m_cmd_list.CopyBufferRegion(readback_buffer, res, 0);
			return readback_buffer;
		}
	};

	using GraphicsJob = GpuJob<D3D12_COMMAND_LIST_TYPE_DIRECT>;
	using ComputeJob = GpuJob<D3D12_COMMAND_LIST_TYPE_COMPUTE>;
	
	// Calculate the number of dispatches needed to process 'total' items in groups of 'group_size'
	inline int DispatchCount(int total, int group_size)
	{
		return (total + group_size - int(1)) / group_size;
	}
	inline iv3 DispatchCount(iv3 total, iv3 group_size)
	{
		return (total + group_size - iv3(1)) / group_size;
	}
}

#if PR_UNITTESTS
#include "pr/common/unittests.h"
#include "pr/compute/utility/update_resource.h"
namespace pr::compute::tests
{
	// Check that the per-job reuse caches return a cached GPU location only for an exact match.
	PRUnitTestClass(GpuJobReuseCacheTests)
	{
		// Two different 8-byte values with the same 32-bit FNV-1a hash.
		struct HashCollisionPair
		{
			int32_t a;
			int32_t b;
		};
		static constexpr HashCollisionPair CollisionA = { 1785990244, 450153144 };
		static constexpr HashCollisionPair CollisionB = { 989274597, 444590511 };

		// The upload buffer must not share an upload between different items whose bytes have the same hash.
		PRUnitTestMethod(UploadBufferReusesOnlyIdenticalItems, Extended)
		{
			// Confirm the test values really collide, otherwise the test proves nothing
			PR_EXPECT(hash::HashBytes32(&CollisionA, &CollisionA + 1) == hash::HashBytes32(&CollisionB, &CollisionB + 1));

			ComGpu gpu;
			ComputeJob job(gpu, "UploadBufferReuse", 0xFF000000);
			auto addr_a = job.m_upload.Add(CollisionA, 8, true).VirtualAddress();
			auto addr_b = job.m_upload.Add(CollisionB, 8, true).VirtualAddress();
			PR_EXPECT(addr_a != addr_b);
			PR_EXPECT(job.m_upload.Add(CollisionA, 8, true).VirtualAddress() == addr_a);

			// A cached upload that does not satisfy a stricter alignment must not be reused. Pad the buffer so the next 4-byte aligned
			// allocation starts at an address that is not 8-byte aligned.
			auto pad = job.m_upload.Add(int32_t{0}, 4, false).VirtualAddress();
			if (pad % 8 == 4)
				job.m_upload.Add(int32_t{0}, 4, false);

			auto const item = HashCollisionPair{ 1, 2 };
			auto addr_4 = job.m_upload.Add(item, 4, true).VirtualAddress();
			PR_EXPECT(addr_4 % 8 == 4);
			auto addr_8 = job.m_upload.Add(item, 8, true).VirtualAddress();
			PR_EXPECT(addr_8 % 8 == 0 && addr_8 != addr_4);

			// Invalid alignments are rejected even when the item would be a cache hit
			PR_THROWS(job.m_upload.Add(CollisionA, 0, true), std::runtime_error);
			PR_THROWS(job.m_upload.Add(CollisionA, 12, true), std::runtime_error);

			// Complete the job so the upload buffer's pending sync point is signalled before it is destroyed
			job.Run();
		}

		// The view heap must not share a GPU block between descriptor tables for different descriptor-store slot generations.
		PRUnitTestMethod(ViewHeapReusesOnlyIdenticalTables, Extended)
		{
			ComGpu gpu;
			ComputeJob job(gpu, "ViewHeapReuse", 0xFF000000, 8);

			// Create two null SRVs in a CPU-only heap to act as descriptor sources
			auto type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;
			auto heap_desc = D3D12_DESCRIPTOR_HEAP_DESC{ .Type = type, .NumDescriptors = 2, .Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE, .NodeMask = 0 };
			D3DPtr<ID3D12DescriptorHeap> cpu_heap;
			Check(gpu->CreateDescriptorHeap(&heap_desc, __uuidof(ID3D12DescriptorHeap), (void**)cpu_heap.address_of()));
			auto des_size = gpu->GetDescriptorHandleIncrementSize(type);
			auto cpu0 = cpu_heap->GetCPUDescriptorHandleForHeapStart();
			auto cpu1 = D3D12_CPU_DESCRIPTOR_HANDLE{ cpu0.ptr + des_size };
			auto srv_desc = D3D12_SHADER_RESOURCE_VIEW_DESC{ .Format = DXGI_FORMAT_R32_FLOAT, .ViewDimension = D3D12_SRV_DIMENSION_BUFFER, .Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING, .Buffer = { .FirstElement = 0, .NumElements = 1 } };
			gpu->CreateShaderResourceView(nullptr, &srv_desc, cpu0);
			gpu->CreateShaderResourceView(nullptr, &srv_desc, cpu1);

			// Tables with the same indices but a different generation must get separate GPU blocks
			Descriptor const seq_a[] = { Descriptor(0, type, cpu0, 1), Descriptor(1, type, cpu1, 1) };
			Descriptor const seq_b[] = { Descriptor(0, type, cpu0, 1), Descriptor(1, type, cpu1, 2) };
			auto handle_a = job.m_view_heap.Add(std::span{ seq_a });
			auto handle_b = job.m_view_heap.Add(std::span{ seq_b });
			PR_EXPECT(handle_a.ptr != handle_b.ptr);
			PR_EXPECT(job.m_view_heap.Add(std::span{ seq_a }).ptr == handle_a.ptr);

			// Complete the job so the view heap's pending sync point is signalled before it is destroyed
			job.Run();
		}
	};

	// Check resource creation and sub-resource updates against what the GPU actually copies.
	PRUnitTestClass(GpuResourceTests)
	{
		// A readback copy of one texture subresource. 'Row' is valid once the job that recorded the copy has completed.
		struct SubresourceReadback
		{
			GpuReadbackBuffer::Allocation m_staging;
			D3D12_PLACED_SUBRESOURCE_FOOTPRINT m_layout;

			// Return the first element of row 'y'
			template <typename T> T const* Row(int y) const
			{
				// The footprint offset is relative to the allocation, so the row pitch is the only layout needed
				return type_ptr<T>(m_staging.ptr<uint8_t>() + m_layout.Footprint.RowPitch * y);
			}
		};

		// Record a copy of subresource 'sub' of 'tex' into 'job's readback buffer. 'tex' must be in the COPY_SOURCE state.
		static SubresourceReadback ReadBackSubresource(GraphicsJob& job, ID3D12Resource* tex, uint32_t sub)
		{
			// Get the layout of the subresource and stage space for it
			auto desc = tex->GetDesc();
			auto total_size = 0ULL;
			SubresourceReadback rb = {};
			job.m_device->GetCopyableFootprints(&desc, sub, 1, 0ULL, &rb.m_layout, nullptr, nullptr, &total_size);
			rb.m_staging = job.m_readback.Alloc(s_cast<int64_t>(total_size), D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT);

			// Copy the subresource into the staging memory
			auto placed = rb.m_layout;
			placed.Offset = s_cast<UINT64>(rb.m_staging.m_ofs);
			D3D12_TEXTURE_COPY_LOCATION dst = { .pResource = rb.m_staging.m_res, .Type = D3D12_TEXTURE_COPY_TYPE_PLACED_FOOTPRINT, .PlacedFootprint = placed };
			D3D12_TEXTURE_COPY_LOCATION src = { .pResource = tex, .Type = D3D12_TEXTURE_COPY_TYPE_SUBRESOURCE_INDEX, .SubresourceIndex = sub };
			job.m_cmd_list.CopyTextureRegion(&dst, 0, 0, 0, &src, nullptr);
			return rb;
		}

		// A descriptor heap must hold exactly 'capacity' descriptors, including a capacity of one.
		PRUnitTestMethod(DescriptorHeapHoldsCapacityDescriptors, Extended)
		{
			// One null SRV fits, and a second overflows
			ComGpu gpu;
			ComputeJob job(gpu, "DescriptorHeapCapacity", 0xFF000000, 1);
			D3D12_SHADER_RESOURCE_VIEW_DESC srv_desc = {
				.Format = DXGI_FORMAT_R32_UINT,
				.ViewDimension = D3D12_SRV_DIMENSION_BUFFER,
				.Shader4ComponentMapping = D3D12_DEFAULT_SHADER_4_COMPONENT_MAPPING,
				.Buffer = { .FirstElement = 0, .NumElements = 1, .StructureByteStride = 0, .Flags = D3D12_BUFFER_SRV_FLAG_NONE },
			};
			job.m_view_heap.Add(nullptr, srv_desc);
			PR_THROWS(job.m_view_heap.Add(nullptr, srv_desc), std::runtime_error);
			job.Run();
		}

		// Setting the default state back to COMMON must replace a previously stored state.
		PRUnitTestMethod(DefaultResStateCanBeSetToCommon, Extended)
		{
			ComGpu gpu;
			ComputeJob job(gpu, "DefaultResState", 0xFF000000);
			auto buf = gpu.CreateResource(ResDesc::Buf<int>(4, {}), job, "DefaultState");
			DefaultResState(buf.get(), D3D12_RESOURCE_STATE_COPY_DEST);
			PR_EXPECT(DefaultResState(static_cast<ID3D12Resource const*>(buf.get())) == D3D12_RESOURCE_STATE_COPY_DEST);
			DefaultResState(buf.get(), D3D12_RESOURCE_STATE_COMMON);
			PR_EXPECT(DefaultResState(static_cast<ID3D12Resource const*>(buf.get())) == D3D12_RESOURCE_STATE_COMMON);
			job.Run();
		}

		// The staging allocation must cover every mip footprint, including the aligned offsets between mips.
		PRUnitTestMethod(UpdateSubresourceStagesAllMips, Extended)
		{
			// A 5x3 texture with two mips needs an aligned offset before mip 1
			GfxGpu gpu;
			GraphicsJob job(gpu, "UpdateSubresourceMips", 0xFF000000);
			auto tex = gpu.CreateResource(ResDesc::Tex2D(Image(5, 3, nullptr, DXGI_FORMAT_R32_FLOAT), 2), job, "Mips");
			GfxUpdateSubresourceScope update(job.m_cmd_list, job.m_upload, tex.get(), 0, 0, 2, D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT);

			// The last row of the last mip must end inside the allocation. The padding after the last row is not needed by the copy.
			auto const& last = update.m_layout.back();
			auto rows = 1LL * last.Footprint.Height * last.Footprint.Depth;
			auto end = s_cast<int64_t>(last.Offset) + last.Footprint.RowPitch * (rows - 1) + last.Footprint.Width * s_cast<int64_t>(sizeof(float));
			PR_EXPECT(end <= update.m_staging.m_ofs + update.m_staging.m_size);

			// Copy both mips so the debug layer validates the staged footprints
			float const mip0[15] = {};
			float const mip1[2] = {};
			Image const images[] = { Image(5, 3, mip0, DXGI_FORMAT_R32_FLOAT), Image(2, 1, mip1, DXGI_FORMAT_R32_FLOAT) };
			update.Write(images);
			update.Commit();
			job.Run();
		}

		// A partial texture update must copy the staged box to its position in the texture.
		PRUnitTestMethod(UpdateSubresourceCopiesPartialBox, Extended)
		{
			// Update a 3x2 box at (1,1) of a zero-initialised 5x3 texture
			GfxGpu gpu;
			GraphicsJob job(gpu, "UpdateSubresourceBox", 0xFF000000);
			auto tex = gpu.CreateResource(ResDesc::Tex2D(Image(5, 3, nullptr, DXGI_FORMAT_R32_FLOAT), 1), job, "Box");
			float const box[] = { 1, 2, 3, 4, 5, 6 };
			GfxUpdateSubresourceScope update(job.m_cmd_list, job.m_upload, tex.get(), 0, 0, 1, D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT, { 1, 1, 0 }, { 3, 2, 1 });
			update.Write(Image(3, 2, box, DXGI_FORMAT_R32_FLOAT));
			update.Commit(EFinalState::Override, D3D12_RESOURCE_STATE_COPY_SOURCE);
			auto readback = ReadBackSubresource(job, tex.get(), 0);
			job.Run();

			// Only the box changes, and it holds the staged values in order
			for (int y = 0; y != 3; ++y)
			{
				for (int x = 0; x != 5; ++x)
				{
					auto in_box = x >= 1 && x < 4 && y >= 1 && y < 3;
					auto expected = in_box ? box[(y - 1) * 3 + (x - 1)] : 0.0f;
					PR_EXPECT(readback.Row<float>(y)[x] == expected);
				}
			}
		}

		// An update clipped to nothing stages no memory, so 'ptr' must reject access while 'Write' and 'Commit' do nothing.
		PRUnitTestMethod(UpdateSubresourceClippedAwayHasNoStaging, Extended)
		{
			// Update a box that starts outside a 4x4 texture
			GfxGpu gpu;
			GraphicsJob job(gpu, "UpdateSubresourceClipped", 0xFF000000);
			auto tex = gpu.CreateResource(ResDesc::Tex2D(Image(4, 4, nullptr, DXGI_FORMAT_R32_FLOAT), 1), job, "Clipped");
			GfxUpdateSubresourceScope update(job.m_cmd_list, job.m_upload, tex.get(), 0, 0, 1, D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT, { 4, 0, 0 }, { 2, 2, 1 });
			PR_EXPECT(update.m_layout.empty());
			PR_THROWS(update.ptr<float>(), std::runtime_error);
			PR_THROWS(update.ptr<float>(iv3(0, 0, 0)), std::runtime_error);

			// 'Write' and 'Commit' record nothing, so the job still runs cleanly
			float const data[] = { 1, 2, 3, 4 };
			update.Write(Image(2, 2, data, DXGI_FORMAT_R32_FLOAT), true);
			update.Commit();
			job.Run();
		}

		// Restoring after an update must return each subresource to its own prior state, not the state of another array slice.
		PRUnitTestMethod(UpdateSubresourceRestoresSliceState, Extended)
		{
			// Put slice 1 of a two-slice array in a different state from slice 0
			GfxGpu gpu;
			GraphicsJob job(gpu, "UpdateSubresourceRestore", 0xFF000000);
			auto desc = ResDesc::Tex2D(Image(4, 4, nullptr, DXGI_FORMAT_R32_FLOAT), 1);
			desc.DepthOrArraySize = 2;
			auto tex = gpu.CreateResource(desc, job, "Array");
			job.m_barriers.Transition(tex.get(), D3D12_RESOURCE_STATE_COPY_SOURCE, 1);
			job.m_barriers.Commit();

			// Update slice 1 and restore its state
			float const pixels[16] = {};
			GfxUpdateSubresourceScope update(job.m_cmd_list, job.m_upload, tex.get(), 1, 0, 1, D3D12_TEXTURE_DATA_PLACEMENT_ALIGNMENT);
			update.Write(Image(4, 4, pixels, DXGI_FORMAT_R32_FLOAT));
			update.Commit(EFinalState::Restore);

			// Each slice keeps its own state
			auto state = job.m_cmd_list.ResState(tex.get());
			PR_EXPECT(state[0] == D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE);
			PR_EXPECT(state[1] == D3D12_RESOURCE_STATE_COPY_SOURCE);
			job.Run();
		}

		// Initial data must be staged and copied by the job that creates the resource, and the 'Gpu' must not wait on work it never submitted.
		PRUnitTestMethod(CreateResourceStagesInitialDataInJob, Extended)
		{
			// Create a buffer with initial data and read it back in the same job
			ComGpu gpu;
			ComputeJob job(gpu, "CreateResourceInitData", 0xFF000000);
			int const data[] = { 3, 1, 4, 1, 5 };
			auto buf = gpu.CreateResource(ResDesc::Buf<int>(_countof(data), data).def_state(D3D12_RESOURCE_STATE_COPY_SOURCE), job, "InitData");
			auto readback = job.m_readback.Alloc(sizeof(data), alignof(int));
			job.m_cmd_list.CopyBufferRegion(readback.m_res, readback.m_ofs, buf.get(), 0, sizeof(data));
			job.Run();

			// The read back values match the initial data
			PR_EXPECT(std::equal(std::begin(data), std::end(data), readback.ptr<int>()));
		}
	};
}
#endif
