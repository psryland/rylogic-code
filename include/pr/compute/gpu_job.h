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

	// Owns recording and submission state. Before destruction, callers must complete submitted work or successfully TryClose.
	// After a submission/recording failure, retain this object and its dependent resources until TryClose succeeds.
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
			, m_recording(true)
			, m_unfenced(false)
			, m_closing(false)
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
		~GpuJob()
		{
			// Runtime failures belong to the explicit close operation, not stack unwinding. An unsafe destruction is a caller contract violation.
			assert((!m_unfenced && m_gsync.CompletedSyncPoint() >= m_gsync.LastAddedSyncPoint()) || FAILED(m_gsync.DeviceRemovedReason()));
			m_upload.CancelUnsubmitted();
			m_readback.CancelUnsubmitted();
		}

		uint64_t m_pending_sync_point; // The submitted-but-not-yet-completed sync point.
		bool m_recording;              // The command list can still accept a complete recording.
		bool m_unfenced;               // Commands were queued but no successful fence signal covers them yet.
		bool m_closing;                // Closing is terminal, even when its wait has to be retried.

		// True while normal recording is allowed.
		bool IsRecording() const
		{
			// A failed or closing recording cannot be submitted again.
			return m_recording && !m_closing;
		}

		// Stop accepting work after a partial-operation failure. Outstanding submissions remain owned until TryClose succeeds.
		void Stop() noexcept
		{
			// Do not submit the partial recording or change the completion state of already queued work.
			m_closing = true;
			m_recording = false;
		}

		// Permanently close this job within 'timeout' milliseconds (INFINITE is allowed). S_OK permits destruction, S_FALSE means timeout,
		// and a failed HRESULT leaves ownership intact for retry. No observers run, no recorded commands execute, and no process policy is imposed.
		// A removed device is quiescent for destruction. Callers must release allocation handles before destroying the closed job.
		HRESULT TryClose(DWORD timeout) noexcept
		{
			// Closing never reopens recording, including after a failed wait or signal.
			auto const started = GetTickCount64();
			Stop();
			if (!FAILED(m_gsync.DeviceRemovedReason()) && m_gsync.CompletedSyncPoint() != UINT64_MAX)
			{
				// Execution may have succeeded before Signal failed. Append only a fence, never the commands themselves.
				if (m_unfenced && m_gsync.LastAddedSyncPoint() < m_pending_sync_point)
				{
					auto const sync_point = m_gsync.NextSyncPoint();
					auto const result = m_queue->Signal(m_gsync.m_fence.get(), sync_point);
					if (FAILED(result) && !FAILED(m_gsync.DeviceRemovedReason()))
						return result;

					if (SUCCEEDED(result))
					{
						// Publish the fence without fallible observers; all reservations remain retained until closing completes.
						m_gsync.m_sync = sync_point;
						m_pending_sync_point = sync_point;
						m_unfenced = false;
					}
				}

				// A timeout or host wait failure is not proof that the GPU has released these resources.
				auto const elapsed = GetTickCount64() - started;
				auto const remaining = timeout == INFINITE ? INFINITE : elapsed >= timeout ? 0 : static_cast<DWORD>(timeout - elapsed);
				auto const result = m_gsync.TryWait(m_gsync.LastAddedSyncPoint(), remaining);
				if (result != S_OK && !FAILED(m_gsync.DeviceRemovedReason()) && m_gsync.CompletedSyncPoint() != UINT64_MAX)
					return result;
			}

			// Only unsubmitted reservations are cancelled. The actual fence value is never forged.
			m_unfenced = false;
			m_pending_sync_point = 0;
			m_upload.CancelUnsubmitted();
			m_readback.CancelUnsubmitted();
			return S_OK;
		}

		// Submit the recorded commands and return without waiting for completion.
		RunHandle Submit(RunProfile* profile = nullptr)
		{
			if (!IsRecording())
				throw std::runtime_error("GpuJob::Submit requires an active recording");

			auto const timestamp = [profile]
			{
				return profile != nullptr ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
			};
			auto const prepare_beg = timestamp();

			// Job complete
			CloseRecording();

			auto const execute_beg = timestamp();

			// Run the job.
			ExecuteRecorded();

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
		// 'profile', when supplied, receives the close, execute and recording-reset durations of this flush.
		void Flush(RunProfile* profile = nullptr)
		{
			// Flushing is part of recording, which is not possible while a submission is pending.
			if (!IsRecording())
				throw std::runtime_error("GpuJob::Flush requires an active recording");

			auto const timestamp = [profile]
			{
				return profile != nullptr ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
			};
			auto const prepare_beg = timestamp();

			// Execute the recorded prefix, then record later commands with an allocator the GPU is not using.
			CloseRecording();
			auto const execute_beg = timestamp();
			ExecuteRecorded();
			auto const reset_beg = timestamp();
			ResetRecording();
			m_pending_sync_point = 0;

			// Use the same phase definitions as Submit and Complete, without adding a wait.
			if (profile != nullptr)
			{
				auto const reset_end = timestamp();
				auto elapsed_ms = [](auto beg, auto end)
				{
					return std::chrono::duration<double, std::milli>(end - beg).count();
				};
				profile->m_prepare_ms = elapsed_ms(prepare_beg, execute_beg);
				profile->m_execute_ms = elapsed_ms(execute_beg, reset_beg);
				profile->m_reset_ms = elapsed_ms(reset_beg, reset_end);
			}
		}

		// Wait for a submitted job, then reset command recording for the next submission.
		void Complete(RunHandle& handle, RunProfile* profile = nullptr)
		{
			if (m_closing || m_unfenced || !handle || handle.m_sync_point != m_pending_sync_point)
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

			// GPU completion is final even if an observer or reopening recording subsequently fails.
			m_pending_sync_point = 0;
			handle = {};

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

		}

		// Run the job and block till complete.
		void Run(RunProfile* profile = nullptr)
		{
			auto handle = Submit(profile);
			Complete(handle, profile);
		}

	private:

		// Close exactly once, including when barrier preparation, D3D12 Close, or resource retention fails.
		void CloseRecording()
		{
			// A failed close must never be retried as though the same list were still recording.
			m_recording = false;
			m_barriers.Commit();
			Check(m_cmd_list.Close());
		}

		// Execute the closed list once and retain its completion state even if signalling or an observer throws.
		void ExecuteRecorded()
		{
			// Associate the allocator before execution, since a later Signal failure must not make it reusable.
			m_pending_sync_point = m_gsync.NextSyncPoint();
			m_cmd_list.SyncPoint(m_pending_sync_point);
			CmdListCollection cmd_lists = { m_cmd_list.get() };
			m_unfenced = true;
			m_queue->ExecuteCommandLists(cmd_lists.count(), cmd_lists.data());
			try
			{
				// AddSyncPoint publishes its fence value before invoking observers.
				m_gsync.AddSyncPoint(m_queue.get());
			}
			catch (...)
			{
				// An observer failure does not undo a successful queue signal.
				m_unfenced = m_gsync.LastAddedSyncPoint() < m_pending_sync_point;
				throw;
			}
			m_unfenced = false;
			if (m_closing)
				throw std::runtime_error("GpuJob was closed by a submission observer");
		}

		// Reopen the command list with an available allocator, ready for further recording.
		void ResetRecording()
		{
			// An observer may have requested closure while completing the preceding submission.
			if (m_closing)
				throw std::runtime_error("GpuJob cannot reopen a closing job");

			// Reset clears the bound descriptor heaps, so rebind the view heap.
			m_cmd_list.Reset(m_cmd_pool.Get());
			auto heaps = { m_view_heap.get() };
			m_cmd_list.SetDescriptorHeaps({ heaps.begin(), heaps.size() });
			m_recording = true;
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
			memcpy(upload_buffer.template ptr<T>(), data.data(), data.size() * sizeof(T));
			job.m_cmd_list.CopyBufferRegion(res, 0, upload_buffer);
		}

		// Upload 'data' to 'res' piecemeal
		template <typename T> friend void UploadData(GpuJob& job, ID3D12Resource* res, size_t count, std::invocable<T*, T*> auto cb)
		{
			assert(job.m_cmd_list.ResState(res).Mip0State() == D3D12_RESOURCE_STATE_COPY_DEST && "'res' should be in the COPY_DEST state");

			auto upload_buffer = job.m_upload.Alloc<T>(count);
			cb(upload_buffer.template ptr<T>(), upload_buffer.template end<T>());
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

	// Only the interfaces of the forwarded object's base chain are answered, so D3D12 never receives this object as a different type.
	template <typename Interface>
	struct ForwardingChild : Interface
	{
		D3DPtr<Interface> m_inner;  // The real object that receives the forwarded calls.
		std::atomic<ULONG> m_refs;  // References held by callers, starting with the owner's one.

		explicit ForwardingChild(Interface* inner)
			: m_inner(inner, true)
			, m_refs(1)
		{
		}
		ForwardingChild(ForwardingChild const&) = delete;
		ForwardingChild& operator=(ForwardingChild const&) = delete;

		// The number of references, including the owner's one.
		ULONG RefCount() const
		{
			return m_refs.load();
		}

		// IUnknown
		HRESULT STDMETHODCALLTYPE QueryInterface(REFIID riid, void** ppv) override
		{
			// Answer only the interfaces this object implements.
			if (ppv == nullptr)
				return E_POINTER;

			if (riid == __uuidof(IUnknown) || riid == __uuidof(ID3D12Object) || riid == __uuidof(ID3D12DeviceChild) || riid == __uuidof(ID3D12Pageable) || riid == __uuidof(Interface))
			{
				*ppv = static_cast<Interface*>(this);
				AddRef();
				return S_OK;
			}
			*ppv = nullptr;
			return E_NOINTERFACE;
		}
		ULONG STDMETHODCALLTYPE AddRef() override
		{
			return ++m_refs;
		}
		ULONG STDMETHODCALLTYPE Release() override
		{
			return --m_refs;
		}

		// ID3D12Object
		HRESULT STDMETHODCALLTYPE GetPrivateData(REFGUID guid, UINT* data_size, void* data) override
		{
			return m_inner->GetPrivateData(guid, data_size, data);
		}
		HRESULT STDMETHODCALLTYPE SetPrivateData(REFGUID guid, UINT data_size, void const* data) override
		{
			return m_inner->SetPrivateData(guid, data_size, data);
		}
		HRESULT STDMETHODCALLTYPE SetPrivateDataInterface(REFGUID guid, IUnknown const* data) override
		{
			return m_inner->SetPrivateDataInterface(guid, data);
		}
		HRESULT STDMETHODCALLTYPE SetName(LPCWSTR name) override
		{
			return m_inner->SetName(name);
		}

		// ID3D12DeviceChild
		HRESULT STDMETHODCALLTYPE GetDevice(REFIID riid, void** device) override
		{
			return m_inner->GetDevice(riid, device);
		}
	};

	// A command queue that forwards to a real queue, except that 'Signal' returns E_OUTOFMEMORY without signalling while armed.
	// Command lists always execute on the real queue, so the GPU runs real work. The failure is a simulated result, not a driver fault.
	struct ForwardingQueue : ForwardingChild<ID3D12CommandQueue>
	{
		bool m_fail_signals = false; // While true, 'Signal' fails without reaching the real queue.
		int m_execute_calls = 0;     // The number of 'ExecuteCommandLists' calls forwarded.
		int m_signal_attempts = 0;   // The number of 'Signal' calls, whether forwarded or failed.

		explicit ForwardingQueue(ID3D12CommandQueue* inner)
			: ForwardingChild(inner)
		{
		}

		// ID3D12CommandQueue
		void STDMETHODCALLTYPE UpdateTileMappings(ID3D12Resource* resource, UINT num_regions, D3D12_TILED_RESOURCE_COORDINATE const* region_starts, D3D12_TILE_REGION_SIZE const* region_sizes, ID3D12Heap* heap, UINT num_ranges, D3D12_TILE_RANGE_FLAGS const* range_flags, UINT const* heap_range_starts, UINT const* range_tile_counts, D3D12_TILE_MAPPING_FLAGS flags) override
		{
			m_inner->UpdateTileMappings(resource, num_regions, region_starts, region_sizes, heap, num_ranges, range_flags, heap_range_starts, range_tile_counts, flags);
		}
		void STDMETHODCALLTYPE CopyTileMappings(ID3D12Resource* dst, D3D12_TILED_RESOURCE_COORDINATE const* dst_start, ID3D12Resource* src, D3D12_TILED_RESOURCE_COORDINATE const* src_start, D3D12_TILE_REGION_SIZE const* region_size, D3D12_TILE_MAPPING_FLAGS flags) override
		{
			m_inner->CopyTileMappings(dst, dst_start, src, src_start, region_size, flags);
		}
		void STDMETHODCALLTYPE ExecuteCommandLists(UINT count, ID3D12CommandList* const* lists) override
		{
			++m_execute_calls;
			m_inner->ExecuteCommandLists(count, lists);
		}
		void STDMETHODCALLTYPE SetMarker(UINT metadata, void const* data, UINT size) override
		{
			m_inner->SetMarker(metadata, data, size);
		}
		void STDMETHODCALLTYPE BeginEvent(UINT metadata, void const* data, UINT size) override
		{
			m_inner->BeginEvent(metadata, data, size);
		}
		void STDMETHODCALLTYPE EndEvent() override
		{
			m_inner->EndEvent();
		}
		HRESULT STDMETHODCALLTYPE Signal(ID3D12Fence* fence, UINT64 value) override
		{
			// Inject failure without signalling the real queue, while retaining the attempted submission count.
			++m_signal_attempts;
			if (m_fail_signals)
				return E_OUTOFMEMORY;

			return m_inner->Signal(fence, value);
		}
		HRESULT STDMETHODCALLTYPE Wait(ID3D12Fence* fence, UINT64 value) override
		{
			return m_inner->Wait(fence, value);
		}
		HRESULT STDMETHODCALLTYPE GetTimestampFrequency(UINT64* frequency) override
		{
			return m_inner->GetTimestampFrequency(frequency);
		}
		HRESULT STDMETHODCALLTYPE GetClockCalibration(UINT64* gpu_timestamp, UINT64* cpu_timestamp) override
		{
			return m_inner->GetClockCalibration(gpu_timestamp, cpu_timestamp);
		}
		D3D12_COMMAND_QUEUE_DESC STDMETHODCALLTYPE GetDesc() override
		{
			return m_inner->GetDesc();
		}
	};

	// A fence that forwards to a real fence, except that 'SetEventOnCompletion' returns E_OUTOFMEMORY while armed.
	// It is for host-side waits only. It must never be passed to a queue, which needs the real fence.
	struct ForwardingFence : ForwardingChild<ID3D12Fence>
	{
		bool m_fail_set_event = false; // While true, 'SetEventOnCompletion' fails without registering the event.
		int m_set_event_attempts = 0;  // The number of 'SetEventOnCompletion' calls made while armed.
		std::function<void(UINT64)> m_on_set_event; // Optional observer called with the value of each 'SetEventOnCompletion' call, before it is forwarded or failed. Must not throw.

		explicit ForwardingFence(ID3D12Fence* inner)
			: ForwardingChild(inner)
		{
		}

		// ID3D12Fence
		UINT64 STDMETHODCALLTYPE GetCompletedValue() override
		{
			return m_inner->GetCompletedValue();
		}
		HRESULT STDMETHODCALLTYPE SetEventOnCompletion(UINT64 value, HANDLE event) override
		{
			// Let the observer see the registration, then count and fail each attempt while armed.
			if (m_on_set_event)
				m_on_set_event(value);

			if (m_fail_set_event)
			{
				++m_set_event_attempts;
				return E_OUTOFMEMORY;
			}
			return m_inner->SetEventOnCompletion(value, event);
		}
		HRESULT STDMETHODCALLTYPE Signal(UINT64 value) override
		{
			return m_inner->Signal(value);
		}
	};


	// Exercise failed submissions and closure against real queues while injecting only host-side HRESULT and observer failures.
	PRUnitTestClass(GpuJobLifecycleTests)
	{
		// D3D12's failed Close result must stop submission before any queue execution.
		PRUnitTestMethod(FailedNativeCloseNeverExecutes, Quick)
		{
			// Keep D3D validation but not its process-breaking error policy: this test deliberately closes an already closed native list.
			ComGpu gpu(IsDebug ? EGpuOptions::DeviceDebug : EGpuOptions::None);
			ForwardingQueue proxy(gpu.queue());
			ComputeJob job(gpu.device(), &proxy, "CloseFailure", 0);
			Check(job.m_cmd_list.get()->Close());
			PR_THROWS(job.Submit(), std::runtime_error);
			PR_EXPECT(!job.IsRecording());
			PR_EXPECT(proxy.m_execute_calls == 0 && proxy.m_signal_attempts == 0);
			PR_EXPECT(job.TryClose(0) == S_OK);
		}

		// Execution followed by a failed Signal remains owned; retries append only a fence.
		PRUnitTestMethod(FailedSignalRetainsWorkWithoutReplay, Quick)
		{
			// Keep the proxy, allocations and job alive until the explicit cleanup guard succeeds.
			ComGpu gpu(TestGpuOptions);
			ForwardingQueue proxy(gpu.queue());
			ComputeJob job(gpu.device(), &proxy, "SignalFailure", 0);
			auto allocation = job.m_upload.Alloc<int>(4);
			Scope<void> close([&]
			{
				// The fault is synthetic; disarming it restores the real healthy queue for cleanup.
				proxy.m_fail_signals = false;
				PR_EXPECT(job.TryClose(INFINITE) == S_OK);
			});
			proxy.m_fail_signals = true;
			PR_THROWS(job.Submit(), std::runtime_error);
			PR_EXPECT(job.m_unfenced);
			PR_EXPECT(proxy.m_execute_calls == 1 && proxy.m_signal_attempts == 1);
			PR_EXPECT(job.TryClose(0) == E_OUTOFMEMORY);
			PR_EXPECT(job.m_unfenced && allocation.m_page->m_sync_point == 1);
			PR_EXPECT(proxy.m_execute_calls == 1 && proxy.m_signal_attempts == 2);

			// Successful retirement does not make the job reusable or execute the recorded list a second time.
			proxy.m_fail_signals = false;
			PR_EXPECT(job.TryClose(INFINITE) == S_OK);
			PR_EXPECT(!job.m_unfenced && job.m_pending_sync_point == 0);
			PR_EXPECT(proxy.m_execute_calls == 1 && proxy.m_signal_attempts == 3);
			PR_THROWS(job.Submit(), std::runtime_error);
			PR_EXPECT(job.TryClose(0) == S_OK);
			PR_EXPECT(proxy.m_signal_attempts == 3);
		}

		// A flushed prefix is still outstanding even though no Submit handle exists.
		PRUnitTestMethod(TimeoutAndWaitFailureRetainFlushedWork, Quick)
		{
			// Gate a real queue, then inject failure only into the host-side fence event registration.
			ComGpu gpu(TestGpuOptions);
			ComputeJob job(gpu, "WaitFailure", 0);
			auto real_fence = job.m_gsync.m_fence;
			ForwardingFence proxy(real_fence.get());
			D3DPtr<ID3D12Fence> gate;
			Check(gpu->CreateFence(0, D3D12_FENCE_FLAG_NONE, IID_PPV_ARGS(gate.address_of())));
			GpuTransferAllocation allocation;
			Scope<void> close([&]
			{
				// Restore the real fence before the proxy is destroyed, and unblock before waiting.
				job.m_gsync.m_fence = real_fence;
				(void)gate->Signal(1);
				PR_EXPECT(job.TryClose(INFINITE) == S_OK);
			});
			Check(job.m_queue->Wait(gate.get(), 1));
			job.Flush();
			allocation = job.m_readback.Alloc<int>(4);
			job.m_gsync.m_fence = D3DPtr<ID3D12Fence>(&proxy, true);
			proxy.m_fail_set_event = true;

			// Neither a poll nor a failed host wait cancels the later recording's still-owned reservation.
			PR_EXPECT(job.TryClose(0) == S_FALSE);
			PR_EXPECT(proxy.m_set_event_attempts == 0);
			PR_EXPECT(job.TryClose(10) == E_OUTOFMEMORY);
			PR_EXPECT(proxy.m_set_event_attempts == 1);
			PR_EXPECT(allocation.m_page->m_sync_point == 2);
			proxy.m_fail_set_event = false;
			PR_EXPECT(job.TryClose(1) == S_FALSE);

			// Completion, not elapsed time or a caught exception, permits cancelling the unsubmitted reservation.
			Check(gate->Signal(1));
			PR_EXPECT(job.TryClose(INFINITE) == S_OK);
			PR_EXPECT(allocation.m_page->m_sync_point == 1);
			PR_EXPECT(job.m_gsync.CompletedSyncPoint() == 1);
		}

		// An observer exception after a successful Signal does not require another fence or another execution.
		PRUnitTestMethod(SubmissionObserverFailureKeepsTheSuccessfulFence, Quick)
		{
			// The observer throws after GpuSync has published its successful signal.
			ComGpu gpu(TestGpuOptions);
			ForwardingQueue proxy(gpu.queue());
			ComputeJob job(gpu.device(), &proxy, "SubmissionObserver", 0);
			AutoSub sub = job.m_gsync.SyncPointAdded += [](GpuSync&, EmptyArgs const&)
			{
				// Simulate arbitrary user code failing after submission.
				throw std::runtime_error("Injected submission observer failure");
			};
			PR_THROWS(job.Submit(), std::runtime_error);
			PR_EXPECT(!job.m_unfenced && job.m_pending_sync_point == 1);
			PR_EXPECT(job.TryClose(INFINITE) == S_OK);
			PR_EXPECT(proxy.m_execute_calls == 1 && proxy.m_signal_attempts == 1);
		}

		// GPU completion invalidates the run handle before fallible callbacks, and cleanup never invokes them again.
		PRUnitTestMethod(CompletionObserverFailureConsumesTheHandle, Quick)
		{
			// Record exactly one completion notification.
			ComGpu gpu(TestGpuOptions);
			ComputeJob job(gpu, "CompletionObserver", 0);
			auto notifications = 0;
			AutoSub sub = job.m_gsync.SyncPointCompleted += [&](GpuSync&, EmptyArgs const&)
			{
				// Completion itself is irreversible even when an observer rejects its notification.
				++notifications;
				throw std::runtime_error("Injected completion observer failure");
			};
			auto handle = job.Submit();
			PR_THROWS(job.Complete(handle), std::runtime_error);
			PR_EXPECT(!handle && job.m_pending_sync_point == 0);
			PR_EXPECT(!job.IsRecording());
			PR_EXPECT(job.TryClose(0) == S_OK);
			PR_EXPECT(notifications == 1);
		}

		// Reentrant closure cannot make a surrounding submission report a usable run handle.
		PRUnitTestMethod(SubmissionObserverClosureStopsTheOuterCall, Quick)
		{
			// The callback closes an already signalled submission without recursively invoking itself.
			ComGpu gpu(TestGpuOptions);
			ComputeJob job(gpu, "ReentrantClosure", 0);
			AutoSub sub = job.m_gsync.SyncPointAdded += [&](GpuSync&, EmptyArgs const&)
			{
				// The callback owns the decision to wait for this submission.
				PR_EXPECT(job.TryClose(INFINITE) == S_OK);
			};
			PR_THROWS(job.Submit(), std::runtime_error);
			PR_EXPECT(!job.IsRecording() && job.m_pending_sync_point == 0);
			PR_EXPECT(job.TryClose(0) == S_OK);
		}
	};

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

			ComGpu gpu(TestGpuOptions);
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
			ComGpu gpu(TestGpuOptions);
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
			ComGpu gpu(TestGpuOptions);
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
			ComGpu gpu(TestGpuOptions);
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
			GfxGpu gpu(TestGpuOptions);
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
			GfxGpu gpu(TestGpuOptions);
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
			GfxGpu gpu(TestGpuOptions);
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
			GfxGpu gpu(TestGpuOptions);
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
			ComGpu gpu(TestGpuOptions);
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
