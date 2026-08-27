//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/utility/device_removed.h"

namespace pr::compute
{
	struct GpuSync
	{
		// Notes:
		//  - There can be any number of 'GpuSync' (i.e. fences) in use at any time.
		//  - m_fence->Signal() sets the fence value from the CPU side (i.e. immediately)
		//  - m_queue->Signal() gets the GPU to set the fence value when it encounters it in the command queue.
		//
		// How to use this:
		//  - Add a sync point to a command list => get a number
		//  - Call Wait using the number to block until the GPU has reached that point in the command list.
		//  - This GpuSync will only get notified about fence values that it has added itself, so you can have multiple GpuSync
		//    objects in use at the same time without them interfering with each other.
		// 
		// Polling/Sweep:
		//  - Owners of these objects should use 'rdr.AddPollCB({ &m_gsync, &GpuSync::Poll });' to add
		//    Poll() function to the renderer's periodic timer. This should completely automate the notification
		//    of sync points being reached.

		ID3D12Device4* m_device;     // The device used to create the fence
		D3DPtr<ID3D12Fence> m_fence; // For signalling completed execution of commands.
		Handle m_event;              // The event that is signalled by Dx12 when a command list is complete
		uint64_t m_sync;             // The sync value of the last queued job.
		uint64_t m_notified;         // The sync value last notified

		GpuSync()
			: m_device()
			, m_fence()
			, m_event()
			, m_sync()
			, m_notified()
		{}
		explicit GpuSync(ID3D12Device4* device)
			: GpuSync()
		{
			Init(device);
		}
		GpuSync(GpuSync&&) = default;
		GpuSync(GpuSync const&) = delete;
		GpuSync& operator=(GpuSync&&) = default;
		GpuSync& operator=(GpuSync const&) = delete;
		~GpuSync()
		{
			Release();
		}

		// Return the device used to create this gpu fence
		ID3D12Device4* D3DDevice()
		{
			return m_device;
		}

		// Create sync objects
		void Init(ID3D12Device4* device)
		{
			Release();

			m_device = device;
			Check(device->CreateFence(0, D3D12_FENCE_FLAG_NONE, __uuidof(ID3D12Fence), (void**)m_fence.address_of()));
			m_event = CreateEventExW(nullptr, nullptr, 0, EVENT_ALL_ACCESS);
			Check(m_event != nullptr, "Creating an event for the thread fence failed");
		}

		// Release COM pointers
		void Release()
		{
			// Ensure there are no outstanding tasks on the GPU
			if (m_fence != nullptr)
				Wait();

			// Release
			m_event.close();
			m_fence = nullptr;
			m_device = nullptr;
		}

		// The sync point last added by this GpuSync instance.
		uint64_t LastAddedSyncPoint() const
		{
			return m_sync;
		}

		// The sync point that will be added next time 'AddSyncPoint' is called.
		uint64_t NextSyncPoint() const
		{
			return m_sync + 1;
		}

		// The sync point that this GpuSync (a.k.a fence) has reached so far.
		uint64_t CompletedSyncPoint() const
		{
			return m_fence->GetCompletedValue();
		}

		// Return the reason that the owning D3D12 device was removed, or S_OK while it remains usable.
		HRESULT DeviceRemovedReason() const
		{
			return m_device != nullptr ? m_device->GetDeviceRemovedReason() : S_OK;
		}

		// Add a synchronisation point to 'queue'. Returns the sync point number to wait for.
		uint64_t AddSyncPoint(ID3D12CommandQueue* queue)
		{
			auto next_sync = m_sync + 1;
			auto signal_result = queue->Signal(m_fence.get(), next_sync);
			if (FAILED(signal_result))
			{
				auto reason = DeviceRemovedReason();
				if (FAILED(reason))
					throw DeviceRemovedException(m_device, reason);

				Check(signal_result);
			}
			m_sync = next_sync;
			SyncPointAdded(*this);
			return m_sync;
		}

		// Wait until the given sync point value is reached.
		// Returns true if 'sync_point' is reached, false it timeout.
		bool Wait(uint64_t sync_point, DWORD timeout = INFINITE) const
		{
			// Wait until the fence reports a completed sync point >= 'sync_point'
			for (auto sp = CompletedSyncPoint(); sp < sync_point; sp = CompletedSyncPoint())
			{
				Check(m_fence->SetEventOnCompletion(sync_point, m_event));
				switch (WaitForSingleObject(m_event, timeout))
				{
					case WAIT_OBJECT_0: break; // Event signalled, go round again
					case WAIT_TIMEOUT: return false; // Timeout,
					case WAIT_FAILED: throw std::runtime_error(HrMsg(GetLastError()));
					case WAIT_ABANDONED: throw std::runtime_error("Wait for sync point abandoned");
				}
			}
			return !FAILED(DeviceRemovedReason());
		}
		
		// Wait till the last sync point is reached
		void Wait() const
		{
			Wait(LastAddedSyncPoint());
		}

		// Raised when AddSyncPoint is called
		EventHandler<GpuSync&, EmptyArgs const&, true> SyncPointAdded;

		// Raised when the GPU reaches a sync point
		EventHandler<GpuSync&, EmptyArgs const&, true> SyncPointCompleted;

		// Polling function to monitor for sync points reached
		void Poll()
		{
			for (;;)
			{
				// While the last notified sync point is less than the
				// completed sync point, notify all up to the current.
				auto completed = CompletedSyncPoint();
				if (completed == m_notified) break;
				m_notified = completed;
				SyncPointCompleted(*this);
			}
		}
		static void __stdcall Poll(void* ctx)
		{
			static_cast<GpuSync*>(ctx)->Poll();
		}
	};
}
