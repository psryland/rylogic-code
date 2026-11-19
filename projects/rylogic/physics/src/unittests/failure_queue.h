//*********************************************
// Physics GPU Test Support
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/physics/forward.h"

namespace pr::physics::tests
{
	// Stack-owned queue proxy: work runs on the real queue, but selected Signal calls fail without reaching it.
	// Keep the proxy alive until borrowed wrappers release their references. This simulates an HRESULT, not a driver fault.
	struct FailureQueue final :ID3D12CommandQueue
	{
		D3DPtr<ID3D12CommandQueue> m_inner;
		std::atomic<ULONG> m_refs = 1;
		int m_failures = 0;
		int m_executes = 0;
		int m_signals = 0;

		// Retain the real queue for every forwarded operation.
		explicit FailureQueue(ID3D12CommandQueue* inner)
			: m_inner(inner, true)
		{}
		FailureQueue(FailureQueue const&) = delete;
		FailureQueue& operator=(FailureQueue const&) = delete;

		// Expose only interfaces actually implemented by this proxy.
		HRESULT STDMETHODCALLTYPE QueryInterface(REFIID iid, void** object) override
		{
			if (object == nullptr)
				return E_POINTER;

			if (iid == __uuidof(IUnknown) || iid == __uuidof(ID3D12Object) || iid == __uuidof(ID3D12DeviceChild) || iid == __uuidof(ID3D12Pageable) || iid == __uuidof(ID3D12CommandQueue))
			{
				*object = static_cast<ID3D12CommandQueue*>(this);
				AddRef();
				return S_OK;
			}
			*object = nullptr;
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
		HRESULT STDMETHODCALLTYPE GetPrivateData(REFGUID guid, UINT* size, void* data) override
		{
			return m_inner->GetPrivateData(guid, size, data);
		}
		HRESULT STDMETHODCALLTYPE SetPrivateData(REFGUID guid, UINT size, void const* data) override
		{
			return m_inner->SetPrivateData(guid, size, data);
		}
		HRESULT STDMETHODCALLTYPE SetPrivateDataInterface(REFGUID guid, IUnknown const* data) override
		{
			return m_inner->SetPrivateDataInterface(guid, data);
		}
		HRESULT STDMETHODCALLTYPE SetName(LPCWSTR name) override
		{
			return m_inner->SetName(name);
		}
		HRESULT STDMETHODCALLTYPE GetDevice(REFIID iid, void** device) override
		{
			return m_inner->GetDevice(iid, device);
		}
		void STDMETHODCALLTYPE UpdateTileMappings(ID3D12Resource* resource, UINT region_count, D3D12_TILED_RESOURCE_COORDINATE const* starts, D3D12_TILE_REGION_SIZE const* sizes, ID3D12Heap* heap, UINT range_count, D3D12_TILE_RANGE_FLAGS const* flags, UINT const* offsets, UINT const* counts, D3D12_TILE_MAPPING_FLAGS mapping_flags) override
		{
			m_inner->UpdateTileMappings(resource, region_count, starts, sizes, heap, range_count, flags, offsets, counts, mapping_flags);
		}
		void STDMETHODCALLTYPE CopyTileMappings(ID3D12Resource* dest, D3D12_TILED_RESOURCE_COORDINATE const* dest_start, ID3D12Resource* source, D3D12_TILED_RESOURCE_COORDINATE const* source_start, D3D12_TILE_REGION_SIZE const* size, D3D12_TILE_MAPPING_FLAGS flags) override
		{
			m_inner->CopyTileMappings(dest, dest_start, source, source_start, size, flags);
		}
		void STDMETHODCALLTYPE ExecuteCommandLists(UINT count, ID3D12CommandList* const* lists) override
		{
			++m_executes;
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
			++m_signals;
			if (m_failures != 0)
			{
				--m_failures;
				return E_OUTOFMEMORY;
			}
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
		HRESULT STDMETHODCALLTYPE GetClockCalibration(UINT64* gpu, UINT64* cpu) override
		{
			return m_inner->GetClockCalibration(gpu, cpu);
		}
		D3D12_COMMAND_QUEUE_DESC STDMETHODCALLTYPE GetDesc() override
		{
			return m_inner->GetDesc();
		}
	};
}
