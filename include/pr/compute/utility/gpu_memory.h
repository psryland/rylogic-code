//*********************************************
// GPU allocation diagnostics
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/compute/forward.h"

// Remove the arguments as well as the bookkeeping from non-Debug builds.
#ifdef NDEBUG
#define PR_GPU_MEMORY_TRACK(...) ((void)0)
#else
#define PR_GPU_MEMORY_TRACK(...) ::pr::compute::TrackGpuMemory(__VA_ARGS__)

namespace pr::compute
{
	// Receives one live backing allocation. The name is valid only during the call; callbacks must not re-enter the registry.
	using GpuMemoryVisitor = void (__cdecl*)(void* context, char const* name, uint64_t bytes, int heap_type, int dimension);

	// One device-wide registry shared by modules that statically link compute. The interface deliberately has no STL-valued ABI.
	struct __declspec(uuid("D92F3B1A-9B23-48B2-AC68-5534BB232763")) IGpuMemoryRegistry : IUnknown
	{
		// Record a new backing allocation without retaining its resource.
		virtual void Add(uintptr_t id, char const* name, uint64_t bytes, int heap_type, int dimension) = 0;
		// Remove an allocation when its D3D object is actually destroyed.
		virtual void Remove(uintptr_t id) = 0;
		// Visit a coherent live set while holding the registry lock.
		virtual void Visit(GpuMemoryVisitor visitor, void* context) = 0;
	};

	namespace gpu_memory
	{
		inline constexpr GUID LifetimeKey = {0xd996a369, 0x5308, 0x48cb, {0xa8, 0x6a, 0x31, 0xcf, 0xc3, 0xee, 0xea, 0x9b}};

		// Metadata for an allocation; no resource or device reference is kept here.
		struct Allocation
		{
			std::string m_name;
			uint64_t m_bytes;
			int m_heap_type;
			int m_dimension;
		};

		// Owns only the live metadata, so deferred GPU releases remain visible without delaying destruction.
		class Registry : public Microsoft::WRL::RuntimeClass<Microsoft::WRL::RuntimeClassFlags<Microsoft::WRL::ClassicCom>, IGpuMemoryRegistry>
		{
			std::mutex m_mutex;
			std::unordered_map<uintptr_t, Allocation> m_allocations;

		public:
			// Add one allocation with its caller-supplied purpose label.
			void Add(uintptr_t id, char const* name, uint64_t bytes, int heap_type, int dimension) override
			{
				// Duplicate tracking would count one backing allocation twice.
				std::lock_guard lock(m_mutex);
				if (!m_allocations.emplace(id, Allocation{name, bytes, heap_type, dimension}).second)
					throw std::logic_error("GPU backing allocation is already tracked");
			}

			// Forget exactly the allocation whose lifetime token is ending.
			void Remove(uintptr_t id) override
			{
				// No D3D calls occur under this lock.
				std::lock_guard lock(m_mutex);
				m_allocations.erase(id);
			}

			// Read one coherent set without synchronizing any GPU queues.
			void Visit(GpuMemoryVisitor visitor, void* context) override
			{
				// Callbacks copy metadata only and must not allocate D3D resources.
				std::lock_guard lock(m_mutex);
				for (auto const& [id, allocation] : m_allocations)
				{
					// The label stays valid until the callback has copied it.
					(void)id;
					visitor(context, allocation.m_name.c_str(), allocation.m_bytes, allocation.m_heap_type, allocation.m_dimension);
				}
			}
		};

		// An object's private COM data ends with its true D3D lifetime, not with one owner's smart pointer.
		class Lifetime : public Microsoft::WRL::RuntimeClass<Microsoft::WRL::RuntimeClassFlags<Microsoft::WRL::ClassicCom>, IUnknown>
		{
			Microsoft::WRL::ComPtr<IGpuMemoryRegistry> m_registry;
			uintptr_t m_id;

		public:
			// Retain the registry, but never the tracked object or its device.
			Lifetime(IGpuMemoryRegistry* registry, uintptr_t id, char const* name, uint64_t bytes, int heap_type, int dimension)
				: m_registry(registry)
				, m_id(id)
			{
				// A failed registration never constructs a token that could remove another allocation.
				m_registry->Add(m_id, name, bytes, heap_type, dimension);
			}

			// End tracking when the object's last actual reference is released.
			~Lifetime()
			{
				// Removal does not depend on device liveness or GPU completion queries.
				m_registry->Remove(m_id);
			}
		};

		// Find the device's registry. Only first-time creation needs the cross-module initialization mutex.
		inline Microsoft::WRL::ComPtr<IGpuMemoryRegistry> GetRegistry(ID3D12Device* device)
		{
			// D3D private interface data takes an independent reference on retrieval.
			Microsoft::WRL::ComPtr<IGpuMemoryRegistry> registry;
			auto size = UINT(sizeof(IGpuMemoryRegistry*));
			auto hr = device->GetPrivateData(__uuidof(IGpuMemoryRegistry), &size, registry.GetAddressOf());
			if (hr != DXGI_ERROR_NOT_FOUND)
			{
				// All later allocation calls avoid the kernel mutex.
				Check(hr);
				return registry;
			}

			// Static compute code exists in multiple DLLs, so a C++ static mutex would not serialize their first calls.
			auto name = std::format(L"Local\\Rylogic.GpuMemory.{}", GetCurrentProcessId());
			auto mutex = CreateMutexW(nullptr, FALSE, name.c_str());
			if (mutex == nullptr)
				Check(HRESULT_FROM_WIN32(GetLastError()));

			auto close = pr::Scope<void>([&]
			{
				// Keep the kernel handle alive until the initialization lock is released.
				CloseHandle(mutex);
			});
			auto wait = WaitForSingleObject(mutex, INFINITE);
			if (wait != WAIT_OBJECT_0 && wait != WAIT_ABANDONED)
				Check(HRESULT_FROM_WIN32(GetLastError()));

			auto unlock = pr::Scope<void>([&]
			{
				// An abandoned lock is owned by this thread too.
				ReleaseMutex(mutex);
			});

			// Another module may have installed the registry while this call was waiting.
			size = UINT(sizeof(IGpuMemoryRegistry*));
			hr = device->GetPrivateData(__uuidof(IGpuMemoryRegistry), &size, registry.GetAddressOf());
			if (hr == DXGI_ERROR_NOT_FOUND)
			{
				// The device and allocation tokens own the registry; the registry owns neither of them.
				registry = Microsoft::WRL::Make<Registry>();
				if (registry == nullptr)
					throw std::bad_alloc();

				Check(device->SetPrivateDataInterface(__uuidof(IGpuMemoryRegistry), registry.Get()));
			}
			else
			{
				// Do not turn a private-data failure into an empty report.
				Check(hr);
			}
			return registry;
		}

		// Attach a destruction notification to one backing allocation.
		inline void Track(ID3D12Device* device, ID3D12Pageable* object, std::string_view name, uint64_t bytes, D3D12_HEAP_TYPE heap_type, D3D12_RESOURCE_DIMENSION dimension)
		{
			// Copy the purpose now; callers can pass temporary labels.
			auto registry = GetRegistry(device);
			auto label = std::string(name);
			auto id = reinterpret_cast<uintptr_t>(object);
			auto lifetime = Microsoft::WRL::Make<Lifetime>(registry.Get(), id, label.c_str(), bytes, int(heap_type), int(dimension));
			if (lifetime == nullptr)
				throw std::bad_alloc();

			Check(object->SetPrivateDataInterface(LifetimeKey, lifetime.Get()));
		}
	}

	// Track a committed resource (or swap-chain buffer). Do not use for placed or reserved resources; track their backing heap instead.
	inline void TrackGpuMemory(ID3D12Resource* resource, std::string_view name)
	{
		// D3D's allocation footprint includes alignment, all mip levels, array slices and multisampling.
		Microsoft::WRL::ComPtr<ID3D12Device> device;
		Check(resource->GetDevice(IID_PPV_ARGS(device.GetAddressOf())));
		auto desc = resource->GetDesc();
		auto info = device->GetResourceAllocationInfo(0, 1, &desc);
		if (info.SizeInBytes == UINT64_MAX)
			throw std::runtime_error("Cannot determine GPU allocation size");

		D3D12_HEAP_PROPERTIES properties = {};
		D3D12_HEAP_FLAGS flags = {};
		Check(resource->GetHeapProperties(&properties, &flags));
		gpu_memory::Track(device.Get(), resource, name, info.SizeInBytes, properties.Type, desc.Dimension);
	}

	// Track a heap once, irrespective of how many placed resources alias its storage.
	inline void TrackGpuMemory(ID3D12Heap* heap, std::string_view name)
	{
		// Heap size is the backing allocation; placed resource footprints must not be added again.
		Microsoft::WRL::ComPtr<ID3D12Device> device;
		Check(heap->GetDevice(IID_PPV_ARGS(device.GetAddressOf())));
		auto desc = heap->GetDesc();
		gpu_memory::Track(device.Get(), heap, name, desc.SizeInBytes, desc.Properties.Type, D3D12_RESOURCE_DIMENSION_UNKNOWN);
	}

	// Visit this device's live backing allocations, including allocations made by other modules using the same SDK.
	inline void VisitGpuMemory(ID3D12Device* device, GpuMemoryVisitor visitor, void* context)
	{
		// Registry metadata is locked only while the caller copies it; this never waits for the GPU.
		gpu_memory::GetRegistry(device)->Visit(visitor, context);
	}
}
#endif
