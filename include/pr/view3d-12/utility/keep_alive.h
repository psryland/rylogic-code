//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/view3d-12/utility/gpu_sync.h"

namespace pr::rdr12
{
	struct KeepAlive
	{
		// Notes:
		//  - This class simply keeps a ref ptr to an object until the 'gsync' notifies that the 
		//    GPU has reached a certain sync point. The 'SyncPointCompleted' event should be 
		//    called automatically by the Timer on the dummy window in the Renderer. This relies
		//    on the GpuSync object registering it's Poll function with the Renderer.

		struct Ref
		{
			D3DPtr<ID3D12Object> ptr;
			D3DPtr<IRefCounted> ptr2;
			uint64_t sync_point;
		};
		using Objects = pr::vector<Ref, 4>;

		Objects m_objects;
		GpuSync& m_gsync;
		AutoSub m_ev_sweep;

		explicit KeepAlive(GpuSync& gsync)
			:m_objects()
			,m_gsync(gsync)
			,m_ev_sweep(m_gsync.SyncPointCompleted += std::bind(&KeepAlive::Sweep, this_ptr(), _1, _2))
		{}
		KeepAlive(KeepAlive const&) = delete;
		KeepAlive& operator = (KeepAlive const&) = delete;
		KeepAlive* this_ptr() { return this; }

		// Add an object to be kept alive until 'sync_point' is reached
		template <RefCountedType T> void Add(D3DPtr<T> obj, std::optional<uint64_t> sync_point = {})
		{
			Ref ref = {};
			if constexpr (std::is_base_of_v<ID3D12Object, T>) ref.ptr = obj;
			if constexpr (std::is_base_of_v<IRefCounted, T>) ref.ptr2 = obj;
			ref.sync_point = sync_point ? *sync_point : m_gsync.NextSyncPoint();
			m_objects.push_back(ref);
		}
		template <RefCountedType T> void Add(T* obj, std::optional<uint64_t> sync_point = {})
		{
			Add(D3DPtr<T>(obj, true), sync_point);
		}

	private:

		// Remove objects whose sync point has been reached
		void Sweep(GpuSync&, EmptyArgs const&)
		{
			auto sync_point = m_gsync.CompletedSyncPoint();
			pr::erase_if_unstable(m_objects, [=](auto obj) { return obj.sync_point <= sync_point; });
		}
	};
}
