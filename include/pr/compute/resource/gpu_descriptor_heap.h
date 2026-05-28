//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/resource/descriptor.h"
#include "pr/compute/utility/lookup.h"
#include "pr/compute/utility/gpu_sync.h"

namespace pr::compute
{
	template <D3D12_DESCRIPTOR_HEAP_TYPE HeapType>
	struct GpuDescriptorHeap
	{
		// TODO: The GpuDescriptorHeap needs to be dynamic because we don't know how many we'll
		//  need at draw time, and we're forced to set the descriptor heaps in the command list
		//  up front.
		// The modern way to do this is to use a Bindless table. If you’re on D3D12 Tier 2+ or
		//  Vulkan descriptor indexing, use a bindless table. You create one giant descriptor
		//  heap (e.g. all SRVs for the whole scene). Shaders receive indices into that heap
		//  instead of fixed bindings. The command list doesn’t need per-object descriptor
		//  sets at all.
		//  Pros:
		//    - Super flexible: no per-draw descriptor binding.
		//    - Perfect for modern forward+ or clustered shading.
		//    - Great GPU parallelism — CPU stays out of the way.
		//  Cons:
		//    - Slightly more complex resource lifetime tracking.
		//    - Requires GPU feature support.
		//
		// Notes:
		//  - This heap is used to pass descriptors to the GPU. Use a DescriptorStore for long
		//    term storage of the descriptors. This heap type can be bound to a command list,
		//    and descriptors are copied from the store into here.
		//  - The heap is treated like a ring buffer, with 'sync points' interleaved.
		//  - Have one of these per command list, per heap type (SRV, Sampler).
		//  - A sync point marks a new "frame" of GPU descriptors.
		//  - The tail of the ring buffer advances as sync points are reached by the GPU.
		//
		// Usage Patterns:
		//  Textures:
		//    For textures created with the Resource Manager, SRV/UAVs are created and added to
		//    the ResourceManager's descriptor store. When these textures are rendered by RenderForward
		//    there is a shared GpuDescriprHeap owned by the window. Descriptors are copied from the 
		//    store to the heap using 'Add' which creates a GPU handle pointing at the first descriptor.
		//    This handle is then set on the command list using 'SetGraphicsRootDescriptorTable'.
		//  Compute Shaders:
		//    For resources used in compute shaders, the ResourceManager's descriptor store can still be
		//    used, but the code managing the compute shader should create it's own GpuDescriptorHeap.
		//    (If using ComputeJob, there's one already in there). Copy the UAV/SRV descriptors into the heap
		//    for each job run.
		//  Constant Buffer View:
		//    1. Use an Upload Heap
		//    2. In Shader use: cbuffer my_cbuf : register(bN);
		//    3. In RootSig use: RootSig(ERootSigFlags::ComputeOnly).CBuf(ECBufReg::uN)
		//    4. In Job use: cmd_list.SetComputeRootConstantBufferView(n, upload.Add(my_cbuf, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true));
		//  Texture in Compute Shader:
		//    1. In Shader use: RWTexture2D<float4> my_tex : register(uN);
		//    2. In RootSig use: RootSig(ERootSigFlags::ComputeOnly).Uav(EUAVReg::uN)
		//    3. In Job use: cmd_list.SetComputeRootDescriptorTable(n, upload.Add(my_tex->m_uav));
		//    4. Use Barrier.UAV(my_tex->m_res.get()) before using the texture.
		//    
	private:

		using HeapPtr = D3DPtr<ID3D12DescriptorHeap>;
		using Lookup = Lookup<uint64_t, D3D12_GPU_DESCRIPTOR_HANDLE>;
		using SyncPoint = struct { uint64_t m_sync_point; int m_index; };
		using SyncPoints = pr::deque<SyncPoint>;

		HeapPtr    m_heap;     // The shader visible heap for descriptors.
		int        m_size;     // The total size of the heap
		GpuSync*   m_gsync;    // The GPU fence marking GPU progress.
		SyncPoints m_sync;     // Positions in the ring buffer and associated sync points.
		Lookup     m_lookup;   // A lookup for descriptors added since the last sync point.
		AutoSub    m_eh0;      // Event subscription
		int        m_des_size; // The size of one descriptor
		int        m_head;     // Insert point for added descriptors
	
	public:
	
		GpuDescriptorHeap(int size, GpuSync& gsync)
			:m_heap()
			,m_size(size)
			,m_gsync(&gsync)
			,m_sync()
			,m_lookup()
			,m_eh0()
			,m_des_size(s_cast<int>(device()->GetDescriptorHandleIncrementSize(HeapType)))
			,m_head()
		{
			if (size < 1)
				throw std::runtime_error("Heap capacity must be >= 1");

			// Create the GPU heap
			D3D12_DESCRIPTOR_HEAP_DESC desc = {
				.Type = HeapType,
				.NumDescriptors = s_cast<UINT>(m_size),
				.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_SHADER_VISIBLE,
				.NodeMask = 0U,
			};
			Check(device()->CreateDescriptorHeap(&desc, __uuidof(ID3D12DescriptorHeap), (void**)m_heap.address_of()));
			
			// Add a dummy sync point to be the current tail.
			m_sync.push_back({0ULL, 0});

			// Sign up for notifications when a sync point is added
			m_eh0 = m_gsync->SyncPointAdded += [this](GpuSync&, EmptyArgs const&)
			{
				// This begins a new "frame" of descriptors. 
				m_sync.push_back({m_gsync->LastAddedSyncPoint(), m_head});
				m_lookup.clear();
				PurgeCompleted();
			};
		}

		// The pointer to the base of the shader visible heap
		ID3D12DescriptorHeap const* get() const
		{
			return m_heap.get();
		}
		ID3D12DescriptorHeap* get()
		{
			return const_call(get());
		}
		
		// Given a range of descriptors, ensure they exist in the GPU heap, and return the handle of the first one
		D3D12_GPU_DESCRIPTOR_HANDLE Add(std::span<Descriptor const> descriptors)
		{
			// Checks
			#if PR_DBG_RDR
			for (auto& des : descriptors)
			{
				if (des.m_type != HeapType)
					throw std::runtime_error("Descriptor is the wrong type");
			}
			#endif

			// Hash the full descriptor-table identity to generate a lookup key. The generation distinguishes a descriptor-store slot that has
			// been released and re-used before the GPU heap cache is cleared at the next sync point.
			auto key = hash::FNV_offset_basis64;
			auto const heap_type = static_cast<uint64_t>(HeapType);
			auto const count64 = static_cast<uint64_t>(descriptors.size());
			key = hash::HashBytes64(&heap_type, &heap_type + 1, key);
			key = hash::HashBytes64(&count64, &count64 + 1, key);
			for (auto& des : descriptors)
			{
				auto const index = static_cast<uint64_t>(des.m_index);
				key = hash::HashBytes64(&index, &index + 1, key);
				key = hash::HashBytes64(&des.m_generation, &des.m_generation + 1, key);
			}

			// See if this combination of descriptors has already been added
			auto iter = m_lookup.find(key);
			if (iter != end(m_lookup))
				return iter->second;

			// The number of descriptors to add in a contiguous block
			auto count = isize(descriptors);

			// Assert capacity
			PurgeCompleted();
			EnsureCapacity(count);

			// Get the heap handles (CPU side, and GPU side) at the insert position (m_head).
			D3D12_CPU_DESCRIPTOR_HANDLE dest_cpu = {m_heap->GetCPUDescriptorHandleForHeapStart().ptr + m_head * m_des_size};
			D3D12_GPU_DESCRIPTOR_HANDLE dest_gpu = {m_heap->GetGPUDescriptorHandleForHeapStart().ptr + m_head * m_des_size};

			// Copy the descriptors to the GPU heap (one at a time, because 'descriptors' are not contiguous)
			for (auto& des : descriptors)
			{
				device()->CopyDescriptorsSimple(1U, dest_cpu, des.m_cpu, des.m_type);
				dest_cpu = {dest_cpu.ptr + m_des_size};
			}

			// Return the GPU handle for the start of the block of descriptors
			m_head = Wrap(m_head + count, 0, m_size);
			m_lookup[key] = dest_gpu;
			return dest_gpu;
		}
		D3D12_GPU_DESCRIPTOR_HANDLE Add(Descriptor const& descriptor)
		{
			return Add({&descriptor, 1});
		}

		// Add a CBV descriptor to the GPU heap, and return its handle
		D3D12_GPU_DESCRIPTOR_HANDLE Add(D3D12_CONSTANT_BUFFER_VIEW_DESC const& desc) requires (HeapType == D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV)
		{
			// Don't bother with caching, this is intended for one-off descriptors
			auto count = 1;

			// Assert capacity
			PurgeCompleted();
			EnsureCapacity(count);

			// Get the heap handles (CPU side, and GPU side) at the insert position (m_head).
			D3D12_CPU_DESCRIPTOR_HANDLE dest_cpu = { m_heap->GetCPUDescriptorHandleForHeapStart().ptr + m_head * m_des_size };
			D3D12_GPU_DESCRIPTOR_HANDLE dest_gpu = { m_heap->GetGPUDescriptorHandleForHeapStart().ptr + m_head * m_des_size };
			device()->CreateConstantBufferView(&desc, dest_cpu);

			// Return the GPU handle for the descriptor
			m_head = Wrap(m_head + count, 0, m_size);
			return dest_gpu;
		}

		// Add a SRV descriptor to the GPU heap, and return its handle
		D3D12_GPU_DESCRIPTOR_HANDLE Add(ID3D12Resource* resource, D3D12_SHADER_RESOURCE_VIEW_DESC const& desc) requires (HeapType == D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV)
		{
			// Don't bother with caching, this is intended for one-off descriptors
			auto count = 1;

			// Assert capacity
			PurgeCompleted();
			EnsureCapacity(count);

			// Get the heap handles (CPU side, and GPU side) at the insert position (m_head).
			D3D12_CPU_DESCRIPTOR_HANDLE dest_cpu = { m_heap->GetCPUDescriptorHandleForHeapStart().ptr + m_head * m_des_size };
			D3D12_GPU_DESCRIPTOR_HANDLE dest_gpu = { m_heap->GetGPUDescriptorHandleForHeapStart().ptr + m_head * m_des_size };
			device()->CreateShaderResourceView(resource, &desc, dest_cpu);
			
			// Return the GPU handle for the descriptor
			m_head = Wrap(m_head + count, 0, m_size);
			return dest_gpu;
		}

		// Add a UAV descriptor to the GPU heap, and return its handle
		D3D12_GPU_DESCRIPTOR_HANDLE Add(ID3D12Resource* resource, D3D12_UNORDERED_ACCESS_VIEW_DESC const& desc) requires (HeapType == D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV)
		{
			// Don't bother with caching, this is intended for one-off descriptors
			auto count = 1;

			// Assert capacity
			PurgeCompleted();
			EnsureCapacity(count);

			// Get the heap handles (CPU side, and GPU side) at the insert position (m_head).
			D3D12_CPU_DESCRIPTOR_HANDLE dest_cpu = { m_heap->GetCPUDescriptorHandleForHeapStart().ptr + m_head * m_des_size };
			D3D12_GPU_DESCRIPTOR_HANDLE dest_gpu = { m_heap->GetGPUDescriptorHandleForHeapStart().ptr + m_head * m_des_size };
			device()->CreateUnorderedAccessView(resource, nullptr, &desc, dest_cpu);

			// Return the GPU handle for the descriptor
			m_head = Wrap(m_head + count, 0, m_size);
			return dest_gpu;
		}

		// Remove sync points that the GPU has completed, effectively advancing the tail of the ring buffer
		void PurgeCompleted()
		{
			// Always leave at least one sync point, to mark the tail index.
			auto completed = m_gsync->CompletedSyncPoint();
			for (; m_sync.size() > 1 && m_sync[1].m_sync_point <= completed; )
				m_sync.pop_front();
		}

	private:

		// D3D device
		ID3D12Device* device() const
		{
			return m_gsync->D3DDevice();
		}

		// Return the tail index for the ring buffer
		int tail() const
		{
			// The sync point is the first descriptor "after" the sync point,
			// so the tail is the position before it. Anything between the last
			// completed sync point and 'm_head' is still in use by the GPU.
			return Wrap(m_sync.front().m_index - 1, 0, m_size);
		}

		// Return the amount of free space in the ring buffer
		int free() const
		{
			return (tail() - m_head + m_size) % m_size;
		}

		// Check there is room for 'sz' more descriptors.
		void EnsureCapacity(int sz)
		{
			// Is there room between [m_head : buffer_end)
			if (m_size - m_head < sz) // no, roll head back to 0
			{
				// Is tail between [m_head : buffer_end)
				if (free() < m_size - m_head) // yes, overflow
					throw std::runtime_error("The GPU descriptor heap is full. Make it bigger");

				// Roll head back to 0
				m_head = 0;
			}

			// Is there room between [m_head : tail)
			if (free() < sz) // no, overflow
				throw std::runtime_error("The GPU descriptor heap is full. Make it bigger");

			// There is enough room at 'm_head' for a contiguous block of 'sz' descriptors.
			return;
		}
	};
}

