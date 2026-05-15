//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/compute/resource/descriptor.h"

namespace pr::compute
{
	struct DescriptorStore
	{
		// Notes:
		//  - Descriptor management is confusing a.f.
		//  - You can only bind one SRV heap, and one Sampler heap to a command list.
		//  - "It is recommended (esp. for NVidia cards), to have one shader visible descriptor heap for the lifetime of the application"
		//  - There are two main components:
		//      - A collection of "offline" descriptors that live in CPU memory for the lifetime of the underlying resource.
		//      - A large static ring buffer of GPU visible descriptors that is bound to the GPU once for the life of the application.
		//
		//    When a resource is created, descriptors are created for any views it needs in the CPU memory store.
		//    At draw time, descriptors are copied into the GPU heap on demand. However, many textures will be reused, so we don't want
		//    to blindly add descriptors to the GPU ring buffer, it needs to be smart enough to handle duplicates.
		//    Also, it needs a way to record sync points in the GPU heap ring.
		//
		// "A descriptor heap is not something immutable but an always changing object. When you bind a descriptor table, you are in fact
		//  binding it from any offset. Swapping descriptor heaps is a costly operation you want to avoid at all cost.
		//  The idea is to prepare the descriptors in non GPU visible heaps (as many as you like, they are merely a CPU allocated object)
		//  and copy, on demand, into the GPU visible one in a ring buffer fashion with CopyDescriptor or CopyDescriptorSimple.
		//  Lets say your shader uses a table with 2 CBVs and 2 SRVs, they have to be contiguous, so you will allocate from your GPU heap,
		//  an array of 4, you get a heap offset, copy the needed descriptors to that location, and then bind them with SetGraphicsRootDescriptorTable.
		//  One thing you will have to be careful with is the lifetimes of the descriptors in your GPU heap, as you cannot overwrite them until
		//  the GPU is done processing the commands using them. And last, if many shaders share some common tables, from similar root signature,
		//  you can save on processing by factorizing the updates."

		// Each block of descriptors is 64 long, so that a u64 mask can be used to tell which slots are used.
		// The block index = 'index >> 6' and the index within the block = 'index & 0x3F'
		enum { ShftBlk = 6, MaskIdx = 0x3F, NoIndex = -1 };
		struct Block { D3DPtr<ID3D12DescriptorHeap> heap; uint64_t free; };
		using Store = vector<Block, 16, false>; // A collection of pointers to blocks of descriptors

		ID3D12Device* m_device;
		Store m_store_cpu[D3D12_DESCRIPTOR_HEAP_TYPE_NUM_TYPES]; // A store for each descriptor type
		int m_hint_free[D3D12_DESCRIPTOR_HEAP_TYPE_NUM_TYPES]; // Record the index of block last returned that had free slots.

		explicit DescriptorStore(ID3D12Device* device);

		// Create CPU memory descriptors
		Descriptor Create(D3D12_CONSTANT_BUFFER_VIEW_DESC const& desc);
		Descriptor Create(ID3D12Resource* resource, D3D12_SHADER_RESOURCE_VIEW_DESC const& desc);
		Descriptor Create(ID3D12Resource* resource, D3D12_UNORDERED_ACCESS_VIEW_DESC const& desc);
		Descriptor Create(ID3D12Resource* resource, D3D12_RENDER_TARGET_VIEW_DESC const& desc);
		Descriptor Create(ID3D12Resource* resource, D3D12_DEPTH_STENCIL_VIEW_DESC const& desc);
		Descriptor Create(D3D12_SAMPLER_DESC const& desc);
	
		// Release a descriptor by index
		void Release(Descriptor const& descriptor);

	private:
	
		// Get a block with a free slot
		Block& GetBlock(D3D12_DESCRIPTOR_HEAP_TYPE type);
	};
}

