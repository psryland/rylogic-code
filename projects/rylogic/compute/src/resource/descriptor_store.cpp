//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/compute/resource/descriptor_store.h"

namespace pr::compute
{
	DescriptorStore::DescriptorStore(ID3D12Device* device)
		: m_device(device)
		, m_store_cpu()
		, m_hint_free()
		, m_generation()
	{}

	// Create a CBV descriptor
	Descriptor DescriptorStore::Create(D3D12_CONSTANT_BUFFER_VIEW_DESC const& desc)
	{
		constexpr auto type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;

		// Find the free slot in the block and create the descriptor there
		auto& block = GetBlock(type);
		auto des_index = LowBitIndex(block.free);

		// Create the descriptor at 'index'
		auto handle = block.heap->GetCPUDescriptorHandleForHeapStart();
		handle.ptr += des_index * m_device->GetDescriptorHandleIncrementSize(type);
		m_device->CreateConstantBufferView(&desc, handle);
		auto generation = ++m_generation;
		block.free = SetBits(block.free, 1ULL << des_index, false);

		// Add the block index to 'index' and return
		auto blk_index = s_cast<int>(&block - m_store_cpu[type].data());
		return Descriptor((blk_index << ShftBlk) | des_index, type, handle, generation);
	}

	// Create a SRV descriptor
	Descriptor DescriptorStore::Create(ID3D12Resource* resource, D3D12_SHADER_RESOURCE_VIEW_DESC const& desc)
	{
		constexpr auto type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;

		// Find the free slot in the block and create the descriptor there
		auto& block = GetBlock(type);
		auto des_index = LowBitIndex(block.free);

		// Create the descriptor at 'index'
		auto handle = block.heap->GetCPUDescriptorHandleForHeapStart();
		handle.ptr += des_index * m_device->GetDescriptorHandleIncrementSize(type);
		m_device->CreateShaderResourceView(resource, &desc, handle);
		auto generation = ++m_generation;
		block.free = SetBits(block.free, 1ULL << des_index, false);

		// Add the block index to 'index' and return
		auto blk_index = s_cast<int>(&block - m_store_cpu[type].data());
		return Descriptor((blk_index << ShftBlk) | des_index, type, handle, generation);
	}

	// Create a UAV descriptor
	Descriptor DescriptorStore::Create(ID3D12Resource* resource, D3D12_UNORDERED_ACCESS_VIEW_DESC const& desc)
	{
		constexpr auto type = D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV;

		// Find the free slot in the block and create the descriptor there
		auto& block = GetBlock(type);
		auto des_index = LowBitIndex(block.free);

		// Create the descriptor at 'index'
		auto handle = block.heap->GetCPUDescriptorHandleForHeapStart();
		handle.ptr += des_index * m_device->GetDescriptorHandleIncrementSize(type);
		m_device->CreateUnorderedAccessView(resource, nullptr, &desc, handle);
		auto generation = ++m_generation;
		block.free = SetBits(block.free, 1ULL << des_index, false);

		// Add the block index to 'index' and return
		auto blk_index = s_cast<int>(&block - m_store_cpu[type].data());
		return Descriptor((blk_index << ShftBlk) | des_index, type, handle, generation);
	}

	// Create a RTV descriptor
	Descriptor DescriptorStore::Create(ID3D12Resource* resource, D3D12_RENDER_TARGET_VIEW_DESC const& desc)
	{
		constexpr auto type = D3D12_DESCRIPTOR_HEAP_TYPE_RTV;

		// Find the free slot in the block and create the descriptor there
		auto& block = GetBlock(type);
		auto des_index = LowBitIndex(block.free);

		// Create the descriptor at 'index'
		auto handle = block.heap->GetCPUDescriptorHandleForHeapStart();
		handle.ptr += des_index * m_device->GetDescriptorHandleIncrementSize(type);
		m_device->CreateRenderTargetView(resource, &desc, handle);
		auto generation = ++m_generation;
		block.free = SetBits(block.free, 1ULL << des_index, false);

		// Add the block index to 'index' and return
		auto blk_index = s_cast<int>(&block - m_store_cpu[type].data());
		return Descriptor((blk_index << ShftBlk) | des_index, type, handle, generation);
	}

	// Create a DSV descriptor
	Descriptor DescriptorStore::Create(ID3D12Resource* resource, D3D12_DEPTH_STENCIL_VIEW_DESC const& desc)
	{
		constexpr auto type = D3D12_DESCRIPTOR_HEAP_TYPE_DSV;

		// Find the free slot in the block and create the descriptor there
		auto& block = GetBlock(type);
		auto des_index = LowBitIndex(block.free);

		// Create the descriptor at 'index'
		auto handle = block.heap->GetCPUDescriptorHandleForHeapStart();
		handle.ptr += des_index * m_device->GetDescriptorHandleIncrementSize(type);
		m_device->CreateDepthStencilView(resource, &desc, handle);
		auto generation = ++m_generation;
		block.free = SetBits(block.free, 1ULL << des_index, false);

		// Add the block index to 'index' and return
		auto blk_index = s_cast<int>(&block - m_store_cpu[type].data());
		return Descriptor((blk_index << ShftBlk) | des_index, type, handle, generation);
	}

	// Create a sampler descriptor
	Descriptor DescriptorStore::Create(D3D12_SAMPLER_DESC const& desc)
	{
		constexpr auto type = D3D12_DESCRIPTOR_HEAP_TYPE_SAMPLER;

		// Find the free slot in the block and create the descriptor there
		auto& block = GetBlock(type);
		auto des_index = LowBitIndex(block.free);

		// Create the descriptor at 'index'
		auto handle = block.heap->GetCPUDescriptorHandleForHeapStart();
		handle.ptr += des_index * m_device->GetDescriptorHandleIncrementSize(type);
		m_device->CreateSampler(&desc, handle);
		auto generation = ++m_generation;
		block.free = SetBits(block.free, 1ULL << des_index, false);

		// Add the block index to 'index' and return
		auto blk_index = s_cast<int>(&block - m_store_cpu[type].data());
		return Descriptor((blk_index << ShftBlk) | des_index, type, handle, generation);
	}

	// Release a descriptor by index
	void DescriptorStore::Release(Descriptor const& descriptor)
	{
		if (descriptor.m_type < 0 || descriptor.m_type >= D3D12_DESCRIPTOR_HEAP_TYPE_NUM_TYPES)
			throw std::runtime_error("Invalid descriptor type");

		auto blk_index = descriptor.m_index >> ShftBlk;
		auto des_index = descriptor.m_index & MaskIdx;
		auto& block = m_store_cpu[descriptor.m_type][blk_index];
		block.free = SetBits(block.free, 1ULL << des_index, true);
	}

	// Return a block with a free slot from 'store'
	DescriptorStore::Block& DescriptorStore::GetBlock(D3D12_DESCRIPTOR_HEAP_TYPE type)
	{
		auto& store = m_store_cpu[type];

		// Find a block with a free slot.
		int i = 0, iend = s_cast<int>(store.size());
		if (m_hint_free[type] != iend && store[m_hint_free[type]].free != 0)
			i = m_hint_free[type];
		else
			for (; i != iend && store[i].free == 0; ++i) {}

		// If all blocks are full, allocate a new one
		if (i == iend)
		{
			// Add a new block to the store
			D3D12_DESCRIPTOR_HEAP_DESC heap_desc = {
				.Type = type,
				.NumDescriptors = 1U << ShftBlk,
				.Flags = D3D12_DESCRIPTOR_HEAP_FLAG_NONE,
				.NodeMask = 0U,
			};
			Block block = {.heap = nullptr, .free = ~0ULL};
			Check(m_device->CreateDescriptorHeap(&heap_desc, __uuidof(ID3D12DescriptorHeap), (void**)block.heap.address_of()));
			store.push_back(block);
			i = iend;
		}

		m_hint_free[type] = i;
		return store[i];
	}
}
