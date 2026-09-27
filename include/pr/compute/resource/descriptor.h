//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"

namespace pr::compute
{
	struct Descriptor
	{
		int m_index;                       // Descriptor index for lookup in the store
		D3D12_DESCRIPTOR_HEAP_TYPE m_type; // The type of descriptor this is
		D3D12_CPU_DESCRIPTOR_HANDLE m_cpu; // Handle in CPU memory
		uint64_t m_generation;             // Generation for this descriptor-store slot

		Descriptor()
			: Descriptor(0, D3D12_DESCRIPTOR_HEAP_TYPE_NUM_TYPES, D3D12_CPU_DESCRIPTOR_HANDLE{}, 0)
		{}
		Descriptor(int index, D3D12_DESCRIPTOR_HEAP_TYPE type, D3D12_CPU_DESCRIPTOR_HANDLE cpu, uint64_t generation)
			: m_index(index)
			, m_type(type)
			, m_cpu(cpu)
			, m_generation(generation)
		{}
		explicit operator bool() const
		{
			return m_type != D3D12_DESCRIPTOR_HEAP_TYPE_NUM_TYPES;
		}

		// Compare descriptor identities, including reuse of a CPU descriptor-store slot.
		bool operator ==(Descriptor const& rhs) const
		{
			return m_generation == rhs.m_generation && m_cpu.ptr == rhs.m_cpu.ptr && m_index == rhs.m_index && m_type == rhs.m_type;
		}
	};
}
