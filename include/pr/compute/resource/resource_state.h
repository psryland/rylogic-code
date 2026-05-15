//*********************************************
// Compute
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"

namespace pr::compute
{
	struct ResStateData
	{
		// Resource State Management:
		//  The recommended way to do this is to buffer transitions per resource per command list,
		//  then before a command list is executed, insert transitions from the global state to the
		//  first required state of the command list. After the command list is queued, update the global
		//  state to the final state for the command list.
		//
		//   * I'm not doing this *
		//
		//  Instead, all resources will have a default state (recorded in the ResourceManager). Command lists
		//  can assume any resource they use will be in its default state. When CmdList::Close() is called,
		//  transitions will be added to return any resource back to its default state.
		//
		// Notes:
		//  - This object is used to determine the final state that a resource (or it's mips) is
		//    in at the end of executing a command list.
		//  - Resource states are maintained *per command list*. I.e., each command list tracks
		//    the state changes of any resources that it operates on.
		//  - Command lists are serialised when executed but can be created in parallel so having
		//    one state tracking structure per resource doesn't work.
		//  - 'm_state[0]' is the state for the 'AllSubresources' special case.
		//  - 'm_state[1:]' maps mip-to-state. States are encoded as [state:24, subresource:8] since
		//    resource states have a max value of 0x80_0000, and textures won't have more than 0xFF mips.
		//  - The flat-map list length is determined by a sentinel.

	private:

		inline static constexpr GUID Guid_ResourceStates = { 0x5DFA5A73, 0xA8A0, 0x466B, { 0xA1, 0x0A, 0x3E, 0x3A, 0x35, 0x87, 0x5B, 0xB3 } };
		inline static constexpr uint32_t StateMask = 0x00FFFFFF;
		inline static constexpr uint32_t IndexMask = 0x000000FF;
		inline static constexpr uint32_t IndexBits = 8;
		struct MipState
		{
			// Encoded as [state:24, subresource:8] 
			uint32_t m_data;

			// Get/Set the sub resource index (i.e. mip level). -1 == ALL mips
			int sub() const
			{
				return int(m_data & IndexMask) - 1;
			}
			void sub(int s)
			{
				assert(s >= -1 && s < int(IndexMask));
				auto idx = s_cast<int>((s + 1) & IndexMask);
				m_data = SetBits(m_data, int(IndexMask), idx);
				assert(sub() == s);
			}

			// Get/Set the resource state
			D3D12_RESOURCE_STATES state() const
			{
				return s_cast<D3D12_RESOURCE_STATES>(m_data >> IndexBits);
			}
			void state(D3D12_RESOURCE_STATES s)
			{
				assert(s_cast<int>(s & StateMask) == s);
				m_data = SetBits(m_data, StateMask << IndexBits, s_cast<uint32_t>(s) << IndexBits);
				assert(state() == s);
			}
		};

		// Records the independent states of up to N mips.
		MipState m_state[16];         // last value is a sentinel

	public:

		ResStateData(D3D12_RESOURCE_STATES default_state)
			: m_state()
		{
			Apply(default_state);
		}

		// Return the main mip state
		D3D12_RESOURCE_STATES Mip0State() const
		{
			return m_state[0].state();
		}

		// True if not all mips have the same (default) state
		bool HasMipSpecificStates() const
		{
			// If 'm_state[1]' is not the sentinel, then there is at least one mip-specific state.
			return m_state[1].sub() != D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES;
		}

		// Return the state of subresource 'sub'.
		D3D12_RESOURCE_STATES operator[](int sub) const
		{
			// Convert 'sub'([-1,N]) to an index in 'm_state'
			int idx = mip_state_index(sub);

			// 'idx' shouldn't equal the sentinel. This would mean
			// 'm_state' is full which should be prevented in 'Apply'.
			assert(idx != _countof(m_state) - 1);

			// If 'sub' == ALL(-1), then the caller is expecting all
			// subresources to have the same state. Throw if this isn't true.
			if (sub == D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES && HasMipSpecificStates())
				throw std::runtime_error("Subresources are not all the same state");

			// Return the state for 'sub'. This might be the default (i.e. m_state[0].state()).
			return m_state[idx].state();
		}

		// Set subresource 'sub' to 'state'.
		void Apply(D3D12_RESOURCE_STATES state, int sub = D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES)
		{
			// If 'sub' == ALL, set all states to 'state'.
			if (sub == D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES)
			{
				memset(&m_state[0], 0, sizeof(m_state));
				m_state[0].state(state);
			}
			
			// Otherwise, find the mip-specific state
			else
			{
				int idx = mip_state_index(sub);

				// If 'idx != 0' then a mip-specific state was found
				if (idx != 0)
				{
					// If 'state' == the default state, remove 'idx'
					if (state == Mip0State())
					{
						assert(idx >= 0 && idx < _countof(m_state) - 1);
						for (auto i = idx; i < _countof(m_state) - 1; ++i)
							m_state[i] = m_state[i+1];
					}
					else // Otherwise, update 'idx'
					{
						assert(m_state[idx].sub() == sub);
						m_state[idx].state(state);
					}
				}

				// If 'idx == 0' then there is no mip-speicifc state for 'sub' yet.
				else
				{
					// If 'state' == the default state, no need to add a mip-specific state
					if (state != Mip0State())
					{
						// Find the next free slot. If 'idx' is the sentinel,
						// then 'm_state' is full. It might need enlarging.
						auto free = mip_state_count();
						if (free == _countof(m_state) - 1)
							throw std::runtime_error("Too many unique mip states");

						m_state[free].sub(sub);
						m_state[free].state(state);
					}
				}
			}
			assert((*this)[sub] == state);
		}

		// Return the mip-specific states
		template <std::invocable<int, D3D12_RESOURCE_STATES> CB>
		void EnumMipSpecificStates(CB cb) const
		{
			for (int i = 1; m_state[i].sub() != D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES && i != _countof(m_state); ++i)
				cb(m_state[i].sub(), m_state[i].state());
		}

		// True if the resource state for all mips equals 'rhs'
		friend bool operator == (ResStateData const& lhs, D3D12_RESOURCE_STATES rhs)
		{
			// The default state == 'rhs' and there are no mip-specific states
			return lhs.Mip0State() == rhs && !lhs.HasMipSpecificStates();
		}
		friend bool operator != (ResStateData const& lhs, D3D12_RESOURCE_STATES rhs)
		{
			return !(lhs == rhs);
		}

	private:

		// Find the index in 'm_state' for 'sub'
		int mip_state_index(int sub) const
		{
			// ALL(-1) is the default state at index 0
			if (sub == int(D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES))
				return 0;

			// Search the mip-specific states
			for (int i = 1; i != _countof(m_state); ++i)
			{
				// Found the state for 'sub'? Return its index
				if (m_state[i].sub() == sub)
					return i;

				// Found the sentinel? No mip-specific state found, return the index for the default state
				if (m_state[i].sub() == D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES)
					return 0;
			}
			throw std::runtime_error("The sentinel has been overwritten");
		}

		// Returns the index of the first free slot
		int mip_state_count() const
		{
			int i = 1; // [0] is the default state and is never free
			for (; i != _countof(m_state) && m_state[i].sub() != D3D12_RESOURCE_BARRIER_ALL_SUBRESOURCES; ++i) {}
			return i;
		}
	};
}

