//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/compute/forward.h"
#include "pr/view3d-12/utility/cmd_list.h"

namespace pr::rdr12
{
	// Builds a list of raw pointers from various arguments
	template <D3D12_COMMAND_LIST_TYPE ListType>
	struct CmdListCollection
	{
	private:

		using ICommandList = std::conditional_t<ListType == D3D12_COMMAND_LIST_TYPE_DIRECT, ID3D12GraphicsCommandList, ID3D12CommandList>;
		pr::vector<ICommandList*, 16> m_list;

	public:

		template <typename... Args>
		CmdListCollection(Args&&... x)
			:m_list()
		{
			(Add(x), ...);
		}

		UINT count() const
		{
			return s_cast<UINT>(m_list.size());
		}
		ID3D12CommandList* const* data() const
		{
			return reinterpret_cast<ID3D12CommandList* const*>(m_list.data());
		}

	private:

		void Add(ICommandList* list)
		{
			m_list.push_back(list);
		}
		void Add(CmdList<ListType>& list)
		{
			m_list.push_back(list.get());
		}
		void Add(std::span<ICommandList* const> lists)
		{
			m_list.insert(m_list.end(), begin(lists), end(lists));
		}
	};

	// Flavours
	using GfxCmdListCollection = CmdListCollection<D3D12_COMMAND_LIST_TYPE_DIRECT>;
	using ComCmdListCollection = CmdListCollection<D3D12_COMMAND_LIST_TYPE_COMPUTE>;
}
