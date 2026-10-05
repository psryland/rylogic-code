//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/utility/pipe_state.h"
#include "pr/view3d-12/main/renderer.h"
#include "pr/view3d-12/main/window.h"

namespace pr::rdr12
{
	PipeStatePool::PipeStatePool(Window& wnd)
		:m_wnd(&wnd)
		,m_pool()
	{}

	// Return a pipeline state instance for the given description
	ID3D12PipelineState* PipeStatePool::Get(PipeStateDesc const& desc)
	{
		auto frame_number = m_wnd->FrameNumber();

		// See if a pipeline state object already exists
		auto iter = pr::find_if(m_pool, [=](auto& pso) { return pso.m_hash == desc.hash(); });
		if (iter == m_pool.end())
		{
			// If it doesn't exists, create it now
			auto device = m_wnd->rdr().D3DDevice();

			// Delete any states that haven't been used for a while
			constexpr size_t MaxFrameAge = 30;
			constexpr size_t DrainPoolCount = 100;
			auto pool_size = m_pool.size();
			if (pool_size > DrainPoolCount)
			{
				erase_if_unstable(m_pool, [=](auto& item) { return frame_number - item.m_frame_number > MaxFrameAge; });

				#if PR_DBG_RDR
				if (m_pool.size() == pool_size)
					throw std::runtime_error("Too many unique pipeline states");
				#endif
			}

			// Create the pipeline state instance. Append it, because draining the pool invalidates 'iter'.
			D3DPtr<ID3D12PipelineState> pso;
			Check(device->CreateGraphicsPipelineState(desc, __uuidof(ID3D12PipelineState), (void**)pso.address_of()));
			m_pool.emplace_back(pso, frame_number, desc.hash());
			iter = m_pool.end() - 1;
		}

		auto& pso = *iter;
		pso.m_frame_number = frame_number;
		return pso.m_pso.get();
	}
}