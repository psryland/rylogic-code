//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/back_buffer.h"
#include "pr/view3d-12/resource/gpu_descriptor_heap.h"
#include "pr/view3d-12/utility/wrappers.h"
#include "pr/view3d-12/utility/cmd_alloc.h"
#include "pr/view3d-12/utility/cmd_list.h"
#include "pr/view3d-12/utility/pipe_state.h"
#include "pr/view3d-12/utility/diagnostics.h"

namespace pr::rdr12
{
	struct Frame
	{
		using GfxCmdLists = pr::vector<ID3D12GraphicsCommandList*, 4, false>;

		GpuSync& m_gsync;         // The GPU sync object used to track GPU progress and manage resources
		GpuUploadBuffer m_upload; // A GPU buffer for the global light data
		GfxCmdAllocPool& m_cmd_alloc_pool; // The command allocator pool to create allocators from

		GfxCmdList m_prepare; // Commands before the first scene is rendered
		GfxCmdList m_resolve; // Commands that resolve/copy the MSAA buffer and seed the K-buffer opaque colour
		GfxCmdList m_present; // Commands after post-resolve work, including alpha resolve and final present-state transitions

		GfxCmdLists m_main; // Command lists to execute before the MSAA buffer is resolved
		GfxCmdLists m_post; // Command lists to execute after MSAA resolve and before alpha resolve/final present

		BackBuffer const* m_bb_main; // The back buffer to render the scene to that will be anti-aliased.
		BackBuffer const* m_bb_post; // The back buffer for post-processing effects (assume main has been rendered into post).

		Frame(ID3D12Device4* device, GpuSync& gsync, GfxCmdAllocPool& cmd_alloc_pool, BackBuffer const& bb_main, BackBuffer const& bb_post)
			: m_gsync(gsync)
			, m_upload(m_gsync, 1ULL * 1024 * 1024)
			, m_cmd_alloc_pool(cmd_alloc_pool)
			, m_prepare(device, cmd_alloc_pool.Get(), nullptr, "Prepare", EColours::Orange)
			, m_resolve(device, cmd_alloc_pool.Get(), nullptr, "Resolve", EColours::Orange)
			, m_present(device, cmd_alloc_pool.Get(), nullptr, "Present", EColours::Orange)
			, m_main()
			, m_post()
			, m_bb_main(&bb_main)
			, m_bb_post(&bb_post)
		{
			m_prepare.Close();
			m_resolve.Close();
			m_present.Close();
		}
		Frame(Frame&&) = default;
		Frame(Frame const&) = delete;
		Frame& operator =(Frame&&) = default;
		Frame& operator =(Frame const&) = delete;

		BackBuffer const& bb_main() const { return *m_bb_main; };
		BackBuffer const& bb_post() const { return *m_bb_post; };

		void Reset(BackBuffer const& bb_main, BackBuffer const& bb_post)
		{
			m_bb_main = &bb_main;
			m_bb_post = &bb_post;

			m_prepare.Reset(m_cmd_alloc_pool.Get());
			m_resolve.Reset(m_cmd_alloc_pool.Get());
			m_present.Reset(m_cmd_alloc_pool.Get());

			m_main.resize(0);
			m_post.resize(0);
		}
	};
}
