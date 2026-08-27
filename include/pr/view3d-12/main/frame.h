//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/render/back_buffer.h"
#include "pr/view3d-12/utility/pipe_state.h"
#include "pr/view3d-12/utility/diagnostics.h"

namespace pr::rdr12
{
	struct Frame
	{
		using GfxCmdLists = pr::vector<ID3D12GraphicsCommandList*, 4, false>;
		using GpuSync = ::pr::compute::GpuSync;
		using GpuUploadBuffer = ::pr::compute::GpuUploadBuffer;
		using GfxCmdAllocPool = ::pr::compute::GfxCmdAllocPool;
		using GfxCmdList = ::pr::compute::GfxCmdList;

		GpuSync& m_gsync;         // The GPU sync object used to track GPU progress and manage resources
		GpuUploadBuffer m_upload; // A GPU buffer for the global light data
		GfxCmdAllocPool& m_cmd_alloc_pool; // The command allocator pool to create allocators from

		GfxCmdList m_prepare;       // Commands before the first scene is rendered
		GfxCmdList m_world_depth;   // Scene-adjacent commands rendered into the multi-sampled scene target with the scene depth buffer bound
		GfxCmdList m_resolve;       // Commands that resolve/copy the MSAA buffer and seed the K-buffer opaque colour
		GfxCmdList m_composite;     // Commands that composite transparent and diagnostic scene output
		GfxCmdList m_depth_resolve; // Commands that produce the single-sample read-only copy of the scene depth buffer
		GfxCmdList m_world_overlay; // Commands for world-anchored overlays, after alpha resolution and before screen-space overlays
		GfxCmdList m_final_overlay; // Commands for optional final overlays after all scene output
		GfxCmdList m_present;       // The authoritative final transition to the present state

		GfxCmdLists m_main; // Command lists to execute before the MSAA buffer is resolved
		GfxCmdLists m_post; // Command lists to execute after MSAA resolve and before alpha resolve/final present

		BackBuffer const* m_bb_main; // The back buffer to render the scene to that will be anti-aliased.
		BackBuffer const* m_bb_post; // The back buffer for post-processing effects (assume main has been rendered into post).

		Frame(ID3D12Device4* device, GpuSync& gsync, GfxCmdAllocPool& cmd_alloc_pool, BackBuffer const& bb_main, BackBuffer const& bb_post)
			: m_gsync(gsync)
			, m_upload(m_gsync, 1ULL * 1024 * 1024)
			, m_cmd_alloc_pool(cmd_alloc_pool)
			, m_prepare(device, cmd_alloc_pool.Get(), nullptr, "Prepare", EColours::Orange)
			, m_world_depth(device, cmd_alloc_pool.Get(), nullptr, "WorldDepth", EColours::Orange)
			, m_resolve(device, cmd_alloc_pool.Get(), nullptr, "Resolve", EColours::Orange)
			, m_composite(device, cmd_alloc_pool.Get(), nullptr, "Composite", EColours::Orange)
			, m_depth_resolve(device, cmd_alloc_pool.Get(), nullptr, "DepthResolve", EColours::Orange)
			, m_world_overlay(device, cmd_alloc_pool.Get(), nullptr, "WorldOverlay", EColours::Orange)
			, m_final_overlay(device, cmd_alloc_pool.Get(), nullptr, "FinalOverlay", EColours::Orange)
			, m_present(device, cmd_alloc_pool.Get(), nullptr, "Present", EColours::Orange)
			, m_main()
			, m_post()
			, m_bb_main(&bb_main)
			, m_bb_post(&bb_post)
		{
			m_prepare.Close();
			m_world_depth.Close();
			m_resolve.Close();
			m_composite.Close();
			m_depth_resolve.Close();
			m_world_overlay.Close();
			m_final_overlay.Close();
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
			m_world_depth.Reset(m_cmd_alloc_pool.Get());
			m_resolve.Reset(m_cmd_alloc_pool.Get());
			m_composite.Reset(m_cmd_alloc_pool.Get());
			m_depth_resolve.Reset(m_cmd_alloc_pool.Get());
			m_world_overlay.Reset(m_cmd_alloc_pool.Get());
			m_final_overlay.Reset(m_cmd_alloc_pool.Get());
			m_present.Reset(m_cmd_alloc_pool.Get());

			m_main.resize(0);
			m_post.resize(0);
		}
	};
}
