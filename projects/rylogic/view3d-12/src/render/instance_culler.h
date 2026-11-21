//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2026
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	// Per-view culling of the instances of nuggets drawn with GPU-written draw arguments (see NuggetDrawArgs).
	// For each (instance, nugget, view), the culler keeps a view buffer that holds the draw arguments of the instances that the view may see,
	// followed by the list of their original instance indices. See 'instance_cull.hlsl' for the layout.
	struct InstanceCuller
	{
		using GfxCmdList = ::pr::compute::GfxCmdList;
		using GpuUploadBuffer = ::pr::compute::GpuUploadBuffer;

		// The byte offset of the visible instance list in a view buffer.
		inline static constexpr int64_t ListOffset = 32;

		// The state that the draws read a view buffer in. The forward root view of the list is visible to every shader stage, so it needs both shader resource states.
		inline static constexpr D3D12_RESOURCE_STATES DrawState = D3D12_RESOURCE_STATE_INDIRECT_ARGUMENT | D3D12_RESOURCE_STATE_ALL_SHADER_RESOURCE;

	private:

		// The view buffer of one (instance, nugget, view), and the last frame that used it.
		struct ViewBuffer
		{
			D3DPtr<ID3D12Resource> m_res;
			int64_t m_size = 0;
			uint64_t m_frame = 0;
		};
		using Key = std::tuple<BaseInstance const*, Nugget const*, int>;
		struct KeyHash
		{
			size_t operator()(Key const& key) const;
		};

		Renderer& m_rdr;
		D3DPtr<ID3D12RootSignature> m_signature;
		D3DPtr<ID3D12PipelineState> m_pso_reset;
		D3DPtr<ID3D12PipelineState> m_pso_cull;
		std::unordered_map<Key, ViewBuffer, KeyHash> m_buffers;
		uint64_t m_frame;

	public:

		explicit InstanceCuller(Renderer& rdr);
		InstanceCuller(InstanceCuller const&) = delete;
		InstanceCuller& operator=(InstanceCuller const&) = delete;
		~InstanceCuller();

		// Start a frame. Releases the view buffers that the previous frame did not use.
		void BeginFrame();

		// Cull the instances of 'nugget' of 'inst' for the view identified by 'view', whose world to clip transform is 'w2s'. Records compute work
		// into 'cmd_list', and leaves the view buffer in 'DrawState' on that list. Does nothing if this frame has already culled the same key.
		// 'nugget' must have draw arguments. Leaves the compute root signature and PSO changed.
		void Cull(GfxCmdList& cmd_list, GpuUploadBuffer& upload, BaseInstance const& inst, Nugget const& nugget, int view, m4x4 const& w2s);

		// The view buffer that this frame's 'Cull' wrote for (inst, nugget, view), or null if there is none.
		ID3D12Resource* Find(BaseInstance const& inst, Nugget const& nugget, int view);
	};
}
