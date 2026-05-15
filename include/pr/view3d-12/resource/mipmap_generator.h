//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"

namespace pr::rdr12
{
	struct MipMapGenerator
	{
	private:

		using SignaturePtr = D3DPtr<ID3D12RootSignature>;
		using PipelineStatePtr = D3DPtr<ID3D12PipelineState>;
		using GpuViewHeap = ::pr::compute::GpuDescriptorHeap<D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV>;
		using GpuSync = ::pr::compute::GpuSync;
		using GfxCmdList = ::pr::compute::GfxCmdList;
		using KeepAlive = ::pr::compute::KeepAlive;

		Renderer&        m_rdr;            // The owning renderer instance
		GpuSync&         m_gsync;          // The GPU fence
		GfxCmdList&      m_cmd_list;       // Command list for mip generator operations.
		KeepAlive        m_keep_alive;     // Keeps D3D objects alive until the GPU has finished with them
		GpuViewHeap      m_view_heap;      // GPU visible descriptor heap for CBV/SRV/UAV
		SignaturePtr     m_mipmap_sig;     // Root signature for the mip map generator
		PipelineStatePtr m_mipmap_pso;     // Pipeline state for the mip map generator
		bool             m_flush_required; // True when there is mip-map generation work pending

	public:

		MipMapGenerator(Renderer& rdr, GpuSync& gsync, GfxCmdList& cmd_list);
		MipMapGenerator(MipMapGenerator&&) = delete;
		MipMapGenerator(MipMapGenerator const&) = delete;
		MipMapGenerator& operator = (MipMapGenerator&&) = delete;
		MipMapGenerator& operator = (MipMapGenerator const&) = delete;
		~MipMapGenerator() = default;

		// Generate mip maps for a texture
		void Generate(ID3D12Resource* texture, int mip_first = 1, int mip_count = limits<int>::max());

	private:

		// Generate mip maps for a resource that supports UAV
		void GenerateCore(ID3D12Resource* uav_resource, int mip_first, int mip_count);
	};
}
