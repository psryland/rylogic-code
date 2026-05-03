//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/resource/gpu_descriptor_heap.h"
#include "pr/view3d-12/texture/texture_2D.h"
#include "pr/view3d-12/utility/cmd_list.h"
#include "pr/view3d-12/utility/wrappers.h"

namespace pr::rdr12
{
	struct AlphaKBuffer
	{
		using GpuViewHeap = GpuDescriptorHeap<D3D12_DESCRIPTOR_HEAP_TYPE_CBV_SRV_UAV>;

		Texture2DPtr m_opaque_colour_1x; // 1x resolved opaque colour used as the base for alpha resolve
		Texture2DPtr m_alpha_colour;     // Per-pixel nearest four alpha colours, RGBA8 packed in uint lanes
		Texture2DPtr m_alpha_depth;      // Per-pixel nearest four D24 depths, with packed OIA bytes in high bits
		D3DPtr<ID3D12RootSignature> m_signature_alpha_resolve;
		D3DPtr<ID3D12PipelineState> m_pso_alpha_resolve;

		AlphaKBuffer();

		// Release all owned resources
		void Release();

		// Resize the buffers to 'size'
		void Resize(Renderer& rdr, iv2 size, ClearValue rt_clear);

		// Create the buffers for the next frame
		void Clear(GfxCmdList& cmd_list, GpuViewHeap& heap_view);

		// Copy the opaque buffer 'source' into the 1x resolved buffer using resolve or direct copy
		void CopyOpaqueBuffer(GfxCmdList& cmd_list, ID3D12Resource* source, DXGI_FORMAT format, bool resolve);

		// Composite the collected alpha buffer over the resolved opaque colour
		void ResolveAlpha(GfxCmdList& cmd_list, GpuViewHeap& heap_view, BackBuffer const& bb_post, Viewport const& viewport, D3D12_RECT const& scissor);

		// True if this AlphaKBuffer is valid and ready for use
		explicit operator bool() const;
	};
}
