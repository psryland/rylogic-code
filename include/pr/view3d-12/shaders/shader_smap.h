//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/shader.h"

namespace pr::rdr12::shaders
{
	namespace smap
	{
		enum class ERootParam
		{
			DrawViews = 0,
			ElementIndex,
			CBufProcedural,
			DiffTexture,
			DiffTextureSampler,
			ShadowViews,
			ProceduralBuffer,
			Elements,
		};

		enum class ESampParam
		{
		};
	}

	// Renders scene depth into the shadow atlas. Each instance of a draw renders into one shadow view.
	struct ShadowMap :Shader
	{
		explicit ShadowMap(Renderer& rdr);

		// Bind the frame's shadow view array (see 'UploadShadowViews') and the step's element constants table (see 'UploadElements')
		void SetupFrame(ID3D12GraphicsCommandList* cmd_list, D3D12_GPU_VIRTUAL_ADDRESS shadow_views, D3D12_GPU_VIRTUAL_ADDRESS elements);

		// Set the shadow views for the next draw. Instance 'i' of the draw renders into 'views[i]', using viewport 'views[i] - first_viewport_view'.
		void SetupDrawViews(ID3D12GraphicsCommandList* cmd_list, std::span<uint32_t const> views, int first_viewport_view);

		// Upload one set of element constants per entry in 'drawlist', in drawlist order. Shadow views supply the projection, so no camera is needed.
		// 'draw_mask' has one entry per drawlist element. Entries whose mask is zero are not drawn, so their constants are left unwritten.
		static D3D12_GPU_VIRTUAL_ADDRESS UploadElements(GpuUploadBuffer& upload, std::span<DrawListElement const> drawlist, std::span<uint32_t const> draw_mask);

		// Select the entry in the bound element table used by the next draw
		static void SetupElement(ID3D12GraphicsCommandList* cmd_list, int index);
	};
}
