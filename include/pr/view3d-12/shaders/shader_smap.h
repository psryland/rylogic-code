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
			CBufNugget,
			CBufProcedural,
			DiffTexture,
			DiffTextureSampler,
			ShadowViews,
		};

		enum class ESampParam
		{
		};
	}

	// Renders scene depth into the shadow atlas. Each instance of a draw renders into one shadow view.
	struct ShadowMap :Shader
	{
		explicit ShadowMap(Renderer& rdr);

		// Bind the frame's shadow view array (see 'UploadShadowViews')
		void SetupFrame(ID3D12GraphicsCommandList* cmd_list, D3D12_GPU_VIRTUAL_ADDRESS shadow_views);

		// Set the shadow views for the next draw. Instance 'i' of the draw renders into 'views[i]', using viewport 'views[i] - first_viewport_view'.
		void SetupDrawViews(ID3D12GraphicsCommandList* cmd_list, std::span<uint32_t const> views, int first_viewport_view);

		// Set the per-element constants
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, DrawListElement const* dle, CameraTransforms const& camera);
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, DrawListElement const* dle, CameraTransforms const& camera, Material const& material);
	};
}
