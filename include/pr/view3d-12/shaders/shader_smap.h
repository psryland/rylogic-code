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
			CBufFrame = 0,
			CBufNugget,
			DiffTexture,
			DiffTextureSampler,
		};

		enum class ESampParam
		{
		};
	}

	struct ShadowMap :Shader
	{
		explicit ShadowMap(Renderer& rdr);

		// Config the shader.
		void SetupFrame(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, ShadowCaster const& caster);
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, DrawListElement const* dle, SceneCamera const& cam);
	};
}
