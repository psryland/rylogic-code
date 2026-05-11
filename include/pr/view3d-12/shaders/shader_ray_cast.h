//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/shader.h"
#include "pr/view3d-12/utility/ray_cast.h"

namespace pr::rdr12::shaders
{
	namespace ray_cast
	{
		enum class ERootParam
		{
			CBufFrame = 0,
			CBufNugget,
		};

		enum class ESampParam
		{
		};
	}

	struct RayCast :Shader
	{
		explicit RayCast(Renderer& rdr);
		void SetupFrame(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, std::span<HitTestRay const> rays);
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, DrawListElement const* dle);
	};
}
