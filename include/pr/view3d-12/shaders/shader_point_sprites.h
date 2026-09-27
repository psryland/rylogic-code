//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/shader.h"

namespace pr::rdr12::shaders
{
	struct PointSpriteGS :Shader
	{
		v2 m_size; // Default point size. Per-point size can be set using tex0.xy
		bool m_depth;
		PointSpriteGS(Renderer& rdr, v2 size, bool depth);
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, CameraTransforms const& camera, DrawListElement const* dle) override;
	};
}