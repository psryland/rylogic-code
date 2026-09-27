//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/shader.h"

namespace pr::rdr12::shaders
{
	struct ThickLineStripGS :Shader
	{
		float m_width;
		ThickLineStripGS(Renderer& rdr, float width);
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, CameraTransforms const& camera, DrawListElement const* dle) override;
	};

	struct ThickLineListGS :Shader
	{
		float m_width;
		ThickLineListGS(Renderer& rdr, float width);
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, CameraTransforms const& camera, DrawListElement const* dle) override;
	};
}