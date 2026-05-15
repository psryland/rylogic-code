//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/shaders/shader_point_sprites.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/render/drawlist_element.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12::shaders
{
	using namespace ::pr::compute;

	PointSpriteGS::PointSpriteGS(Renderer& rdr, v2 size, bool depth)
		: Shader(rdr)
		, m_size(size)
		, m_depth(depth)
	{
		m_code = ShaderCode
		{
			.VS = shader_code::none,
			.PS = shader_code::none,
			.DS = shader_code::none,
			.HS = shader_code::none,
			.GS = shader_code::point_sprites_gs,
			.CS = shader_code::none,
		};
	}
	void PointSpriteGS::SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, DrawListElement const* dle)
	{
		if (dle != nullptr)
		{
			fwd::CBufScreenSpace cb = {};
			SetScreenSpace(cb, *dle->m_instance, scene, m_size, m_depth);
			auto gpu_address = upload.Add(cb, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
			cmd_list->SetGraphicsRootConstantBufferView((UINT)fwd::ERootParam::CBufScreenSpace, gpu_address);
		}
	}
}
