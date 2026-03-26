//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/shaders/shader_arrow_head.h"
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/scene/scene.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12::shaders
{
	ArrowHeadGS::ArrowHeadGS(Renderer& rdr, v2 size, bool depth)
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
			.GS = shader_code::arrow_head_gs,
			.CS = shader_code::none,
		};
	}
	void ArrowHeadGS::SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, DrawListElement const*)
	{
		fwd::CBufScreenSpace cb = {
			.screen_dim = To<v2>(scene.wnd().BackBufferSize()),
			.size = m_size, // Default if 'size' is not read from 'tex0' in the vertex
			.depth = m_depth,
		};
		auto gpu_address = upload.Add(cb, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
		cmd_list->SetGraphicsRootConstantBufferView((UINT)fwd::ERootParam::CBufScreenSpace, gpu_address);
	}
}
