//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/shaders/shader_smap.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/render/drawlist_element.h"
#include "pr/view3d-12/utility/shadow_caster.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12::shaders
{
	using namespace ::pr::compute;
	using namespace smap;

	struct EReg
	{
		inline static constexpr auto CBufFrame = ECBufReg::b0;
		inline static constexpr auto CBufNugget = ECBufReg::b1;
		inline static constexpr auto DiffTexture = ESRVReg::t0;
		inline static constexpr auto DiffTextureSampler = ESamReg::s0;
	};

	ShadowMap::ShadowMap(Renderer& rdr)
		:Shader(rdr)
	{
		m_code = ShaderCode
		{
			.VS = shader_code::shadow_map_vs,
			.PS = shader_code::shadow_map_ps,
			.DS = shader_code::none,
			.HS = shader_code::none,
			.GS = shader_code::none,
			.CS = shader_code::none,
		};
		
		// Create the root signature
		m_signature = RootSig(ERootSigFlags::VertGeomPixelOnly)
			.CBuf(EReg::CBufFrame)
			.CBuf(EReg::CBufNugget)
			.SRV(EReg::DiffTexture, 1)
			.Samp(EReg::DiffTextureSampler, 1)
			.Create(rdr.d3d(), "ShadowMapSig");
	}

	// Config the shader
	void ShadowMap::SetupFrame(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, ShadowCaster const& caster)
	{
		// Set the frame constants
		CBufFrame cb0 = {};
		cb0.w2l = caster.m_params.m_w2ls;
		cb0.l2s = caster.m_params.m_ls2s;
		auto gpu_address = upload.Add(cb0, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false);
		cmd_list->SetGraphicsRootConstantBufferView((UINT)ERootParam::CBufFrame, gpu_address);
	}
	void ShadowMap::SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, DrawListElement const* dle, SceneCamera const& cam)
	{
		SetupElement(cmd_list, upload, dle, cam, dle->m_nugget->mat());
	}
	void ShadowMap::SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, DrawListElement const* dle, SceneCamera const& cam, Material const& material)
	{
		// Set the per-element constants
		auto& inst = *dle->m_instance;
		auto& nug = *dle->m_nugget;

		CBufNugget cb1 = {};
		SetFlags(cb1, inst, material, nug, false);
		SetTxfm(cb1, inst, nug.m_model, cam); // todo frame constant?
		SetTint(cb1, inst, material);
		SetTex2Surf(cb1, inst, material);
		auto gpu_address = upload.Add(cb1, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false);
		cmd_list->SetGraphicsRootConstantBufferView((UINT)ERootParam::CBufNugget, gpu_address);
	}
}
