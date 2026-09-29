//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/shaders/shader_smap.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/render/drawlist_element.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12::shaders
{
	using namespace ::pr::compute;
	using namespace smap;

	struct EReg
	{
		inline static constexpr auto DrawViews = ECBufReg::b0;
		inline static constexpr auto CBufNugget = ECBufReg::b1;
		inline static constexpr auto CBufProcedural = ECBufReg::b2;
		inline static constexpr auto DiffTexture = ESRVReg::t0;
		inline static constexpr auto DiffTextureSampler = ESamReg::s0;
		inline static constexpr auto ShadowViews = ESRVReg::t1;
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

		// Create the root signature. The draw views are root constants because they change for every draw.
		m_signature = RootSig(ERootSigFlags::VertGeomPixelOnly)
			.U32(EReg::DrawViews, sizeof(CBufDrawViews) / sizeof(uint32_t), D3D12_SHADER_VISIBILITY_VERTEX)
			.CBuf(EReg::CBufNugget)
			.CBuf(EReg::CBufProcedural, D3D12_SHADER_VISIBILITY_VERTEX)
			.SRV(EReg::DiffTexture, 1)
			.Samp(EReg::DiffTextureSampler, 1)
			.SRV(EReg::ShadowViews, D3D12_SHADER_VISIBILITY_VERTEX)
			.Create(rdr.d3d(), "ShadowMapSig");
	}

	// Bind the frame's shadow view array
	void ShadowMap::SetupFrame(ID3D12GraphicsCommandList* cmd_list, D3D12_GPU_VIRTUAL_ADDRESS shadow_views)
	{
		cmd_list->SetGraphicsRootShaderResourceView((UINT)ERootParam::ShadowViews, shadow_views);
	}

	// Set the shadow views for the next draw
	void ShadowMap::SetupDrawViews(ID3D12GraphicsCommandList* cmd_list, std::span<uint32_t const> views, int first_viewport_view)
	{
		// Pack the view indices, one per instance, into the root constants
		pr_assert(views.size() <= ShadowViewBatchSize && "Too many views for one draw");
		CBufDrawViews cb = {};
		for (int i = 0; i != isize(views); ++i)
			cb.views[i / 4][i % 4] = views[i];

		cb.info.x = static_cast<uint32_t>(first_viewport_view);
		cmd_list->SetGraphicsRoot32BitConstants((UINT)ERootParam::DrawViews, sizeof(cb) / sizeof(uint32_t), &cb, 0);
	}

	// Set the per-element constants
	void ShadowMap::SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, DrawListElement const* dle, CameraTransforms const& camera)
	{
		SetupElement(cmd_list, upload, dle, camera, dle->m_nugget->mat());
	}
	void ShadowMap::SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, DrawListElement const* dle, CameraTransforms const& camera, Material const& material)
	{
		// Set the per-element constants
		auto& inst = *dle->m_instance;
		auto& nug = *dle->m_nugget;

		CBufNugget cb1 = {};
		SetFlags(cb1, inst, material, nug, false);
		SetTxfm(cb1, inst, nug.m_model, camera);
		SetTint(cb1, inst, material);
		SetTex2Surf(cb1, inst, material);
		auto gpu_address = upload.Add(cb1, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false);
		cmd_list->SetGraphicsRootConstantBufferView((UINT)ERootParam::CBufNugget, gpu_address);
	}
}
