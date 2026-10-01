//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/shaders/shader_smap.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/render/drawlist_element.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/instance/instance.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12::shaders
{
	using namespace ::pr::compute;
	using namespace smap;

	struct EReg
	{
		inline static constexpr auto DrawViews = ECBufReg::b0;
		inline static constexpr auto ElementIndex = ECBufReg::b1;
		inline static constexpr auto CBufProcedural = ECBufReg::b2;
		inline static constexpr auto DiffTexture = ESRVReg::t0;
		inline static constexpr auto DiffTextureSampler = ESamReg::s0;
		inline static constexpr auto ShadowViews = ESRVReg::t1;
		inline static constexpr auto Elements = ESRVReg::t2;
		inline static constexpr auto ProceduralBuffer = ESRVReg::t14;
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

		// Create the root signature. The draw views and element index are root constants because they change for every draw.
		m_signature = RootSig(ERootSigFlags::VertGeomPixelOnly)
			.U32(EReg::DrawViews, sizeof(CBufDrawViews) / sizeof(uint32_t), D3D12_SHADER_VISIBILITY_VERTEX)
			.U32(EReg::ElementIndex, sizeof(CBufElement) / sizeof(uint32_t))
			.CBuf(EReg::CBufProcedural, D3D12_SHADER_VISIBILITY_VERTEX)
			.SRV(EReg::DiffTexture, 1)
			.Samp(EReg::DiffTextureSampler, 1)
			.SRV(EReg::ShadowViews, D3D12_SHADER_VISIBILITY_VERTEX)
			.SRV(EReg::ProceduralBuffer, D3D12_SHADER_VISIBILITY_VERTEX)
			.SRV(EReg::Elements)
			.Create(rdr.d3d(), "ShadowMapSig");
	}

	// Bind the frame's shadow view array and the step's element constants table
	void ShadowMap::SetupFrame(ID3D12GraphicsCommandList* cmd_list, D3D12_GPU_VIRTUAL_ADDRESS shadow_views, D3D12_GPU_VIRTUAL_ADDRESS elements)
	{
		cmd_list->SetGraphicsRootShaderResourceView((UINT)ERootParam::ShadowViews, shadow_views);
		cmd_list->SetGraphicsRootShaderResourceView((UINT)ERootParam::Elements, elements);
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

	// Upload the per-element constants table for a drawlist
	D3D12_GPU_VIRTUAL_ADDRESS ShadowMap::UploadElements(GpuUploadBuffer& upload, std::span<DrawListElement const> drawlist, std::span<uint32_t const> draw_mask)
	{
		// Allocate at least one entry so the table always has a valid address, even for an empty drawlist
		pr_assert(draw_mask.size() == drawlist.size() && "One draw mask per drawlist element expected");
		auto count = std::max<int>(isize(drawlist), 1);
		auto alex = upload.Alloc(count * sizeof(ElementConstants), alignof(ElementConstants));
		auto* elements = alex.ptr<ElementConstants>();

		// Fill one entry per drawn element, using the same material selection as the material passes
		for (int e = 0, eend = isize(drawlist); e != eend; ++e)
		{
			if (draw_mask[e] == 0)
				continue;

			auto& inst = *drawlist[e].m_instance;
			auto& nug = *drawlist[e].m_nugget;
			auto material_override = FindMaterial(inst);
			auto const& material = material_override != nullptr ? *material_override.get() : nug.mat();

			// Build the entry in cached memory, then copy it in one write. The upload heap is write-combined (see Forward::UploadElements).
			ElementConstants cb = {};
			SetFlags(cb, inst, material, nug, false);
			SetPlacement(cb, inst, nug.m_model);
			SetTint(cb, inst, material);
			SetTex2Surf(cb, inst, material);
			elements[e] = cb;
		}

		// Return the GPU address of the table
		return alex.m_res->GetGPUVirtualAddress() + alex.m_ofs;
	}

	// Select the element table entry for the next draw
	void ShadowMap::SetupElement(ID3D12GraphicsCommandList* cmd_list, int index)
	{
		cmd_list->SetGraphicsRoot32BitConstant((UINT)ERootParam::ElementIndex, s_cast<UINT>(index), 0);
	}
}
