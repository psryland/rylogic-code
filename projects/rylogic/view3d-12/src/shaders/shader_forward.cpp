//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#include "pr/view3d-12/shaders/shader_forward.h"
#include "pr/view3d-12/scene/scene.h"
#include "pr/view3d-12/render/drawlist_element.h"
#include "pr/view3d-12/model/nugget.h"
#include "pr/view3d-12/instance/instance.h"
#include "view3d-12/src/shaders/common.h"

namespace pr::rdr12::shaders
{
	using namespace ::pr::compute;
	using namespace fwd;

	struct EReg
	{
		inline static constexpr auto CBufFrame = ECBufReg::b0;
		inline static constexpr auto ElementIndex = ECBufReg::b1;
		inline static constexpr auto CBufFade = ECBufReg::b2;
		inline static constexpr auto CBufScreenSpace = ECBufReg::b3;
		inline static constexpr auto CBufPbrSurface = ECBufReg::b4;
		inline static constexpr auto CBufDiag = ECBufReg::b5;
		inline static constexpr auto CBufProcedural = ECBufReg::b6;

		inline static constexpr auto DiffTexture = ESRVReg::t0;
		inline static constexpr auto EnvMap = ESRVReg::t1;
		inline static constexpr auto ShadowAtlas = ESRVReg::t2;
		inline static constexpr auto ProjTex = ESRVReg::t3;
		inline static constexpr auto PbrMetallicTexture = ESRVReg::t4;
		inline static constexpr auto PbrRoughnessTexture = ESRVReg::t5;
		inline static constexpr auto OpaqueDepth = ESRVReg::t6;
		inline static constexpr auto PbrEmissiveTexture = ESRVReg::t7;
		inline static constexpr auto Tex1Stream = ESRVReg::t8;
		inline static constexpr auto Tex2Stream = ESRVReg::t9;
		inline static constexpr auto Tex3Stream = ESRVReg:: t10;
		inline static constexpr auto Tex4Stream = ESRVReg:: t11;
		inline static constexpr auto PbrNormalTexture = ESRVReg::t12;
		inline static constexpr auto EnvMapPrev = ESRVReg::t13;
		inline static constexpr auto SkyTexture = ESRVReg::t18;
		inline static constexpr auto ProceduralBuffer = ESRVReg::t14;
		inline static constexpr auto Lights = ESRVReg::t15;
		inline static constexpr auto ShadowViews = ESRVReg::t16;
		inline static constexpr auto Elements = ESRVReg::t17;
		inline static constexpr auto AlphaColour = EUAVReg::u0;
		inline static constexpr auto AlphaDepth = EUAVReg::u1;
		inline static constexpr auto AlphaRtAttrs = EUAVReg::u2;
	};
	struct ESamp
	{
		inline static constexpr auto Diff = ESamReg::s0;
		inline static constexpr auto EnvMap = SamDescStatic(ESamReg::s1);
		inline static constexpr auto ShadowAtlas = SamDescStatic(ESamReg::s2).addr(D3D12_TEXTURE_ADDRESS_MODE_CLAMP).filter(D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT).compare(D3D12_COMPARISON_FUNC_LESS_EQUAL);
		inline static constexpr auto ProjTex = SamDescStatic(ESamReg::s3);
		inline static constexpr auto SkyNoise = SamDescStatic(ESamReg::s8).addr(D3D12_TEXTURE_ADDRESS_MODE_WRAP);
		inline static constexpr auto PbrMetallic = ESamReg::s4;
		inline static constexpr auto PbrRoughness = ESamReg::s5;
		inline static constexpr auto PbrEmissive = ESamReg::s6;
		inline static constexpr auto PbrNormal = ESamReg::s7;
	};

	Forward::Forward(Renderer& rdr)
		:Shader(rdr)
	{
		m_code = ShaderCode
		{
			.VS = shader_code::forward_vs,
			.PS = shader_code::forward_ps,
			.DS = shader_code::none,
			.HS = shader_code::none,
			.GS = shader_code::none,
			.CS = shader_code::none,
		};
		
		// Create the root signature. The element index is a root constant because it changes for every draw.
		m_signature = RootSig(ERootSigFlags::GraphicsOnly)
			.CBuf(EReg::CBufFrame)
			.U32(EReg::ElementIndex, sizeof(CBufElement) / sizeof(uint32_t))
			.CBuf(EReg::CBufFade)
			.CBuf(EReg::CBufScreenSpace)
			.CBuf(EReg::CBufPbrSurface)
			.CBuf(EReg::CBufDiag)
			.CBuf(EReg::CBufProcedural, D3D12_SHADER_VISIBILITY_VERTEX)
			.SRV(EReg::DiffTexture, 1)
			.SRV(EReg::EnvMap, 1)
			.SRV(EReg::ShadowAtlas, 1, D3D12_SHADER_VISIBILITY_PIXEL)
			.SRV(EReg::ProjTex, shaders::MaxProjectedTextures)
			.SRV(EReg::PbrMetallicTexture, 1)
			.SRV(EReg::PbrRoughnessTexture, 1)
			.SRV(EReg::OpaqueDepth, 1, D3D12_SHADER_VISIBILITY_PIXEL)
			.SRV(EReg::PbrEmissiveTexture, 1)
			.SRV(EReg::Tex1Stream, 1, D3D12_SHADER_VISIBILITY_VERTEX)
			.SRV(EReg::Tex2Stream, 1, D3D12_SHADER_VISIBILITY_VERTEX)
			.SRV(EReg::Tex3Stream, 1, D3D12_SHADER_VISIBILITY_VERTEX)
			.SRV(EReg::Tex4Stream, 1, D3D12_SHADER_VISIBILITY_VERTEX)
			.SRV(EReg::PbrNormalTexture, 1)
			.Samp(ESamp::Diff, shaders::MaxSamplers)
			.Samp(ESamp::PbrMetallic, 1)
			.Samp(ESamp::PbrRoughness, 1)
			.Samp(ESamp::PbrEmissive, 1)
			.Samp(ESamp::PbrNormal, 1)
			.Samp(ESamp::EnvMap)
			.Samp(ESamp::ShadowAtlas)
			.Samp(ESamp::ProjTex)
			.Samp(ESamp::SkyNoise)
			.UAV(EReg::AlphaColour, 1)
			.UAV(EReg::AlphaDepth, 1)
			.UAV(EReg::AlphaRtAttrs, 1)
			.SRV(EReg::SkyTexture, 3)
			.SRV(EReg::Lights, D3D12_SHADER_VISIBILITY_PIXEL)
			.SRV(EReg::ShadowViews, D3D12_SHADER_VISIBILITY_PIXEL)
			.SRV(EReg::ProceduralBuffer, D3D12_SHADER_VISIBILITY_VERTEX)
			.SRV(EReg::Elements)
			.SRV(EReg::EnvMapPrev, 1)
			.Create(rdr.d3d(), "ForwardSig");
	}

	// Config the shader
	void Forward::SetupFrame(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene)
	{
		// Set the frame constants
		CBufFrame cb0 = {};
		SetViewConstants(cb0.cam, scene.m_cam);
		SetLightingConstants(cb0, scene);
		SetEnvMapConstants(cb0.env_map, scene.m_global_envmap.get(), scene.m_global_envmap_prev.get(), scene.m_global_envmap_blend, scene.m_global_envmap_proxy_radius);
		cb0.output = v4(scene.wnd().m_dither_amount, 0, 0, 0);
		auto gpu_address = upload.Add(cb0, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
		cmd_list->SetGraphicsRootConstantBufferView((UINT)ERootParam::CBufFrame, gpu_address);

		// Bind the frame's lights as a root structured buffer
		auto lights_address = UploadLights(upload, scene);
		cmd_list->SetGraphicsRootShaderResourceView((UINT)ERootParam::Lights, lights_address);

		// Bind the frame's shadow views as a root structured buffer. The shadow atlas is bound by the render step.
		auto smap_step = scene.FindRStep<RenderSmap>();
		auto shadow_views_address = smap_step != nullptr
			? UploadShadowViews(upload, smap_step->Views().m_views, smap_step->Settings())
			: UploadShadowViews(upload, {}, ShadowSettings{});
		cmd_list->SetGraphicsRootShaderResourceView((UINT)ERootParam::ShadowViews, shadow_views_address);
	}
	// Upload the per-element constants table for a drawlist
	D3D12_GPU_VIRTUAL_ADDRESS Forward::UploadElements(GpuUploadBuffer& upload, Scene const& scene, CameraTransforms const& camera, std::span<DrawListElement const> drawlist)
	{
		// Allocate at least one entry so the table always has a valid address, even for an empty drawlist
		auto count = std::max<int>(isize(drawlist), 1);
		auto alex = upload.Alloc(count * sizeof(ElementConstants), alignof(ElementConstants));
		auto* elements = alex.ptr<ElementConstants>();
		auto const env_mapped = scene.m_global_envmap != nullptr;

		// The far clip fade range is the same for all elements. Only the source mode depends on the element's sort group.
		auto const fade = scene.FarClipFadeProperties();
		auto const fade_range = fade.m_enabled ? fade.DepthRange(scene.m_cam.ClipPlanes(false).y) : v2::Zero();

		// Fill one entry per element, using the same material selection as the material passes
		for (auto const& dle : drawlist)
		{
			auto& inst = *dle.m_instance;
			auto& nug = *dle.m_nugget;
			auto material_override = FindMaterial(inst);
			auto const& material = material_override != nullptr ? *material_override.get() : nug.mat();

			// Build the entry in cached memory, then copy it in one write. The upload heap is write-combined, so the setters'
			// read-modify-write accesses would be uncached reads if made directly on the table.
			ElementConstants cb = {};
			SetFlags(cb, inst, material, nug, env_mapped);
			SetTxfm(cb, inst, nug.m_model, camera);
			SetTint(cb, inst, material);
			SetTex2Surf(cb, inst, material);
			SetReflectivity(cb, inst, material);

			// Keep background and post-alpha overlays outside the scene's world-opacity policy.
			auto group = dle.m_sort_key.Group();
			if (fade.m_enabled && FarClipFadeApplies(group))
				cb.far_clip_fade = v3{ fade_range.x, fade_range.y, group < ESortGroup::AlphaBack ? 1.0f : 2.0f };

			// Write the complete entry to the table
			*elements++ = cb;
		}

		// Return the GPU address of the table
		return alex.m_res->GetGPUVirtualAddress() + alex.m_ofs;
	}

	// Bind a table of element constants
	void Forward::SetupElements(ID3D12GraphicsCommandList* cmd_list, D3D12_GPU_VIRTUAL_ADDRESS elements)
	{
		cmd_list->SetGraphicsRootShaderResourceView((UINT)ERootParam::Elements, elements);
	}

	// Select the element table entry for the next draw
	void Forward::SetupElement(ID3D12GraphicsCommandList* cmd_list, int index)
	{
		cmd_list->SetGraphicsRoot32BitConstant((UINT)ERootParam::ElementIndex, s_cast<UINT>(index), 0);
	}
}
