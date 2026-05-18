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
		inline static constexpr auto CBufNugget = ECBufReg::b1;
		inline static constexpr auto CBufFade = ECBufReg::b2;
		inline static constexpr auto CBufScreenSpace = ECBufReg::b3;
		inline static constexpr auto CBufPbrSurface = ECBufReg::b4;
		inline static constexpr auto CBufDiag = ECBufReg::b5;

		inline static constexpr auto DiffTexture = ESRVReg::t0;
		inline static constexpr auto EnvMap = ESRVReg::t1;
		inline static constexpr auto SMap = ESRVReg::t2;
		inline static constexpr auto ProjTex = ESRVReg::t3;
		inline static constexpr auto PbrMetallicTexture = ESRVReg::t4;
		inline static constexpr auto PbrRoughnessTexture = ESRVReg::t5;
		inline static constexpr auto OpaqueDepth = ESRVReg::t6;
		inline static constexpr auto PbrEmissiveTexture = ESRVReg::t7;
		inline static constexpr auto AlphaColour = EUAVReg::u0;
		inline static constexpr auto AlphaDepth = EUAVReg::u1;
		inline static constexpr auto AlphaRtAttrs = EUAVReg::u2;
	};
	struct ESamp
	{
		inline static constexpr auto Diff = ESamReg::s0;
		inline static constexpr auto EnvMap = SamDescStatic(ESamReg::s1);
		inline static constexpr auto SMap = SamDescStatic(ESamReg::s2).addr(D3D12_TEXTURE_ADDRESS_MODE_CLAMP).filter(D3D12_FILTER_COMPARISON_MIN_MAG_LINEAR_MIP_POINT).compare(D3D12_COMPARISON_FUNC_GREATER_EQUAL);
		inline static constexpr auto ProjTex = SamDescStatic(ESamReg::s3);
		inline static constexpr auto PbrMetallic = ESamReg::s4;
		inline static constexpr auto PbrRoughness = ESamReg::s5;
		inline static constexpr auto PbrEmissive = ESamReg::s6;
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
		
		// Create the root signature
		m_signature = RootSig(ERootSigFlags::GraphicsOnly)
			.CBuf(EReg::CBufFrame)
			.CBuf(EReg::CBufNugget)
			.CBuf(EReg::CBufFade)
			.CBuf(EReg::CBufScreenSpace)
			.CBuf(EReg::CBufPbrSurface)
			.CBuf(EReg::CBufDiag)
			.SRV(EReg::DiffTexture, 1)
			.SRV(EReg::PbrMetallicTexture, 1)
			.SRV(EReg::PbrRoughnessTexture, 1)
			.SRV(EReg::PbrEmissiveTexture, 1)
			.SRV(EReg::EnvMap, 1)
			.SRV(EReg::SMap, shaders::MaxShadowMaps)
			.SRV(EReg::ProjTex, shaders::MaxProjectedTextures)
			.SRV(EReg::OpaqueDepth, 1, D3D12_SHADER_VISIBILITY_PIXEL)
			.Samp(ESamp::Diff, shaders::MaxSamplers)
			.Samp(ESamp::PbrMetallic, 1)
			.Samp(ESamp::PbrRoughness, 1)
			.Samp(ESamp::PbrEmissive, 1)
			.Samp(ESamp::EnvMap)
			.Samp(ESamp::SMap)
			.Samp(ESamp::ProjTex)
			.UAV(EReg::AlphaColour, 1)
			.UAV(EReg::AlphaDepth, 1)
			.UAV(EReg::AlphaRtAttrs, 1)
			.Create(rdr.d3d(), "ForwardSig");
	}

	// Config the shader
	void Forward::SetupFrame(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene)
	{
		// Set the frame constants
		CBufFrame cb0 = {};
		SetViewConstants(cb0.cam, scene.m_cam);
		SetLightingConstants(cb0.global_light, scene.m_global_light, scene.m_cam);
		SetShadowMapConstants(cb0.shadow, scene.FindRStep<RenderSmap>());
		SetEnvMapConstants(cb0.env_map, scene.m_global_envmap.get());
		auto gpu_address = upload.Add(cb0, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, true);
		cmd_list->SetGraphicsRootConstantBufferView((UINT)ERootParam::CBufFrame, gpu_address);
	}
	void Forward::SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, DrawListElement const* dle)
	{
		SetupElement(cmd_list, upload, scene, dle, dle->m_nugget->mat());
	}
	void Forward::SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, DrawListElement const* dle, Material const& material)
	{
		// Set the per-element constants
		auto& inst = *dle->m_instance;
		auto& nug = *dle->m_nugget;

		CBufNugget cb1 = {};
		SetFlags(cb1, inst, material, nug, scene.m_global_envmap != nullptr);
		SetTxfm(cb1, inst, nug.m_model, scene.m_cam);
		SetTint(cb1, inst, material);
		SetTex2Surf(cb1, inst, material);
		SetReflectivity(cb1, inst, material);
		auto gpu_address = upload.Add(cb1, D3D12_CONSTANT_BUFFER_DATA_PLACEMENT_ALIGNMENT, false);
		cmd_list->SetGraphicsRootConstantBufferView((UINT)ERootParam::CBufNugget, gpu_address);
	}
}

