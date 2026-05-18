//*********************************************
// View 3d
//  Copyright (c) Rylogic Ltd 2022
//*********************************************
#pragma once
#include "pr/view3d-12/forward.h"
#include "pr/view3d-12/shaders/shader.h"

namespace pr::rdr12::shaders
{
	namespace fwd
	{
		// This is the index order of parameters added to the root signature
		enum class ERootParam
		{
			CBufFrame = 0,
			CBufNugget,
			CBufFade,
			CBufScreenSpace,
			CBufPbrSurface,
			CBufDiag,
			DiffTexture,
			PbrMetallicTexture,
			PbrRoughnessTexture,
			PbrEmissiveTexture,
			EnvMap,
			SMap,
			ProjTex,
			OpaqueDepth,
			DiffTextureSampler,
			PbrMetallicSampler,
			PbrRoughnessSampler,
			PbrEmissiveSampler,
			AlphaColour,
			AlphaDepth,
			AlphaRtAttrs,
		};

		enum class ESampParam
		{
			EnvMap,
			SMap,
			ProjTex,
		};
	}

	struct Forward :Shader
	{
		explicit Forward(Renderer& rdr);
		void SetupFrame(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene) override;
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, DrawListElement const* dle) override;
		void SetupElement(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene, DrawListElement const* dle, Material const& material);
	};
}
