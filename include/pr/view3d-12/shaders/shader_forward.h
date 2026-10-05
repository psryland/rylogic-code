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
			ElementIndex,
			CBufFade,
			CBufScreenSpace,
			CBufPbrSurface,
			CBufDiag,
			CBufProcedural,
			DiffTexture,
			EnvMap,
			ShadowAtlas,
			ProjTex,
			PbrMetallicTexture,
			PbrRoughnessTexture,
			OpaqueDepth,
			PbrEmissiveTexture,
			Tex1Stream,
			Tex2Stream,
			Tex3Stream,
			Tex4Stream,
			PbrNormalTexture,
			DiffTextureSampler,
			PbrMetallicSampler,
			PbrRoughnessSampler,
			PbrEmissiveSampler,
			PbrNormalSampler,
			AlphaColour,
			AlphaDepth,
			AlphaRtAttrs,
			SkyTexture,
			Lights,
			ShadowViews,
			ProceduralBuffer,
			Elements,
			EnvMapPrev,
		};

		enum class ESampParam
		{
			EnvMap,
			ShadowAtlas,
			ProjTex,
		};
	}

	struct Forward :Shader
	{
		explicit Forward(Renderer& rdr);
		void SetupFrame(ID3D12GraphicsCommandList* cmd_list, GpuUploadBuffer& upload, Scene const& scene) override;

		// Upload one set of element constants per entry in 'drawlist', in drawlist order, using 'camera' for the projection.
		// Returns the GPU address of the table, for use with 'SetupElements'.
		static D3D12_GPU_VIRTUAL_ADDRESS UploadElements(GpuUploadBuffer& upload, Scene const& scene, CameraTransforms const& camera, std::span<DrawListElement const> drawlist);

		// Bind a table of element constants created by 'UploadElements'
		static void SetupElements(ID3D12GraphicsCommandList* cmd_list, D3D12_GPU_VIRTUAL_ADDRESS elements);

		// Select the entry in the bound element table used by the next draw
		static void SetupElement(ID3D12GraphicsCommandList* cmd_list, int index);
	};
}
